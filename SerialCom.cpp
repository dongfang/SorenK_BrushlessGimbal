//************************************************************************************
// general config parameter access routines
//************************************************************************************
#include "Util.h"
#include "Configuration.h"
#include "RCdecode.h"
#include "I2C.h" // debug only
#include <string.h>
#include <math.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

static uint8_t _debug;

static void writeEEPROM() {
	wdt_enable(WDTO_1S);
	config.writeEEPROM();
	printf_P(PSTR("Wrote.\r\n"));
	wdt_enable(WDT_TIMEOUT);
}

static void readEEPROM() {
	wdt_enable(WDTO_1S);
	config.readEEPROMOrDefault();
	printf_P(PSTR("Read.\r\n"));
	wdt_enable(WDT_TIMEOUT);
}

// print single parameter value
void printConfig(ConfigDef_t * def) {
	if (def != NULL) {
		printf_P(PSTR("par %s"), def->name);
		printf_P(PSTR(" "));
		switch (def->type) {
		case BOOL:
			if (*(bool*) def->address)
				printf_P(PSTR("true"));
			else
				printf_P(PSTR("false"));
			break;
		case UINT8:
			printf_P(PSTR("%u"), *(uint8_t*) def->address);
			break;
		case UINT16:
			printf_P(PSTR("%u"), *(uint16_t*) def->address);
			break;
		case UINT32:
			printf_P(PSTR("%lu"), *(uint32_t*) def->address);
			break;
		case INT8:
			printf_P(PSTR("%d"), *(int8_t*) def->address);
			break;
		case INT16:
			printf_P(PSTR("%d"), *(int16_t*) def->address);
			break;
		case INT32:
			printf_P(PSTR("%ld"), *(int32_t*) def->address);
			break;
		}
		printf_P(PSTR("\r\n"));
	} else {
		printf_P(PSTR("ERROR: illegal parameter\r\n"));
	}
}

// print all parameters
void printConfigAll(ConfigDef_t * p) {
	while (true) {
		// EEPROM can do blocks but not pgmspace.
		uint8_t i;
		for (i = 0; i < sizeof(ConfigDef_t); i++) {
			configUnion.asBytes[i] = pgm_read_byte((uint8_t*)p+i);
		}

		if (configUnion.asConfig.address == NULL)
			break;

		printConfig(&configUnion.asConfig);
		p++;
	}
	printf_P(PSTR("done.\r\n"));
}

//******************************************************************************
// general parameter modification function
//      par                           print all parameters
//      par <parameter_name>          print parameter <parameter_name>
//      par <parameter_name> <value>  set parameter <parameter_name>=<value>
//*****************************************************************************
void parameterMod() {
	char * paraName = NULL;
	char * paraValue = NULL;
	int32_t val = 0;

	if ((paraName = sCmd.next()) == NULL) {
		// no command parameter, print all config parameters
		printConfigAll((ConfigDef_t *) configListPGM);
	} else if ((paraValue = sCmd.next()) == NULL) {
		ConfigDef_t * def = getConfigDef(paraName);
		if (def != NULL) {
			printConfig(def);
		} else {
			printf_P(PSTR("ERROR: illegal parameter\r\n"));
		}
		// one parameter, print single parameter
	} else {
		// two parameters, set specified parameter
		ConfigDef_t * def = getConfigDef(paraName);
		if (def != NULL) {
			if (def->type == BOOL) {
				val = strcmp_P(paraValue, PSTR("true")) == 0;
			} else {
				val = atol(paraValue);
			}
			writeConfig(def, val);
			printConfig(def);
		} else {
			printf_P(PSTR("ERROR: illegal parameter\r\n"));
		}
	}
}
//************************************************************************************

void oscillation(uint8_t val);
void setOscillation() {
	char * paraValue;
	uint16_t val = 0;

	paraValue = sCmd.next();
	if (paraValue == NULL) {
		val = 10;
	} else {
		val = atol(paraValue);
		if (val < 1)
			val = 1;
		else if (val > 255)
			val = 255;
	}
	oscillation(val);
}

void transients(uint8_t axes, int16_t val);
void setTransients() {
	char * axisName;
	char * paraValue;
	uint8_t axis = 0;
	int16_t val = 0;

	if ((axisName = sCmd.next()) == NULL) {
		// do nothing
	} else {
		paraValue = sCmd.next();
		if (paraValue == NULL)
			val = 10;
		else {
			val = atol(paraValue);
			if (val < -500)
				val = -500;
			else if (val > 500)
				val = 500;
		}
	}

	if (!strcmp_P(axisName, PSTR("roll"))) {
		axis = 1;
	} else if (!strcmp_P(axisName, PSTR("pitch"))) {
		axis = 2;
	} else if (!strcmp_P(axisName, PSTR("both"))) {
		axis = 3;
	}
	transients(axis, val);
}

void setDefaultParametersAndUpdate() {
	config.setDefaults();
	updateAllParameters();
	printf_P(PSTR("Reverted to defaults.\r\n"));
}

void printHelpUsage() {
	printf_P(PSTR("This gives you a list of all commands with usage.\r\n"));
	printf_P(PSTR("Explanations are in brackets(), use integer values only !\r\n"));
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("These are the preferred commands, use them for new GUIs.\r\n"));
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("sd    Set Defaults\r\n"));
	printf_P(PSTR("we    Writes active config to eeprom\r\n"));
	printf_P(PSTR("re    Restores values from eeprom to active config\r\n"));
	printf_P(PSTR("cal   Recalibrates the gyro\r\n"));
	printf_P(PSTR("level Sets level (place gimbal firmly level and run)\r\n"));
	printf_P(PSTR("debug <category> Prints troubleshooting info\r\n"));
	printf_P(PSTR("        debug usage:\r\n"));
	printf_P(PSTR("        debug off               ... turns off debug\r\n"));
	printf_P(PSTR("        debug acc               ... prints accelerometer values\r\n"));
	printf_P(PSTR("        debug gyro              ... prints gyro values\r\n"));
	printf_P(PSTR("        debug att               ... prints attitude\r\n"));
	printf_P(PSTR("        debug pid               ... prints PID outputs\r\n"));
	printf_P(PSTR("par   <parName> <parValue>   General parameter read/set command\r\n"));
	printf_P(PSTR("        example usage:\r\n"));
	printf_P(PSTR("        par                     ... list all config parameters\r\n"));
	printf_P(PSTR("        par pitchKi             ... list pitchKi\r\n"));
	printf_P(PSTR("        par pitchKi 12000       ... set pitchKi to 12000\r\n"));
#ifdef DO_PERFORMANCE
	printf_P(PSTR("perf	 Prints performace info\r\n"));
#endif
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("osc <n> Starts oscillation with speed n (0 to stop)\r\n"));
	printf_P(PSTR("trans <rol|pitch|both|off> n Starts transients with amplitude ntest\r\n"));
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("reset  Reboot system\r\n"));
	printf_P(PSTR("help   print this output\r\n"));
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Commands and parameter names are case-insensitive.\r\n"));
}

void unrecognized(const char *command) {
	printf_P(PSTR("Huh? type \"help\" for help ...\r\n"));
}

#define DEBUG_OFF 0
#define DEBUG_ACCVALUES 1
#define DEBUG_GYROVALUES 2
#define DEBUG_ESTG 3
#define DEBUG_ATTITUDE 4
#define DEBUG_PID 5
#define DEBUG_DGYRO 6
#define DEBUG_I2C 7
#define DEBUG_RC 8

static const char DEBUG_OFF_ARG[] PROGMEM = "off";
static const char DEBUG_ACCVALUES_ARG[] PROGMEM = "acc";
static const char DEBUG_GYROVALUES_ARG[] PROGMEM = "gyro";
static const char DEBUG_ESTG_ARG[] PROGMEM = "estg";
static const char DEBUG_ATTITUDE_ARG[] PROGMEM = "att";
static const char DEBUG_PID_ARG[] PROGMEM = "pid";
static const char DEBUG_DGYRO_ARG[] PROGMEM = "dgyro";
static const char DEBUG_I2C_ARG[] PROGMEM = "i2c";
static const char DEBUG_RC_ARG[] PROGMEM = "rc";

static PGM_P const DEBUG_COMMANDS[] PROGMEM = { DEBUG_OFF_ARG, DEBUG_ACCVALUES_ARG, DEBUG_GYROVALUES_ARG,
		DEBUG_ESTG_ARG, DEBUG_ATTITUDE_ARG, DEBUG_PID_ARG, DEBUG_DGYRO_ARG, DEBUG_I2C_ARG, DEBUG_RC_ARG };

/*
 void toggleEcho() {
 sCmd.setEcho(!sCmd.getEcho());
 }
 */

void debugControl() {
	char * itemName = NULL;
	if ((itemName = sCmd.next()) == NULL) {
		// no command parameter, print all config parameters
		printf_P(PSTR("Debug off.\r\n"));
		_debug = 0;
		return;
	}
	bool found;
	uint8_t i;
	for (i = 0; i < sizeof(DEBUG_COMMANDS) / 2; i++) {
		PGM_P ptr = (PGM_P) pgm_read_word(&DEBUG_COMMANDS[i]);
		uint8_t j = 0;
		found = true;
		char stored;
		do {
			stored = pgm_read_byte(ptr+j);
			char given = itemName[j++];
			if (tolower(stored) != tolower(given)) {
				found = false;
				break;
			}
		} while (stored);
		if (found)
			break;
	}
	if (found) {
		_debug = i;
	} else {
		printf_P(PSTR("Huh? Use ")); //\"off\", \"acc\", \"gyro\" or \"att\".\r\n"));
		for (i = 0; i < sizeof(DEBUG_COMMANDS) / 2; i++) {
			PGM_P ptr = (PGM_P) pgm_read_word(&DEBUG_COMMANDS[i]);
			printf_P(PSTR("\"%S\""), ptr);
			if (i < sizeof(DEBUG_COMMANDS) / 2 - 2)
				printf_P(PSTR(", "));
			else if (i == sizeof(DEBUG_COMMANDS) / 2 - 2)
				printf_P(PSTR(" or "));
		}
		printf_P(PSTR("\r\n"));
	}
}

void debug() {
	char temp[16];
	//uint8_t i;
	wdt_reset();
	int16_t x, y, z;

	switch (_debug) {
	case DEBUG_ATTITUDE:
		cli();
		x = imu.angle_i16[ROLL];
		y = imu.angle_i16[PITCH];
		sei();
		sprintf_P(temp, PSTR("%d"), x / (32768 / 1800));
		printf_P(PSTR("att: roll %s"), temp);
		sprintf_P(temp, PSTR("%d"), y / (32768 / 1800));
		//insertComma(temp);
		printf_P(PSTR("\tpitch %s\r\n"), temp);
		break;
	case DEBUG_ACCVALUES:
		cli();
		x = imu.acc[X];
		y = imu.acc[Y];
		z = imu.acc[Z];
		sei();
		printf_P(PSTR("acc: x %d\t y %d\t z %d\r\n"), x, y, z);
		break;
	case DEBUG_GYROVALUES:
		cli();
		x = imu.gyro[X];
		y = imu.gyro[Y];
		z = imu.gyro[Z];
		sei();
		printf_P(PSTR("gyro: xt %d\t y %d\t z %d\r\n"), x, y, z);
		break;
	case DEBUG_ESTG:
		cli();
		x = (int) imu.estG[X];
		y = (int) imu.estG[Y];
		z = (int) imu.estG[Z];
		sei();
		printf_P(PSTR("estg: x %d\t y %d\t, z %d\t\r\n"), x, y, z);
		break;
	case DEBUG_PID:
		printf_P(PSTR("pid: roll %d\t, rolldelta %d\t pitch %d\tpitchdelta:%d\r\n"), rollPIDVal, rollPIDDelta,
				pitchPIDVal, pitchPIDDelta);
		break;
	case DEBUG_DGYRO:
		printf_P(PSTR("dgyro: cosp %d\t sinp %d\t roll %d\t pitch%d\r\n"), imu.cosPitch, imu.sinPitch, imu.rollRate,
				imu.pitchRate);
		break;
	case DEBUG_I2C:
		printf_P(PSTR("errors %u\r\n"), i2c_errors_count);
		break;
	case DEBUG_RC:
		printf_P(PSTR("rc: switch %d\troll %d\tvalid %S\tpitch %d\tvalid %S\r\n"), switchPos,
				rcData[RC_DATA_ROLL].setpoint, (rcData[RC_DATA_ROLL].isValid ? PSTR("y") : PSTR("n")),
				rcData[RC_DATA_PITCH].setpoint, (rcData[RC_DATA_PITCH].isValid ? PSTR("y") : PSTR("n")));
		break;
	default:
		break;
	}
}

void reset() {
	watchdogResetWasIntended = true;
	wdt_enable(WDTO_15MS);
	while (1)
		;
}

void calibrateGyro() {
	calibrateSensor(MPU6050::GYRO);
}

void calibrateAcc() {
	calibrateSensor(MPU6050::ACC);
}

void showSensorCal() {
	printf_P(PSTR("Gyro roll\t%d, pitch\t%d, yaw\t%d\r\n"), mpu.sensorOffset[0], mpu.sensorOffset[1],
			mpu.sensorOffset[2]);
	printf_P(PSTR("Acc X\t%d, Y\t%d, Z\t%d\r\n"), mpu.sensorOffset[3], mpu.sensorOffset[4], mpu.sensorOffset[5]);
}

void setSerialProtocol() {
	// Setup callbacks for SerialCommand commands
	sCmd.addCommand("sd", setDefaultParametersAndUpdate);
	sCmd.addCommand("we", writeEEPROM);
	sCmd.addCommand("re", readEEPROM);
	sCmd.addCommand("par", parameterMod);
	sCmd.addCommand("cal", calibrateGyro);
	sCmd.addCommand("level", calibrateAcc);
	sCmd.addCommand("osc", setOscillation);
	sCmd.addCommand("trans", setTransients);
	sCmd.addCommand("help", printHelpUsage);
#ifdef DO_PERFORMANCE
	sCmd.addCommand("perf", reportPerformance);
#endif
	sCmd.addCommand("debug", debugControl);
	sCmd.addCommand("reset", reset);
	sCmd.addCommand("cal", showSensorCal);
	sCmd.setDefaultHandler(unrecognized); // Handler for command that isn't matched  (says "What?")
}
