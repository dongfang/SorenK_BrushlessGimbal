//************************************************************************************
// general config parameter access routines
//************************************************************************************
/*
 Groups?
 1) MPU orientation
 2) Filters
 3) PIDs
 4) Motors
 5) RC in
 */
#include "Util.h"
#include "Configuration.h"
#include "RCdecode.h"

#include <string.h>
#include <math.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

static uint8_t _debug;

static inline void writeEEPROM() {
	wdt_enable(WDTO_1S);
	config.writeEEPROM();
	wdt_enable(WDT_TIMEOUT);
}

static inline void fixme_readEEPROM() {
	config.readEEPROMOrDefault();
}

// print single parameter value
void printConfig(ConfigDef_t * def) {
	if (def != NULL) {
		printf_P(PSTR("%s"), def->name);
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
	printf_P(PSTR("cal   Recalibrates the Gyro Offsets\r\n"));
	printf_P(PSTR("debug <category> Prints troubleshooting info\r\n"));
	printf_P(PSTR("        debug usage:\r\n"));
	printf_P(PSTR("        debug off               ... turns off debug\r\n"));
	printf_P(PSTR("        debug acc               ... prints accelerometer values\r\n"));
	printf_P(PSTR("        debug gyro              ... prints gyro values\r\n"));
	printf_P(PSTR("        debug att               ... prints attitude\r\n"));
	printf_P(PSTR("par   <parName> <parValue>   General parameter read/set command\r\n"));
	printf_P(PSTR("        example usage:\r\n"));
	printf_P(PSTR("        par                     ... list all config parameters\r\n"));
	printf_P(PSTR("        par pitchKi             ... list pitchKi\r\n"));
	printf_P(PSTR("        par pitchKi 12000       ... set pitchKi to 12000\r\n"));
#ifdef DO_PERFORMANCE
	printf_P(PSTR("perf	 Prints performace info\r\n"));
#endif
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("reset  Reboot system\r\n"));
	printf_P(PSTR("help   print this output\r\n"));
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Note: command input is case-insensitive, commands are accepted in both upper/lower case\r\n"));
}

void unrecognized(const char *command) {
	printf_P(PSTR("Huh? type \"help\" for help ...\r\n"));
}

#define DEBUG_OFF 0
#define DEBUG_ACCVALUES 1
#define DEBUG_GYROVALUES 2
#define DEBUG_ESTG 3
#define DEBUG_ATTITUDE 4

static const char DEBUG_OFF_ARG[] PROGMEM = "off";
static const char DEBUG_ACCVALUES_ARG[] PROGMEM = "acc";
static const char DEBUG_GYROVALUES_ARG[] PROGMEM = "gyro";
static const char DEBUG_ESTG_ARG[] PROGMEM = "estg";
static const char DEBUG_ATTITUDE_ARG[] PROGMEM = "att";

static PGM_P const DEBUG_COMMANDS[] PROGMEM = {
	DEBUG_OFF_ARG, DEBUG_ACCVALUES_ARG, DEBUG_GYROVALUES_ARG, DEBUG_ESTG_ARG, DEBUG_ATTITUDE_ARG
};

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
	bool found = false;
	uint8_t i;
	for (i=0; i<sizeof(DEBUG_COMMANDS)/2; i++) {
		PGM_P ptr = (PGM_P)pgm_read_word(&DEBUG_COMMANDS[i]);
		if (strcmp_P(itemName, ptr) == 0) found = true;
		if (found) break;
	}
	if (found) {
		_debug = i;
	} else {
		printf_P(PSTR("Huh? Use "));//\"off\", \"acc\", \"gyro\" or \"att\".\r\n"));
		for (i=0; i<sizeof(DEBUG_COMMANDS)/2; i++) {
			PGM_P ptr = (PGM_P)pgm_read_word(&DEBUG_COMMANDS[i]);
			printf_P(PSTR("\"%S\""), ptr);
			if (i<sizeof(DEBUG_COMMANDS)/2-2)
				printf_P(PSTR(", "));
			else if (i == sizeof(DEBUG_COMMANDS)/2-2)
				printf_P(PSTR(" or "));
		}
		printf_P(PSTR("\r\n"));
	}
}

void insertComma(char* temp) {
	// index of the \0
	uint8_t end = strlen(temp);
	uint8_t ip = end - LOG_ANGLE_SCALING;
	for (uint8_t i=end; i>=ip; i--) {
		temp[i+1] = temp[i];
	}
	temp[ip] = '.';
}

void debug() {
	char temp[15];
	// printf_P(PSTR("debug %d\r\n"), _debug);
	switch(_debug) {
	case DEBUG_ATTITUDE:
		// This stunt is to avoid having to draw in the printf_flt stuff which is a pain.
		sprintf_P(temp, PSTR("%ld"), imu.angle_md[ROLL]);
		insertComma(temp);
		printf_P(PSTR("roll:%s"), temp);
		sprintf_P(temp, PSTR("%ld"), imu.angle_md[PITCH]);
		insertComma(temp);
		printf_P(PSTR("\tpitch:%s\r\n"), temp);
		break;
	case DEBUG_ACCVALUES:	printf_P(PSTR("x:%d, y:%d, z:%d\r\n"),  imu.acc[X], imu.acc[Y], imu.acc[Z]); break;
	case DEBUG_GYROVALUES:	printf_P(PSTR("x:%d, y:%d, z:%d\r\n"),  imu.gyro[X], imu.gyro[Y], imu.gyro[Z]); break;
	case DEBUG_ESTG:		printf_P(PSTR("x:%d, y:%d, z:%d (mag: %ld)\r\n"),
			(int)(imu.estG[X]/imu.accMagnitude_g_100),
			(int)(imu.estG[Y]/imu.accMagnitude_g_100),
			(int)(imu.estG[Z]/imu.accMagnitude_g_100),
			imu.accMagnitude_g_100
			);
	break;
	default: break;
	}
}

void reset() {
	watchdogResetWasIntended = true;
	wdt_enable(WDTO_15MS);
	while(1);
}

void showGyroCal() {
	printf_P(PSTR("x:%d, y:%d, z:%d\r\n"), imu.gyroOffset[0], imu.gyroOffset[1], imu.gyroOffset[2]);
}

void setSerialProtocol() {
	// Setup callbacks for SerialCommand commands
	sCmd.addCommand("sd", setDefaultParametersAndUpdate);
	sCmd.addCommand("we", writeEEPROM);
	sCmd.addCommand("re", fixme_readEEPROM);
	sCmd.addCommand("par", parameterMod);
	sCmd.addCommand("cal", calibrateGyro);
	// sCmd.addCommand("echo", toggleEcho);
	sCmd.addCommand("help", printHelpUsage);
#ifdef DO_PERFORMANCE
	sCmd.addCommand("perf", reportPerformance);
#endif
	sCmd.addCommand("debug", debugControl);
	sCmd.addCommand("reset", reset);
	sCmd.addCommand("gcal", showGyroCal);
	sCmd.setDefaultHandler(unrecognized); // Handler for command that isn't matched  (says "What?")
}
