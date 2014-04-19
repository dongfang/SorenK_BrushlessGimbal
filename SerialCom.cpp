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

uint8_t _debug;

static inline void fixme_writeEEPROM() {
	config.writeEEPROM();
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
		// one parameter, print single parameter
		printConfig(getConfigDef(paraName));
	} else {
		// two parameters, set specified parameter
		val = atol(paraValue);
		writeConfig(getConfigDef(paraName), val);
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
	printf_P(PSTR("these are the preferred commands, use them for new GUIs.\r\n"));
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("SD    (Set Defaults)\r\n"));
	printf_P(PSTR("WE    (Writes active config to eeprom)\r\n"));
	printf_P(PSTR("RE    (Restores values from eeprom to active config)\r\n"));
	printf_P(PSTR("GC    (Recalibrates the Gyro Offsets)\r\n"));
	printf_P(PSTR("par <parName> <parValue>   (general parameter read/set command)\r\n"));
	printf_P(PSTR("    example usage:\r\n"));
	printf_P(PSTR("       par                     ... list all config parameters\r\n"));
	printf_P(PSTR("       par pitchKi             ... list pitchKi\r\n"));
	printf_P(PSTR("       par pitchKi 12000       ... set pitchKi to 12000\r\n"));
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("HE     (print this output)\r\n"));
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Note: command input is case-insensitive, commands are accepted in both upper/lower case\r\n"));
}

void unrecognized(const char *command) {
	printf_P(PSTR("What? type in HE for Help ..."));
}


#define DEBUG_OFF 0
#define DEBUG_ACCVALUES 1
#define DEBUG_GYROVALUES 2
#define DEBUG_ATTITUDE 3

static const char DEBUG_OFF_CMD[] PROGMEM = "off";
static const char DEBUG_ACCVALUES_CMD[] PROGMEM = "acc";
static const char DEBUG_GYROVALUES_CMD[] PROGMEM = "gyros";
static const char DEBUG_ATTITUDE_CMD[] PROGMEM = "att";

static PGM_P const DEBUG_COMMANDS[] PROGMEM = {
	DEBUG_OFF_CMD, DEBUG_ACCVALUES_CMD, DEBUG_GYROVALUES_CMD, DEBUG_ATTITUDE_CMD
};

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
	for (i=0; i<sizeof(DEBUG_COMMANDS)/2 && !found; i++) {
		PGM_P ptr = (PGM_P)pgm_read_word(&DEBUG_COMMANDS[i]);
		if (strncmp_P(itemName, ptr, 6) == 0) found = true;
	}
	if (found) {
		_debug = i;
	} else {
		printf_P(PSTR("Huh? Use \"off\", \"acc\", \"gyros\" or \"att\".\r\n"));
	}
}

void insertComma(char* temp) {
	uint8_t end = strlen(temp);
	uint8_t ip = end - 2;
	for (uint8_t i=end; i>=ip; i--) {
		temp[i+1] = temp[i];
	}
	temp[ip] = '.';
}

void debug() {
	char temp[10];
	switch(_debug) {
	case DEBUG_ATTITUDE:
		// This stunt is to avoid having to draw in the printf_flt stuff which is a pain.
		sprintf_P(temp, PSTR("%ld"), imu.angle[0]);
		insertComma(temp);
		printf_P(PSTR("roll:%s"), temp);
		sprintf_P(temp, PSTR("%ld"), imu.angle[1]);
		insertComma(temp);
		printf_P(PSTR("pitch:%f\r\n"), temp);
		break;
	case DEBUG_ACCVALUES:	printf_P(PSTR("x:%d, y:%d, z:%d\r\n"),  imu.acc[X], imu.acc[Y], imu.acc[Z]); break;
	case DEBUG_GYROVALUES:	printf_P(PSTR("x:%d, y:%d, z:%d\r\n"),  imu.gyro[X], imu.gyro[Y], imu.gyro[Z]); break;
	default: break;
	}
}

void setSerialProtocol() {
	// Setup callbacks for SerialCommand commands
	sCmd.addCommand("sd", setDefaultParametersAndUpdate);
	sCmd.addCommand("we", fixme_writeEEPROM);
	sCmd.addCommand("re", fixme_readEEPROM);
	sCmd.addCommand("par", parameterMod);
	sCmd.addCommand("gc", calibrateGyro);
	sCmd.addCommand("he", printHelpUsage);
	sCmd.addCommand("debug", debugControl);

	sCmd.setDefaultHandler(unrecognized); // Handler for command that isn't matched  (says "What?")
}
