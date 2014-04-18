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
//#include "Globals.h"
#include "Configuration.h"
#include "RCdecode.h"

#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

// Fixme
extern void recalcMotorStuff();

// types of config parameters
enum confType {
	BOOL, INT8, INT16, INT32, UINT8, UINT16, UINT32
};

#define CONFIGNAME_MAX_LEN 17

struct ConfigDef_t {
	char name[CONFIGNAME_MAX_LEN]; // name of config parameter
	confType type; // type of config parameters
	void * address; // address of config parameter
	void (*updateFunction)(void); // function is called when parameter update happens
};

ConfigDef_t configDef;

// access descriptor as array of bytes as well
union ConfigUnion_t {
	ConfigDef_t asConfig;
	uint8_t asBytes[sizeof(ConfigDef_t)];
};

ConfigUnion_t configUnion;

static inline void fixme_writeEEPROM() {
	config.writeEEPROM();
}

static inline void fixme_readEEPROM() {
	config.readEEPROMOrDefault();
}

static inline void fixme_initIMU() {
	imu.init();
}

static inline void fixme_initSensorOrientation() {
	imu.initSensorOrientation();
}

// list of all config parameters
// to be accessed by par command
//
// descriptor is stored in PROGMEN to preserve RAM space
// see http://www.arduino.cc/en/Reference/PROGMEM
// and http://jeelabs.org/2011/05/23/saving-ram-space/
const ConfigDef_t PROGMEM configListPGM[] = {
{ "vers", UINT8, &config.vers, NULL },
{ "pitchKp", INT32, &config.pitchKp, &initPIDs },
{ "pitchKi", INT32, &config.pitchKi, &initPIDs },
{ "pitchKd", INT32, &config.pitchKd, &initPIDs },
{ "rollKp", INT32, &config.rollKp, &initPIDs },
{ "rollKi", INT32, &config.rollKi, &initPIDs },
{ "rollKd", INT32, &config.rollKd, &initPIDs },
{ "timeConstant", INT16, &config.accTimeConstant, &fixme_initIMU },
{ "mpuLPF", INT8, &config.mpuLPF, &initMPUlpf },
{ "angleOffsetPitch", INT16, &config.angleOffsetPitch, NULL },
{ "angleOffsetRoll", INT16, &config.angleOffsetRoll, NULL },
{ "dirMotorPitch", INT8, &config.dirMotorPitch, NULL },
{ "dirMotorRoll", INT8, &config.dirMotorRoll, NULL },
{ "motorNumberPitch", UINT8, &config.motorNumberPitch, NULL },
{ "motorNumberRoll", UINT8, &config.motorNumberRoll, NULL },
{ "maxPWMmotorPitch", UINT8, &config.maxPWMmotorPitch, &recalcMotorStuff },
{ "maxPWMmotorRoll", UINT8, &config.maxPWMmotorRoll, &recalcMotorStuff },
{ "minRCPitch", INT8, &config.minRCPitch, NULL },
{ "maxRCPitch", INT8, &config.maxRCPitch, NULL },
{ "minRCRoll", INT8, &config.minRCRoll, NULL },
{ "maxRCRoll", INT8, &config.maxRCRoll, NULL },
{ "rcGain", INT16, &config.rcGain, NULL },
{ "rcLPF", INT16, &config.rcLPF, &initRC },
{ "rcMid", INT16, &config.rcMid, NULL },
{ "rcAbsolute", BOOL, &config.rcAbsolute, NULL },
{ "majorAxis", UINT8, &config.majorAxis, 		&fixme_initSensorOrientation },
{ "axisReverseZ", BOOL, &config.axisReverseZ, 	&fixme_initSensorOrientation },
{ "axisSwapXY", BOOL, &config.axisSwapXY, 		&fixme_initSensorOrientation },
{ "", BOOL, NULL, NULL } // terminating NULL required !!
};

// find Config Definition for named parameter
ConfigDef_t * getConfigDef(char* name) {
	bool found = false;
	ConfigDef_t * p = (ConfigDef_t *) configListPGM;

	while (true) {
		uint8_t i;
		for (i = 0; i < sizeof(ConfigDef_t); i++) {
			configUnion.asBytes[i] = pgm_read_byte((uint8_t*)p + i);
		}
		if (configUnion.asConfig.address == NULL)
			break;
		if (strncmp(configUnion.asConfig.name, name, CONFIGNAME_MAX_LEN) == 0) {
			found = true;
			break;
		}
		p++;
	}
	if (found)
		return &configUnion.asConfig;
	else
		return NULL;
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

// write single parameter with value
void writeConfig(ConfigDef_t* def, int32_t val) {
	if (def != NULL) {
		 switch (def->type) {
		 case BOOL   : *(bool *)(def->address)     = val; break;
		 case UINT8  : *(uint8_t *)(def->address)  = val; break;
		 case UINT16 : *(uint16_t *)(def->address) = val; break;
		 case UINT32 : *(uint32_t *)(def->address) = val; break;
		 case INT8   : *(int8_t *)(def->address)   = val; break;
		 case INT16  : *(int16_t *)(def->address)  = val; break;
		 case INT32  : *(int32_t *)(def->address)  = val; break;
		 }
		 //def->address = (void*)val;
		// call update function
		if (def->updateFunction != NULL)
			def->updateFunction();
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

void updateAllParameters() {
	recalcMotorStuff();
	initPIDs();
	imu.init();
	initMPUlpf();
	//imu.initSensorOrientation();
	initRCFilter();
	initRC();
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

void setSerialProtocol() {
	// Setup callbacks for SerialCommand commands
	sCmd.addCommand("sd", setDefaultParametersAndUpdate);
	sCmd.addCommand("we", fixme_writeEEPROM);
	sCmd.addCommand("re", fixme_readEEPROM);
	sCmd.addCommand("par", parameterMod);
	sCmd.addCommand("gc", calibrateGyro);
	sCmd.addCommand("he", printHelpUsage);
	sCmd.setDefaultHandler(unrecognized); // Handler for command that isn't matched  (says "What?")
}
