#include "Configuration.h"
#include "Definitions.h"
#include "Util.h"
#include "Globals.h"
#include <avr/wdt.h>
#include <avr/eeprom.h>

static Configuration configInEEPROM EEMEM;
Configuration config;
ConfigDef_t configDef;
ConfigUnion_t configUnion;

void Configuration::setDefaults() {
	vers = VERSION;
	versEEPROM = VERSION_EEPROM;
	pitchKp = 7500;
	pitchKi = 10000;
	pitchKd = 11000;
	rollKp = 20000;
	rollKi = 8000;
	rollKd = 30000;
	accTimeConstant = 4;
	mpuLPF = 0;
	angleOffsetPitch = 0;
	angleOffsetRoll = 0;
	nPolesMotorPitch = 14;
	nPolesMotorRoll = 14;
	dirMotorPitch = 1;
	dirMotorRoll = -1;
	motorNumberPitch = 0;
	motorNumberRoll = 1;
	maxPWMmotorPitch = 100;
	maxPWMmotorRoll = 100;
	minRCPitch = 0;
	maxRCPitch = 90;
	minRCRoll = -30;
	maxRCRoll = 30;
	rcGain = 5;
	rcLPF = 5; // 0.5 sec
	rcMid = MID_RC;
	rcAbsolute = true;
	calibrateGyro = true;
	axisReverseZ = true;
	axisSwapXY = false;
	majorAxis = 0;
	crc16 = 0;
}

uint16_t Configuration::CRC() {
	return ::crc16((uint8_t*)this, sizeof(Configuration) - 2);
}

void Configuration::writeEEPROM() {
	crc16 = CRC();
	eeprom_write_block(this, &configInEEPROM, sizeof(Configuration));
}

void Configuration::readEEPROMOrDefault() {
	wdt_reset();
	eeprom_read_block(this, &configInEEPROM, sizeof(Configuration));
	if (crc16 == CRC()) {
		if ((vers != VERSION) || (versEEPROM != VERSION_EEPROM)) {
			printf_P(PSTR("EEPROM version mismatch, initialize EEPROM"));
			setDefaults();
			writeEEPROM();
		}
	} else {
		wdt_reset();
		// crc failed intialize directly here, as readEEPROM is void
		printf_P(PSTR("EEPROM CRC failed, initialize EEPROM\r\n"));
		setDefaults();
		writeEEPROM();
	}
}

// Fixme
extern void recalcMotorStuff();
extern void initRCFilter();

static inline void fixme_initIMU() {
	imu.init();
}

static inline void fixme_initSensorOrientation() {
	mpu.initSensorOrientation();
}

static inline void fixme_initMPUlpf() {
	mpu.setDLPFMode(config.mpuLPF);
}

const ConfigDef_t PROGMEM configListPGM[] = {
{ "vers", UINT8, &config.vers, NULL },
{ "pitchKp", INT32, &config.pitchKp, &initPIDs },
{ "pitchKi", INT32, &config.pitchKi, &initPIDs },
{ "pitchKd", INT32, &config.pitchKd, &initPIDs },
{ "rollKp", INT32, &config.rollKp, &initPIDs },
{ "rollKi", INT32, &config.rollKi, &initPIDs },
{ "rollKd", INT32, &config.rollKd, &initPIDs },
{ "accTimeConstant", INT16, &config.accTimeConstant, &fixme_initIMU },
{ "mpuLPF", INT8, &config.mpuLPF, &fixme_initMPUlpf },
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
{ "rcLPF", INT16, &config.rcLPF, &initRCFilter },
{ "rcMid", INT16, &config.rcMid, NULL },
{ "rcAbsolute", BOOL, &config.rcAbsolute, NULL },
{ "majorAxis", UINT8, &config.majorAxis, 		&fixme_initSensorOrientation },
{ "axisReverseZ", BOOL, &config.axisReverseZ, 	&fixme_initSensorOrientation },
{ "axisSwapXY", BOOL, &config.axisSwapXY, 		&fixme_initSensorOrientation },
{ "", BOOL, NULL, NULL } // terminating NULL required !!
};

/*
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
*/

/*
 * Is now case insensitive.
 */
ConfigDef_t * getConfigDef(char* name) {
	bool found = false;
	ConfigDef_t * p = (ConfigDef_t *) configListPGM;
	uint8_t i;

	while (!found) {
		for (i = 0; i < sizeof(ConfigDef_t); i++) {
			configUnion.asBytes[i] = pgm_read_byte((uint8_t*)p + i);
		}
		if (configUnion.asConfig.address == NULL)
			break;
		for (i = 0; i < CONFIGNAME_MAX_LEN && !found; i++) {
			char stored = configUnion.asConfig.name[i];
			char given = name[i];
			if (stored == 0) found = true;
			else if (tolower(stored) != tolower(given)) break;
		}
		if (found) {
			break;
		}
		p++;
	}
	if (found) {
		return &configUnion.asConfig;
	}
	else
		return NULL;
}

// write single parameter with value
void writeConfig(ConfigDef_t* def, int32_t val) {
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
}

void updateAllParameters() {
	recalcMotorStuff();
	initPIDs();
	mpu.setDLPFMode(config.mpuLPF);
	mpu.initSensorOrientation();
	initRCFilter();
}
