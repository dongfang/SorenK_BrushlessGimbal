#include "Configuration.h"
#include "Definitions.h"
#include "Util.h"
#include "Globals.h"
#include <avr/wdt.h>
#include <avr/eeprom.h>

Configuration configInEEPROM EEMEM;
Configuration config;
ConfigDef_t configDef;
ConfigUnion_t configUnion;

void Configuration::setDefaults() {
	vers = VERSION;
	versEEPROM = VERSION_EEPROM;
	pitchKp = 320;
	pitchKi = 800;
	pitchKd = 1500;
	rollKp = 650;
	rollKi = 3000;
	rollKd = 5000;
	accTimeConstant = 4;
	angleOffsetPitch = 0;
	angleOffsetRoll = 0;
	nPolesMotorPitch = 14;
	nPolesMotorRoll = 14;
	dirMotorPitch = -1;
	dirMotorRoll = 1;
	motorNumberPitch = 0;
	motorNumberRoll = 1;
	maxPWMmotorPitch = 100;
	maxPWMmotorRoll = 100;
	minRCPitch = 0;
	maxRCPitch = 90;
	minRCRoll = -30;
	maxRCRoll = 30;
	rcGain = 5;
	angleOffsetRoll = 0;
	angleOffsetPitch = 0;
	rcLPF = 5; // 0.5 sec
	rcMid = MID_RC;
	rcAbsolute = true;
	axisReverseZ = true;
	axisSwapXY = false;
	majorAxis = 2;
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
	mpu.initSensorOrientation(config.majorAxis, config.axisReverseZ, config.axisSwapXY);
}

/*
static inline void fixme_initMPUlpf() {
	mpu.setDLPFMode(config.mpuLPF);
}
*/

const ConfigDef_t PROGMEM configListPGM[] = {
{ "vers", UINT8, &config.vers, NULL },
{ "pitchKp", INT16, &config.pitchKp, &initPIDs },
{ "pitchKi", INT16, &config.pitchKi, &initPIDs },
{ "pitchKd", INT16, &config.pitchKd, &initPIDs },
{ "rollKp", INT16, &config.rollKp, &initPIDs },
{ "rollKi", INT16, &config.rollKi, &initPIDs },
{ "rollKd", INT16, &config.rollKd, &initPIDs },
{ "accTime", INT16, &config.accTimeConstant, &fixme_initIMU },
// { "mpuFilter", INT8, &config.mpuLPF, &fixme_initMPUlpf }, // This only makes everything worse.
{ "pitchOffset", INT16, &config.angleOffsetPitch, NULL },
{ "rollOffset", INT16, &config.angleOffsetRoll, NULL },
//{ "pitchDir", INT8, &config.dirMotorPitch, NULL },
//{ "rollDir", INT8, &config.dirMotorRoll, NULL },
{ "pitchMotor", UINT8, &config.motorNumberPitch, NULL },
{ "rollMotor", UINT8, &config.motorNumberRoll, NULL },
{ "pitchPower", UINT8, &config.maxPWMmotorPitch, &recalcMotorStuff },
{ "rollPower", UINT8, &config.maxPWMmotorRoll, &recalcMotorStuff },
{ "pitchMinRC", INT8, &config.minRCPitch, NULL },
{ "pitchMaxRC", INT8, &config.maxRCPitch, NULL },
{ "rollMinRC", INT8, &config.minRCRoll, NULL },
{ "rollMaxRC", INT8, &config.maxRCRoll, NULL },
{ "rcGain", INT16, &config.rcGain, NULL },
{ "rcFilter", INT16, &config.rcLPF, &initRCFilter },
{ "rcMid", INT16, &config.rcMid, NULL },
{ "rcAbsolute", BOOL, &config.rcAbsolute, NULL },
{ "majorAxis", UINT8, &config.majorAxis, 		&fixme_initSensorOrientation },
{ "reverseZ", BOOL, &config.axisReverseZ, 	&fixme_initSensorOrientation },
{ "swapXY", BOOL, &config.axisSwapXY, 		&fixme_initSensorOrientation },
{ "", BOOL, NULL, NULL } // terminating NULL required !!
};

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
	// mpu.setDLPFMode(config.mpuLPF);
	mpu.initSensorOrientation(config.majorAxis, config.axisReverseZ, config.axisSwapXY);
	initRCFilter();
}
