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
	/*
	pitchKp = 1100;
	pitchKi = 1500;
	pitchKd = 200;
	rollKp = 800;
	rollKi = 800;
	rollKd = 500;
	*/
	pitchKp = 400;
	pitchKi = 600;
	pitchKd = 180;
	rollKp = 800;
	rollKi = 800;
	rollKd = 700;

	ILimit = 20000;

	accTimeConstant = 4;

	// default nothing at all.
	rollMotorPower = 0;
	pitchMotorPower = 0;

	RCRoll.defaultAngle = 0;
	RCRoll.minAngle = TO_NERD_DEGREES(-20);
	RCRoll.maxAngle = TO_NERD_DEGREES(20);
	RCRoll.speed = 10;

	RCPitch.defaultAngle = TO_NERD_DEGREES(0);
	RCPitch.minAngle = TO_NERD_DEGREES(-90);
	RCPitch.maxAngle = TO_NERD_DEGREES(0);
	RCPitch.speed = 10;

	rollOutputRateLimit = 20;
	pitchOutputRateLimit = 20;

	rcAbsolute = true;
	axisReverseZ = true;
	axisRotateZ  = 0;
	majorAxis = 0;

	frozenGimbalPower = 10;

	LEDMask = LED_SERIAL_RX;

	mavlinkSystemId = 42;
	mavlinkComponentId = 1;
	mavlinkUseRelativealtitudes = true;
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

void Configuration::checkRAMImageValid() {
	if (crc16 != CRC()) {
		readEEPROMOrDefault();
	}
}

extern void recalcMotorPower();
extern void initSerial();
extern void initMPU6050();

static void fixme_initIMU() {
	imu.init();
}

static inline void fixme_initSensorOrientation() {
	mpu.initSensorOrientation(config.majorAxis, config.axisReverseZ, config.axisRotateZ);
}

const ConfigDef_t PROGMEM configListPGM[] = {
{ "vers", UINT8, &config.vers, NULL },

{ "pitchKp", INT16, &config.pitchKp, &initPIDs },
{ "pitchKi", INT16, &config.pitchKi, &initPIDs },
{ "pitchKd", INT16, &config.pitchKd, &initPIDs },

{ "rollKp", INT16, &config.rollKp, &initPIDs },
{ "rollKi", INT16, &config.rollKi, &initPIDs },
{ "rollKd", INT16, &config.rollKd, &initPIDs },

{ "ILimit", INT32, &config.ILimit, &initPIDs },

{ "accTime", UINT8, &config.accTimeConstant, &fixme_initIMU },

{ "pitchPower", UINT8, &config.pitchMotorPower, static_cast <void(*)()>(&recalcMotorPower) },
{ "rollPower", UINT8, &config.rollMotorPower, static_cast <void(*)()>(&recalcMotorPower) },

{ "pitchRateLimit", UINT8, &config.pitchOutputRateLimit, NULL },
{ "rollRateLimit", UINT8, &config.rollOutputRateLimit, NULL },

{ "rollMin", INT16, &config.RCRoll.minAngle, },
{ "rollMax", INT16, &config.RCRoll.maxAngle, },
{ "rollSpeed", UINT8, &config.RCRoll.speed, NULL },
{ "rollDefault", INT16, &config.RCRoll.defaultAngle, },

{ "pitchMin", INT16, &config.RCPitch.minAngle,  },
{ "pitchMax", INT16, &config.RCPitch.maxAngle,  },
{ "pitchSpeed", UINT8, &config.RCPitch.speed, NULL },
{ "pitchDefault", INT16, &config.RCPitch.defaultAngle, },

{ "rcAbsolute", BOOL, &config.rcAbsolute, NULL },
{ "majorAxis", UINT8, &config.majorAxis, 		&fixme_initSensorOrientation },
{ "reverseZ", BOOL, &config.axisReverseZ, 	&fixme_initSensorOrientation },
{ "rotateZ", UINT8, &config.axisRotateZ, 		&fixme_initSensorOrientation },

{ "LEDMask", UINT8, &config.LEDMask, 		NULL },
{ "MPU6050Addr", UINT8, &config.mpu6050Address, &initMPU6050 },
{ "baudRate", UINT32, &config.serialBaudRate, &initSerial },

{ "systemId", UINT8, &config.mavlinkSystemId, NULL },
{ "componentId", UINT8, &config.mavlinkComponentId, NULL },
{ "relativeAlt", UINT32, &config.mavlinkUseRelativealtitudes, NULL },

{ "", BOOL, NULL, NULL } // terminating NULL required !!
};

/*
 * Is now case insensitive.
 * FIXME: Stops too early on prefix match. Eg. versEEPROM gets vers.
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
			if (given == 0) found = true;
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
	recalcMotorPower();
	initPIDs();
	// mpu.setDLPFMode(config.mpuLPF);
	fixme_initSensorOrientation();
}

void initPIDs(void) {
	rollPID.setCoefficients(config.rollKp, config.rollKi / 10, config.rollKd, config.ILimit);
	pitchPID.setCoefficients(config.pitchKp, config.pitchKi / 10, config.pitchKd, config.ILimit);
}
