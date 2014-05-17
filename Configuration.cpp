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

LiveControlAxisDef liveControlDefs[2];

void initControlLimits() {
	liveControlDefs[ROLL].convertFrom(&config.controlInput[ROLL]);
	liveControlDefs[PITCH].convertFrom(&config.controlInput[PITCH]);
}

void Configuration::setDefaults() {
	vers = VERSION;
	versEEPROM = VERSION_EEPROM;
	/*
	pitchKp = 400;
	pitchKi = 1000;
	pitchKd = 160;
	rollKp = 750;
	rollKi = 1200;
	rollKd = 400;
	*/
	pitchKp = 300;
	pitchKi = 600;
	pitchKd = 150;
	rollKp = 650;
	rollKi = 800;
	rollKd = 500;

	ILimit = 15000;

	accTimeConstant = 4;

	// default nothing at all.
	//rollMotorPower = 180;
	//pitchMotorPower = 165;
	rollMotorPower = 75;
	pitchMotorPower = 75;

	controlInput[ROLL].defaultAngle = 0;
	controlInput[ROLL].minAngle = -20;
	controlInput[ROLL].maxAngle = 20;
	controlInput[ROLL].maxSlewRate = 15;

	controlInput[PITCH].defaultAngle = 0;
	controlInput[PITCH].minAngle = -90;
	controlInput[PITCH].maxAngle = 0;
	controlInput[PITCH].maxSlewRate = 15;

	rollOutputRateLimit = 35;
	pitchOutputRateLimit = 35;

	yawServoLimit = 90;
	yawServoDirection = 1;

	rcAbsolute = true;
	axisReverseZ = true;
	axisRotateZ  = 0;
	majorAxis = 0;

	frozenGimbalPower = 10;

	LEDMask = LED_SCHEDULER_OVERLOAD_MASK;
	serialBaudRate = 115200;

	mavlinkSystemId = 42;
	mavlinkComponentId = 1;
	mavlinkUseRelativealtitudes = true;

	retractedServoVal = 30;
	extendedServoVal = 60;

	mpu6050Address = 0x68;
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
		updateAllParameters();
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

{ "rollMin", INT8, &config.controlInput[ROLL].minAngle, &initControlLimits },
{ "rollMax", INT8, &config.controlInput[ROLL].maxAngle, &initControlLimits },
{ "rollSpeed", UINT8, &config.controlInput[ROLL].maxSlewRate, &initControlLimits },
{ "rollDefault", INT8, &config.controlInput[ROLL].defaultAngle, &initControlLimits },

{ "pitchMin", INT8, &config.controlInput[PITCH].minAngle, &initControlLimits },
{ "pitchMax", INT8, &config.controlInput[PITCH].maxAngle, &initControlLimits },
{ "pitchSpeed", UINT8, &config.controlInput[PITCH].maxSlewRate, &initControlLimits },
{ "pitchDefault", INT8, &config.controlInput[PITCH].defaultAngle, &initControlLimits },

{ "rcAbsolute", BOOL, &config.rcAbsolute, NULL },
{ "majorAxis", UINT8, &config.majorAxis, 		&fixme_initSensorOrientation },
{ "reverseZ", BOOL, &config.axisReverseZ, 	&fixme_initSensorOrientation },
{ "rotateZ", UINT8, &config.axisRotateZ, 		&fixme_initSensorOrientation },

{ "LEDMask", UINT8, &config.LEDMask, 		NULL },
{ "MPU6050Addr", UINT8, &config.mpu6050Address, &initMPU6050 },
{ "baudRate", UINT32, &config.serialBaudRate, &initSerial },

{ "retractServo", UINT8, &config.retractedServoVal, NULL },
{ "extendServo", UINT8, &config.extendedServoVal, NULL },

{ "systemId", UINT8, &config.mavlinkSystemId, NULL },
{ "componentId", UINT8, &config.mavlinkComponentId, NULL },
{ "relativeAlt", BOOL, &config.mavlinkUseRelativealtitudes, NULL },

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

void initPIDs(void) {
	rollPID.setCoefficients(config.rollKp, config.rollKi / 10, config.rollKd, config.ILimit);
	pitchPID.setCoefficients(config.pitchKp, config.pitchKi / 10, config.pitchKd, config.ILimit);
}

void updateAllParameters() {
	recalcMotorPower();
	initPIDs();
	initControlLimits();
	fixme_initSensorOrientation();
}

