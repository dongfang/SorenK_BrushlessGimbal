#include "Configuration.h"
#include "Definitions.h"
#include "Util.h"
#include <avr/wdt.h>
#include <avr/eeprom.h>

static Configuration configInEEPROM EEMEM;

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
	return ::crc16((uint8_t*)this, sizeof(this) - 2);
}

void Configuration::writeEEPROM() {
	wdt_reset();
	crc16 = CRC();
	eeprom_write_block(this, &configInEEPROM, sizeof(this));
}

void Configuration::readEEPROMOrDefault() {
	wdt_reset();
	eeprom_read_block(this, &configInEEPROM, sizeof(this));
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
