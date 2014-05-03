#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

#include <stdint.h>
#include "Definitions.h"

class Configuration {
public:
	uint8_t vers;
	uint8_t versEEPROM;

// Input settings
	int16_t accTimeConstant;
	//int8_t mpuLPF; // mpu LPF 0..6, 0=fastest(256Hz) 6=slowest(5Hz)
	bool calibrateGyro; // Else use EEPROM value
	uint8_t majorAxis;
	bool axisReverseZ;
	bool axisSwapXY;

// PID settings
	int32_t pitchKp;
	int32_t pitchKi;
	int32_t pitchKd;
	int32_t rollKp;
	int32_t rollKi;
	int32_t rollKd;

// Output settings
	uint8_t motorNumberPitch;
	uint8_t motorNumberRoll;
	uint8_t maxPWMmotorPitch;
	uint8_t maxPWMmotorRoll;
	//int8_t dirMotorPitch;
	//int8_t dirMotorRoll;

// RC settings
	int8_t minRCPitch;
	int8_t maxRCPitch;
	int8_t minRCRoll;
	int8_t maxRCRoll;
	int16_t rcGain;
	int16_t rcLPF; // low pass filter for RC absolute mode, units=1/10 sec

// RC channels are not configurable. Makes no sense, when it is so easy
// to just move the connections around instead.
// PPM mode is disabled for now (until I get a new interpreter written).
// Of course for PPM mode, channel settings will be needed.
// bool rcModePPM;          // RC mode, true=common RC PPM channel, false=separate RC channels
// int8_t rcChannelPitch;     // input channel for pitch
// int8_t rcChannelRoll;      // input channel for roll
// int8_t rcChannelSwitch;    // input channel for switch

	int16_t angleOffsetPitch;
	int16_t angleOffsetRoll;

	int16_t rcMid; // rc channel center ms
	bool rcAbsolute;
	uint16_t crc16;

	void setDefaults();
	void readEEPROMOrDefault();
	void writeEEPROM();

private:
	uint16_t CRC();
};

// types of config parameters
enum confType {
	BOOL, INT8, INT16, INT32, UINT8, UINT16, UINT32
};

struct ConfigDef_t {
	char name[CONFIGNAME_MAX_LEN]; // name of config parameter
	confType type; // type of config parameters
	void * address; // address of config parameter
	void (*updateFunction)(void); // function is called when parameter update happens
};

// access descriptor as array of bytes as well
union ConfigUnion_t {
	ConfigDef_t asConfig;
	uint8_t asBytes[sizeof(ConfigDef_t)];
};

extern ConfigDef_t configDef;
extern ConfigUnion_t configUnion;
extern const ConfigDef_t configListPGM[];

// find Config Definition for named parameter
ConfigDef_t * getConfigDef(char* name);

// write single parameter with value
void writeConfig(ConfigDef_t* def, int32_t val);

void updateAllParameters();

#endif
