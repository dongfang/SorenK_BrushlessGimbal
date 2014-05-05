#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

#include <stdint.h>
#include "Definitions.h"

struct RCChannelDef {
	int16_t minAngle;			// min may be greater than max to reverse.
	int16_t maxAngle;			// min may be greater than max to reverse.
	int16_t defaultAngle;		// if no signal
	uint8_t speed; 				// integrating mode : Integration speed. Absolute mode: max. speed.
	//uint8_t LPF; 				// low pass filter for RC absolute mode. I don't like LPFs.
};

class Configuration {
public:
	uint8_t vers;
	uint8_t versEEPROM;

// Input settings
	uint8_t accTimeConstant;
	//int8_t mpuLPF; // mpu LPF 0..6, 0=fastest(256Hz) 6=slowest(5Hz)
	bool calibrateGyro; // Else use EEPROM value
	uint8_t majorAxis;
	bool axisReverseZ;
	bool axisSwapXY;

// PID settings
	int16_t pitchKp;
	int16_t pitchKi;
	int16_t pitchKd;
	int16_t rollKp;
	int16_t rollKi;
	int16_t rollKd;

// Output settings
	uint8_t maxPWMmotorRoll;
	uint8_t maxPWMmotorPitch;

	// There should be no need for these. One can just swap and turn the connectors.
	//uint8_t motorNumberPitch;
	//uint8_t motorNumberRoll;
	//int8_t dirMotorPitch;
	//int8_t dirMotorRoll;

	// Source rules could be:
	// If MAVLink seen, use that.
	// If no RC signal, use default angle (a default rate is meaningless)
	// RC settings
	RCChannelDef RCPitch;
	RCChannelDef RCRoll;
	bool rcAbsolute;

	// RC channels are not configurable. Makes no sense, when it is so easy
	// to just move the connectors around instead.
	// PPM mode is disabled for now (until I get a new interpreter written).
	// Of course for PPM mode, channel settings will be needed.
	// bool rcModePPM;          // RC mode, true=common RC PPM channel, false=separate RC channels
	// int8_t rcChannelPitch;     // input channel for pitch
	// int8_t rcChannelRoll;      // input channel for roll
	// int8_t rcChannelSwitch;    // input channel for switch

	// No need for these. There is an acc. leverl feature for that.
	// If somebody wants a way to set the output angle directly, fine, but this applied to the
	// input/attitude estimation side and not to output. The 2 should be kept separate.
	//int16_t pitchAngleOffset;
	//int16_t rollAngleOffset;

	// Experiment: Seems to help against tumbling motors when losing sync.
	uint8_t pitchSpeedLimit;
	uint8_t rollSpeedLimit;

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

void initPIDs();

#endif
