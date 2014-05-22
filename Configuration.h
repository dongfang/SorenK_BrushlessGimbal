#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

#include <stdint.h>
#include "Definitions.h"
#include "Util.h"

extern uint8_t profile;

class Configuration {
public:
	uint8_t vers;
	uint8_t versEEPROM;

	uint32_t serialBaudRate;
	uint8_t mpu6050Address;

// Input settings
	uint8_t accTimeConstant;
	//int8_t mpuLPF; // mpu LPF 0..6, 0=fastest(256Hz) 6=slowest(5Hz)
	bool calibrateGyro; // Else use EEPROM value
	uint8_t majorAxis;		// valid values are 0, 1, 2		(3)
	bool axisReverseZ;		// true, false					(2)
	uint8_t axisRotateZ;	// valid values are 0, 1, 2, 3 	(4)
							// total combinations			3*2*4 = 24
	// How to auto-config:
	// 1) Ask user to set gimbal about horizontal
	// 2) Perform test roll rotation 1 way
	// 3) Ask user which way that was (now we know both gyro and motor sign)
	// 5) Perform test pitch rotation 1 way
	// 6) Ask user which way that was (now we know both gyro and motor sign)
	// 7) We should now be able to infer majorAxis and Z rotation.
	// 8) The only thing missing is yaw gyro sign.

// PID settings
	int16_t pitchKp;
	int16_t pitchKi;
	int16_t pitchKd;
	int16_t rollKp;
	int16_t rollKi;
	int16_t rollKd;

	int32_t ILimit;

// Output settings
	uint8_t rollMotorPower;
	uint8_t pitchMotorPower;

	uint8_t rollFrozenValue;
	uint8_t pitchFrozenValue;

	// There should be no need for these. One can just swap and turn the connectors.
	//uint8_t motorNumberPitch;
	//uint8_t motorNumberRoll;
	//int8_t dirMotorPitch;
	//int8_t dirMotorRoll;

	// Source rules could be:
	// If MAVLink seen, use that.
	// If no RC signal, use default angle (a default rate is meaningless)
	// RC settings
	ControlAxisDef controlInput[2];
	bool rcAbsolute;

	uint8_t yawServoLimit;
	int8_t yawServoDirection;

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
	uint8_t pitchOutputRateLimit;
	uint8_t rollOutputRateLimit;

	// in 1:16 units.
	uint8_t frozenGimbalPower;
	uint8_t LEDMask;

	uint8_t mavlinkSystemId;
	uint8_t mavlinkComponentId;
	bool mavlinkUseRelativealtitudes;

	uint16_t retractedServoUsec;
	uint16_t extendedServoUsec;

	uint16_t crc16;

	void setDefaults();
	void readEEPROMOrDefault();
	void checkRAMImageValid();
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
