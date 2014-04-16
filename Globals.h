#ifndef __GLOBALS_H
#define __GLOBALS_H

#include "MPU6050.h"
#include "IMU.h"
#include "SerialCommand.h"
#include "PID.h"
#include "SerialStream.h"

extern MPU6050 mpu;            // Create MPU object
extern IMU imu;
extern SerialCommand sCmd;     // Create SerialCommand object
extern PID pitchPID;
extern PID rollPID;
extern Serial serial0;
extern bool runMainLoop;

extern uint8_t timer1Extension;
extern uint8_t syncCounter;

extern uint8_t motorPhases[2][3];
extern uint8_t softStart;

extern uint8_t pwmSinMotorPitch[N_SIN];
extern uint8_t pwmSinMotorRoll[N_SIN];

struct Config_t {
	uint8_t vers;
	uint8_t versEEPROM;

// Input settings
	int16_t accTimeConstant;
	int8_t mpuLPF; // mpu LPF 0..6, 0=fastest(256Hz) 6=slowest(5Hz)
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
	int16_t angleOffsetPitch; // angle offset, deg*100
	int16_t angleOffsetRoll;
	uint8_t nPolesMotorPitch;
	uint8_t nPolesMotorRoll;
	int8_t dirMotorPitch;
	int8_t dirMotorRoll;
	uint8_t motorNumberPitch;
	uint8_t motorNumberRoll;
	uint8_t maxPWMmotorPitch;
	uint8_t maxPWMmotorRoll;

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

	int16_t rcMid; // rc channel center ms
	bool rcAbsolute;
	uint16_t crc16;
};

extern Config_t config;

struct RCData_t {
	uint16_t microsRisingEdge;
	uint16_t microsLastUpdate;
	uint16_t rx;
	bool isFresh;
	bool isValid;
	float rcSpeed;
	float setpoint;
};

extern RCData_t rcData[RC_DATA_SIZE];
extern int8_t switchPos;

// DEBUG only
extern uint32_t stackTop;
extern uint32_t stackBottom;
extern uint32_t heapTop;
extern uint32_t heapBottom;

#endif
