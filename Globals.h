#ifndef __GLOBALS_H
#define __GLOBALS_H

#include "Configuration.h"
#include "SerialCommand.h"
#include "SerialStream.h"
#include "MPU6050.h"
#include "IMU.h"
#include "PID.h"

extern uint8_t mcusr_mirror;	// Why did the CPU reset status

extern MPU6050 mpu;
extern IMU imu;
extern SerialCommand sCmd;     	// Create SerialCommand object
extern PID pitchPID;
extern PID rollPID;
extern UARTSerial serial0;		// Serial stream
extern Configuration config;

extern volatile bool runMainLoop;
extern bool watchdogResetWasIntended;

extern uint8_t syncCounter;		// Timer1 overflow divider to mainloop frequency.
extern bool doubleFault;		// Set true after a WDT reset. Set false after mainloop completion.
								// If true at WDT reset time, init not only HW but also state.

extern int32_t pitchPIDVal;
extern int32_t rollPIDVal;

extern uint8_t motorPhases[2][3];
extern uint8_t softStart;

extern uint8_t pwmSinMotorPitch[N_SIN];
extern uint8_t pwmSinMotorRoll[N_SIN];

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

extern float rcLPF_tc;

#endif
