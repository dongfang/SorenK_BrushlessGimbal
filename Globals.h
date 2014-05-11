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
extern int16_t rollAngleSet;
extern int16_t pitchAngleSet;

extern int16_t rollPIDVal;
extern int16_t rollPIDDelta;
extern int16_t pitchPIDVal;
extern int16_t pitchPIDDelta;

extern uint8_t motorPhases[2][3];
extern volatile uint8_t softStart;

extern uint8_t pwmSinMotorPitch[N_SIN];
extern uint8_t pwmSinMotorRoll[N_SIN];

struct RCData_t {
	//uint16_t microsRisingEdge;
	//uint16_t microsFallingEdge;
	uint16_t m_16;
	bool pulseComplete;
	uint8_t timeout;
	int16_t setpoint;
	inline bool isTimedOut() { return timeout >= 200; }
};

extern RCData_t rcData[RC_DATA_SIZE];
extern int8_t switchPos;

//extern int16_t pitchTransient;
//extern int16_t rollTransient;

extern volatile bool newPWMData;
extern volatile uint8_t gimbalState;
extern volatile bool mediumTaskHasRun;

extern uint8_t interfaceState;

#endif

