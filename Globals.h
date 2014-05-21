#ifndef __GLOBALS_H
#define __GLOBALS_H

#include "Configuration.h"
#include "SerialCommand.h"
#include "SerialStream.h"
#include "MPU6050.h"
#include "IMU.h"
#include "PID.h"
#include "Board.h"

extern uint8_t mcusr_mirror;		// Why did the CPU reset status

extern MPU6050 mpu;					// The inertial sensor abstraction
extern IMU imu;						// The IMU/AHRS instance, calculating attitude
extern SerialCommand sCmd;     		// Serial command object
extern PID pitchPID;				// PID controller
extern PID rollPID;					// PID controller
extern UARTSerial serial0;			// Serial stream
extern Configuration config;		// Static config
extern LiveControlAxisDef liveControlDefs[];	// Angles in converted units

extern bool watchdogResetWasIntended; // The reset command uses a WDT timeout to reset system; in this case we want no warning about it.

extern bool doubleFault;			// Set true after a WDT reset. Set false after mainloop completion.
									// If true at WDT reset time, init not only HW but also state.

extern int16_t rollAngleSet;		// Roll angle the gimbal should aim at (in Nerd Degrees, 1 full circle = 1<<16)
extern int16_t pitchAngleSet;		// Pitch angle the gimbal should aim at (in Nerd Degrees, 1 full circle = 1<<16)

extern int16_t rollPIDVal;			// Latest PID output
extern int16_t rollPIDDelta;		// Difference from previous one
extern int16_t pitchPIDVal;
extern int16_t pitchPIDDelta;


//extern uint8_t motorPhases[2][3];	// Output signals, 2x3 phases
extern volatile uint8_t softStart;	// 0..15

extern uint8_t pwmSinMotorPitch[N_SIN];
extern uint8_t pwmSinMotorRoll[N_SIN];

extern RCData_t rcData[RC_DATA_SIZE];
extern int8_t switchPos;

extern volatile bool newPWMData;		// Whether there is new output data to latch out
extern volatile uint8_t gimbalState;	// Control flags for gimbal control
#ifdef SUPPORT_AUTOSETUP
extern volatile uint8_t autosetupState;
#endif
extern volatile bool mediumTaskHasRun;	// Sync main loop to medium task

extern uint8_t interfaceState;			// Control flags for serial interface

/*
 * How often was the max. PID output rotation rates exceeded,
 * this is indicative of self-oscillation / runaway
 */
extern volatile uint8_t overrate;

extern int16_t targetSources[][2];
extern uint8_t targetSource;

extern int16_t transient[2];

#endif

