#include "Globals.h"
#include <avr/wdt.h>
#include <avr/io.h>

int16_t pitchPIDVal;
int16_t rollPIDVal;

int16_t prevPitchPIDVal;
int16_t prevRollPIDVal;

int16_t rollPIDDelta;
int16_t pitchPIDDelta;

uint8_t rollMotorTest;
uint8_t pitchMotorTest;

//volatile uint8_t overrate;

void outputPitch(uint8_t value) {
	uint8_t a = pwmSinMotorPitch[value] * softStart >> 4;
	uint8_t b = pwmSinMotorPitch[(uint8_t) (value + 85)] * softStart >> 4;
	uint8_t c = pwmSinMotorPitch[(uint8_t) (value + 170)] * softStart >> 4;

	PWM_A_MOTOR1 = a;
	PWM_B_MOTOR1 = b;
	PWM_C_MOTOR1 = c;
}

void outputRoll(uint8_t value) {
	uint8_t a = pwmSinMotorRoll[value] * softStart >> 4;
	uint8_t b = pwmSinMotorRoll[(uint8_t) (value + 85)] * softStart >> 4;
	uint8_t c = pwmSinMotorRoll[(uint8_t) (value + 170)] * softStart >> 4;

	PWM_A_MOTOR0 = a;
	PWM_B_MOTOR0 = b;
	PWM_C_MOTOR0 = c;
}

//#define DO_LIMIT_RATES

void fastTask() {
	static uint8_t setupMoveDivider;
	static uint8_t setupMoveCounter;

	PERFORMANCE_NEW_CYCLE;

	// update IMU data
	imu.fastUpdateCycle();

	// We do this after I2C comms so we should fail fast if I2C is stuck.
	wdt_reset();

	//****************************
	// pitch and roll PIDs
	//****************************
	rollPIDVal = rollPID.compute(imu.angle_i16[ROLL], rollAngleSet, imu.rollRate);
	pitchPIDVal = pitchPID.compute(imu.angle_i16[PITCH], pitchAngleSet, imu.pitchRate);

	rollPIDDelta = rollPIDVal - prevRollPIDVal;
	pitchPIDDelta = pitchPIDVal - prevPitchPIDVal;

	/*
	bool didOverspeed = false;
	if (rollPIDDelta > config.rollOutputRateLimit) {
		LEDEvent(LED_SPEED_LIMIT_MASK);
		didOverspeed = true;
		//Actually limiting brings little.
#ifdef DO_LIMIT_RATES
		rollPIDVal = prevRollPIDVal + config.rollOutputRateLimit;
#endif
	} else if (rollPIDDelta < -config.rollOutputRateLimit) {
		LEDEvent(LED_SPEED_LIMIT_MASK);
		didOverspeed = true;
		//Actually limiting brings little.
#ifdef DO_LIMIT_RATES
		rollPIDVal = prevRollPIDVal - config.rollOutputRateLimit;
#endif
	}
	if (pitchPIDDelta > config.pitchOutputRateLimit) {
		LEDEvent(LED_SPEED_LIMIT_MASK);
		didOverspeed = true;
		//Actually limiting brings little.
#ifdef DO_LIMIT_RATES
		pitchPIDVal = prevPitchPIDVal + config.pitchOutputRateLimit;
#endif
	} else if (pitchPIDDelta < -config.pitchOutputRateLimit) {
		LEDEvent(LED_SPEED_LIMIT_MASK);
		didOverspeed = true;
		//Actually limiting brings little.
#ifdef DO_LIMIT_RATES
		pitchPIDVal = prevPitchPIDVal - config.pitchOutputRateLimit;
#endif
	}

	if (didOverspeed && overrate < 64) overrate++;
	*/

	prevRollPIDVal = rollPIDVal;
	prevPitchPIDVal = pitchPIDVal;

	// motor control
	if (gimbalState & GS_PIDS_ARE_OUTPUT) {
		//int motorDrive = pitchPIDVal; // * config.dirMotorPitch;
		outputRoll(rollPIDVal);
		outputPitch(pitchPIDVal);
	}
#ifdef SUPPORT_AUTOSETUP
	else if (autosetupState & AS_RUNNING) {
		if (autosetupState & AS_RESET) {
			setupMoveCounter = 0;
			setupMoveDivider = SETUP_MOVE_DIVIDER;
			autosetupState &= ~AS_RESET;
		}
		if (!setupMoveDivider) {
			setupMoveDivider = SETUP_MOVE_DIVIDER;
			if (setupMoveCounter < SETUP_MOVE_LIMIT) {
				setupMoveCounter++;
			} else {
				autosetupState &= ~AS_RUNNING;
			}
		}
		if (autosetupState & AS_IS_PITCH) {
			outputPitch(setupMoveCounter);
		} else {
			outputRoll(setupMoveCounter);
		}
		//printf("%d\r\n", setupMoveCounter);
		setupMoveDivider--;
	}
#endif
	else if (gimbalState & GS_GIMBAL_FROZEN) {
		outputPitch(config.pitchFrozenValue);
		outputRoll(config.rollFrozenValue);
	} else if (gimbalState & GS_GIMBAL_MOTORTEST) {
		outputRoll(rollMotorTest++);
		outputPitch(pitchMotorTest++);
	}
}
