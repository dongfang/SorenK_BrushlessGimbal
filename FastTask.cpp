#include "Globals.h"
#include <avr/wdt.h>

int16_t pitchPIDVal;
int16_t rollPIDVal;

int16_t prevPitchPIDVal;
int16_t prevRollPIDVal;

int16_t rollPIDDelta;
int16_t pitchPIDDelta;

volatile uint8_t overrate;

inline void outputPitch(uint8_t value) {
	motorPhases[PITCH][0] = pwmSinMotorPitch[value] * softStart >> 4;
	motorPhases[PITCH][1] = pwmSinMotorPitch[(uint8_t) (value + 85)] * softStart >> 4;
	motorPhases[PITCH][2] = pwmSinMotorPitch[(uint8_t) (value + 170)] * softStart >> 4;
}

inline void outputRoll(uint8_t value) {
	motorPhases[ROLL][0] = pwmSinMotorRoll[value] * softStart >> 4;
	motorPhases[ROLL][1] = pwmSinMotorRoll[(uint8_t) (value + 85)] * softStart >> 4;
	motorPhases[ROLL][2] = pwmSinMotorRoll[(uint8_t) (value + 170)] * softStart >> 4;
}

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

	bool didOverspeed = false;
	if (rollPIDDelta > config.rollOutputRateLimit) {
		LEDEvent(LED_SPEED_LIMIT_MASK);
		didOverspeed = true;
		//Actually limiting brings little.
		//rollPIDVal = prevRollPIDVal + config.rollOutputRateLimit;
	} else if (rollPIDDelta < -config.rollOutputRateLimit) {
		LEDEvent(LED_SPEED_LIMIT_MASK);
		didOverspeed = true;
		//Actually limiting brings little.
		//rollPIDVal = prevRollPIDVal - config.rollOutputRateLimit;
	}
	if (pitchPIDDelta > config.pitchOutputRateLimit) {
		LEDEvent(LED_SPEED_LIMIT_MASK);
		didOverspeed = true;
		//Actually limiting brings little.
		//pitchPIDVal = prevPitchPIDVal + config.pitchOutputRateLimit;
	} else if (pitchPIDDelta < -config.pitchOutputRateLimit) {
		LEDEvent(LED_SPEED_LIMIT_MASK);
		didOverspeed = true;
		//Actually limiting brings little.
		//pitchPIDVal = prevPitchPIDVal - config.pitchOutputRateLimit;
	}

	if (didOverspeed) {
		if (overrate < 255) overrate++;
	} else {
		//if (overrate) overrate--;
	}

	prevRollPIDVal = rollPIDVal;
	prevPitchPIDVal = pitchPIDVal;

	// motor control
	if (gimbalState & PIDS_ARE_OUTPUT) {
		//int motorDrive = pitchPIDVal; // * config.dirMotorPitch;
		outputRoll(rollPIDVal);
		outputPitch(pitchPIDVal);
		/*
		 uint8_t posStep = pitchPIDVal;
		 motorPhases[PITCH][0] = pwmSinMotorPitch[posStep] * softStart >> 4;
		 motorPhases[PITCH][1] = pwmSinMotorPitch[(uint8_t) (posStep + 85)] * softStart >> 4;
		 motorPhases[PITCH][2] = pwmSinMotorPitch[(uint8_t) (posStep + 170)] * softStart >> 4;

		 posStep = rollPIDVal;
		 motorPhases[ROLL][0] = pwmSinMotorRoll[posStep] * softStart >> 4;
		 motorPhases[ROLL][1] = pwmSinMotorRoll[(uint8_t) (posStep + 85)] * softStart >> 4;
		 motorPhases[ROLL][2] = pwmSinMotorRoll[(uint8_t) (posStep + 170)] * softStart >> 4;
		 */
		newPWMData = true;
	} else if (gimbalState & SETUP_TASK_RUNS) {
		if (gimbalState & SETUP_RESET) {
			setupMoveCounter = 0;
			setupMoveDivider = SETUP_MOVE_DIVIDER;
		}
		if (!setupMoveDivider) {
			setupMoveDivider = SETUP_MOVE_DIVIDER;
			if (setupMoveCounter < SETUP_MOVE_LIMIT) {
				setupMoveCounter++;
			} else {
				gimbalState &= ~SETUP_TASK_RUNS;
			}
		}
		if (gimbalState & SETUP_AXIS) {
			outputPitch(setupMoveCounter);
		} else {
			outputRoll(setupMoveCounter);
		}
		newPWMData = true;
		setupMoveDivider--;
	}
}
