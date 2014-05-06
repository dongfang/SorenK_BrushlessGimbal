#include "Globals.h"
#include "RCdecode.h"
#include "Util.h"
#include "Board.h"

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <math.h>

#define RC 0
#define OSC 1

uint8_t gimbalState;

int16_t rollAngleSet;
int16_t pitchAngleSet;

int16_t pitchPIDVal;
int16_t rollPIDVal;

int16_t prevPitchPIDVal;
int16_t prevRollPIDVal;

int16_t rollPIDDelta;
int16_t pitchPIDDelta;

int16_t oscPitch;
int16_t oscRoll;
uint8_t oscSpeed;
uint8_t transientsMag;
uint8_t transientsAxes;

int8_t pitchTransient;
int8_t rollTransient;

uint8_t targetSource;
uint8_t LEDFlags;

/**********************************************/
/* Main Loop                                  */
/**********************************************/
void fastTask() {
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

	if (rollPIDDelta > config.rollSpeedLimit) {
		LEDEvent(SPEED_LIMIT_MASK);
		rollPIDVal = prevRollPIDVal + config.rollSpeedLimit;
	} else if (rollPIDDelta < -config.rollSpeedLimit) {
		LEDEvent(SPEED_LIMIT_MASK);
		rollPIDVal = prevRollPIDVal - config.rollSpeedLimit;
	}
	if (pitchPIDDelta > config.pitchSpeedLimit) {
		LEDEvent(SPEED_LIMIT_MASK);
		pitchPIDVal = prevPitchPIDVal + config.pitchSpeedLimit;
	} else if (pitchPIDDelta < -config.pitchSpeedLimit) {
		LEDEvent(SPEED_LIMIT_MASK);
		pitchPIDVal = prevPitchPIDVal - config.pitchSpeedLimit;
	}

	prevRollPIDVal = rollPIDVal;
	prevPitchPIDVal = pitchPIDVal;

	// motor control
	if (gimbalState == GIMBAL_FROZEN) {
	} else {
		//int motorDrive = pitchPIDVal; // * config.dirMotorPitch;
		uint8_t posStep = pitchPIDVal;
		motorPhases[PITCH][0] = pwmSinMotorPitch[posStep] * softStart >> 4;
		motorPhases[PITCH][1] = pwmSinMotorPitch[(uint8_t) (posStep + 85)] * softStart >> 4;
		motorPhases[PITCH][2] = pwmSinMotorPitch[(uint8_t) (posStep + 170)] * softStart >> 4;

		posStep = rollPIDVal;
		motorPhases[ROLL][0] = pwmSinMotorRoll[posStep] * softStart >> 4;
		motorPhases[ROLL][1] = pwmSinMotorRoll[(uint8_t) (posStep + 85)] * softStart >> 4;
		motorPhases[ROLL][2] = pwmSinMotorRoll[(uint8_t) (posStep + 170)] * softStart >> 4;
	}
}

int16_t xformRCToInt16(int16_t a) {
	return a * (65536 / 3600);
}

int16_t getPitchTarget() {
	int16_t result = 0;
	if (targetSource == RC) {
		result = xformRCToInt16(rcData[RC_DATA_PITCH].setpoint);
	}
	if (targetSource == OSC) {
		result = oscPitch;
	}
	return result + pitchTransient;
}

int16_t getRollTarget() {
	int16_t result = 0;
	if (targetSource == RC) {
		result = xformRCToInt16(rcData[RC_DATA_ROLL].setpoint);
	}
	if (targetSource == OSC) {
		result = oscRoll;
	}
	return result + rollTransient;
}

void oscillation(uint8_t val) {
	if (val) {
		oscSpeed = val;
		oscPitch = getPitchTarget();
		oscRoll = getRollTarget();
		targetSource = OSC;
	} else {
		targetSource = RC; // TODO: Restore other value if not initially RC.
	}
}

void transients(uint8_t axes, int16_t val) {
	transientsAxes = axes;
	transientsMag = val;
}

void mediumTask() {
	imu.blendAccToAttitude();
	imu.calculateAttitudeAngles();

	// and the target-angles:
	rollAngleSet = getRollTarget();
	pitchAngleSet = getPitchTarget();
}

inline bool checkMediumLoop() {
	uint8_t sreg = SREG;
	cli();
	bool result = mediumTaskHasRun;
	mediumTaskHasRun = false;
	SREG = sreg;
	return result;
}

void updateGimbalState() {
	///if (switchPos < 0) balahblah .. implement switch to state logic here.
	gimbalState = GIMBAL_RUNNING;
}

void updateSoftStart() {
// gimbal state actions
	switch (gimbalState) {
	case GIMBAL_OFF:
		if (softStart)
			softStart--;
		break;
	case GIMBAL_RUNNING:
		if (softStart < 16)
			softStart++;
		break;
	case GIMBAL_FROZEN:
		if (softStart < config.frozenGimbalPower)
			++softStart;
		else if (softStart > config.frozenGimbalPower)
			--softStart;
		break;
	}
}

void complainAboutSensorMotion() {
	printf_P(PSTR("Motion detected during calibration. Starting over!\r\n"));
}

// This thing is the only task blocking the slow task and the only thing stopping the fast/medium interrupt driven tasks.
// Therefore it does strange things and it is placed here.
void calibrateSensor(uint8_t which) {
	// Gyro Offset calibration
	// TODO if softstart works VERY well we can add a divider.
	// Or we can use freeze instead.
	printf_P(PSTR("Sensor calibration: do not move\r\n"));
	gimbalState = GIMBAL_OFF;
	// This is all a stupid hack. Should be better structured.
	runFastTask = runMediumTask = false;
	softStart = 0;
	// Silly hack get rid of it.
	PWM_A_MOTOR0 = 0;
	PWM_B_MOTOR0 = 0;
	PWM_C_MOTOR0 = 0;
	PWM_A_MOTOR1 = 0;
	PWM_B_MOTOR1 = 0;
	PWM_C_MOTOR1 = 0;
	mpu.recalibrateSensor(&complainAboutSensorMotion, which);
	runFastTask = runMediumTask = true;
	printf_P(PSTR("Sensor calibration: done\r\n"));
	updateGimbalState();
}

void slowLoop() {
	static uint8_t debugDivider;
	static uint8_t RCDivider;
	static uint8_t softstartDivider;
	static uint8_t LEDDivider;
	static uint8_t heartbeatDivider;
	//static uint8_t accMagDivider;
	static uint8_t oscDivider;
	static uint16_t transientsDivider;

	static int8_t pitchOscDir = 1;
	static int8_t rollOscDir = 1;

	while (true) {
		sCmd.readSerial();

		bool ticked = checkMediumLoop();

		/* No need for this, it did more harm than good.
		 if (ticked && !accMagDivider) {
		 accMagDivider = ACCMAG_LATCH;
		 imu.updateAccMagnitude();
		 }
		 */

		if (ticked && !RCDivider) {
			RCDivider = RC_LATCH;
			// Evaluate RC-Signals
			evaluateRCControl();
			evaluateRCSwitch();
			updateGimbalState();
		}

		if (ticked && !softstartDivider) {
			softstartDivider = SOFTSTART_LATCH;
			updateSoftStart();
		}

		if (ticked && !debugDivider) {
			debugDivider = HUMAN_DEBUG_LATCH;
			debug();
		}

		if (ticked && !heartbeatDivider) {
			LEDEvent(HEARTBEAT_MASK);
			heartbeatDivider = LED_LATCH*2;
		}

		if (ticked && !LEDDivider) {
			LEDDivider = LED_LATCH;
			if (LEDFlags & config.LEDMask) {
				LED_PORT |= (1<<LED_BIT);
				LEDFlags &= ~config.LEDMask;
			} else {
				LED_PORT &= ~(1<<LED_BIT);
			}
		}

		if (ticked && !oscDivider) {
			oscDivider = OSCILLATION_LATCH;

			if (oscRoll >= xformRCToInt16(config.RCRoll.maxAngle))
				rollOscDir = -1;
			else if (oscRoll <= xformRCToInt16(config.RCRoll.minAngle))
				rollOscDir = 1;
			oscRoll += rollOscDir * oscSpeed;

			if (oscPitch >= xformRCToInt16(config.RCPitch.maxAngle))
				pitchOscDir = -1;
			else if (oscPitch <= xformRCToInt16(config.RCPitch.minAngle))
				pitchOscDir = 1;
			oscPitch += pitchOscDir * oscSpeed;
		}

		if (ticked && !transientsDivider) {
			transientsDivider = MEDIUMLOOP_FREQ;
			if (!rollTransient && (transientsAxes & 1))
				rollTransient = transientsMag;
			else
				pitchTransient = 0;
			if (!pitchTransient && (transientsAxes & 2))
				pitchTransient = transientsMag;
			else
				pitchTransient = 0;
		}
#ifdef STACKHEAPCHECK_ENABLE
		stackHeapEval(false);
#endif

		if (ticked) {
			// --accMagDivider;
			--debugDivider;
			--RCDivider;
			--softstartDivider;
			--LEDDivider;
			--heartbeatDivider;
			--oscDivider;
			--transientsDivider;
		}

		doubleFault = false; // If we have run the mainloop successfully, we reset the double WDT fault status.
	}
}
