#include "Globals.h"
#include "RCdecode.h"
#include "Util.h"

#include <avr/wdt.h>
#include <avr/io.h>
#include <math.h>

uint8_t loopCount;
uint8_t gimbalState;

float rollPhiSet;
float pitchPhiSet;
float rollAngleSet;
float pitchAngleSet;

void flashLED() {
	static uint8_t count;
	count++;
	if (count > 15) count = 0;
	if (count <= softStart)
		LED_PIN |= (1<<LED_BIT);
}

/**********************************************/
/* Main Loop                                  */
/**********************************************/
void mainLoop() {
	int32_t pitchPIDVal;
	int32_t rollPIDVal;

	static char pOutCnt = 0;
	static int stateCount = 0;

	if (runMainLoop) // loop runs with motor ISR update rate (1000Hz)
	{
		wdt_reset();
		runMainLoop = false;

		// update IMU data
		imu.fastUpdateCycle();

		// Evaluate RC-Signals
		if (config.rcAbsolute == 1) {
			BENCHMARK(BM_RC_DECODE);
			evaluateRCAbsolute(); // t=30/142us,  returns rollRCSetPoint, pitchRCSetpoint
			utilLP_float(&pitchAngleSet, pitchPhiSet, rcLPF_tc); // t=16us
			utilLP_float(&rollAngleSet, rollPhiSet, rcLPF_tc); // t=28us
			BENCHMARK(BM_NOTHING);
		} else {
			BENCHMARK(BM_RC_DECODE);
			evaluateRCIntegrating(); // gives rollRCSpeed, pitchRCSpeed
			utilLP_float(&pitchAngleSet, pitchPhiSet, 0.01);
			utilLP_float(&rollAngleSet, rollPhiSet, 0.01);
			BENCHMARK(BM_NOTHING);
		}

		evaluateRCSwitch();
		switchPos = 1;

		//****************************
		// pitch and roll PIDs
		//****************************
		// t=94us
		BENCHMARK(BM_PIDS);
		pitchPIDVal = //ComputePID(DT_INT_MS, angle[PITCH], pitchAngleSet*1000, &pitchErrorSum, &pitchErrorOld, pitchPIDpar.Kp, pitchPIDpar.Ki, pitchPIDpar.Kd);
				pitchPID.compute(DT_INT_MS, imu.angle[PITCH], pitchAngleSet * 1000);
		// t=94us
		rollPIDVal = //ComputePID(DT_INT_MS, angle[ROLL], rollAngleSet*1000, &rollErrorSum, &rollErrorOld, rollPIDpar.Kp, rollPIDpar.Ki, rollPIDpar.Kd);
				rollPID.compute(DT_INT_MS, imu.angle[ROLL], rollAngleSet * 1000);
		BENCHMARK(BM_NOTHING);

		// motor control
		if (switchPos >= 0) {
			BENCHMARK(BM_MOTORPHASES);
			int motorDrive = pitchPIDVal * config.dirMotorPitch;
			uint8_t posStep = motorDrive >> 3;
			motorPhases[config.motorNumberPitch][0] = pwmSinMotorPitch[posStep] * softStart >> 4;
			motorPhases[config.motorNumberPitch][1] = pwmSinMotorPitch[(uint8_t) (posStep + 85)] * softStart >> 4;
			motorPhases[config.motorNumberPitch][2] = pwmSinMotorPitch[(uint8_t) (posStep + 170)] * softStart >> 4;

			motorDrive = rollPIDVal * config.dirMotorRoll;
			posStep = motorDrive >> 3;
			motorPhases[config.motorNumberRoll][0] = pwmSinMotorRoll[posStep] * softStart >> 4;
			motorPhases[config.motorNumberRoll][1] = pwmSinMotorRoll[(uint8_t) (posStep + 85)] * softStart >> 4;
			motorPhases[config.motorNumberRoll][2] = pwmSinMotorRoll[(uint8_t) (posStep + 170)] * softStart >> 4;
			BENCHMARK(BM_NOTHING);
		}

		//****************************
		// slow rate actions
		//****************************
		BENCHMARK(BM_SLOWLOOP);
		switch (loopCount) {
		case 1:
			imu.readAcc(0);
			break;
		case 2:
			imu.readAcc(1);
			break;
		case 3:
			imu.readAcc(2);
			break;
		case 4:
			imu.updateAccVector();
			break;
		case 5:
			flashLED();
			break;
		case 6:
			// gimbal state transitions
			if (gimbalState == GIMBAL_OFF) {
				// wait 2 sec to settle ACC, before PID controller becomes active
				// Is this really necessary?
				// How about this state scheme:
				// Calibrating-running (softstart)
				stateCount++;
				if (stateCount >= LOOPUPDATE_FREQ / 10 * LOCK_TIME_SEC) {
					gimbalState = GIMBAL_RUNNING;
					stateCount = 0;
				}
			}

			// gimbal state actions
			switch (gimbalState) {
			case GIMBAL_OFF:
				softStart = 0;
				break;
			case GIMBAL_RUNNING:
				if (switchPos <= 0) {
					// slask
					if (softStart > 0)
						softStart--;
				} else {
					// normal
					if (softStart < 16)
						softStart++;
				}
				break;
			}
			break;
		case 7:
			// RC Pitch function
			if (rcData[RC_DATA_PITCH].isValid) {
				if (config.rcAbsolute == 1) {
					pitchPhiSet = rcData[RC_DATA_PITCH].setpoint;
				} else {
					if (fabs(rcData[RC_DATA_PITCH].rcSpeed) > 0.01) {
						pitchPhiSet += rcData[RC_DATA_PITCH].rcSpeed * 0.01;
					}
				}
			} else {
				pitchPhiSet = 0;
			}
			if (config.minRCPitch < config.maxRCPitch) {
				pitchPhiSet = constrain_f(pitchPhiSet, config.minRCPitch, config.maxRCPitch);
			} else {
				pitchPhiSet = constrain_f(pitchPhiSet, config.maxRCPitch, config.minRCPitch);
			}
			break;
		case 8:
			// RC roll function
			if (rcData[RC_DATA_ROLL].isValid) {
				if (config.rcAbsolute == 1) {
					rollPhiSet = rcData[RC_DATA_ROLL].setpoint;
				} else {
					if (fabs(rcData[RC_DATA_ROLL].rcSpeed) > 0.01) {
						rollPhiSet += rcData[RC_DATA_ROLL].rcSpeed * 0.01;
					}
				}
			} else {
				rollPhiSet = 0;
			}
			if (config.minRCRoll < config.maxRCRoll) {
				rollPhiSet = constrain_f(rollPhiSet, config.minRCRoll, config.maxRCRoll);
			} else {
				rollPhiSet = constrain_f(rollPhiSet, config.maxRCRoll, config.minRCRoll);
			}
			break;
		case 9:
			// regular debug output
			pOutCnt++;
			if (pOutCnt == (LOOPUPDATE_FREQ / 10 / POUT_FREQ)) {
				pOutCnt = 0;
				debug();
			}
			break;
		case 10:
#ifdef STACKHEAPCHECK_ENABLE
			stackHeapEval(false);
#endif
			loopCount = 0;
			break;
		default:
			break;
		}

		loopCount++;
		BENCHMARK(BM_NOTHING);

		//****************************
		// check RC channel timeouts
		//****************************

		BENCHMARK(BM_TIMEOUTS);
		checkRcTimeouts();
		BENCHMARK(BM_NOTHING);

		//****************************
		// Evaluate Serial inputs
		//****************************
		BENCHMARK(BM_SERIAL);
		sCmd.readSerial();
		BENCHMARK(BM_NOTHING);
	}
}
