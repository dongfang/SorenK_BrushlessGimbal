#include "Globals.h"
#include "RCdecode.h"
#include "Util.h"
#include "Board.h"

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <math.h>

static uint8_t slowLoopTask;
uint8_t slowLoopTaskPerformanceTimed;
uint8_t gimbalState;

float rollPhiSet;
float pitchPhiSet;
float rollAngleSet;
float pitchAngleSet;

int16_t pitchPIDVal;
int16_t rollPIDVal;

void flashLED() {
	static uint8_t count;
	count++;
	if (count >= 25) {
		count = 0;
//		LED_PIN |= (1 << LED_BIT);
	}
}

/**********************************************/
/* Main Loop                                  */
/**********************************************/
void mainLoop() {
	PERFORMANCE(BM_OTHER);
	PERFORMANCE_NEW_CYCLE;

	// update IMU data
	imu.fastUpdateCycle();

	// We do this after I2C comms so we should fail fast if I2C is stuck.
	wdt_reset();

	/*
	 * TODO: Faked controls. Make them work.
	*/
	switchPos = 1;
	pitchAngleSet = rollAngleSet = 0;

	//****************************
	// pitch and roll PIDs
	//****************************
	PERFORMANCE(BM_PIDS);
	pitchPIDVal =pitchPID.compute(DT_INT_MS, imu.angle_md[PITCH], pitchAngleSet * 1000);
	rollPIDVal = rollPID.compute(DT_INT_MS, imu.angle_md[ROLL], rollAngleSet * 1000);
	PERFORMANCE(BM_OTHER);

	// motor control
	if (switchPos >= 0) {
		PERFORMANCE(BM_MOTORPHASES);
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
		PERFORMANCE(BM_OTHER);
	}
}

void slowLoop() {
	static uint8_t pOutCnt = 0;
	static int stateCount = 0;

  //****************************
  // slow rate actions
  //****************************
  imu.updateAccMagnitude();
  flashLED();
  
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

  // Evaluate RC-Signals
  if (config.rcAbsolute == 1) {
    PERFORMANCE(BM_RC_DECODE);
    evaluateRCAbsolute(); // t=30/142us,  returns rollRCSetPoint, pitchRCSetpoint
    utilLP_float(&pitchAngleSet, pitchPhiSet, rcLPF_tc); // t=16us
    utilLP_float(&rollAngleSet, rollPhiSet, rcLPF_tc); // t=28us
    PERFORMANCE(BM_OTHER);
  } else {
    PERFORMANCE(BM_RC_DECODE);
    evaluateRCIntegrating(); // gives rollRCSpeed, pitchRCSpeed
    utilLP_float(&pitchAngleSet, pitchPhiSet, 0.01);
    utilLP_float(&rollAngleSet, rollPhiSet, 0.01);
    PERFORMANCE(BM_OTHER);
  }
  evaluateRCSwitch();
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

   // regular debug output
   pOutCnt++;
   if (pOutCnt == (LOOPUPDATE_FREQ / 10 / POUT_FREQ)) {
     pOutCnt = 0;
     debug();
   }

   checkRcTimeouts();

   sCmd.readSerial();
   slowLoopTask = 0;

#ifdef STACKHEAPCHECK_ENABLE
   stackHeapEval(false);
#endif
   
   doubleFault = false; // If we have run the mainloop successfully, we reset the double WDT fault status.
}
