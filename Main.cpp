/*
 Brushless Gimbal Controller Software by Christian Winkler and Alois Hahn (C) 2013

 Brushless Gimbal Controller Hardware and Software support
 by Ludwig Fäerber, Alexander Rehfeld and Martin Eckart

 Special Contributions:
 Michael Schätzel

 Rewrite to non Arduino C++:
 Soren Kuula

 Project homepage: http://brushlessgimbal.de/
 Discussions:
 http://fpv-community.de/showthread.php?20795-Brushless-Gimbal-Controller-SOFTWARE
 http://fpv-community.de/showthread.php?22617-Gimbal-Brushless-Controller-V3-0-50x50mm-by-Martinez
 http://fpv-community.de/showthread.php?19252-Brushless-Gimbal-Controller

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>

 Anyhow, if you start to commercialize our work, please read on http://code.google.com/p/brushless-gimbal/ on how to contribute

 // I2Cdev library collection - MPU6050 I2C device class
 // Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
 // 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
 // Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
 */

// FOR CHANGES PLEASE READ: ReleaseHistory.txt
#define VERSION_STATUS B // A = Alpha; B = Beta , N = Normal Release
#define VERSION 49
#define VERSION_EEPROM 2 // change this number when eeprom data strcuture has changed
/*************************/
/* Include Header Files  */
/*************************/
#include <avr/eeprom.h>
#include <stdio.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

#include "Globals.h"
#include "SerialStream.h"
#include "SerialCommand.h"

#include "FastMathRoutines.h"     // fast Math functions required by orientationRoutines.h
#include "RCdecode.h"             // RC Decoder to move camera by servo signal input
#include "BLcontroller.h"         // Motor Movement Functions and Timer Config

// The globals
MPU6050 mpu; // Create MPU object
IMU imu(mpu);
SerialCommand sCmd; // Create SerialCommand object
PID pitchPID;
PID rollPID;

bool runMainLoop;
uint8_t gimbalState;
uint8_t loopCount;
uint8_t timer1Extension;
uint8_t syncCounter;

float rollPhiSet;
float pitchPhiSet;
float rollAngleSet;
float pitchAngleSet;
float rcLPF_tc = 1.0;

uint8_t motorPhases[2][3];
uint8_t softStart;

uint8_t pwmSinMotorPitch[N_SIN];
uint8_t pwmSinMotorRoll[N_SIN];

RCData_t rcData[RC_DATA_SIZE];
int8_t switchPos = SW_UNKNOWN;

// DEBUG only
uint32_t stackTop = 0xffffffff;
uint32_t stackBottom = 0;
uint32_t heapTop = 0;
uint32_t heapBottom = 0xffffffff;

void initMPUlpf() {
	// Set Gyro Low Pass Filter(0..6, 0=fastest, 6=slowest)
	switch (config.mpuLPF) {
	case 0:
		mpu.setDLPFMode(MPU6050_DLPF_BW_256);
		break;
	case 1:
		mpu.setDLPFMode(MPU6050_DLPF_BW_188);
		break;
	case 2:
		mpu.setDLPFMode(MPU6050_DLPF_BW_98);
		break;
	case 3:
		mpu.setDLPFMode(MPU6050_DLPF_BW_42);
		break;
	case 4:
		mpu.setDLPFMode(MPU6050_DLPF_BW_20);
		break;
	case 5:
		mpu.setDLPFMode(MPU6050_DLPF_BW_10);
		break;
	case 6:
		mpu.setDLPFMode(MPU6050_DLPF_BW_5);
		break;
	default:
		mpu.setDLPFMode(MPU6050_DLPF_BW_256);
		break;
	}
}

void setDefaultParameters() {
	config.vers = VERSION;
	config.versEEPROM = VERSION_EEPROM;
	config.pitchKp = 7500;
	config.pitchKi = 10000;
	config.pitchKd = 11000;
	config.rollKp = 20000;
	config.rollKi = 8000;
	config.rollKd = 30000;
	config.accTimeConstant = 4;
	config.mpuLPF = 0;
	config.angleOffsetPitch = 0;
	config.angleOffsetRoll = 0;
	config.nPolesMotorPitch = 14;
	config.nPolesMotorRoll = 14;
	config.dirMotorPitch = 1;
	config.dirMotorRoll = -1;
	config.motorNumberPitch = 0;
	config.motorNumberRoll = 1;
	config.maxPWMmotorPitch = 100;
	config.maxPWMmotorRoll = 100;
	config.minRCPitch = 0;
	config.maxRCPitch = 90;
	config.minRCRoll = -30;
	config.maxRCRoll = 30;
	config.rcGain = 5;
	config.rcLPF = 5; // 0.5 sec
	config.rcMid = MID_RC;
	config.rcAbsolute = true;
	config.calibrateGyro = true;
	config.axisReverseZ = true;
	config.axisSwapXY = false;
	config.majorAxis = 2;
	config.crc16 = 0;
}

void initSerial() {
	serial0.init(115200, _FDEV_SETUP_RW);
	fdevopen(&serial0.*_putchar, &serial0.*_getchar);
}

void calibrateGyro() {
	// Gyro Offset calibration
	printf_P(PSTR("Gyro calibration: do not move\r\n"));
	// mpu.setDLPFMode(MPU6050_DLPF_BW_5);  // experimental AHa: set to slow mode during calibration
	imu.recalibrateGyros();
	// initMPUlpf();
	printf_P(PSTR("Gyro calibration: done\r\n"));
}

/**********************************************/
/* Initialization                             */
/**********************************************/
void coldStart() {
	// Make timeout long.
	wdt_enable(WDTO_1S);

	// just for debugging
#ifdef STACKHEAPCHECK_ENABLE
	stackCheck();
	heapCheck();
#endif

	DDRB |= 1;

	// We don't use this, whatever it was.
	// CH2_PINMODE
	// CH3_PINMODE

	initSerial();
	// Set Serial Protocol Commands
	setSerialProtocol();

	// Read Config, fill with default settings if versions do not match or CRC fails
	readEEPROM();

	if ((config.vers != VERSION) || (config.versEEPROM != VERSION_EEPROM)) {
		printf_P(PSTR("EEPROM version mismatch, initialize EEPROM"));
		setDefaultParameters();
		writeEEPROM();
	}

	// Init Sinus Arrays and Motor Stuff
	recalcMotorStuff();

	// Init PIDs to reduce floating point operations.
	initPIDs();

	// init RC variables
	initRC();

	// Init BL Controller
	initBlController();

	mpu.init();

	// Auto detect MPU address
	mpu.setAddr(MPU6050_ADDRESS_AD0_HIGH);
	if (mpu.testConnection()) {
		printf_P(PSTR("MPU6050 ok (HIGH)\r\n"));
	} else {
		mpu.setAddr(MPU6050_ADDRESS_AD0_LOW);
		if (mpu.testConnection()) {
			printf_P(PSTR("MPU6050 ok (LOW)\r\n"));
		} else {
			printf_P(PSTR("MPU6050 init FAILED\r\n"));
		}
	}

	// Init MPU Stuff
	mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO); // Set Clock to ZGyro
	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS); // Set Gyro Sensitivity to config.h
	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS); //+- 2G
	initMPUlpf(); // Set Gyro Low Pass Filter
	mpu.setRate(0); // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
	mpu.setSleepEnabled(false);

	// set sensor orientation (from config)
	imu.init();
	if (!imu.loadGyroCalibration())
		calibrateGyro();

	// Init BL Controller
	initBlController();
	// motorTest();

	// Init RC-Input
	initRCPins();
	printf_P(PSTR("GO! Type HE for help, activate NL in Arduino Terminal!"));

	wdt_enable(WDTO_15MS);
}

void flashLED() {

}

/**********************************************/
/* Main Loop                                  */
/**********************************************/
void loop() {
	int32_t pitchPIDVal;
	int32_t rollPIDVal;
	static int32_t pitchErrorSum;
	static int32_t rollErrorSum;
	static int32_t pitchErrorOld;
	static int32_t rollErrorOld;

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
			evaluateRCAbsolute(); // t=30/142us,  returns rollRCSetPoint, pitchRCSetpoint
			utilLP_float(&pitchAngleSet, pitchPhiSet, rcLPF_tc); // t=16us
			utilLP_float(&rollAngleSet, rollPhiSet, rcLPF_tc); // t=28us
		} else {
			evaluateRCIntegrating(); // gives rollRCSpeed, pitchRCSpeed
			utilLP_float(&pitchAngleSet, pitchPhiSet, 0.01);
			utilLP_float(&rollAngleSet, rollPhiSet, 0.01);
		}

		evaluateRCSwitch();

		//****************************
		// pitch and roll PIDs
		//****************************
		// t=94us
		pitchPIDVal = //ComputePID(DT_INT_MS, angle[PITCH], pitchAngleSet*1000, &pitchErrorSum, &pitchErrorOld, pitchPIDpar.Kp, pitchPIDpar.Ki, pitchPIDpar.Kd);
				pitchPID.compute(DT_INT_MS, imu.angle[PITCH], pitchAngleSet * 1000);
		// t=94us
		rollPIDVal = //ComputePID(DT_INT_MS, angle[ROLL], rollAngleSet*1000, &rollErrorSum, &rollErrorOld, rollPIDpar.Kp, rollPIDpar.Ki, rollPIDpar.Kd);
				rollPID.compute(DT_INT_MS, imu.angle[ROLL], rollAngleSet * 1000);

		// motor control
		if (switchPos >= 0) {
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
		}

		//****************************
		// slow rate actions
		//****************************
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
				// wait 2 sec to settle ACC, before PID controlerbecomes active
				// Is this really necessary?
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
				if (switchPos > 0) {
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
			// regular ACC output
			pOutCnt++;
			if (pOutCnt == (LOOPUPDATE_FREQ / 10 / POUT_FREQ)) {
				// 600 us
//        if(config.accOutput==1) { 
//        Serial.print(angle[PITCH]); Serial.print(F(" ACC "));Serial.println(angle[ROLL]);
//      }
				pOutCnt = 0;
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

		//****************************
		// check RC channel timeouts
		//****************************

		checkRcTimeouts();

		//****************************
		// Evaluate Serial inputs
		//****************************
		sCmd.readSerial();
	}
}
