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

#include <avr/eeprom.h>
#include <stdio.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

#include "Board.h"
#include "Globals.h"
#include "SerialStream.h"
#include "SerialCommand.h"

#include "Util.h"     			  // fast Math functions required by orientationRoutines.h
#include "RCdecode.h"             // RC Decoder to move camera by servo signal input
#include "BLcontroller.h"         // Motor Movement Functions and Timer Config

// The globals
MPU6050 mpu; // Create MPU object
IMU imu(mpu);
SerialCommand sCmd; // Create SerialCommand object
PID pitchPID;
PID rollPID;

uint8_t mcusr_mirror  __attribute__ ((section (".noinit")));

bool runMainLoop;
uint8_t syncCounter;

uint8_t motorPhases[2][3];
uint8_t softStart;

uint8_t pwmSinMotorPitch[N_SIN];
uint8_t pwmSinMotorRoll[N_SIN];

RCData_t rcData[RC_DATA_SIZE];
int8_t switchPos = SW_UNKNOWN;

extern void mainLoop();

// D'uh!
int _putchar(char c, FILE* f) {
	serial0.put(c);
	return c;
}

void initSerial() {
	serial0.init(115200, _FDEV_SETUP_WRITE);
	fdevopen(_putchar, NULL);
}

/**********************************************/
/* After reset, whether caused by WDT or
 * otherwise, HW is reset. Init all in one.
 * Stuff that inits state (ram memory things)
 * should not go here.
 **********************************************/
void initHW() {
	// HW
	LED_DDR |= (1<<LED_BIT);

	// We don't use this, whatever it was.
	// CH2_PINMODE
	// CH3_PINMODE

	//HW
	initSerial();

	// HW
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

	// Init BL Controller
	// Set all outputs to zero.
	initBlController();
	// motorTest();

	// Init RC-Input
	initRC();
}

/**********************************************/
/* Initialization                             */
/**********************************************/
void initState() {
	// just for debugging
#ifdef STACKHEAPCHECK_ENABLE
	stackCheck();
	heapCheck();
#endif

	// Set Serial Protocol Commands
	setSerialProtocol();

	// Read Config, fill with default settings if versions do not match or CRC fails
	config.readEEPROMOrDefault();
	updateAllParameters();

	// Init Sinus Arrays and Motor Stuff
	recalcMotorStuff();

	// Init PIDs to reduce floating point operations.
	initPIDs();

	// init RC variables
	initRCFilter();

	// mpu.setDLPFMode(config.mpuLPF);
	// initMPUlpf();
	// set sensor orientation (from config)
	// This needs a working acc. sensor.
	imu.init();

	if (!imu.loadGyroCalibration())
		calibrateGyro();

	printf_P(PSTR("GO! Type HE for help, activate NL in Arduino Terminal!\r\n"));
}

void checkwatchdog(void) __attribute__((naked))
__attribute__((section(".init3")));

void checkwatchdog(void) {
	mcusr_mirror = MCUSR;
	MCUSR = 0;

	// Set it to what we want while the system is up and running.
	//wdt_enable(WDTO_1S);
}

int main() {
	// TODO: Can this be moved down?
	sei();

	initHW();

	// If it was not a WDT reset
	if (!(mcusr_mirror & (1<<3)))
		initState();

	//wdt_enable(WDTO_15MS);

	while(1) {
		mainLoop();
	}
}
