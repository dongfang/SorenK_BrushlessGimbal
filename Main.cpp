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
*/
// I2Cdev: Removed. Too fat, too slow.

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

#include "I2C.h"

// The globals
MPU6050 mpu; // Create MPU object
IMU imu(&mpu);
SerialCommand sCmd; // Create SerialCommand object
PID pitchPID;
PID rollPID;

uint8_t mcusr_mirror  __attribute__ ((section (".noinit")));
bool watchdogResetWasIntended __attribute__ ((section (".noinit")));
bool doubleFault __attribute__ ((section (".noinit")));

uint8_t syncCounter;

uint8_t motorPhases[2][3];
volatile uint8_t softStart;

uint8_t pwmSinMotorRoll[N_SIN];
uint8_t pwmSinMotorPitch[N_SIN];

RCData_t rcData[RC_DATA_SIZE];
int8_t switchPos = SW_UNKNOWN;

extern void slowLoop();

// D'uh!
int _putchar(char c, FILE* f) {
	serial0.put(c);
	return c;
}

int _getchar(FILE* f) {
	return serial0.get();
}

void initSerial() {
	serial0.init(config.serialBaudRate, _FDEV_SETUP_RW);
	fdevopen(_putchar, _getchar);
}

/**********************************************/
/* After reset, whether caused by WDT or
 * otherwise, HW is reset. Init all in one.
 * Stuff that inits state (ram memory things)
 * should not go here.
 **********************************************/

// Auto detect MPU address
/*
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
*/

void initMPU6050() {
	// HW
	// TODO: Only unjam in case of jam.
	// TODO: Auto addressing. Combine with unjam?
	mpu.setAddr(config.mpu6050Address);
	mpu.unjam();
	mpu.init();
}

void initHW() {
	// HW
	LED_DDR |= (1<<LED_BIT);
#ifdef DEBUG_SIGNALS
	DEBUG_DDR |= 1<<DEBUG_BIT1 | 1<<DEBUG_BIT2;
#endif

	// We don't use this, whatever it was.
	// CH2_PINMODE
	// CH3_PINMODE

	//HW
	initSerial();

	i2c_init();

	// HW
	// TODO: Only unjam in case of jam.
	// TODO: Auto addressing. Combine with unjam?
	mpu.setAddr(0x68);
	mpu.unjam();
	mpu.init();

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
	// config.readEEPROMOrDefault();

	// Init Sinus Arrays and Motor Stuff
	recalcMotorPower();

	// Init PIDs to reduce floating point operations.
	initPIDs();

	mpu.initSensorOrientation(config.majorAxis, config.axisReverseZ, config.axisRotateZ);
	//mpu.setDLPFMode(config.mpuLPF);

	// set sensor orientation (from config)
	// This needs a working acc. sensor.
	imu.init();

	if (!mpu.loadSensorCalibration()) {
		calibrateSensor(MPU6050::GYRO);
		mpu.resetAccCalibration();
	}

	printf_P(PSTR("Type \"help\" for help.\r\n"));
}

void checkwatchdog(void) __attribute__((naked))
__attribute__((section(".init3")));

int main();

void checkwatchdog(void) {
	mcusr_mirror = MCUSR;
	MCUSR = 0;

	// Set it to what we want while the system is up and running.
	wdt_enable(WDTO_1S);

	// Skip init
	//if ((mcusr_mirror & (1<<3)) && !watchdogResetWasIntended && !doubleFault)
	//	main();
}

int main() {
	// TODO: Can this be moved down?
	config.checkRAMImageValid();

	initHW();
	sei();

	// If it was not a WDT reset
	// if (watchdogResetWasIntended || doubleFault || !(mcusr_mirror & (1<<3))) {
	if (true)
	{
		initState();
		watchdogResetWasIntended = false;
		doubleFault = false;
	} else {
		printf_P(PSTR("WDT restart!\r\n"));
		doubleFault = true;
	}

	wdt_enable(WDT_TIMEOUT);
	printf_P(PSTR("Go!\r\n"));

	gimbalState = BACKGROUND_TASKS_RUN | PIDS_ARE_OUTPUT | MOTORS_POWERED;

	// Enable Timer1 Interrupt for timing
	TIMSK1 |= 1<<TOIE1;

	mpu.startRotationRatesAsync();

	slowLoop();
}
