#include "Globals.h"
#include "RCdecode.h"
#include "Util.h"
#include "Board.h"

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <math.h>

int16_t rollAngleSet;
int16_t pitchAngleSet;

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
	uint8_t before = gimbalState & ~GS_POWER_RAMPING_COMPLETE;
	gimbalState &= GS_GIMBAL_RETRACTED;
	mpu.recalibrateSensor(&complainAboutSensorMotion, which);
	printf_P(PSTR("done.\r\n"));
	gimbalState = before;
}

static uint8_t state;

void startAutosetup() {
	state = 0;
	interfaceState = INTERFACE_STATE_AUTOSETUP;
}

#ifdef SUPPORT_AUTOSETUP
const char MOVED_PROMPT[] PROGMEM
		= "What did the gimbal do?\r\n1)Pitch down\r\n2)Pitch up\r\n3)Roll left\r\n4)Roll right\r\na)Do again\r\n";
const char REVERSE_PROMPT[] PROGMEM = "Please reverse the direction of the %S motor (plug it the other way).\r\n";
const char OBSERVE_PROMPT[] PROGMEM = "Please see in which direction the gimbal moves after pressing SPACE\r\n";
const char SWAP_PROMPT[] PROGMEM = "Please swap the pitch and roll motor connections.\r\n";

volatile uint8_t autosetupState;

extern void recalcMotorPower(uint8_t rollPower, uint8_t pitchPower);
extern void recalcMotorPower();

#define GYRO_FILTER 32L

void runAutosetup() {
	static uint8_t resultingMajorAxis;
	static uint8_t resultingZRotation;
	static bool resultingZReversal;

	static int16_t pitchGyroSeen;
	static int16_t rollGyroSeen;

	uint16_t largestAcc = 0;
	uint8_t i;

	int ch;

	switch (state) {
	case 0:
		gimbalState = 0; // No output and no power.
		mpu.initSensorOrientation(0, false, 0);
		mpu.resetAccCalibration();
		printf_P(PSTR("Please place gimbal about horizontal and press SPACE\r\n"));
		serial0.clear();
		state++;
		break;
	case 1:
		if (gimbalState & GS_POWER_RAMPING_COMPLETE) {
			ch = getchar();
			if (ch == ' ') {
				state++;
			} else if (ch == 27) {
				interfaceState = INTERFACE_STATE_CONSOLE;
			}
		}
		break;
	case 2:
		delay_ms(10); // make sure a sample is taken.
		resultingMajorAxis = 0;
		for (i = 0; i < 3; i++) {
			if (abs16(mpu.acc[i]) > largestAcc) {
				largestAcc = abs16(mpu.acc[i]);
				resultingMajorAxis = 2 - i;
			}
		}
		printf_P(PSTR("majorAxis: %d\r\n"), resultingMajorAxis);
		// We really should have a test of largest acc is sufficiently larger than the others..
		mpu.initSensorOrientation(resultingMajorAxis, false, 0);
		state++;
		break;
	case 3:
		delay_ms(10);  // make sure a sample is taken.
		if (mpu.acc[Z] < 0) {
			resultingZReversal = true;
			printf_P(PSTR("Reversed Z\r\n"));
		} else {
			resultingZReversal = false;
			printf_P(PSTR("Did not reverse Z\r\n"));
		}
		mpu.initSensorOrientation(resultingMajorAxis, resultingZReversal, 0);
		state++;
		break;
	case 4:
		pitchGyroSeen = rollGyroSeen = 0;
		recalcMotorPower(125, 125);
		printf_P(OBSERVE_PROMPT);
		serial0.clear();
		gimbalState = GS_MOTORS_POWERED;
		autosetupState = AS_RUNNING | AS_RESET;
		state++;
		break;
	case 5:
		if (gimbalState & GS_POWER_RAMPING_COMPLETE) {
			ch = getchar();
			if (ch == ' ') {
				autosetupState = AS_RUNNING;
				state++;
			} else if (ch == 27) {
				interfaceState = INTERFACE_STATE_CONSOLE;
			}
		}
		break;
	case 6:
		if (!(autosetupState & AS_RUNNING)) { // has finished run.
			printf_P(PSTR("%S"), MOVED_PROMPT);
			serial0.clear();
			state++;
		} else {
			rollGyroSeen = (mpu.gyro[ROLL] + rollGyroSeen * (GYRO_FILTER - 1)) / GYRO_FILTER;
			pitchGyroSeen = (mpu.gyro[PITCH] + pitchGyroSeen * (GYRO_FILTER - 1)) / GYRO_FILTER;
		}
		break;
	case 7:
		resultingZRotation = 0;
		ch = getchar();
		if (ch == 'a') {
			state = 4;
		} else if (ch == '1' || ch == '2') {
			printf_P(SWAP_PROMPT);
			state = 4;
		} else if (ch == '3') {
			printf_P(REVERSE_PROMPT, PSTR("roll"));
			state = 4;
		} else if (ch == '4') {
			if (abs16(pitchGyroSeen) > abs16(rollGyroSeen)) {
				resultingZRotation = 1; // will make current pitch future roll.
				if (pitchGyroSeen < 0) {
					printf_P(PSTR("Rotated sensor 270\r\n"));
					resultingZRotation = 3; // will make current pitch minus future roll.
				} else {
					printf_P(PSTR("Rotated sensor 90\r\n"));
				}
			} else if (rollGyroSeen < 0) {
				resultingZRotation = 2;
				printf_P(PSTR("Rotated sensor 180\r\n"));
			} else
				printf_P(PSTR("Rotated sensor 0\r\n"));

			mpu.initSensorOrientation(resultingMajorAxis, resultingZReversal, resultingZRotation);
			state++;
		} else if (ch == 27) {
			interfaceState = INTERFACE_STATE_CONSOLE;
		} else if (ch >= 0) {
			printf_P(PSTR("What? %d\r\n"), ch);
		}
		break;
	case 8:
		printf_P(OBSERVE_PROMPT);
		pitchGyroSeen = rollGyroSeen = 0;
		gimbalState = GS_MOTORS_POWERED;
		autosetupState = AS_RUNNING | AS_RESET | AS_IS_PITCH;
		state++;
		break;
	case 9:
		if (gimbalState & GS_POWER_RAMPING_COMPLETE) {
			ch = getchar();
			if (ch == ' ') {
				gimbalState = GS_MOTORS_POWERED;
				autosetupState = AS_RUNNING | AS_IS_PITCH;
				state++;
			} else if (ch == 27) {
				interfaceState = INTERFACE_STATE_CONSOLE;
			}
		}
		break;
	case 10:
		if (!(autosetupState & AS_RUNNING)) {
			printf_P(PSTR("%S"), MOVED_PROMPT);
			serial0.clear();
			state++;
		} else {
			rollGyroSeen = (mpu.gyro[ROLL] + rollGyroSeen * (GYRO_FILTER - 1)) / GYRO_FILTER;
			pitchGyroSeen = (mpu.gyro[PITCH] + pitchGyroSeen * (GYRO_FILTER - 1)) / GYRO_FILTER;
			// printf_P(PSTR("%d\r\n"), pitchGyroSeen);
		}
		break;
	case 11:
		ch = getchar();
		if (ch == 'a') {
			state = 8;
		} else if (ch == '3' || ch == '4') {
			printf_P(SWAP_PROMPT);
			state = 4;
		} else if (ch == '1') {
			printf_P(REVERSE_PROMPT, PSTR("pitch"));
			state = 8;
		} else if (ch == '2') {
			if (abs16(rollGyroSeen) > abs16(pitchGyroSeen)) {
				printf_P(PSTR("Unexpected error: Z rotation\r\n"));
			} else if (pitchGyroSeen < 0) {
				printf_P(PSTR("Unexpected error: pitch direction (%d)\r\n"), pitchGyroSeen);
			} else {
				config.majorAxis = resultingMajorAxis;
				config.axisReverseZ = resultingZReversal;
				config.axisRotateZ = resultingZRotation;
			}
			state++;
		} else if (ch == 27) {
			interfaceState = INTERFACE_STATE_CONSOLE;
		} else if (ch >= 0) {
			printf_P(PSTR("What? %d\r\n"), ch);
		}
		break;
	case 12:
		recalcMotorPower(); // back to default
		// we just assume nobody ever could dream of (or be able to) starting this from any other interface than console.
		interfaceState = INTERFACE_STATE_CONSOLE;
		printf_P(PSTR("Done. Don't forget to write the new settings.\r\n"));
		break;
	}
}
#endif
