#include "Commands.h"
#include "Definitions.h"
#include "Globals.h"
#include "Mavlink.h"
#include "Util.h"
#include <avr/interrupt.h>
#include <avr/wdt.h>

void reset() {
	watchdogResetWasIntended = true;
	wdt_enable(WDTO_15MS);
	cli();
	while (1)
		;
}

void calibrateGyro() {
	calibrateSensor(MPU6050::GYRO);
}

void calibrateAcc() {
	calibrateSensor(MPU6050::ACC);
}

void run() {
	extend();
	gimbalState = GS_MOTORS_POWERED | GS_PIDS_ARE_OUTPUT;
}

void stop() {
	gimbalState = GS_PIDS_ARE_OUTPUT;
}

#ifdef SUPPORT_RETRACT
extern void setYawServoOut(uint16_t);
extern void setRetractServoOut(uint16_t);
void retract() {
	if (!(gimbalState & GS_GIMBAL_RETRACTED)) {
		freeze();
		setYawServoOut(1500);
		delay_ms(1000);
		gimbalState |= GS_GIMBAL_RETRACTED;
		setRetractServoOut(config.retractedServoUsec);
		delay_ms(1000);
	}
}

void extend() {
	if (gimbalState & GS_GIMBAL_RETRACTED) {
		gimbalState &= ~GS_GIMBAL_RETRACTED;
		setRetractServoOut(config.extendedServoUsec);
		delay_ms(1000);
	}
}
#endif

void freeze() {
	gimbalState = GS_MOTORS_POWERED | GS_GIMBAL_FROZEN;
}

void motorTest() {
	gimbalState = GS_MOTORS_POWERED | GS_GIMBAL_MOTORTEST;
}

#ifdef SUPPORT_MAVLINK
void goMavlink() {
	if (interfaceState != INTERFACE_STATE_MAVLINK) {
		interfaceState = INTERFACE_STATE_MAVLINK;
		targetSource = TARGET_SOURCE_MAVLINK;
		mavlink_updateTarget();
	}
}
#endif
