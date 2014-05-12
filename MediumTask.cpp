#include "Globals.h"
#include "Definitions.h"

inline void updateSoftStart() {
// gimbal state actions
	if (!(gimbalState & POWER_RAMPING_COMPLETE)) {
		if (MOTORS_POWERED) {
			if (softStart < 16)
				softStart++;
			else {
				gimbalState |= POWER_RAMPING_COMPLETE;
				//LEDEvent(LED_SOFTSTART_MASK);
			}
		} else {
			if (softStart)
				softStart--;
			else {
				gimbalState |= POWER_RAMPING_COMPLETE;
				//LEDEvent(LED_SOFTSTART_MASK);
			}
		}
	}
}

extern int16_t getRollTarget();
extern int16_t getPitchTarget();

void mediumTask() {
	static uint8_t softStartDivider;

	imu.blendAccToAttitude();
	imu.calculateAttitudeAngles();

	// and the target-angles:
	rollAngleSet = getRollTarget();
	pitchAngleSet = getPitchTarget();

	if (!softStartDivider) {
		softStartDivider = 	SOFTSTART_LATCH;
		updateSoftStart();
	}

	softStartDivider--;
}
