#include "Globals.h"
#include "Definitions.h"

extern int16_t getRollTarget();
extern int16_t getPitchTarget();

void mediumTask() {
	static uint8_t softStartDivider;

	imu.blendGyrosToAttitude();
	imu.blendAccToAttitude();
	imu.calculateAttitudeAngles();

	// and the target-angles:
	rollAngleSet = getRollTarget();
	pitchAngleSet = getPitchTarget();

	if (!softStartDivider) {
		softStartDivider = SOFTSTART_LATCH;

		// An attempt to stop self oscillations or at least reduce damage.
		if (overrate) {
			overrate--;
			if (softStart >= 12) {
				softStart--;
				softStartDivider = SOFTSTART_LATCH*12;
				gimbalState &= ~(POWER_RAMPING_COMPLETE);
			}
		}

		else if (!(gimbalState & POWER_RAMPING_COMPLETE)) {
			if (MOTORS_POWERED) {
				if (softStart < 16)
					softStart++;
				else {
					gimbalState |= POWER_RAMPING_COMPLETE;
					overrate = 0;
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

	softStartDivider--;
}
