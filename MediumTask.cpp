#include "Globals.h"
#include "Definitions.h"

uint8_t targetSource;
int16_t targetSources[TARGET_SOURCES_END][2];

int16_t getTarget(uint8_t axis) {
	static int16_t lastResults[2];

	int16_t result = targetSources[targetSource][axis];
	int16_t diff = result - lastResults[axis];

	if (diff) {
		if (diff > config.controlInput[PITCH].maxSlewRate) {
			result = lastResults[axis] + config.controlInput[PITCH].maxSlewRate;
		} else if (diff < -config.controlInput[PITCH].maxSlewRate) {
			result = lastResults[axis] - config.controlInput[PITCH].maxSlewRate;
		}

		LiveControlAxisDef* def = liveControlDefs + axis;

		if (result > def->maxAngleND)
			result = def->maxAngleND;
		if (result < def->minAngleND)
			result = def->minAngleND;

		lastResults[axis] = result;
	}
	return result + transient[axis];
}

void mediumTask() {
	static uint8_t softStartDivider;

	imu.blendGyrosToAttitude();
	imu.blendAccToAttitude();
	imu.calculateAttitudeAngles();

	// and the target-angles:
	rollAngleSet = getTarget(ROLL);
	pitchAngleSet = getTarget(PITCH);

	if (!softStartDivider) {
		softStartDivider = SOFTSTART_LATCH;

		// An attempt to stop self oscillations or at least reduce damage.
		if (overrate) {
			overrate--;
			if (softStart >= 12) {
				softStart--;
				softStartDivider = SOFTSTART_LATCH * 12;
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
