#include <avr/interrupt.h>
#include "Definitions.h"
#include "RCdecode.h"

uint8_t targetSource;

int16_t pitchTransient;
int16_t rollTransient;

int32_t oscPitch;
int32_t oscRoll;
uint8_t oscSpeed;
uint16_t transientsMag;
uint8_t transientsAxes;

void transients(uint8_t axes, int16_t val) {
	transientsAxes = axes;
	transientsMag = val;
	pitchTransient = 0;
	rollTransient = 0;
}

int16_t getPitchTarget() {
	int16_t result = 0;
	if (targetSource == TARGET_SOURCE_RC) {
		result = rcData[RC_DATA_PITCH].setpoint; //xformRCToInt16(rcData[RC_DATA_PITCH].setpoint);
	}
	else if (targetSource == TARGET_SOURCE_OSC) {
		result = oscPitch;
	}
	return result + pitchTransient;
}

int16_t getRollTarget() {
	int16_t result = 0;
	if (targetSource == TARGET_SOURCE_RC) {
		result = rcData[RC_DATA_ROLL].setpoint; //xformRCToInt16(rcData[RC_DATA_ROLL].setpoint);
	}
	else if (targetSource == TARGET_SOURCE_OSC) {
		result = oscRoll;
	}
	return result + rollTransient;
}

void oscillation(uint8_t val) {
	oscSpeed = val;
	if (val) {
		oscPitch = getPitchTarget();
		oscRoll = getRollTarget();
		targetSource = TARGET_SOURCE_OSC;
	} else {
		targetSource = TARGET_SOURCE_RC; // TODO: Restore other value if not initially RC.
	}
}

int16_t xformRCToInt16(int16_t a) {
	return a * (65536L / 3600);
}

void oscillationTask() {
	static int8_t pitchOscDir = 1;
	static int8_t rollOscDir = 1;

	if (oscRoll >= xformRCToInt16(config.RCRoll.maxAngle))
		rollOscDir = -1;
	else if (oscRoll <= xformRCToInt16(config.RCRoll.minAngle))
		rollOscDir = 1;
	oscRoll += rollOscDir * oscSpeed;

	if (oscPitch >= xformRCToInt16(config.RCPitch.maxAngle))
		pitchOscDir = -1;
	else if (oscPitch <= xformRCToInt16(config.RCPitch.minAngle))
		pitchOscDir = 1;
	oscPitch += pitchOscDir * oscSpeed;
}

void transientTask() {
	if (!rollTransient && (transientsAxes & 1))
		rollTransient = transientsMag;
	else
		rollTransient = 0;
	if (!pitchTransient && (transientsAxes & 2))
		pitchTransient = transientsMag;
	else
		pitchTransient = 0;

}
