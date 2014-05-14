#include <avr/interrupt.h>
#include "Definitions.h"
#include "RCdecode.h"

int16_t transient[2];

uint8_t oscSpeed;
uint16_t transientsMag;
uint8_t transientsAxes;

void transients(uint8_t axes, int16_t val) {
	transientsAxes = axes;
	transientsMag = val;
	transient[PITCH] = 0;
	transient[ROLL] = 0;
}

void oscillation(uint8_t val) {
	static uint8_t pushOldSource;
	oscSpeed = val;
	if (val) {
		targetSources[TARGET_SOURCE_OSC][PITCH] = targetSources[targetSource][PITCH];
		targetSources[TARGET_SOURCE_OSC][ROLL] = targetSources[targetSource][ROLL];
		pushOldSource = targetSource;
		targetSource = TARGET_SOURCE_OSC;
	} else {
		targetSource = pushOldSource;
	}
}

void oscillationTask() {
	static int8_t oscDir[2] = { 1, 1 };
	for (uint8_t axis = ROLL; axis <= PITCH; axis++) {
		if (targetSources[TARGET_SOURCE_OSC][axis] >= liveControlDefs[axis].maxAngleND)
			oscDir[axis] = -1;
		else if (targetSources[TARGET_SOURCE_OSC][axis] <= liveControlDefs[axis].minAngleND)
			oscDir[axis] = 1;
		targetSources[TARGET_SOURCE_OSC][axis] += oscDir[axis] * oscSpeed;
	}
}

void transientTask() {
	for (uint8_t axis = ROLL; axis <= PITCH; axis++) {
		if (!transient[axis] && (transientsAxes & (1 << axis)))
			transient[axis] = transientsMag;
		else
			transient[axis] = -transientsMag;
	}
}
