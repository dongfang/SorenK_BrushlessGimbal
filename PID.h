#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "Util.h"
#include "Board.h"

// The dt parameter was removed (there was an error in its use anyway, and since
// it was the integral number of millis, the rounding error was huge.
// The only implication is that if you change the main loop frequency by factor k,
// you have to multiply ki by 1/k, and multiply kd by k.
class PID {
public:
	int16_t Kp;
	int16_t Ki;
	int16_t Kd;

	int32_t errorIntegral;

	PID() {}

	PID(int16_t Kp, int16_t Ki, int16_t Kd) :
			Kp(Kp), Ki(Ki), Kd(Kd) {
	}

	void setCoefficients(int16_t Kp, int16_t Ki, int16_t Kd) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		errorIntegral =  0;
	}

	// Yes I have checked if int16's would do and the answer was no.
	int16_t compute(int16_t actual, int16_t target, int16_t d) {
		int32_t error = target - actual;
		int32_t Ierror = error * Ki;

		Ierror = constrain_int32(Ierror, -10000L, 10000L);
		errorIntegral += Ierror; // The integration

		int32_t out = (int32_t)Kp * error + errorIntegral - (int32_t)Kd * d;

		return out/256;
	}
};

#endif
