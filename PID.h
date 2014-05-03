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
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;

	int32_t errorIntegral;
	int32_t errorOld;

	PID() {}

	PID(int16_t Kp, int16_t Ki, int16_t Kd) :
			Kp(Kp), Ki(Ki), Kd(Kd) {
	}

	void setCoefficients(int32_t Kp, int32_t Ki, int32_t Kd) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		errorIntegral = errorOld = 0;
	}

	// Yes I have checked if int16's would do and the answer was no.
	int16_t compute(int16_t actual, int16_t target) {
		int32_t error_ref = target - actual;
		int32_t Ierror_ref = error_ref * Ki;

		Ierror_ref = constrain_int32(Ierror_ref, -1000L, 1000L);
		errorIntegral += Ierror_ref; // The integration

		// int32_t f1_ref = (int32_t)Kp * error_ref;
		// int32_t f2_ref = (int32_t)Kd * (error_ref - errorOld_ref);
		// int32_t out = f1_ref + errorIntegral + f2_ref;

		int32_t out = (int32_t)Kp * error_ref + errorIntegral + (int32_t)Kd * (error_ref - errorOld);

		errorOld = error_ref;

		return out/256;
	}
};

#endif
