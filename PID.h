#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "Util.h"

/* Scaled down by factor: We want coeffs to fit into plain int16s. They were up to 150000, so maybe factor 8 smaller.
   inputs are factor 10 less so we are at about 80 times smaller. 
   4096 / 80 about = 64 */

class PID {
public:
	int16_t Kp;
	int16_t Ki;
	int16_t Kd;

	int32_t errorIntegral;
	int16_t errorOld;

	PID() {}

	PID(int16_t Kp, int16_t Ki, int16_t Kd) :
			Kp(Kp), Ki(Ki), Kd(Kd) {
	}

	void setCoefficients(int16_t Kp, int16_t Ki, int16_t Kd) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		errorIntegral = errorOld = 0;
	}

	int16_t compute(int16_t dt_ms, int16_t actual, int16_t target) {
		int16_t error = target - actual;
		int32_t Ierror = error * Ki * dt_ms;
		Ierror = constrain_int32(Ierror, -10000L, 10000L);
		errorIntegral += Ierror; // The integration

		int32_t out = (Kp * error) + errorIntegral + Kd * (error - errorOld) * dt_ms;
		errorOld = error;

		return out / 64;
	}
};

#endif
