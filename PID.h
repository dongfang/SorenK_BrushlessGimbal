#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "Util.h"

class PID {
public:
	int16_t Kp;
	int16_t Ki;
	int16_t Kd;

	int32_t errorSum;
	int32_t errorOld;

	PID() {}

	PID(int16_t Kp, int16_t Ki, int16_t Kd) :
			Kp(Kp), Ki(Ki), Kd(Kd) {
	}

	void setCoefficients(int16_t Kp, int16_t Ki, int16_t Kd) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		errorSum = errorOld = 0;
	}

	int32_t compute(int16_t dt_ms, int16_t actual, int16_t target) {
		int16_t error = target - actual;
		int32_t Ierror = error * Ki * dt_ms;
		Ierror = constrain_int32(Ierror, -100000L, 100000L);
		errorSum += Ierror;

		int16_t out = (Kp * error) + errorSum + Kd * (error - errorOld) * dt_ms;
		errorOld = error;

		return out / 64;
	}
};

#endif
