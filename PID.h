#include <stdint.h>
#include "FastMathRoutines.h"

class PID {
public:
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;

	int32_t errorSum;
	int32_t errorOld;

	PID() {}

	PID(int32_t Kp, int32_t Ki, int32_t Kd) :
			Kp(Kp), Ki(Ki), Kd(Kd) {
	}

	void setCoefficients(int32_t Kp, int32_t Ki, int32_t Kd) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		errorSum = errorOld = 0;
	}

	int32_t compute(int32_t dt_ms, int32_t actual, int32_t target) {
		int32_t error = target - actual;
		int32_t Ierror = error * Ki * dt_ms;
		Ierror = constrain_int32(Ierror, -100000L, 100000L);
		errorSum += Ierror;

		int32_t out = (Kp * error) + errorSum + Kd * (error - errorOld) * dt_ms;
		errorOld = error;

		return out / 4096;
	}
};
