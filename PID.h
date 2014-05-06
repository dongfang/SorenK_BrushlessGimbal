#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "Util.h"

extern MPU6050 mpu; // ouch thats ugly.

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
	int32_t ILimit;

	PID() {
	}

	PID(int16_t Kp, int16_t Ki, int16_t Kd) :
			Kp(Kp), Ki(Ki), Kd(Kd) {
	}

	void setCoefficients(int16_t Kp, int16_t Ki, int16_t Kd, int32_t ILimit) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		this->ILimit = ILimit;
		errorIntegral = 0;
	}

	// Yes I have checked if int16's would do and the answer was no.
	int16_t compute(int16_t actual, int16_t target, int16_t d) {
		int32_t error = target - actual;
		int32_t Ierror = error * Ki;

		if (Ierror < -ILimit) {
			LEDEvent(I_LIMIT_MASK);
			Ierror = -ILimit;
		} else if (Ierror > ILimit) {
			LEDEvent(I_LIMIT_MASK);
			Ierror = ILimit;
		}

		errorIntegral += Ierror; // The integration

		// The >> shift count is just an arbitrary number I came up with,
		// to make reasonably valued Kd factors.
		int32_t out = (int32_t) Kp * error + errorIntegral - ((int32_t) Kd * d >> (mpu.logGyroToDeg_s() - 3));

		// This is not the "same" division by 4096 as in Martinez: The subsequent division by 8
		// in MainLoop has been removed. The angles (error and target) are to a different scale.
		// Kp, Ki and Kd are much smaller here (16 bit)
		return out / 4096;
	}
};

#endif
