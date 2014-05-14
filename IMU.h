#ifndef __IMU_H
#define __IMU_H

#include "MPU6050.h"
#include "Util.h"
#include "Definitions.h"
#include <avr/interrupt.h>

class IMU {
public:
	IMU(MPU6050* mpu) : mpu(mpu) {}

	// Initially set vectors and such.
	// Do not call from WDT restart.
	void init();

	inline void blendGyrosToAttitude() {
		uint8_t axis;
		float deltaGyroAngle[3];
		for (axis = 0; axis < 3; axis++) {
			deltaGyroAngle[axis] = mpu->gyroSum[axis] * gyroADCToRad_dt;
		}
		rotateV(estG, deltaGyroAngle);
		mpu->resetGyroSums();
	}

	inline void blendAccToAttitude() {
		uint8_t axis;
		static bool estGInitialized;
		// Apply complimentary filter (Gyro drift correction)
		for (axis = 0; axis < 3; axis++) {
			cli();
			int16_t _acc = mpu->acc[axis];
			float _eg = estG[axis];
			sei();
			if (estGInitialized)
				_eg = _eg * (1.0 - accComplFilterConstant) + _acc * accComplFilterConstant; // note: this is different from MultiWii (wrong brackets postion in MultiWii ??.
			else
				_eg = _acc;
			cli();
			estG[axis] = _eg;
			sei();
			estGInitialized = true;
		}
	}

	inline void calculateAttitudeAngles() {
		// attitude of the estimated vector
		// Here, the traditional meanings of pitch and roll are reversed.
		// That is ultimately okay! In an airframe, it is first pitch then roll.
		// On the typical gimbal frame, it is opposite.
		int16_t resultRoll, resultPitch;

		cli();
		float x = estG[X];
		float y = estG[Y];
		float z = estG[Z];
		sei();

		float hypo = sqrt(y * y + z * z);
		resultRoll = ANGLE_SCALING * Rajan_FastArcTan2(x, hypo);
		resultPitch = ANGLE_SCALING * Rajan_FastArcTan2(y, z);

		// Assume the G vector is normal (meaning it has length mpu->accToG()
		cli();
		angle_i16[ROLL] = resultRoll;
		angle_i16[PITCH] = resultPitch;
		sei();

		//sinPitch = ((int16_t)y) >> (mpu->logAccToG()-LOG_SIN_RES);
		//cosPitch = ((int16_t)z) >> (mpu->logAccToG()-LOG_SIN_RES);

		sinPitch = (1 << LOG_SIN_RES) * y / hypo;
		cosPitch = (1 << LOG_SIN_RES) * z / hypo;
	}

	inline void prepareRates() {
		cli();
		int16_t p = mpu->gyro[PITCH];
		int32_t r = mpu->gyro[ROLL];
		int32_t y = mpu->gyro[YAW];
		sei();
		pitchRate = p;
		rollRate = (r * cosPitch - y * sinPitch) >> LOG_SIN_RES;
	}

	// Get attitude data
	inline void fastUpdateCycle() {
		mpu->getAllSensorsAsync();
	    mpu->startAllSensorsAsync();
	    // TODO: This should be doable at the medium rate too. Might work just fine.
	    // Can maybe increase fast rate to even faster, and medium also.
	    // blendGyrosToAttitude();
	    prepareRates();
	}

	// Public just so we can debug them. They are of no public use.
	static const uint8_t LOG_SIN_RES = 6;
	int8_t sinPitch;
	int8_t cosPitch;

	// The pitch and roll rates, on the axes of the motors (not the gyros).
	// This assumes the platform (roll axis) is about horizontal.
	int16_t pitchRate;
	int16_t rollRate;

	// Apparently the accelerations and estG have the same scale, and divided by
	// accMagnitude_g_100 should yield 1/100 g units.
	float estG[3];

	// Now in units depending on the acc. meters scale.
	// This is not necessary to to.
	// uint32_t accMagnitude;
	// uint32_t minAccMagnitude;
	// uint32_t maxAccMagnitude;

	int16_t angle_i16[2];  // absolute angle inclination in int16-degrees

private:
	MPU6050* mpu;

	float gyroADCToRad_dt;
	float accComplFilterConstant;  // filter constant for complementary filter

	// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
	// needs angle in radian units !
	inline void rotateV(float* v, float* delta) {
		float v_tmp[3] = { v[X], v[Y], v[Z] };
		v[Z] -= delta[ROLL] * v_tmp[X] + delta[PITCH] * v_tmp[Y];
		v[X] += delta[ROLL] * v_tmp[Z] - delta[YAW] * v_tmp[Y];
		v[Y] += delta[PITCH] * v_tmp[Z] + delta[YAW] * v_tmp[X];
	}
};

#endif
