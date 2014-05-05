#ifndef __IMU_H
#define __IMU_H

#include "MPU6050.h"
#include "Util.h"
#include <avr/interrupt.h>

class IMU {
public:
// Ctor
	IMU(MPU6050* mpu) : mpu(mpu) {}

	// Initially set vectors and such.
	// Do not call from WDT restart.
	void init();
	void updateAccMagnitude();
	void blendAccToAttitude();
	void calculateAttitudeAngles();

	// Coming in same units as the gyro outputs, whatever that be.
	inline int16_t pitchDTerm() {
		cli();
		int16_t p = gyro[PITCH];
		sei();
		return p;
	}

	// Coming in same units as the gyro outputs, whatever that be.
	inline int16_t rollDTerm() {
		cli();
		int32_t r = gyro[ROLL];
		int32_t y = gyro[YAW];
		sei();
		return (r * cosPitch - y * sinPitch) >> LOG_SIN_RES;
	}

	// Get attitude data
	void fastUpdateCycle() {
		PERFORMANCE(BM_READ_GYROS);
		// There is a case of optimistic scheduling here: We start the background
		// fetching of acc. data before the gyro data has been read from the (same)
		// buffer. It should be verified that there is no collision.
		// OK at first, we just execute it the safe way around, missing the opportunity
		// to start acc.meter sampling early.
		mpu->getRotationRatesAsync(gyro);
	    mpu->startAccelerationsAsync();
	    PERFORMANCE(BM_BLEND_GYROS);
	    blendGyrosToAttitude();
	    PERFORMANCE(BM_BLEND_ACC);
	    readAccelerations();
	    // blendAccToAttitude();
	    // PERFORMANCE(BM_CALCULATE_AA);
	    //calculateAttitudeAngles();
	    mpu->startRotationRatesAsync();
	    PERFORMANCE(BM_OTHER);
	}

	int16_t gyro[3];
	int16_t acc[3];

	static const uint8_t LOG_SIN_RES = 6;
	int8_t sinPitch;
	int8_t cosPitch;

	// Apparently the accelerations and estG have the same scale, and divided by
	// accMagnitude_g_100 should yield 1/100 g units.
	float estG[3];

	// Now in units depending on the acc. meters scale.
	uint32_t accMagnitude;
	uint32_t minAccMagnitude;
	uint32_t maxAccMagnitude;

	int16_t angle_i16[2];  // absolute angle inclination in int16-degrees

private:
	MPU6050* mpu;
	// TODO: This sensor orientation stuff is hardware related only.
	// Should that be moved into the MPU class?
	float gyroADCToRad_dt;

	float accComplFilterConstant;  // filter constant for complementary filter
	int32_t tmp_accMagnitude_g_2;

	void readRotationRates();
	void readAccelerations();
	void blendGyrosToAttitude();
	// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
	// needs angle in radian units !
	inline void rotateV(float* v, float* delta) {
		float v_tmp[3] = { v[X], v[Y], v[Z] };
		v[Z] -= delta[ROLL] * v_tmp[X] + delta[PITCH] * v_tmp[Y];
		v[X] += delta[ROLL] * v_tmp[Z] - delta[YAW] * v_tmp[Y];
		v[Y] += delta[PITCH] * v_tmp[Z] + delta[YAW] * v_tmp[X];
	}

	/*
	 * Stupid approach to fast control!
	 * pitch motor comp = pitch gyro, as simply as that.
	 * roll motor comp = cos(pitch deflect) * roll gyro + sin(pitch deflect) * yaw gyro
	 * pitch deflect is pitch angle, if the airframe is horizontal.
	 * As pitch deflect is not a small angle, small-angle approx will not cut it. But we might 
	 * be fortunate that linear blending is good enough.
	 */
};

#endif
