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

	inline void prepareRates() {
		cli();
		int16_t p = gyro[PITCH];
		int32_t r = gyro[ROLL];
		int32_t y = gyro[YAW];
		sei();
		pitchRate = p;
		rollRate = (r * cosPitch - y * sinPitch) >> LOG_SIN_RES;
	}

	// Get attitude data
	void fastUpdateCycle() {
		uint8_t i;
		mpu->getAllSensorsAsync(gyro, acc);
	    mpu->startAllSensorsAsync();
	    // TODO: This should be doable at the medium rate too. Might work just fine.
	    // Can maybe increase fast rate to even faster, and medium also.
	    // blendGyrosToAttitude();
	    prepareRates();
	    for (i=0; i<3; i++)
	    	gyroSum[i] += gyro[i];
	}

	int16_t gyro[3];
	int16_t gyroSum[3];
	int16_t acc[3];

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

	void blendGyrosToAttitude();

private:
	MPU6050* mpu;
	// TODO: This sensor orientation stuff is hardware related only.
	// Should that be moved into the MPU class?
	float gyroADCToRad_dt;

	float accComplFilterConstant;  // filter constant for complementary filter
	int32_t tmp_accMagnitude_g_2;

	//void readRotationRates();
	//void readAccelerations();
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
