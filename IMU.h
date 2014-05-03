#ifndef __IMU_H
#define __IMU_H

#include "MPU6050.h"
#include "Util.h"
/*
 * IMU is always used like:
 *
    // update IMU data
    readGyros();   // t=386us
    updateGyroAttitude();  // t=260us
    updateACCAttitude();   // t=146us
    getAttitudeAngles(); // t=468us
 *
 */

class IMU {
public:
// Ctor
	IMU(MPU6050* mpu) : mpu(mpu) {}

	// Initially set vectors and such.
	// Do not call from WDT restart.
	void init();
	void updateAccMagnitude();

	// Get attitude data
	void fastUpdateCycle() {
		PERFORMANCE(BM_READ_GYROS);
	    readRotationRates();
	    mpu->startAccelerationsAsync();
	    PERFORMANCE(BM_BLEND_GYROS);
	    blendGyrosToAttitude();
	    PERFORMANCE(BM_BLEND_ACC);
	    readAccelerations();
	    blendAccToAttitude();
	    PERFORMANCE(BM_CALCULATE_AA);
	    calculateAttitudeAngles();
	    mpu->startRotationRatesAsync();
	    PERFORMANCE(BM_OTHER);
	}

	int16_t gyro[3];
	int16_t acc[3];

	// Apparently the accelerations and estG have the same scale, and divided by
	// accMagnitude_g_100 should yield 1/100 g units.
	float estG[3];

	int32_t angle_cd[2];  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000
	int32_t accMagnitude;
	int32_t minAccMagnitude;
	int32_t maxAccMagnitude;

private:
	MPU6050* mpu;
	// TODO: This sensor orientation stuff is hardware related only.
	// Should that be moved into the MPU class?
	float gyroADCToRad_s;

	//float accLPF_f[3];

	float accComplFilterConstant;  // filter constant for complementary filter

	// int16_t acc_25deg;      //** TODO: check

	void readRotationRates();
	void readAccelerations();
	void blendGyrosToAttitude();
	void blendAccToAttitude();
	void calculateAttitudeAngles();
	// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
	// needs angle in radian units !
	inline void rotateV(float* v, float* delta) {
		float v_tmp[3] = { v[X], v[Y], v[Z] };
		v[Z] -= delta[ROLL] * v_tmp[X] + delta[PITCH] * v_tmp[Y];
		v[X] += delta[ROLL] * v_tmp[Z] - delta[YAW] * v_tmp[Y];
		v[Y] += delta[PITCH] * v_tmp[Z] + delta[YAW] * v_tmp[X];
	}
};

// void initMPUlpf();
void initPIDs();

#endif
