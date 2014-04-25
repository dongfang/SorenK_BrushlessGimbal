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

	//void updateAccVector();
	void updateAccMagnitude1();
	void updateAccMagnitude2();

	// Get attitude data
	void fastUpdateCycle() {
		PERFORMANCE(BM_READ_GYROS);
	    readRotationRates();
	    mpu->startAccelerationsAsync();
	    PERFORMANCE(BM_BLEND_GYROS);
	    blendGyrosToAttitude();  // t=260us
	    PERFORMANCE(BM_BLEND_ACC);
	    readAccelerations();
	    blendAccToAttitude();   // t=146us
	    PERFORMANCE(BM_CALCULATE_AA);
	    calculateAttitudeAngles(); // t=468us
	    mpu->startRotationRatesAsync();
	    PERFORMANCE(BM_OTHER);
	}

	int16_t gyro[3];
	int16_t acc[3];

	// Apparently the accelerations and estG have the same scale, and divided by
	// accMagnitude_g_100 should yield 1/100 g units.
	float estG[3];
	int16_t accMagnitude_g_100;

	int16_t angle_cd[2];  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000

private:
	MPU6050* mpu;
	// TODO: This sensor orientation stuff is hardware related only.
	// Should that be moved into the MPU class?
	float gyroADCToRad_s;

	//float accLPF_f[3];

	float accComplFilterConstant;  // filter constant for complementary filter
	int32_t tmp_accMagnitude_g_2;

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

	/*
	 * Stupid approach to fast control!
	 * pitch motor comp = pitch gyro, as simply as that.
	 * roll motor comp = cos(pitch deflect) * roll gyro + sin(pitch deflect) * yaw gyro
	 * pitch deflect is pitch angle, if the airframe is horizontal.
	 * As pitch deflect is not a small angle, small-angle approx will not cut it. But we might 
	 * be fortunate that linear blending is good enough.
	 */
};

void initMPUlpf();
void initPIDs();

#endif
