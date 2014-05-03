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
	void blendAccToAttitude();
	void calculateAttitudeAngles();

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

	// Apparently the accelerations and estG have the same scale, and divided by
	// accMagnitude_g_100 should yield 1/100 g units.
	float estG[3];

	// Now in units depending on the acc. meters scale.
	uint32_t accMagnitude;
	// These limits are to the same scale.
	uint32_t accMagnitudeLowerLimit;
	uint32_t accMagnitudeUpperLimit;

//	int32_t angle_md[2];  // absolute angle inclination in multiple of 0.001 degree    180 deg = 180000
	int16_t angle_cd[2];  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000

private:
	MPU6050* mpu;
	// TODO: This sensor orientation stuff is hardware related only.
	// Should that be moved into the MPU class?
	float gyroADCToRad_s;

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

// void initMPUlpf();
void initPIDs();

#endif
