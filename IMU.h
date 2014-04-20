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
	struct SensorAxisDef {
	  char idx;
	  int  dir;
	} t_sensorAxisDef;

	struct SensorOrientationDef {
	  SensorAxisDef Gyro[3];
	  SensorAxisDef Acc[3];
	};

// Ctor
	IMU(MPU6050 mpu) : mpu(mpu) {}

	// Initially set vectors and such.
	// Do not call from WDT restart.
	void init();

	// Configure:
	void initSensorOrientation();

	// Load EEPROM-stored offsets. Return true if success.
	bool loadGyroCalibration();

	// Do the calibration ritual and store result in EEPROM.
	void recalibrateGyros();

	void readAcc(uint8_t axis);

	void updateAccVector();

	// Get attitude data
	void fastUpdateCycle() {
		PERFORMANCE(BM_READ_GYROS);
	    readGyros();   // t=386us
	    PERFORMANCE(BM_BLEND_GYROS);
	    blendGyrosToAttitude();  // t=260us
	    PERFORMANCE(BM_BLEND_ACC);
	    blendAccToAttitude();   // t=146us
	    PERFORMANCE(BM_CALCULATE_AA);
	    calculateAttitudeAngles(); // t=468us
	    PERFORMANCE(BM_OTHER);
	}

	// Offset and sign-fixed values from MPU data.
	// TODO: That processing might as will be done by the MPU itself.
	int16_t gyro[3];
	int16_t acc[3];
	int32_t angle[2];  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000

private:
	MPU6050 mpu;
	// TODO: This sensor orientation stuff is hardware related only.
	// Should that be moved into the MPU class?
	SensorOrientationDef sensorDef;

	// gyro calibration value
	int16_t gyroOffset[4];

	float gyroScale;
	float accComplFilterConstant;

	float EstG[3];
	float accLPF_f[3];
	int32_t accLPF_i[3];
	int32_t accMagnitude_g_100;

	float AccComplFilterConst;  // filter constant for complementary filter

	// int16_t acc_25deg;      //** TODO: check

	// TODO: Use transformations instead of the magic constants (not important)
	void initSensorOrientationFaceUp();
	void initSensorOrientationChipTextRightSideUp();
	void initSensorOrientationChipTextStandingOnEnd();

	void readGyros();
	void blendGyrosToAttitude();
	void blendAccToAttitude();
	void calculateAttitudeAngles();
	void rotateV(float* v, float* delta);

	// swap two char items
	inline void swap_char(char * a, char * b) {
	  char tmp = *a;
	  *a = *b;
	  *b = tmp;
	}

	// swap two int items
	inline void swap_int(int * a, int * b) {
	  int tmp = *a;
	  *a = *b;
	  *b = tmp;
	}

	void saveGyroCalibration();

	uint16_t CRC();
};

void calibrateGyro();
void initMPUlpf();
void initPIDs();

#endif
