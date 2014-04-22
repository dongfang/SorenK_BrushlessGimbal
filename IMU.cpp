// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

#include "IMU.h"
#include "Globals.h"
#include <math.h>

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
#ifndef ACC_LPF_FACTOR
#define ACC_LPF_FACTOR 40
#endif

// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

// Do not run this in hot starts.
void IMU::init() {
	// 102us
	gyroADCToRad_s = 1.0 / mpu->gyroToDeg_s() / 180.0 * M_PI; // convert to radians/s
	accComplFilterConstant = (float) DT_FLOAT / (config.accTimeConstant + DT_FLOAT);

	uint8_t axis;

	// Take gravity vector directly from acc. meter
	mpu->getAccelerations(acc);

	for (axis = 0; axis < 3; axis++) {
		estG[axis] = accLPF_f[axis] = acc[axis];
		accLPF_i[axis] = acc[axis];
	}

	accMagnitude_g_100 = 100;
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
// needs angle in radian units !
void IMU::rotateV(float* v, float* delta) {
	float v_tmp[3] = { v[X], v[Y], v[Z] };
	v[Z] -= delta[ROLL] * v_tmp[X] + delta[PITCH] * v_tmp[Y];
	v[X] += delta[ROLL] * v_tmp[Z] - delta[YAW] * v_tmp[Y];
	v[Y] += delta[PITCH] * v_tmp[Z] + delta[YAW] * v_tmp[X];
}

void IMU::readRotationRates() {
	mpu->getRotationRatesAsync(gyro);
}

void IMU::readAccelerations() {
	mpu->getAccelerationsAsync(acc);
}

void IMU::blendGyrosToAttitude() {
	uint8_t axis;
	float deltaGyroAngle[3];
	for (axis = 0; axis < 3; axis++) {
		deltaGyroAngle[axis] = gyro[axis] * gyroADCToRad_s * DT_FLOAT;
	}
	// 168 us
	rotateV(estG, deltaGyroAngle);
}

// Called from slow-loop in main.
// Same units as MPU uses, whatever they are.
/*
void IMU::readAcc(uint8_t axis) {
	// get acceleration
	// 382 us
	uint8_t idx;
	int16_t val;
	idx = sensorDef.Acc[axis].idx;
	val = mpu->getAcceleration(idx); // TODO: 370us
	if (sensorDef.Acc[axis].dir==-1)
		acc[axis] = -val;
	else
		acc[axis] = val;
}
*/

// Called from slow-loop in main.
void IMU::updateAccVector() {
	uint8_t axis;

	// 179 us
	accMagnitude_g_100 = 0;
	for (axis = 0; axis < 3; axis++) {
		accLPF_f[axis] = accLPF_f[axis] * (1.0f - (1.0f / ACC_LPF_FACTOR)) + acc[axis] * (1.0f / ACC_LPF_FACTOR);
		accLPF_i[axis] = accLPF_f[axis];
		accMagnitude_g_100 += (int32_t) accLPF_i[axis] * accLPF_i[axis];
	}

	uint16_t scale = mpu->accToG();

	// accMag = accMag*100/((int32_t)ACC_1G*ACC_1G);
	// 130 us
	// split operation to avoid 32-bit overflow, TODO: no division may happen !!!
	accMagnitude_g_100 = accMagnitude_g_100 / scale;
	accMagnitude_g_100 = accMagnitude_g_100 * 100;
	accMagnitude_g_100 = accMagnitude_g_100 / scale;
}

void IMU::blendAccToAttitude() {
	uint8_t axis;
	// 255 us
	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip filter, as EstV already rotated by Gyro
	if ((36 < accMagnitude_g_100 && accMagnitude_g_100 < 196)) {
		for (axis = 0; axis < 3; axis++) {
			int32_t acc = accLPF_i[axis];
			estG[axis] = estG[axis] * (1.0 - accComplFilterConstant) + acc * accComplFilterConstant; // note: this is different from MultiWii (wrong brackets postion in MultiWii ??.
		}
	}
}

void IMU::calculateAttitudeAngles() {
	// attitude of the estimated vector
	// Here, the traditional meanings of pitch and roll are reversed.
	// That is ultimately okay! In an airframe, it is first pitch then roll.
	// On the typical gimbal frame, it is opposite.
	angle_md[ROLL] = (config.angleOffsetRoll * 10) + Rajan_FastArcTan2_scaled(estG[X], sqrt(estG[Z] * estG[Z] + estG[Y] * estG[Y]));
	angle_md[PITCH] = (config.angleOffsetPitch * 10) + Rajan_FastArcTan2_scaled(estG[Y], estG[Z]);
}

void initPIDs(void) {
	rollPID.setCoefficients(config.rollKp / 10, config.rollKi / 1000, config.rollKd / 10);
	pitchPID.setCoefficients(config.pitchKp / 10, config.pitchKi / 1000, config.pitchKd / 10);
}

