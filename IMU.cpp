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
//#ifndef ACC_LPF_FACTOR
//#define ACC_LPF_FACTOR 40
//#endif

// Do not run this in hot starts.
void IMU::init() {
	// 102us
	gyroADCToRad_s = 1.0 / mpu->gyroToDeg_s() / 180.0 * M_PI; // convert to radians/s
	accComplFilterConstant = DT_FLOAT / ((float)config.accTimeConstant + DT_FLOAT);
	uint8_t axis;

	// Take gravity vector directly from acc. meter
	mpu->getAccelerations(acc);

	for (axis = 0; axis < 3; axis++) {
		estG[axis] = acc[axis];
	}

	uint32_t temp = (uint32_t)mpu->accToG() * mpu->accToG();
	accMagnitude = temp;
	minAccMagnitude = temp * 2/3;
	maxAccMagnitude = temp * 3/2;
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
	rotateV(estG, deltaGyroAngle);
}

// Called from slow-loop in main.
void IMU::updateAccMagnitude() {
	uint8_t axis;

	// 179 us
	accMagnitude = 0;
	for (axis = 0; axis < 3; axis++) {
		//accMagnitude_g_100 += accLPF_f[axis] * accLPF_f[axis];
		accMagnitude += (uint32_t)acc[axis] * acc[axis];
	}
}
void IMU::blendAccToAttitude() {
	uint8_t axis;
	// 255 us
	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip filter, as EstV already rotated by Gyro
	if ((minAccMagnitude < accMagnitude && accMagnitude < maxAccMagnitude)) {
		for (axis = 0; axis < 3; axis++) {
			estG[axis] = estG[axis] * (1.0 - accComplFilterConstant) + acc[axis] * accComplFilterConstant; // note: this is different from MultiWii (wrong brackets postion in MultiWii ??.
		}
	}
}

void IMU::calculateAttitudeAngles() {
	// attitude of the estimated vector
	// Here, the traditional meanings of pitch and roll are reversed.
	// That is ultimately okay! In an airframe, it is first pitch then roll.
	// On the typical gimbal frame, it is opposite.

	angle_cd[ROLL] = config.angleOffsetRoll + Rajan_FastArcTan2_scaled(estG[X], sqrt(estG[Z] * estG[Z] + estG[Y] * estG[Y]));
	angle_cd[PITCH] = config.angleOffsetPitch + Rajan_FastArcTan2_scaled(estG[Y], estG[Z]);
}

void initPIDs(void) {
	rollPID.setCoefficients(config.rollKp, config.rollKi / 100, config.rollKd);
	pitchPID.setCoefficients(config.pitchKp, config.pitchKi / 100, config.pitchKd);
}
