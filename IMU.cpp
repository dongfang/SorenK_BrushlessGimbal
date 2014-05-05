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
	gyroADCToRad_dt = FASTLOOP_DT_F_S * M_PI / mpu->gyroToDeg_s() / 180.0;
	accComplFilterConstant = (float) MEDIUMLOOP_DT_F_S / (config.accTimeConstant + MEDIUMLOOP_DT_F_S);
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
		deltaGyroAngle[axis] = gyro[axis] * gyroADCToRad_dt;
	}
	rotateV(estG, deltaGyroAngle);
}

// Called from slow-loop in main.
void IMU::updateAccMagnitude() {
	uint8_t axis;

	// 179 us
	uint32_t tmp_accMagnitude = 0;
	for (axis = 0; axis < 3; axis++) {
		//accMagnitude_g_100 += accLPF_f[axis] * accLPF_f[axis];
		tmp_accMagnitude += (uint32_t)acc[axis] * acc[axis];
	}
	cli();
	accMagnitude = tmp_accMagnitude;
	sei();
}
void IMU::blendAccToAttitude() {
	uint8_t axis;
	// 255 us
	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip filter, as EstV already rotated by Gyro
	if ((minAccMagnitude < accMagnitude && accMagnitude < maxAccMagnitude)) {
		for (axis = 0; axis < 3; axis++) {
			cli();
			int16_t _acc = acc[axis];
			float _eg = estG[axis];
			sei();
			_eg = _eg * (1.0 - accComplFilterConstant) + _acc * accComplFilterConstant; // note: this is different from MultiWii (wrong brackets postion in MultiWii ??.
			cli();
			estG[axis] = _eg;
			sei();
		}
	}
}

void IMU::calculateAttitudeAngles() {
	// attitude of the estimated vector
	// Here, the traditional meanings of pitch and roll are reversed.
	// That is ultimately okay! In an airframe, it is first pitch then roll.
	// On the typical gimbal frame, it is opposite.
	int16_t resultRoll, resultPitch;

	cli();
	float x = estG[X]; float y = estG[Y]; float z = estG[Z];
	sei();

	float hypo = sqrt(y*y + z*z);
	resultRoll = ANGLE_SCALING * Rajan_FastArcTan2(x, hypo);
	resultPitch = ANGLE_SCALING * Rajan_FastArcTan2(y, z);

	// Assume the G vector is normal (meaning it has length mpu->accToG()
	cli();
	angle_i16[ROLL] = resultRoll;
	angle_i16[PITCH] = resultPitch;
	sei();

	//sinPitch = ((int16_t)y) >> (mpu->logAccToG()-LOG_SIN_RES);
	//cosPitch = ((int16_t)z) >> (mpu->logAccToG()-LOG_SIN_RES);

	sinPitch = (1<<LOG_SIN_RES) * y/hypo;
	cosPitch = (1<<LOG_SIN_RES) * z/hypo;
}
