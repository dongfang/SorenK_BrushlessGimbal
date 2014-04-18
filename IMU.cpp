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
#include "Configuration.h"
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <math.h>

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
#define ACC_LPF_FACTOR 40
#endif

// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

int16_t gyroOffsetInEEPROM[4] EEMEM;

// set default sensor orientation (sensor upside)
void IMU::initSensorOrientationFaceUp() {
	// This one may not work. not tested in this version!
	// channel assignment
	sensorDef.Gyro[ROLL].idx = 0;
	sensorDef.Gyro[PITCH].idx = 1;
	sensorDef.Gyro[YAW].idx = 2;

	sensorDef.Acc[ROLL].idx = 1; // y
	sensorDef.Acc[PITCH].idx = 0; // x
	sensorDef.Acc[YAW].idx = 2; // z

	// direction
	sensorDef.Gyro[ROLL].dir = 1;
	sensorDef.Gyro[PITCH].dir = -1;
	sensorDef.Gyro[YAW].dir = 1;

	sensorDef.Acc[ROLL].dir = 1;
	sensorDef.Acc[PITCH].dir = 1;
	sensorDef.Acc[YAW].dir = 1;
}

// set sensor orientation (chip end up)
void IMU::initSensorOrientationChipTextRightSideUp() {
	// channel assignment
	sensorDef.Gyro[ROLL].idx = 0;
	sensorDef.Gyro[PITCH].idx = 2;
	sensorDef.Gyro[YAW].idx = 1;

	sensorDef.Acc[ROLL].idx = 1;
	sensorDef.Acc[PITCH].idx = 2;
	sensorDef.Acc[YAW].idx = 0;

	// direction
	sensorDef.Gyro[ROLL].dir = 1;
	sensorDef.Gyro[PITCH].dir = -1;
	sensorDef.Gyro[YAW].dir = -1;

	sensorDef.Acc[ROLL].dir = 1;
	sensorDef.Acc[PITCH].dir = 1;
	sensorDef.Acc[YAW].dir = -1;
}

// set sensor orientation (chip side up)
void IMU::initSensorOrientationChipTextStandingOnEnd() {
	// channel assignment
	sensorDef.Gyro[ROLL].idx = 1;
	sensorDef.Gyro[PITCH].idx = 2;
	sensorDef.Gyro[YAW].idx = 0;

	sensorDef.Acc[ROLL].idx = 2;
	sensorDef.Acc[PITCH].idx = 1;
	sensorDef.Acc[YAW].idx = 0;

	// direction
	sensorDef.Gyro[ROLL].dir = 1;
	sensorDef.Gyro[PITCH].dir = -1;
	sensorDef.Gyro[YAW].dir = 1;

	sensorDef.Acc[ROLL].dir = 1;
	sensorDef.Acc[PITCH].dir = 1;
	sensorDef.Acc[YAW].dir = 1;
}

// set sensor orientation according config (TODO: separate config from class. Not important)
//
//   config.axisReverseZ
//        false ... sensor mounted on top
//        true  ... sensor mounted upside down
//   config.axisSwapXY
//        false ... default XY axes
//        true  ... swap XY (means exchange Roll/Pitch)

void IMU::initSensorOrientation() {
	switch (config.majorAxis) {
	case 1:
		initSensorOrientationChipTextRightSideUp();
		break;
	case 2:
		initSensorOrientationChipTextStandingOnEnd();
		break;
	default:
		initSensorOrientationFaceUp();
		break;
	}

	if (config.axisReverseZ) {
		// flip over roll
		sensorDef.Acc[YAW].dir *= -1;
		sensorDef.Acc[ROLL].dir *= -1;
		sensorDef.Gyro[PITCH].dir *= -1;
		sensorDef.Gyro[YAW].dir *= -1;
	}

	if (config.axisSwapXY) {
		// swap gyro axis
		swap_char(&sensorDef.Gyro[ROLL].idx, &sensorDef.Gyro[PITCH].idx);
		swap_int(&sensorDef.Gyro[ROLL].dir, &sensorDef.Gyro[PITCH].dir);
		sensorDef.Gyro[PITCH].dir *= -1; // try and error ;-)
		// swap acc axis
		swap_char(&sensorDef.Acc[ROLL].idx, &sensorDef.Acc[PITCH].idx);
		swap_int(&sensorDef.Acc[ROLL].dir, &sensorDef.Acc[PITCH].dir);
		sensorDef.Acc[ROLL].dir *= -1;
	}
}

// This functions performs an initial gyro offset calibration
// INCLUDING motion detection
// Board should be still for some seconds
void IMU::recalibrateGyros() {
	int i;
#define TOL 64
#define GYRO_ITERATIONS 2000
	int16_t prevGyro[3], gyro[3];
	int32_t gyroOffsetSums[3];
	bool tiltDetected = false;
	int calibGCounter = GYRO_ITERATIONS;

	// wait 1 second
	for (i = 0; i < 100; i++) {
		wdt_reset();
		_delay_ms(10);
	}

	while (calibGCounter > 0) {
		wdt_reset();
		if (calibGCounter == GYRO_ITERATIONS) {
			for (i = 0; i < 70; i++) { // wait 0.7sec if calibration failed
				_delay_ms(10);
			}
			mpu.getRotationRates(gyro);
			for (i = 0; i < 3; i++) {
				gyroOffsetSums[i] = 0;
				prevGyro[i] = gyro[i];
			}
		}

		mpu.getRotationRates(gyro);

		wdt_reset();

		for (i = 0; i < 3; i++) {
			if (abs16(prevGyro[i] - gyro[i]) > TOL) {
				tiltDetected = true;
				//Serial.print(F(" i="));Serial.print(i);
				//Serial.print(F(" calibGCounter="));Serial.print(calibGCounter);
				//Serial.print(F(" diff="));Serial.print(prevGyro[i] - gyro[i]);
				//Serial.print(F(" gyroi="));Serial.print(gyro[i]);
				//Serial.print(F(" prevgyroi="));Serial.println(prevGyro[i]);
				break;
			}
		}

		for (i = 0; i < 3; i++) {
			wdt_reset();
			gyroOffsetSums[i] += gyro[i];
			prevGyro[i] = gyro[i];
		}

		calibGCounter--;
		if (tiltDetected) {
			printf_P(PSTR("Motion detected during Gyro calibration. Starting over!"));
			calibGCounter = GYRO_ITERATIONS;
			tiltDetected = false;
		}
	}

	// put result into integer
	for (i = 0; i < 3; i++) {
		gyroOffset[i] = (gyroOffsetSums[i] + GYRO_ITERATIONS/2) / GYRO_ITERATIONS;
		//Serial.print(F("gyroOffset="));Serial.println(fp_gyroOffset[i], 3);
	}

	saveGyroCalibration();
}

// Do not run this in hot starts.
void IMU::init() {
	// 102us
	initSensorOrientation();

	gyroScale = 1.0 / mpu.gyroToDeg_s() / 180.0 * 3.14159265359; // convert to radians/s
	accComplFilterConstant = (float) DT_FLOAT / (config.accTimeConstant + DT_FLOAT);

	uint8_t axis;

	// Take gravity vector directly from acc. meter
	for (axis = 0; axis < 3; axis++) {
		readAcc(axis);
		EstG[axis] = accLPF_f[axis] = acc[axis];
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

void IMU::readGyros() {
	int16_t axisRot[3];
	uint8_t idx;
	// 414 us

	// read gyros
	mpu.getRotationRates(axisRot);
	idx = sensorDef.Gyro[0].idx;
	gyros[ROLL] = axisRot[idx] - gyroOffset[idx];
	gyros[ROLL] *= sensorDef.Gyro[0].dir;

	idx = sensorDef.Gyro[1].idx;
	gyros[PITCH] = axisRot[idx] - gyroOffset[idx];
	gyros[PITCH] *= sensorDef.Gyro[1].dir;

	idx = sensorDef.Gyro[2].idx;
	gyros[YAW] = axisRot[idx] - gyroOffset[idx];
	gyros[YAW] *= sensorDef.Gyro[2].dir;
}

void IMU::blendGyrosToAttitude() {
	uint8_t axis;
	float deltaGyroAngle[3];
	for (axis = 0; axis < 3; axis++) {
		deltaGyroAngle[axis] = gyros[axis] * gyroScale * DT_FLOAT;
	}
	// 168 us
	rotateV(EstG, deltaGyroAngle);
}

// Called from slow-loop in main.
// Same units as MPU uses, whatever they are.
void IMU::readAcc(uint8_t axis) {
	// get acceleration
	// 382 us
	uint8_t idx;
	int16_t val;
	idx = sensorDef.Acc[axis].idx;
	val = mpu.getAcceleration(idx); // TODO: 370us
	val *= sensorDef.Acc[axis].dir;
	acc[axis] = val;
}

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

	uint16_t scale = mpu.accToG();

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
			EstG[axis] = EstG[axis] * (1.0 - AccComplFilterConst) + acc * AccComplFilterConst; // note: this is different from MultiWii (wrong brackets postion in MultiWii ??.
		}
	}
}

void IMU::calculateAttitudeAngles() {
	// attitude of the estimated vector
	// 272 us
	// Here, the traditional meanings of pitch and roll are reversed.
	// That is ultimately okay! In an airframe, it is first pitch then roll.
	// On the typical gimbal frame, it is opposite.
	angle[ROLL] = (config.angleOffsetRoll * 10)
			+ Rajan_FastArcTan2_deg1000(EstG[X], sqrt(EstG[Z] * EstG[Z] + EstG[Y] * EstG[Y]));
	// 192 us
	angle[PITCH] = (config.angleOffsetPitch * 10) + Rajan_FastArcTan2_deg1000(EstG[Y], EstG[Z]);
}


/*
 * Scrapyard for different recalibrate routines from here and there.
 */
void calibrateGyro() {
	// Gyro Offset calibration
	printf_P(PSTR("Gyro calibration: do not move\r\n"));
	// mpu.setDLPFMode(MPU6050_DLPF_BW_5);  // experimental AHa: set to slow mode during calibration
	imu.recalibrateGyros();
	// initMPUlpf();
	printf_P(PSTR("Gyro calibration: done\r\n"));
}
/*
void gyroRecalibrate() {
	// Set voltage on all motor phases to zero
	// Nah why not freeze it instead?
	softStart = 0;
	mpu.setDLPFMode(MPU6050_DLPF_BW_5); // experimental AHa: set to slow mode during calibration
	imu.recalibrateGyros();
	initMPUlpf();
	printf_P(PSTR("recalibration: done"));
}
*/

void initPIDs(void) {
	rollPID.setCoefficients(config.rollKp / 10, config.rollKi / 1000, config.rollKd / 10);
	pitchPID.setCoefficients(config.pitchKp / 10, config.pitchKi / 1000, config.pitchKd / 10);
}

void initMPUlpf() {
	// Set Gyro Low Pass Filter(0..6, 0=fastest, 6=slowest)
	switch (config.mpuLPF) {
	case 0:
		mpu.setDLPFMode(MPU6050_DLPF_BW_256);
		break;
	case 1:
		mpu.setDLPFMode(MPU6050_DLPF_BW_188);
		break;
	case 2:
		mpu.setDLPFMode(MPU6050_DLPF_BW_98);
		break;
	case 3:
		mpu.setDLPFMode(MPU6050_DLPF_BW_42);
		break;
	case 4:
		mpu.setDLPFMode(MPU6050_DLPF_BW_20);
		break;
	case 5:
		mpu.setDLPFMode(MPU6050_DLPF_BW_10);
		break;
	case 6:
		mpu.setDLPFMode(MPU6050_DLPF_BW_5);
		break;
	default:
		mpu.setDLPFMode(MPU6050_DLPF_BW_256);
		break;
	}
}

uint16_t IMU::CRC() {
	return ::crc16((uint8_t*)gyroOffset, 6);
}

void IMU::saveGyroCalibration() {
	wdt_reset();
	gyroOffset[3] = CRC();
	eeprom_write_block(gyroOffset, gyroOffsetInEEPROM, sizeof(gyroOffset));
}

bool IMU::loadGyroCalibration() {
	wdt_reset();
	eeprom_read_block(gyroOffset, gyroOffsetInEEPROM, sizeof(gyroOffset));
	bool ok = (uint16_t)gyroOffset[3] == CRC();
	printf_P(PSTR("Reused old gyrocal: %d"), ok);
	return ok;
}
