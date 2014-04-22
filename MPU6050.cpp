#include "MPU6050.h"
#include "I2C.h"
#include "Util.h"
#include "Board.h"
#include "Globals.h"
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

int16_t gyroOffsetInEEPROM[4] EEMEM;

void MPU6050::unjam() {
	// The famous dongfang anti lockup from MultiWii.
	i2c_writeReg(devAddr, 0x6B, 0x80); 	//PWR_MGMT_1    -- DEVICE_RESET 1
	_delay_ms(100);
	i2c_writeReg(devAddr, 0x68, 0x7);   //PWR_MGMT_1    -- RESETS
	_delay_ms(100);
}

void MPU6050::init() {
	setClockSource(MPU6050_CLOCK_PLL_ZGYRO); // Set Clock to ZGyro
	setGyroRange(MPU6050_GYRO_FS); // Set Gyro Sensitivity to config.h
	setAccelRange(MPU6050_ACCEL_FS); //+- 2G
	setRate(0); // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
	setSleepEnabled(false);
	initSensorOrientation();
	_delay_ms(100);
}

/** Verify the I2C connection.
* Make sure the device is connected and responds as expected.
* @return True if connection is valid, false otherwise
*/
bool MPU6050::testConnection() {
	i2c_read_regs(devAddr, MPU6050_RA_WHO_AM_I, 1);
	uint8_t result = selectBits(i2c_buffer[0], MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH);
    return result == 0x34;
}

void MPU6050::setDLPFMode(uint8_t mode) {
	i2c_read_regs(devAddr, MPU6050_RA_CONFIG, 1);
	setBits(&i2c_buffer[0], MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
	i2c_writeReg(devAddr, MPU6050_RA_CONFIG, i2c_buffer[0]);
}

void MPU6050::setClockSource(uint8_t source) {
	i2c_read_regs(devAddr, MPU6050_RA_PWR_MGMT_1, 1);
	setBits(&i2c_buffer[0], MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
	i2c_writeReg(devAddr, MPU6050_RA_PWR_MGMT_1, i2c_buffer[0]);
}

void MPU6050::setGyroRange(uint8_t range) {
	i2c_read_regs(devAddr, MPU6050_RA_GYRO_CONFIG, 1);
	setBits(&i2c_buffer[0], MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
	i2c_writeReg(devAddr, MPU6050_RA_GYRO_CONFIG, i2c_buffer[0]);
}

void MPU6050::setAccelRange(uint8_t range) {
	i2c_read_regs(devAddr, MPU6050_RA_ACCEL_CONFIG, 1);
	setBits(&i2c_buffer[0], MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
	i2c_writeReg(devAddr, MPU6050_RA_ACCEL_CONFIG, i2c_buffer[0]);
}

void MPU6050::setRate(uint8_t rate) {
   i2c_writeReg(devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}

void MPU6050::setSleepEnabled(bool enabled) {
	i2c_read_regs(devAddr, MPU6050_RA_PWR_MGMT_1, 1);
	setBits(&i2c_buffer[0], MPU6050_PWR1_SLEEP_BIT, 1, enabled ? 1 : 0);
	i2c_writeReg(devAddr, MPU6050_RA_PWR_MGMT_1, i2c_buffer[0]);
}


// set default sensor orientation (sensor upside)
void MPU6050::initSensorOrientationFaceUp() {
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
void MPU6050::initSensorOrientationChipTextRightSideUp() {
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
void MPU6050::initSensorOrientationChipTextStandingOnEnd() {
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

void MPU6050::initSensorOrientation() {
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

uint16_t MPU6050::CRC() {
	return ::crc16((uint8_t*)gyroOffset, 6);
}

void MPU6050::saveGyroCalibration() {
	wdt_reset();
	gyroOffset[3] = CRC();
	eeprom_write_block(gyroOffset, gyroOffsetInEEPROM, sizeof(gyroOffset));
}

bool MPU6050::loadGyroCalibration() {
	wdt_reset();
	eeprom_read_block(gyroOffset, gyroOffsetInEEPROM, sizeof(gyroOffset));
	bool ok = (uint16_t)gyroOffset[3] == CRC();
	// printf_P(PSTR("Reused old gyrocal: %d\r\n"), ok);
	return ok;
}

// This functions performs an initial gyro offset calibration
// INCLUDING motion detection
// Board should be still for some seconds
void MPU6050::recalibrateGyros() {
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
			getRotationRates11(gyro);
			for (i = 0; i < 3; i++) {
				gyroOffsetSums[i] = 0;
				prevGyro[i] = gyro[i];
			}
		}

		getRotationRates11(gyro);
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

void MPU6050::transformRotationRates(int16_t* gyro) {
	uint8_t idx;
	uint8_t axis;

	for (axis=0; axis<3; axis++) {
		idx = sensorDef.Gyro[axis].idx;
		gyro[axis] = (((int16_t)i2c_buffer[idx<<1]) << 8) | i2c_buffer[(idx<<1)+1];
		gyro[axis] -= gyroOffset[idx];
		if (sensorDef.Gyro[axis].dir == -1)
			gyro[axis] = -gyro[axis];
	}
}

// Blocking read and identity transform, only used in calibration.
void MPU6050::getRotationRates11(int16_t* result) {
    i2c_read_regs(devAddr, MPU6050_RA_GYRO_XOUT_H, 6);
    result[0] = (((int16_t)i2c_buffer[0]) << 8) | i2c_buffer[1];
    result[1] = (((int16_t)i2c_buffer[2]) << 8) | i2c_buffer[3];
    result[2] = (((int16_t)i2c_buffer[4]) << 8) | i2c_buffer[5];
}

void MPU6050::startRotationRatesAsync() {
	i2c_read_regs_async(devAddr, MPU6050_RA_GYRO_XOUT_H, 6);
}

void MPU6050::getRotationRatesAsync(int16_t* gyro) {
	i2c_wait_async_done();
	transformRotationRates(gyro);
}

void MPU6050::transformAccelerations(int16_t* acc) {
	uint8_t idx;
	uint8_t axis;

	for (axis=0; axis<3; axis++) {
		idx = sensorDef.Acc[axis].idx;
		acc[axis] = (((int16_t)i2c_buffer[idx<<1]) << 8) | i2c_buffer[(idx<<1)+1];
		if (sensorDef.Acc[axis].dir == -1)
			acc[axis] = -acc[axis];
	}
}

// Blocking read, only used in startup.
void MPU6050::getAccelerations(int16_t* acc) {
    i2c_read_regs(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6);
    transformAccelerations(acc);
}

void MPU6050::startAccelerationsAsync() {
	i2c_read_regs_async(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6);
}

void MPU6050::getAccelerationsAsync(int16_t* acc) {
	i2c_wait_async_done();
    transformAccelerations(acc);
}

/*
 * Scrapyard for different recalibrate routines from here and there.
 */
void calibrateGyro() {
	// Gyro Offset calibration
	printf_P(PSTR("Gyro calibration: do not move\r\n"));
	// mpu.setDLPFMode(MPU6050_DLPF_BW_5);  // experimental AHa: set to slow mode during calibration
	mpu.recalibrateGyros();
	// initMPUlpf();
	printf_P(PSTR("Gyro calibration: done\r\n"));
}
