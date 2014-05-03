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

	// This seems not to have any fukkin effect at all! I read a zero back.
	setGyroRange(MPU6050_GYRO_FS); // Set Gyro Sensitivity to config.h

	setAccelRange(MPU6050_ACCEL_FS); //+- 2G
	setRate(7); // 0=8kHz, 1=4kHz, 2=2.66Hz, 3=2kHz, 4=1.6kHz, 5=1.33kHz, 6=1.14Hz, 7=1kHz
	setSleepEnabled(false);
	//_delay_ms(100);
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

/*
void MPU6050::setDLPFMode(uint8_t mode) {
	i2c_read_regs(devAddr, MPU6050_RA_CONFIG, 1);
	setBits(&i2c_buffer[0], MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
	i2c_writeReg(devAddr, MPU6050_RA_CONFIG, i2c_buffer[0]);
}
*/

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

// this is useless for IMU but it will show 1:1 what comes out of the MPU chip with the debug acc / debug gyro commands.
void MPU6050::initSensorOrientationRaw() {
	sensorDef.gyro[ROLL].idx = 0;
	sensorDef.gyro[PITCH].idx = 1;
	sensorDef.gyro[YAW].idx = 2;

	sensorDef.acc[X].idx = 0;
	sensorDef.acc[Y].idx = 1;
	sensorDef.acc[Z].idx = 2;

	// direction
	sensorDef.gyro[ROLL].dir = 1;
	sensorDef.gyro[PITCH].dir = 1;
	sensorDef.gyro[YAW].dir = 1;

	sensorDef.acc[X].dir = 1;
	sensorDef.acc[Y].dir = 1;
	sensorDef.acc[Z].dir = 1;
}

void MPU6050::initSensorOrientationFaceUp() {
	sensorDef.gyro[ROLL].idx = 1;
	sensorDef.gyro[PITCH].idx = 0;
	sensorDef.gyro[YAW].idx = 2;

	sensorDef.acc[X].idx = 0;
	sensorDef.acc[Y].idx = 1;
	sensorDef.acc[Z].idx = 2;

	// direction
	sensorDef.gyro[ROLL].dir = 1;
	sensorDef.gyro[PITCH].dir = 1;
	sensorDef.gyro[YAW].dir = 1;

	sensorDef.acc[X].dir = -1;
	sensorDef.acc[Y].dir = 1;
	sensorDef.acc[Z].dir = 1;
}

void MPU6050::initSensorOrientationChipTextRightSideUp() {
	sensorDef.gyro[ROLL].idx = 0;
	sensorDef.gyro[PITCH].idx = 2;
	sensorDef.gyro[YAW].idx = 1;

	sensorDef.acc[ROLL].idx = 2;
	sensorDef.acc[PITCH].idx = 0;
	sensorDef.acc[YAW].idx = 1;

	// direction
	sensorDef.gyro[ROLL].dir = 1;
	sensorDef.gyro[PITCH].dir = -1;
	sensorDef.gyro[YAW].dir = -1;

	sensorDef.acc[ROLL].dir = 1;
	sensorDef.acc[PITCH].dir = 1;
	sensorDef.acc[YAW].dir = -1;
}

void MPU6050::initSensorOrientationChipTextStandingOnEnd() {
	// channel assignment
	sensorDef.gyro[ROLL].idx = 1;
	sensorDef.gyro[PITCH].idx = 2;
	sensorDef.gyro[YAW].idx = 0;

	sensorDef.acc[ROLL].idx = 2;
	sensorDef.acc[PITCH].idx = 1;
	sensorDef.acc[YAW].idx = 0;

	// direction
	sensorDef.gyro[ROLL].dir = 1;
	sensorDef.gyro[PITCH].dir = -1;
	sensorDef.gyro[YAW].dir = 1;

	sensorDef.acc[ROLL].dir = 1;
	sensorDef.acc[PITCH].dir = 1;
	sensorDef.acc[YAW].dir = 1;
}

// set sensor orientation according config (TODO: separate config from class. Not important)
//
//   config.axisReverseZ
//        false ... sensor mounted on top
//        true  ... sensor mounted upside down
//   config.axisSwapXY
//        false ... default XY axes
//        true  ... swap XY (means exchange Roll/Pitch)

void MPU6050::initSensorOrientation(uint8_t majorAxis, bool reverseZ, bool swapXY) {
	switch (majorAxis) {
	case 0:
		initSensorOrientationFaceUp();
		break;
	case 1:
		initSensorOrientationChipTextRightSideUp();
		break;
	case 2:
		initSensorOrientationChipTextStandingOnEnd();
		break;
	case 3:
		initSensorOrientationRaw();
		break;
	}

	if (reverseZ) {
		// flip over roll
		sensorDef.acc[Z].dir *= -1;
		sensorDef.acc[X].dir *= -1;
		sensorDef.gyro[PITCH].dir *= -1;
		sensorDef.gyro[YAW].dir *= -1;
	}

	if (swapXY) {
		// swap gyro axis
		swap_uint8(&sensorDef.gyro[ROLL].idx, &sensorDef.gyro[PITCH].idx);
		swap_int(&sensorDef.gyro[ROLL].dir, &sensorDef.gyro[PITCH].dir);
		sensorDef.gyro[PITCH].dir *= -1; // try and error ;-)
		// swap acc axis
		swap_uint8(&sensorDef.acc[X].idx, &sensorDef.acc[Y].idx);
		swap_int(&sensorDef.acc[X].dir, &sensorDef.acc[Y].dir);
		sensorDef.acc[X].dir *= -1;
	}
}

uint16_t MPU6050::CRC() {
	return ::crc16((uint8_t*)sensorOffset, 12);
}

void MPU6050::saveSensorCalibration() {
	wdt_reset();
	sensorOffset[6] = CRC();
	eeprom_write_block(sensorOffset, gyroOffsetInEEPROM, sizeof(sensorOffset));
}

bool MPU6050::loadGyroCalibration() {
	wdt_reset();
	eeprom_read_block(sensorOffset, gyroOffsetInEEPROM, sizeof(sensorOffset));
	bool ok = (uint16_t)sensorOffset[6] == CRC();
	// printf_P(PSTR("Reused old gyrocal: %d\r\n"), ok);
	return ok;
}

// This functions performs an initial gyro offset calibration
// INCLUDING motion detection
// Board should be still for some seconds
void MPU6050::recalibrateSensor(void (*complain)(), uint8_t which) {
	int i;
#define TOL 64
#define SENSOR_ITERATIONS 2000
	int16_t prevSensor[3], sensor[3];
	int32_t sensorOffsetSums[3];
	int calibGCounter = SENSOR_ITERATIONS;

	// wait 1 second
	for (i = 0; i < 100; i++) {
		wdt_reset();
		_delay_ms(10);
	}

	while (calibGCounter > 0) {
		bool motionDetected = false;
		wdt_reset();
		if (calibGCounter == SENSOR_ITERATIONS) {
			for (i = 0; i < 70; i++) { // wait 0.7sec if calibration failed
				_delay_ms(10);
			}
			getSensor(sensor, which);
			for (i = 0; i < 3; i++) {
				sensorOffsetSums[i] = 0;
				prevSensor[i] = sensor[i];
			}
		}

		getSensor(sensor, which);
		wdt_reset();
		for (i = 0; i < 3; i++) {
			if (abs16(prevSensor[i] - sensor[i]) > TOL) {
				motionDetected = true;
				break;
			}
		}

		for (i = 0; i < 3; i++) {
			wdt_reset();
			sensorOffsetSums[i] += sensor[i];
			prevSensor[i] = sensor[i];
		}

		calibGCounter--;
		if (motionDetected) {
			complain();
			calibGCounter = SENSOR_ITERATIONS;
		}
	}

	// put result into integer
	for (i = 0; i < 3; i++) {
		sensorOffset[i] = (sensorOffsetSums[i] + SENSOR_ITERATIONS/2) / SENSOR_ITERATIONS;
	}
	if (which == ACC)
		sensorOffset[Z] = 0; // We don't want to calibrate Z accel away.

	saveSensorCalibration();
}

void MPU6050::transformRotationRates(int16_t* gyro) {
	uint8_t idx;
	uint8_t axis;

	for (axis=0; axis<3; axis++) {
		idx = sensorDef.gyro[axis].idx;
		gyro[axis] = (((int16_t)i2c_buffer[idx<<1]) << 8) | i2c_buffer[(idx<<1)+1];
		gyro[axis] -= sensorOffset[idx];
		if (sensorDef.gyro[axis].dir == -1)
			gyro[axis] = -gyro[axis];
	}
}

// Blocking read and identity transform, only used in calibration.
void MPU6050::getSensor(int16_t* result, uint8_t which) {
    i2c_read_regs(devAddr, (which == GYRO) ? MPU6050_RA_GYRO_XOUT_H : MPU6050_RA_ACCEL_XOUT_H, 6);
    result[0] = (((int16_t)i2c_buffer[0]) << 8) | i2c_buffer[1];
    result[1] = (((int16_t)i2c_buffer[2]) << 8) | i2c_buffer[3];
    result[2] = (((int16_t)i2c_buffer[4]) << 8) | i2c_buffer[5];
}

void MPU6050::startRotationRatesAsync() {
	i2c_read_regs_async(devAddr, MPU6050_RA_GYRO_XOUT_H, 6);
}

uint8_t debug_measureing_what ;

void MPU6050::getRotationRatesAsync(int16_t* gyro) {
	debug_measureing_what = 0;
	i2c_wait_async_done();
	transformRotationRates(gyro);
}

void MPU6050::transformAccelerations(int16_t* acc) {
	uint8_t idx;
	uint8_t axis;

	for (axis=0; axis<3; axis++) {
		idx = sensorDef.acc[axis].idx;
		acc[axis] = (((int16_t)i2c_buffer[idx<<1]) << 8) | i2c_buffer[(idx<<1)+1];
		acc[axis] -= sensorOffset[idx+3];
		if (sensorDef.acc[axis].dir == -1)
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
	debug_measureing_what = 1;
	i2c_wait_async_done();
    transformAccelerations(acc);
}
