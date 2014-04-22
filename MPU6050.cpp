#include "MPU6050.h"
#include "I2C.h"
#include <util/delay.h>

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
	_delay_ms(100);
}

/** Verify the I2C connection.
* Make sure the device is connected and responds as expected.
* @return True if connection is valid, false otherwise
*/
bool MPU6050::testConnection() {
	uint8_t result = i2c_readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH);
    return result == 0x34;
}

void MPU6050::setDLPFMode(uint8_t mode) {
	i2c_writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void MPU6050::setClockSource(uint8_t source) {
	i2c_writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050::setGyroRange(uint8_t range) {
    i2c_writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050::setAccelRange(uint8_t range) {
    i2c_writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU6050::setRate(uint8_t rate) {
   i2c_writeReg(devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}

void MPU6050::setSleepEnabled(bool enabled) {
	i2c_writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1, enabled ? 1 : 0);
}

void MPU6050::getRotationRates(int16_t* result) {
    i2c_read_regs_to_buf(devAddr, MPU6050_RA_GYRO_XOUT_H,(uint8_t*) buffer, 6);
    result[0] = (((int16_t)buffer[0]) << 8) | buffer[1];
    result[1] = (((int16_t)buffer[2]) << 8) | buffer[3];
    result[2] = (((int16_t)buffer[4]) << 8) | buffer[5];
}

int16_t MPU6050::getAcceleration(uint8_t axis) {
    uint8_t idxa = axis<<1;
    uint8_t idxb = idxa+1;
    i2c_read_regs_to_buf(devAddr, MPU6050_RA_ACCEL_XOUT_H, buffer, 6);
    return (((int16_t)buffer[idxa]) << 8) | buffer[idxb];
}
