#include "MPU6050.h"
#include "I2C.h"

void MPU6050::init() {
	// Start I2C and Configure Frequency
	TWSR = 0; // no prescaler => prescaler = 1
	TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
	TWCR = 1 << TWEN; // enable twi module, no interrupt

	// There should be no reason to do this 2x.
	//setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	//setFullScaleGyroRange(MPU6050_GYRO_FS_250);
	//setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
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

void MPU6050::setFullScaleGyroRange(uint8_t range) {
    i2c_writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050::setFullScaleAccelRange(uint8_t range) {
    i2c_writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU6050::setRate(uint8_t rate) {
   i2c_writeReg(devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}

void MPU6050::setSleepEnabled(bool enabled) {
	i2c_writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1, enabled ? 1 : 0);
}

void MPU6050::getRotationRates(int16_t* result) {
    i2c_read_regs_to_buf(devAddr, MPU6050_RA_GYRO_XOUT_H, buffer, 6);
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
