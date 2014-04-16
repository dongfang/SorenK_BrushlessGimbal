#include "MPU6050.h"

void MPU6050::init() {
	// Start I2C and Configure Frequency
	TWSR = 0; // no prescaler => prescaler = 1
	TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
	TWCR = 1 << TWEN; // enable twi module, no interrupt
}
