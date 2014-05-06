#ifndef __I2C_H
#define __I2C_H

#include "Util.h"
#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

#define I2C_ASYNC_STARTED_1	0
#define I2C_DEVADDR_SENT_1 	1
#define I2C_REGADDR_SENT 	2
// Stopped? Wait for that?
#define I2C_ASYNC_STARTED_2	3
#define I2C_DEVADR_SENT_2 	4
#define I2C_ASYNC_DATA		5
#define I2C_ASYNC_DONE		6

extern volatile uint8_t i2c_errors_count;
extern volatile uint8_t i2c_async_status;
extern uint8_t i2c_buffer[]; //should really be volatile but too much trouble. I trust the compiler.
extern uint8_t i2c_interrupt_hits[8];

void i2c_init(void);
void i2c_shutdown(void);

void i2c_read_regs(uint8_t add, uint8_t reg, uint8_t size);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);

void i2c_read_regs_async(uint8_t add, uint8_t reg, uint8_t size);

// This will wait till the async handling it completed but not reset the watchdog timer.
// If it takes too long (glitch), bang, watchdog reboot.
void i2c_wait_async_done();

// This just ignores the timeout and gets on with it.
inline bool i2c_is_async_done() {
	bool result = (i2c_async_status == I2C_ASYNC_DONE);
	if (result) return true;
	LEDEvent(I2C_TIMEOUT_MASK);
	_delay_us(5);
	result = (i2c_async_status == I2C_ASYNC_DONE);
	if (!result) i2c_errors_count++;
	return result;
}

#endif
