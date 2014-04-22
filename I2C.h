#ifndef __SENSORS_H
#define __SENSORS_H

#include <stdlib.h>
#include <stdint.h>

extern uint8_t i2c_errors_count;

void i2c_init(void);
void waitTransmissionI2C();
void i2c_rep_start(uint8_t address);
void i2c_write(uint8_t data );
void i2c_stop(void);
void i2c_write(uint8_t data );
void i2c_read_regs_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
void i2c_writeRegs(uint8_t add, uint8_t reg, uint8_t* values, uint8_t length);

uint8_t i2c_readReg(uint8_t add, uint8_t reg);
uint8_t i2c_readAck();
uint8_t i2c_readNak();
void i2c_getSixRawADC(uint8_t add, uint8_t reg, uint8_t* buf);

void swap_endianness6(uint8_t *buf);
void swap_endianness2(uint8_t *buf);

uint8_t i2c_readBits(uint8_t add, uint8_t regAddr, uint8_t bitStart, uint8_t length);
void i2c_writeBits(uint8_t add, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);

extern uint8_t i2c_errors_count;

#endif
