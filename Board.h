#ifndef __BOARD_H
#define __BOARD_H

#define LED_DDR			DDRB
#define LED_PORT		PORTB
#define LED_PIN			PINB

// Hextronik board : 5
#define LED_BIT			5

// RCTimer board: 0
// #define LED_BIT			0

// I2C Frequency
//#define I2C_SPEED 100000L     //100kHz normal mode
#define I2C_SPEED 400000L   //400kHz fast mode
//#define I2C_SPEED 800000L   //800kHz ultra fast mode

#endif
