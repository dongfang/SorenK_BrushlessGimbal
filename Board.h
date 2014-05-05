#ifndef __BOARD_H
#define __BOARD_H

#define LED_DDR			DDRB
#define LED_PORT		PORTB
#define LED_PIN			PINB

#define DEBUG_DDR		DDRD
#define DEBUG_PORT		PORTD
#define DEBUG_PIN		PIND

// Hextronik board : 5
#define LED_BIT			5

#define DEBUG_BIT1		4
#define DEBUG_BIT2		7

// RCTimer board: 0
// #define LED_BIT			0

// I2C Frequency
//#define I2C_SPEED 400000L   //400kHz fast mode
#define I2C_SPEED 500000L   // slightly tune the bus.

#endif
