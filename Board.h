#ifndef __BOARD_H
#define __BOARD_H

#define LED_DDR			DDRB
#define LED_PORT		PORTB
#define LED_PIN			PINB
// Hextronik board : 5
#define LED_BIT			5
// RCTimer board: 0
// #define LED_BIT			0


// Undef this to avoid the fast and medium timing pulses on outputs.
// #define DEBUG_SIGNALS
#define DEBUG_DDR		DDRD
#define DEBUG_PORT		PORTD
#define DEBUG_PIN		PIND
#define DEBUG_BIT1		4
#define DEBUG_BIT2		7


// By Martinez board (V3?)
// A0 C0
// A1 C1
// A2 C2
// C1 D2
// C3 D4
// C4 B4

#define USE_YAWSERVO
// This conflicts with debug-out so only use one at a time.
#define YAWSERVO_DDR	DDRD
#define YAWSERVO_PORT	PORTD
#define YAWSERVO_BIT	4

// I2C Frequency
#define I2C_SPEED 600000L   // slightly tune the bus.

#endif
