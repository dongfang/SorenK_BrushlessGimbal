#ifndef __BOARD_H
#define __BOARD_H

#define LED_DDR			DDRB
#define LED_PORT		PORTB
#define LED_PIN			PINB
// Hextronik board : 	5
#define LED_BIT			5
// RCTimer board: 		0
// #define LED_BIT		0

#define SERVO_DDR		DDRD
#define SERVO_PORT		PORTD

#define REMOTE_SERVO_DDR		DDRD
#define REMOTE_SERVO_PORT		PORTD
#define REMOTE_SERVO_PIN		PIND

#define REMOTE_SERVO_CLKBIT		2
#define REMOTE_SERVO_DATABIT	4

// This conflicts with debug-out so only use one at a time.
#define SUPPORT_YAW_SERVO
#define YAW_REPLACES_RC_ROLL
// #define YAW_SERVOOUT_LOCAL
#define YAW_SERVOOUT_REMOTE
#define YAW_SERVO_BIT	4

#define SUPPORT_RETRACT
// #define RETRACT_SERVOOUT_LOCAL
#define RETRACT_SERVOOUT_REMOTE
#define RETRACT_SERVO_BIT	7

#if !defined(SUPPORT_YAW_SERVO) && !defined(SUPPORT_RETRACT)
// Undef this to avoid the fast and medium timing pulses on outputs.
#define DEBUG_SIGNALS
#define DEBUG_DDR		DDRD
#define DEBUG_PORT		PORTD
#define DEBUG_PIN		PIND
#define DEBUG_BIT1		4
#define DEBUG_BIT2		7
#endif

// By Martinez board (V3?)
// A0 C0
// A1 C1
// A2 C2
// C1 D2
// C3 D4
// C4 B4


// I2C Frequency
#define I2C_SPEED 600000L   // slightly tune the bus.

#endif
