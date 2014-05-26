#include "Board.h"
#include "Definitions.h"
#include "Globals.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>

extern uint8_t timer1Extension;
extern uint16_t timer1ExtensionExtension;

#define IDLE 0
#define RUNNING 1

volatile uint8_t gimbalState;

volatile bool mediumTaskHasRun;

#if defined (SUPPORT_YAW_SERVO)
#if defined (YAW_SERVOOUT_LOCAL)
volatile uint8_t yawServoPulseLength = 46;
void setYawServoOut(uint16_t usec) {
	cli();
	yawServoPulseLength = usec/32 - 1; // An approximation
	sei();
}
#endif
#if defined (YAW_SERVOOUT_REMOTE)
volatile uint16_t yawServoUsec = -1;
void setYawServoOut(uint16_t usec) {
	cli();
	yawServoUsec = usec;
	sei();
}
#endif
#endif

#if defined (SUPPORT_RETRACT)
#if defined (RETRACT_SERVOOUT_LOCAL)
volatile uint8_t retractServoPulseLength = 46;
void setRetractServoOut(uint16_t usec) {
	cli();
	retractServoPulseLength = usec/32 - 1; // An approximation
	sei();
}
#endif
#if defined (RETRACT_SERVOOUT_REMOTE)
volatile uint16_t retractServoUsec = -1;
void setRetractServoOut(uint16_t usec) {
	cli();
	retractServoUsec = usec;
	sei();
}
#endif
#endif

void fastTask();
void mediumTask();

#define FAST_PERIOD (F_CPU/510/FASTLOOP_FREQ)
#define MEDIUM_SUBPERIOD ((F_CPU/510/MEDIUMLOOP_FREQ) / FAST_PERIOD)

/********************************/
/* Motor Control IRQ Routine    */
/********************************/
// Motor control.
// Should trigger at 31.25 kHz. More than often enough that we really can output the PWM values in sync with the overflow.
ISR (TIMER1_OVF_vect) {
	static uint8_t fastState = IDLE;
	static uint8_t mediumState = IDLE;
	static uint8_t fastDivider = FAST_PERIOD;
	static uint8_t mediumDivider = MEDIUM_SUBPERIOD;

#if defined (SUPPORT_RETRACT) && defined (RETRACT_SERVOOUT_LOCAL)
	static uint8_t retractServoDivider;
#endif
#if defined (SUPPORT_YAW_SERVO) && defined (YAW_SERVOOUT_LOCAL)
	static uint8_t yawServoDivider;
#endif
#if (defined (SUPPORT_RETRACT) && defined (RETRACT_SERVOOUT_LOCAL) || defined (SUPPORT_YAW_SERVO) && defined (YAW_SERVOOUT_LOCAL))
	static uint16_t servoPauseDivider;
#endif

#if defined (SUPPORT_YAW_SERVO) && defined (YAW_SERVOOUT_REMOTE) || defined (SUPPORT_RETRACT) && defined (RETRACT_SERVOOUT_REMOTE)
	static uint8_t remoteServoOutState = 0;
	static uint16_t remoteServoOutData = 0;
#endif

	sei();

	fastDivider--; // decrements at F_CPU/256 so after time T it is T*F_CPU/256

	if (!fastDivider) {
		if (fastState == IDLE) {
			//cli();
			fastState = RUNNING;
#ifdef DEBUG_SIGNALS
			DEBUG_PORT |= 1 << DEBUG_BIT1;
#endif
			fastDivider = FAST_PERIOD;
			//sei();
			fastTask();
			fastState = IDLE;
			// TODO: Maybe move to fast-loop itself.
#ifdef DEBUG_SIGNALS
			DEBUG_PORT &= ~(1 << DEBUG_BIT1);
#endif

#if defined (SUPPORT_YAW_SERVO) && defined (YAW_SERVOOUT_REMOTE) || defined (SUPPORT_RETRACT) && defined (RETRACT_SERVOOUT_REMOTE)
	if (remoteServoOutState == 0) {
		remoteServoOutData = yawServoUsec;
	}
	// Max. usec value is 12 bits large so 12 is what we need.
	else if (remoteServoOutState == 24) {
		remoteServoOutData = retractServoUsec;
	}

	if (remoteServoOutState >= 48) {
		// Sync pause. Do nothing.
		if (remoteServoOutState == 96) remoteServoOutState = -1;
	}

	else if (remoteServoOutState & 1) {
		// clk hi
		REMOTE_SERVO_PORT |= 1<<REMOTE_SERVO_CLKBIT;
	} else {
		//clk lo
		REMOTE_SERVO_PORT &= ~(1<<REMOTE_SERVO_CLKBIT);
		if (remoteServoOutData & 1)
			REMOTE_SERVO_PORT |= 1<<REMOTE_SERVO_DATABIT;
		else
			REMOTE_SERVO_PORT &= ~(1<<REMOTE_SERVO_DATABIT);
		remoteServoOutData >>= 1;
	}

	remoteServoOutState++;
#endif

			if (!--mediumDivider) {
				if (mediumState == IDLE) {
					cli();
#ifdef DEBUG_SIGNALS
					DEBUG_PORT |= 1 << DEBUG_BIT2;
#endif
					mediumState = RUNNING;
					mediumDivider = MEDIUM_SUBPERIOD;
					sei();
					mediumTask();
					mediumTaskHasRun = true; // To assist the slow-task stuff to sync
					mediumState = IDLE;
#ifdef DEBUG_SIGNALS
					DEBUG_PORT &= ~(1 << DEBUG_BIT2);
#endif
				} else {
					// Medium task collision has occured.
					LEDEvent(LED_SCHEDULER_OVERLOAD_MASK);
					// try to save it.
					mediumDivider = 1;
				}
			}
		} else {
			// Fast task collision has occured.
			LEDEvent(LED_SCHEDULER_OVERLOAD_MASK);
			// try to save it.
			fastDivider = 1;
		}
	}

#if (defined (SUPPORT_RETRACT) && defined (RETRACT_SERVOOUT_LOCAL) || defined (SUPPORT_YAW_SERVO) && defined (YAW_SERVOOUT_LOCAL))
	static bool emitting;
	if (servoPauseDivider) {
		--servoPauseDivider;
	} else {
#if defined (SUPPORT_YAW_SERVO) && defined (YAW_SERVOOUT_LOCAL)
		if (!emitting) {
			SERVO_PORT |= (1 << YAW_SERVO_BIT);
			yawServoDivider = yawServoPulseLength;
		} else if (yawServoDivider) {
			--yawServoDivider;
		} else {
			SERVO_PORT &= ~(1 << YAW_SERVO_BIT);
		}
#endif
#if defined (SUPPORT_RETRACT) && defined (RETRACT_SERVOOUT_LOCAL)
		if (!emitting) {
			SERVO_PORT |= (1 << RETRACT_SERVO_BIT);
			retractServoDivider = retractServoPulseLength;
		} else if (retractServoDivider) {
			--retractServoDivider;
		} else {
			SERVO_PORT &= ~(1 << RETRACT_SERVO_BIT);
		}
#endif
		if (retractServoDivider || yawServoDivider) emitting = true;
		else {
			servoPauseDivider = 0.05 * (F_CPU/510); // 40 ms
			emitting = false;
		}
	}
#endif

	if (!(++timer1Extension)) { // Overflows at 122.55 Hz
		timer1ExtensionExtension++;
	}
}
