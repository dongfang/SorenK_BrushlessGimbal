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

volatile bool newPWMData;
volatile bool mediumTaskHasRun;

#ifdef SUPPORT_YAW_SERVO
volatile uint8_t yawServoPulseLength = 46;
void setYawServoOut(uint8_t pulse) {
	cli();
	yawServoPulseLength = pulse-1;
	sei();
}
#endif

#ifdef SUPPORT_RETRACT
volatile uint8_t retractServoPulseLength = 46;
void setRetractServoOut(uint8_t pulse) {
	cli();
	retractServoPulseLength = pulse-1;
	sei();
}
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
	static uint8_t yawServoDivider;
	static uint8_t retractServoDivider;
	static uint16_t servoPauseDivider;
	sei();

	fastDivider--; // decrements at F_CPU/256 so after time T it is T*F_CPU/256

	if (newPWMData) {
		// As a further experiment, try put this at exit time of mainLoop (unsynced PWM out).
		PWM_A_MOTOR0 = motorPhases[0][0];
		PWM_B_MOTOR0 = motorPhases[0][1];
		PWM_C_MOTOR0 = motorPhases[0][2];
		PWM_A_MOTOR1 = motorPhases[1][0];
		PWM_B_MOTOR1 = motorPhases[1][1];
		PWM_C_MOTOR1 = motorPhases[1][2];
		newPWMData = false;
	}

	if (!fastDivider) {
		if (fastState == IDLE) {
			//cli();
			fastState = RUNNING;
#ifdef DEBUG_SIGNALS
			DEBUG_PORT |= 1<<DEBUG_BIT1;
#endif
			fastDivider = FAST_PERIOD;
			//sei();
			fastTask();
			fastState = IDLE;
			// TODO: Maybe move to fast-loop itself.
#ifdef DEBUG_SIGNALS
			DEBUG_PORT &= ~(1<<DEBUG_BIT1);
#endif
			if (!--mediumDivider) {
				if (mediumState == IDLE) {
					cli();
#ifdef DEBUG_SIGNALS
					DEBUG_PORT |= 1<<DEBUG_BIT2;
#endif
					mediumState = RUNNING;
					mediumDivider = MEDIUM_SUBPERIOD;
					sei();
					mediumTask();
					mediumTaskHasRun = true; // To assist the slow-task stuff to sync
					mediumState = IDLE;
#ifdef DEBUG_SIGNALS
					DEBUG_PORT &= ~(1<<DEBUG_BIT2);
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
#if defined(SUPPORT_YAW_SERVO) || defined(SUPPORT_RETRACT)
	static bool emitting;
		if (servoPauseDivider) {
			--servoPauseDivider;
		} else {
#if defined(SUPPORT_YAW_SERVO)
			if (!emitting) {
				SERVO_PORT |= (1 << YAW_SERVO_BIT);
				yawServoDivider = yawServoPulseLength;
			} else if (yawServoDivider) {
				--yawServoDivider;
			} else {
				SERVO_PORT &= ~(1 << YAW_SERVO_BIT);
			}
#endif
#if defined(SUPPORT_RETRACT)
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
