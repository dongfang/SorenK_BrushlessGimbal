#include "Board.h"
#include "Definitions.h"
#include "Globals.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>

extern uint8_t timer1Extension;

#define IDLE 0
#define RUNNING 1
#define DONE 2
extern void fastTask();
extern void mediumTask();

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
  sei();
  fastDivider--;	// decrements at F_CPU/256 so after time T it is T*F_CPU/256
  timer1Extension++;
  if (fastState == DONE) {
    // As a further experiment, try put this at exit time of mainLoop (unsynced PWM out).
    PWM_A_MOTOR0 = motorPhases[0][0];
    PWM_B_MOTOR0 = motorPhases[0][1];
    PWM_C_MOTOR0 = motorPhases[0][2];
    PWM_A_MOTOR1 = motorPhases[1][0];
    PWM_B_MOTOR1 = motorPhases[1][1];
    PWM_C_MOTOR1 = motorPhases[1][2];
    fastState = IDLE;
  }
  if (!fastDivider) {
	  if (fastState == IDLE) {
		  cli();
		  fastState = RUNNING;
		  DEBUG_PORT |= 1<<DEBUG_BIT1;
		  fastDivider = FAST_PERIOD;
		  sei();
		  fastTask();
		  fastState = DONE;
		  DEBUG_PORT &= ~(1<<DEBUG_BIT1);
		  mediumDivider--;
		  if (!mediumDivider) {
			  if (mediumState == IDLE) {
				  cli();
				  DEBUG_PORT |= 1<<DEBUG_BIT2;
				  mediumState = RUNNING;
				  mediumDivider = MEDIUM_SUBPERIOD;
				  sei();
				  mediumTask();
				  mediumState = IDLE;
				  DEBUG_PORT &= ~(1<<DEBUG_BIT2);
			  } else {
				  // Medium task collision has occured.
				  printf_P(PSTR("BANG: MC\r\n"));
				  // try to save it.
				  mediumDivider = 1;
			  }
		  }
	  } else {
		  // Fast task collision has occured.
		  printf_P(PSTR("BANG: FC\r\n"));
		  // try to save it.
		  fastDivider = 1;
	  }
  }
}

/*
if (faststate == RUNNING) return;
if (faststate == DONE) lastchout to motors; faststate = IDLE;
if (faststate == IDLE) {
	if ([time for fast loop]) {
		faststate = RUNNING;
		reschedule
		fastLoop()
		faststate = DONE;
	}
	if(mediumstate == DONE && [time for medium loop]) {
		mediumstate = RUNNING;
		reschedule
		mediumLoop();
		mediumstate = DONE;
	}
}

 * Oh shit
struct Task {
	uint8_t state;
	int nextTime;
	int period;
	void (*task)();
	void (*postTask)(); // if there is something that much be synced to the timer's overflow
}

ISR(timern_OVF) {
	uint8_t i;
	int t = timerExtension;
	sei();
	for (i=0; i<NUM_TASKS; i++) {
		if (tasks[i].state == DONE) {
			if (tasks[i].postTask()!=0) tasks[i].postTask();
			tasks[i].state = IDLE;
		}
		// Oh shit here we need to check that no higher priority stuff is waiting.. too comlicated or I lack the right idea right now.
		if (tasks[i].state == IDLE && t >= tasks[i].nextTime) {
			tasks[i] = RUNNING;
			tasks[i].nextTime += tasks[i].period;
			tasks[i].task();
			tasks[i] = DONE;
		}
	}
}
*/
