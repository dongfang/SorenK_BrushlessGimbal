#include "Definitions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

extern uint8_t timer1Extension;

void initBlController() {
  DDRB |= 1<<1 | 1<<2 | 1<<3;
  DDRD |= 1<<3 | 1<<5 | 1<<6;

  TCCR0A = (1<<COM0A1) | (1<<COM0B1) | (1<<WGM00); 
  TCCR0B = (1<<CS00);
  TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10);
  TCCR1B = (1<<CS10);
  TCCR2A = (1<<COM2A1) | (1<<COM2B1) | (1<<WGM20);
  TCCR2B = (1<<CS20);

  // Start out with no power applied.
  OCR2A = 0;  //11  APIN
  OCR2B = 0;  //D3
  OCR1A = 0;  //D9  CPIN
  OCR1B = 0;  //D10 BPIN
  OCR0A = 0;  //D6
  OCR0B = 0;  //D5 
}

// Only for testing?
/*
void fastMoveMotor(uint8_t motorNumber, int dirStep, uint8_t* pwmSin) {
  if (motorNumber == 0) {
    currentStepMotor0 += dirStep;
    currentStepMotor0 &= 0xff;
    PWM_A_MOTOR0 = pwmSin[currentStepMotor0];
    PWM_B_MOTOR0 = pwmSin[(uint8_t)(currentStepMotor0 + 85)];
    PWM_C_MOTOR0 = pwmSin[(uint8_t)(currentStepMotor0 + 170)];
  }
 
  if (motorNumber == 1) {
    currentStepMotor1 += dirStep;
    currentStepMotor1 &= 0xff;
    PWM_A_MOTOR1 = pwmSin[currentStepMotor1] ;
    PWM_B_MOTOR1 = pwmSin[(uint8_t)(currentStepMotor1 + 85)] ;
    PWM_C_MOTOR1 = pwmSin[(uint8_t)(currentStepMotor1 + 170)] ;
  }
}
*/

void calcSinusArray(uint8_t maxPWM, uint8_t *array) {
  for(int i=0; i<N_SIN; i++) {
//    array[i] = maxPWM / 2.0 + sin(2.0 * i / N_SIN * 3.14159265) * maxPWM / 2.0;
    array[i] = 128 + sin(2.0 * i / N_SIN * M_PI) * maxPWM / 2.0;
  }
}

void recalcMotorStuff() {
  cli();
  calcSinusArray(config.maxPWMmotorPitch, pwmSinMotorPitch);
  calcSinusArray(config.maxPWMmotorRoll, pwmSinMotorRoll);
  sei();
}

/*
// switch off motor power
// TODO: for some reason motor control gets noisy, if call from ISR
inline void motorOff(uint8_t motorNumber, uint8_t* pwmSin) {
  if (motorNumber == 0) {
    PWM_A_MOTOR0 = pwmSin[0];
    PWM_B_MOTOR0 = pwmSin[0];
    PWM_C_MOTOR0 = pwmSin[0];
  }
 
  if (motorNumber == 1) {
    PWM_A_MOTOR1 = pwmSin[0];
    PWM_B_MOTOR1 = pwmSin[0];
    PWM_C_MOTOR1 = pwmSin[0];
  }
}
*/

#define IDLE 0
#define RUNNING 1
#define DONE 2
extern void fastTask();
extern void mediumTask();

#define FAST_PERIOD (F_CPU/510/FASTLOOP_FREQ)
#define MEDIUM_PERIOD (F_CPU/510/MEDIUMLOOP_FREQ)

/********************************/
/* Motor Control IRQ Routine    */
/********************************/
// Motor control.
// Should trigger at 31.25 kHz. More than often enough that we really can output the PWM values in sync with the overflow.
ISR (TIMER1_OVF_vect) {
  static uint8_t fastState = DONE;
  static uint8_t mediumState = DONE;
  static uint8_t fastDivider = FAST_PERIOD;
  static uint16_t mediumDivider = MEDIUM_PERIOD;
  fastDivider--;	// decrements at F_CPU/256 so after time T it is T*F_CPU/256
  mediumDivider--;
  timer1Extension++;
  sei(); // Make reentrant (!)
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
  if (fastState == IDLE && !fastDivider) {
    fastState = RUNNING;
    fastDivider = FAST_PERIOD;
	//LED_PORT |= (1 << LED_BIT);
    fastTask();
	//LED_PORT &= ~(1 << LED_BIT);
    fastState = DONE;
  }
  if (fastState != RUNNING && mediumState == IDLE && !mediumDivider) {
    mediumState = RUNNING;
    mediumDivider = MEDIUM_PERIOD;
	//LED_PORT |= (1 << LED_BIT);
    mediumTask();
	//LED_PORT &= ~(1 << LED_BIT);
    mediumState = IDLE;
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

/*
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
/*
void motorTest() {
  #define MOT_DEL 100
  cli();
  _delay_ms(10 * CC_FACTOR);
  // Move Motors to ensure function
  for(int i=0; i<100; i++) { fastMoveMotor(config.motorNumberPitch, 1,pwmSinMotorPitch); _delay_ms(MOT_DEL * CC_FACTOR); }
  for(int i=0; i<100; i++) { fastMoveMotor(config.motorNumberPitch, -1,pwmSinMotorPitch); _delay_ms(MOT_DEL * CC_FACTOR); }
  _delay_ms(200 * CC_FACTOR);
  for(int i=0; i<100; i++) { fastMoveMotor(config.motorNumberRoll, 1,pwmSinMotorRoll); _delay_ms(MOT_DEL * CC_FACTOR); }
  for(int i=0; i<100; i++) { fastMoveMotor(config.motorNumberRoll, -1,pwmSinMotorRoll); _delay_ms(MOT_DEL * CC_FACTOR); }
  sei();  
}
*/

