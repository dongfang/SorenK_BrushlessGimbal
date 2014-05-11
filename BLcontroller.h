#include "Definitions.h"
#include <math.h>
#include <avr/io.h>

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

void recalcMotorPower(uint8_t rollPower, uint8_t pitchPower) {
  calcSinusArray(rollPower, pwmSinMotorRoll);
  calcSinusArray(pitchPower, pwmSinMotorPitch);
}

void recalcMotorPower() {
  cli();
  recalcMotorPower(config.rollMotorPower, config.pitchMotorPower);
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

