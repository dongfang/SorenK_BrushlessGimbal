#include "Util.h"
#include "I2C.h"
#include "Definitions.h"
#include <math.h>
#include <stdlib.h>
#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// DEBUG only
uint32_t stackTop = 0xffffffff;
uint32_t stackBottom = 0;
uint32_t heapTop = 0;
uint32_t heapBottom = 0xffffffff;
uint8_t timer1Extension;
uint16_t timer1ExtensionExtension;
#include "Globals.h"

#ifdef DO_PERFORMANCE
uint16_t lastCycleTime;
uint16_t cycleStartTime;
uint16_t performanceTimers[BM_END];
uint16_t slowLoopPerformanceTimers[12];
uint8_t nowPerformanceTiming;
uint8_t performanceStack;
#endif

uint16_t crc16(uint8_t* data, size_t size) {
  size_t i;
  uint16_t crc = 23456;
  for (i=0; i<size; i++) {
    crc = _crc16_update(crc, data[i]);
  }
  return crc;
}

//***************************************************************
// “Efficient approximations for the arctangent function”,
// Rajan, S. Sichun Wang Inkol, R. Joyal, A., May 2006
//***************************************************************
float Rajan_FastArcTan(float x) {
  return M_PI/4.0*x - x*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x));
}

// atan2 for all quadrants by A. Hahn
float Rajan_FastArcTan2(float y, float x) {
  uint8_t qCode;
  float q;
  float z;

  // 6 us
  bool swap45 = (fabs(y) > fabs(x));

  // 22us
  if ((y >= 0) && (x >= 0)) { qCode = 0; }
  if ((y >= 0) && (x <= 0)) { qCode = 1; }
  if ((y <= 0) && (x <= 0)) { qCode = 2; }
  if ((y <= 0) && (x >= 0)) { qCode = 3; }

  // 54 us
  if (swap45) {
    q = x / y;
  } else {
    q = y / x;
  }

  // 92 us
  z = Rajan_FastArcTan(q);

  if (swap45) {
    switch (qCode) {
      case 0: z = M_PI_2 - z;  break;
      case 1: z = M_PI_2 - z;  break;
      case 2: z = -M_PI_2 - z; break;
      case 3: z = -M_PI_2 - z; break;
    }
  } else {
    switch (qCode) {
      case 0: 		         break;
      case 1: z = M_PI + z;    break;
      case 2: z = -M_PI + z;   break;
      case 3: 		         break;
    }
  }
  return z;
}

uint16_t time() {
  uint8_t sreg = SREG;
  cli();
  uint8_t fine = TCNT1;
  uint8_t after;
  // Get a counter step. That should not take more than 0.1 us at 32kHz.
  do {
   after = TCNT1;
  } while (after==fine);

  uint8_t ext = timer1Extension;
  uint8_t flags = TIFR1;
  SREG = sreg;

  if (flags & (1<<TOV1)) {
// We simulate the interrupt has happened and the fine timer is zero.
      return (ext+1) << 9;
  } else {
    if (after < fine) {
    // We were downcounting.
      return ((ext+1)<<9) - after;
    } else {
      return (ext<<9) + after;
    }
  }
}

// same as time() but with a wider result.
uint32_t time32() {
	cli();
	uint16_t t = time();
	return t + ((uint32_t)timer1ExtensionExtension << 16);
}

/*
 * 1 cycle is 16 microseconds
 */
void wait_16_micros(uint8_t n) {
	uint8_t capture = timer1Extension + n;
	while(timer1Extension != capture);
}

uint8_t selectBits(uint8_t data, uint8_t bitStart, uint8_t length) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data &= mask;
    data >>= (bitStart - length + 1);
    return data;
}

void setBits(uint8_t* writtenTo, uint8_t bitStart, uint8_t length, uint8_t newBits) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	newBits <<= (bitStart - length + 1); // shift data into correct position
	newBits &= mask; // zero all non-important bits in data
	*writtenTo &= ~(mask); // zero all important bits in existing byte
	*writtenTo |= newBits; // combine data with existing byte
}

#ifdef DO_PERFORMANCE
static const char BMS00[] PROGMEM = "Idle   ";
static const char BMS01[] PROGMEM = "ReadGyros";
static const char BMS02[] PROGMEM = "BlendGyros";
static const char BMS03[] PROGMEM = "BlendAcc";
static const char BMS04[] PROGMEM = "Calc AA";
static const char BMS05[] PROGMEM = "RCDecode";
static const char BMS06[] PROGMEM = "PIDs   ";
static const char BMS07[] PROGMEM = "MotorPhases";
static const char BMS08[] PROGMEM = "SlowLoop";
static const char BMS09[] PROGMEM = "This   ";	// The task of printing this benchmark info.
static const char BMS10[] PROGMEM = "Other  ";
static const char BMS11[] PROGMEM = "END";

static PGM_P const performanceItemNames[] PROGMEM = {
	BMS00,BMS01,BMS02,BMS03,BMS04,BMS05,BMS06,BMS07,BMS08,BMS09,BMS10,BMS11
};

static const char BMSLS00[] PROGMEM = "UpdAccMagm";
static const char BMSLS01[] PROGMEM = "UpdAccMagd1";
static const char BMSLS02[] PROGMEM = "UpdAccMagd2";
static const char BMSLS03[] PROGMEM = "FlashLED";
static const char BMSLS04[] PROGMEM = "GimState";
static const char BMSLS05[] PROGMEM = "RCPitch ";
static const char BMSLS06[] PROGMEM = "RCRoll  ";
static const char BMSLS07[] PROGMEM = "Debug   ";
static const char BMSLS08[] PROGMEM = "Timeouts";
static const char BMSLS09[] PROGMEM = "Serial  ";
static const char BMSLS10[] PROGMEM = "Restart ";

static PGM_P const performaceSlowLoopSubitemNames[] PROGMEM = {
		BMSLS00,BMSLS01,BMSLS02,BMSLS03,BMSLS04,BMSLS05,BMSLS06,BMSLS07,BMSLS08,BMSLS09,BMSLS10
};

void reportPerformance() {
	 uint8_t save = nowPerformanceTiming;
	 doPerformance(BM_PRINTBM);
	 for (uint8_t i=0; i<BM_END; i++) {
		 uint16_t t = performanceTimers[i];
		 PGM_P const sprt = (PGM_P)pgm_read_word(&performanceItemNames[i]);
		 printf_P(PSTR("%S:\t%u\t\r\n"), sprt, t);
	 }
	 printf_P(PSTR("\r\nSlow loop:\r\n"));
	 for (uint8_t i=0; i<sizeof(performaceSlowLoopSubitemNames)/2; i++) {
		 uint32_t t = slowLoopPerformanceTimers[i];
		 PGM_P const sprt = (PGM_P)pgm_read_word(&performaceSlowLoopSubitemNames[i]);
		 printf_P(PSTR("%S:\t%lu\t\r\n"), sprt, t);
	 }
	 // TODO: Remove off to some debug/status thing,
	 printf_P(PSTR("I2C error count: %u\r\n"), i2c_errors_count);
	 printf_P(PSTR("Cycle time %u CPU cycles (%u us)\r\n"), lastCycleTime, lastCycleTime/(F_CPU/1000000UL));
	 doPerformance(save);
 }
#endif

