#include "Util.h"
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

#ifdef DO_BENCHMARK
uint32_t benchmarkTimers[BM_END];
uint8_t nowBenchmarking;
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

// atan2 returning degrees * 1000
int32_t Rajan_FastArcTan2_deg1000(float y, float x) {
  return 180/M_PI * 1000 * Rajan_FastArcTan2(y, x);
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

#ifdef DO_BENCHMARK
PGM_P const benchmarkItemNames[] PROGMEM = {
	"OTHER",
	"READ_GYROS",
	"BLEND_GYROS",
	"BLEND_ACC",
	"CALCULATE_AA",
	"RC_DECODE",
	"PIDS",
	"MOTORPHASES",
	"SLOWLOOP",
	"TIMEOUTS",
	"SERIAL",
	"END"
};
#endif
