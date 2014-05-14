#ifndef __UTIL_H
#define __UTIL_H

/************************/
/* Debugging            */
/************************/
#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>
//#include "Globals.h"

extern uint8_t LEDFlags;

// DEBUG only
extern uint32_t stackTop;
extern uint32_t stackBottom;
extern uint32_t heapTop;
extern uint32_t heapBottom;

inline void LEDEvent(uint8_t mask) {
	LEDFlags |= mask;
}

inline void stackCheck() {
  int localVar;
  // stack limits
  if ((uint32_t)&localVar < stackTop) stackTop = (uint32_t)&localVar;
  if ((uint32_t)&localVar > stackBottom) stackBottom = (uint32_t)&localVar;
}

inline void heapCheck() {
  uint32_t * memPtr;
  // heap limits
  memPtr = (uint32_t *)malloc(8);
  if ((uint32_t)memPtr > heapTop) heapTop = (uint32_t)memPtr;
  if ((uint32_t)memPtr < heapBottom) heapBottom = (uint32_t)memPtr;
  free(memPtr);
}

inline void stackHeapEval(bool doPrint) {
  int diff = stackTop - heapTop;
  if (diff < 64) {
    printf_P(PSTR("WARNING: low memory < 64\r\n"));
  }
  if ((diff <64) || doPrint) {
    printf_P(PSTR("free memory = %d\r\n"), diff);
    printf_P(PSTR("stackSize   = %ld\r\n"), stackBottom-stackTop);
    printf_P(PSTR("stackBottom = %lx\r\n"), stackBottom);
    printf_P(PSTR("stackTop    = %lx\r\n"), stackTop);
    printf_P(PSTR("heapSize    = %ld\r\n"), heapTop-heapBottom);
    printf_P(PSTR("heapTop     = %lx\r\n"), heapTop);
    printf_P(PSTR("heapBottom  = %lx\r\n"), heapBottom);
  }
}

uint16_t crc16(uint8_t* data, size_t size);

inline char tolower(char c) {
	if (c >= 'A' && c <= 'Z')
		return c - 'A' + 'a';
	return c;
}

inline bool isprint(char c) {
	return c >= 0x20 && c != 0x7f;
}

// TODO: Proper C++ overloading possible?
inline int16_t constrain_int16(int16_t x, int16_t l, int16_t h) {
	if (x <= l) {
		return l;
	} else if (x >= h) {
		return h;
	} else {
		return x;
	}
}

// TODO: Proper C++ overloading possible?
inline float constrain_f(float x, float l, float h) {
	if (x <= l) {
		return l;
	} else if (x >= h) {
		return h;
	} else {
		return x;
	}
}

//***************************************************************
// “Efficient approximations for the arctangent function”,
// Rajan, S. Sichun Wang Inkol, R. Joyal, A., May 2006
//***************************************************************
float Rajan_FastArcTan(float x);

// atan2 for all quadrants by A. Hahn
float Rajan_FastArcTan2(float y, float x);

// atan2 returnig degrees * 1000
// int32_t Rajan_FastArcTan2_scaled(float y, float x);

/****************************/
/* LP Filter                */
/* Coeff=0: *q not changing */
/* Coeff=1: *q <- i         */
/****************************/
/*
inline void utilLP_float(float* q, float i, float coeff) {
	*q += (i - *q) * coeff;
}
*/

inline void utilLP_int(int16_t* q, int16_t i, uint16_t coeff) {
	*q += (i - *q) * coeff;
	*q += ((i - *q) * (int32_t)coeff >> 8);
}

inline uint16_t abs16(int16_t z) {
	return z < 0 ? -z : z;
}

inline uint32_t abs32(int32_t z) {
	return z < 0 ? -z : z;
}

#define COARSE_TIME_UNITS (510.0/F_CPU)
// In CPU cycles
uint16_t time();

// In units of 510/F_CPU
uint32_t coarseTime();

//void wait_16_micros(uint8_t n);

int16_t degreesToND(int8_t a);
int8_t NDToDegrees(int16_t a);

uint8_t selectBits(uint8_t data, uint8_t bitStart, uint8_t length);
void setBits(uint8_t* writtenTo, uint8_t bitStart, uint8_t length, uint8_t newBits);

enum PERFORMANCE_ITEMS {
	BM_IDLE,
	BM_READ_GYROS,
    BM_BLEND_GYROS,
    BM_BLEND_ACC,
    BM_CALCULATE_AA,
    BM_RC_DECODE,
    BM_PIDS,
    BM_MOTORPHASES,
    BM_SLOWLOOP,
    BM_PRINTBM,
	BM_OTHER,
    BM_END
};

//#define DO_PERFORMANCE 1

#ifdef DO_PERFORMANCE
//extern uint8_t hitCounters[];
extern uint16_t lastCycleTime;
extern uint16_t cycleStartTime;
extern uint16_t performanceTimers[];
extern uint16_t slowLoopPerformanceTimers[];
extern uint8_t nowPerformanceTiming;
// singleton stack only right now.
// extern uint8_t performanceStack;
extern uint8_t slowLoopTaskPerformanceTimed;
/*
 * This method will be reliable if: Timer runs af full blast (16MHz)
 * and nothing is benchmarked that takes longer than 4ms.
 */
inline void doPerformance(uint8_t item) {
	static uint16_t then;
	uint16_t now = time();
	uint16_t t = now - then;
	// The "other" item is multiple. All others single per cycle.
	if (nowPerformanceTiming != -1) {
		if (nowPerformanceTiming == BM_OTHER)
			performanceTimers[nowPerformanceTiming] += t;
		else
			performanceTimers[nowPerformanceTiming] = t;
		if(nowPerformanceTiming == BM_SLOWLOOP)
			slowLoopPerformanceTimers[slowLoopTaskPerformanceTimed] = t;
	}
	nowPerformanceTiming = item;
	then = now;
}

// singleton stack only right now.
/*
inline void performancePush(uint8_t item) {
	performanceStack = nowPerformanceTiming;
	doPerformance(item);
}

inline void performancePop() {
	if (performanceStack != -1)
		doPerformance(performanceStack);
}
*/

inline void performanceNewCycle() {
	uint16_t now = time();
	// Special multiple case.
	performanceTimers[BM_OTHER] = 0;
	lastCycleTime = now - cycleStartTime;
	cycleStartTime = now;
}

void reportPerformance();

#endif

// Maybe this macro circus is a waste of (my) time, as the compiler would optimize away
// an empty impl. anyway. Especially if inline.
#ifdef DO_PERFORMANCE
#define PERFORMANCE(item) doPerformance(item)
#define PERFORMANCE_NEW_CYCLE performanceNewCycle()
//#define PERFORMANCE_PUSH(item) performancePush(item)
//#define PERFORMANCE_POP performancePop()
#else
#define PERFORMANCE(item)
#define PERFORMANCE_NEW_CYCLE
//#define PERFORMANCE_PUSH(item)
//#define PERFORMANCE_POP
#endif

#endif
