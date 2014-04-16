/************************/
/* Debugging            */
/************************/
#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#include "Globals.h"

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

uint16_t crc16(uint8_t* data, size_t size) {
  size_t i;
  uint16_t crc = 23456;
  for (i=0; i<size; i++) {
    crc = _crc16_update(crc, data[i]);
  }
  return crc;
}

char tolower(char c) {
	if (c >= 'A' && c <= 'Z')
		return c - 'A' + 'Z';
	return c;
}

bool isprint(char c) {
	return c >= 0x20 && c != 0x7f;
}
