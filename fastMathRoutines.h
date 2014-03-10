#include <util/crc16.h>

inline int32_t constrain_int32(int32_t x , int32_t l, int32_t h) {
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
inline float Rajan_FastArcTan(float x) {
  return PI/4.0*x - x*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x));
}

// atan2 for all quadrants by A. Hahn
inline float Rajan_FastArcTan2(float y, float x) {

  uint8_t qCode;
  const float pi_2 = PI/2.0;
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
      case 0: z = pi_2 - z;  break;
      case 1: z = pi_2 - z;  break;
      case 2: z = -pi_2 - z; break;
      case 3: z = -pi_2 - z; break;
    }
  } else {
    switch (qCode) {    
      case 0: z = z;         break;
      case 1: z = PI + z;    break;
      case 2: z = -PI + z;   break;
      case 3: z = z;         break;
    }
  }
  return z;
}

// atan2 returnig degrees * 1000
int32_t Rajan_FastArcTan2_deg1000(float y, float x) {
  return 180/PI * 1000 * Rajan_FastArcTan2(y, x);
}

/****************************/
/* LP Filter                */
/* Coeff=0: *q not changing */
/* Coeff=1: *q <- i         */
/****************************/
inline void utilLP_float(float* q, float i, float coeff) {
  *q += (i-*q)*coeff;
}

/************************/
/* Debugging            */
/************************/
// TODO: move functions to other file (utilities ?)

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
    Serial.println(F("WARNING: low memory < 64"));
  }
  if ((diff <64) || doPrint) {
    Serial.print(F("free memory = ")); Serial.println(diff);
    Serial.print(F("stackSize   = ")); Serial.println(stackBottom-stackTop);
    Serial.print(F("stackBottom = 0x")); Serial.println(stackBottom, HEX);
    Serial.print(F("stackTop    = 0x")); Serial.println(stackTop, HEX);
    Serial.print(F("heapSize    = ")); Serial.println(heapTop-heapBottom);
    Serial.print(F("heapTop     = 0x")); Serial.println(heapTop, HEX);
    Serial.print(F("heapBottom  = 0x")); Serial.println(heapBottom, HEX);
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
