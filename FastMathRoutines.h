#ifndef __FASTMATH_H
#define __FASTMATH_H

#include <util/crc16.h>
#include <math.h>

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
inline int32_t constrain_int32(int32_t x, int32_t l, int32_t h) {
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
inline float Rajan_FastArcTan(float x) {
	return M_PI / 4.0 * x - x * (fabs(x) - 1) * (0.2447 + 0.0663 * fabs(x));
}

// atan2 for all quadrants by A. Hahn
inline float Rajan_FastArcTan2(float y, float x) {

	uint8_t qCode;
	float q;
	float z;

	// 6 us
	bool swap45 = (fabs(y) > fabs(x));

	// 22us
	if ((y >= 0) && (x >= 0)) {
		qCode = 0;
	}
	if ((y >= 0) && (x <= 0)) {
		qCode = 1;
	}
	if ((y <= 0) && (x <= 0)) {
		qCode = 2;
	}
	if ((y <= 0) && (x >= 0)) {
		qCode = 3;
	}

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
		case 0:
			z = M_PI_2 - z;
			break;
		case 1:
			z = M_PI_2 - z;
			break;
		case 2:
			z = -M_PI_2 - z;
			break;
		case 3:
			z = -M_PI_2 - z;
			break;
		}
	} else {
		switch (qCode) {
		case 0:
			break;
		case 1:
			z = M_PI + z;
			break;
		case 2:
			z = -M_PI + z;
			break;
		case 3:
			break;
		}
	}
	return z;
}

// atan2 returnig degrees * 1000
int32_t Rajan_FastArcTan2_deg1000(float y, float x) {
	return 180 / M_PI * 1000 * Rajan_FastArcTan2(y, x);
}

/****************************/
/* LP Filter                */
/* Coeff=0: *q not changing */
/* Coeff=1: *q <- i         */
/****************************/
inline void utilLP_float(float* q, float i, float coeff) {
	*q += (i - *q) * coeff;
}

inline uint16_t abs16(int16_t z) {
	return z < 0 ? -z : z;
}

inline uint32_t abs32(int32_t z) {
	return z < 0 ? -z : z;
}

#endif
