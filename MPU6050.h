#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "definitions.h"
#include <avr/pgmspace.h>


#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

// Do not change for now
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_250  // +-250,500,1000,2000 deg/s
#define MPU6050_ACCEL_FS MPU6050_ACCEL_FS_2
#define MPU6050_DLPF_BW MPU6050_DLPF_BW_256  // 5,10,20,42,98,188,256 Hz

/*
 * Except for the address and the configuration sent to the
 * hardware MPU6050, this class is supposed to be stateless.
 */
class MPU6050 {
private:
	uint8_t _address;
public:
	// Init the I2C HW but not any configuration.
	void init();

	void setAddr(uint8_t address) {_address = address;}
	bool testConnection();

	// Configure
	void setClockSource(uint8_t); // Set Clock to ZGyro
	void setFullScaleGyroRange(uint8_t); // Set Gyro Sensitivity to config.h
	void setFullScaleAccelRange(uint8_t); //
	void setRate(uint8_t); // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
	void setSleepEnabled(bool);
	void setDLPFMode(uint8_t); // experimental AHa: set to slow mode during calibration

	// Divide raw values by this to get degrees/sec.
	float gyroToDeg_s() {
	    if(MPU6050_GYRO_FS == MPU6050_GYRO_FS_250) return 	131.0;
	    if(MPU6050_GYRO_FS == MPU6050_GYRO_FS_500) return  	65.5;
	    if(MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000) return  32.8;
	    if(MPU6050_GYRO_FS == MPU6050_GYRO_FS_2000) return  16.4;
	    return 1; // should never happen
	}

	uint16_t accToG() {
	    if(MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2)  return 	16384;
	    if(MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4)  return  8192;
	    if(MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8)  return  4096;
	    if(MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16) return  2048;
	    return 1; // should never happen
	}
	// Shit .. this depends on acc.meter settings!!
	#define ACC_1G 16384.0f

	// Get motion data
	void getRotationRates(int16_t* xyz);
	int16_t getAcceleration(uint8_t idx);
	//int16_t getAccelerations(uint8_t idx);
};

#endif
