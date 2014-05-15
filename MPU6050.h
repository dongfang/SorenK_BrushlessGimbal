#ifndef __MPU6050_H
#define __MPU6050_H

#include "Definitions.h"
#include "I2C.h"
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

// dongfang: I want 500 deg/s. I like the tolerance to bumps (gyros won't saturate so easily).
// But trying to set up the MPU6050 chip to anything else than 250 does not work. Why?
// #define MPU6050_GYRO_FS MPU6050_GYRO_FS_500  // +-250,500,1000,2000 deg/s
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_250  // +-250,500,1000,2000 deg/s
#define MPU6050_ACCEL_FS MPU6050_ACCEL_FS_2
#define MPU6050_DLPF_BW MPU6050_DLPF_BW_256  // 5,10,20,42,98,188,256 Hz
#define MPU6050_RA_WHO_AM_I         	0x75
#define MPU6050_WHO_AM_I_BIT        	6
#define MPU6050_WHO_AM_I_LENGTH     	6

#define MPU6050_RA_SMPLRT_DIV       	0x19

#define MPU6050_RA_CONFIG           	0x1A
#define MPU6050_CFG_DLPF_CFG_BIT    	2
#define MPU6050_CFG_DLPF_CFG_LENGTH 	3

#define MPU6050_RA_GYRO_CONFIG      	0x1B
#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_RA_ACCEL_CONFIG     	0x1C
#define MPU6050_ACONFIG_AFS_SEL_BIT		4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH  2

#define MPU6050_RA_PWR_MGMT_1       	0x6B
#define MPU6050_PWR1_CLKSEL_BIT     	2
#define MPU6050_PWR1_CLKSEL_LENGTH  	3

#define MPU6050_PWR1_SLEEP_BIT          6

#define MPU6050_RA_ACCEL_XOUT_H     	0x3B
#define MPU6050_RA_GYRO_XOUT_H      	0x43

/*
 * Except for the address and the configuration sent to the
 * hardware MPU6050, this class is supposed to be stateless.
 */
class MPU6050 {
private:
	uint8_t devAddr;
//	uint8_t buffer[6];
public:
	static const uint8_t GYRO = 1;
	static const uint8_t ACC = 0;

	int16_t gyro[3];
	int16_t gyroSum[3];
	int16_t acc[3];

	struct SensorAxisDef {
	  uint8_t idx;		// index of physical sensor
	  int  dir;			// sign of logical sensor
	} SensorAxisDef_t;

	struct SensorOrientationDef {
	  SensorAxisDef acc[3];
	  SensorAxisDef gyro[3];
	};

	// Reset signal paths. Often jammed at startup.
	void unjam();

	// Init the I2C HW but not any configuration.
	void init();

	void setAddr(uint8_t address) {
		devAddr = address;
	}

	bool testConnection();

	// Configure
	void setClockSource(uint8_t); // Set Clock
	void setGyroRange(uint8_t); // Set Gyro Sensitivity
	void setAccelRange(uint8_t); //
	void setRate(uint8_t); // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
	void setSleepEnabled(bool);
	void setDLPFMode(uint8_t); // experimental AHa: set to slow mode during calibration

	// Divide raw values by this to get degrees/sec.
	float gyroToDeg_s() {
		if (MPU6050_GYRO_FS == MPU6050_GYRO_FS_250)
			return 131.0;
		if (MPU6050_GYRO_FS == MPU6050_GYRO_FS_500)
			return 65.5;
		if (MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000)
			return 32.8;
		if (MPU6050_GYRO_FS == MPU6050_GYRO_FS_2000)
			return 16.4;
		return 1; // should never happen
	}

	// Divide raw values by 1<<this to get approx. degrees/sec
	uint8_t logGyroToDeg_s() {
		if (MPU6050_GYRO_FS == MPU6050_GYRO_FS_250)
			return 7;
		if (MPU6050_GYRO_FS == MPU6050_GYRO_FS_500)
			return 6;
		if (MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000)
			return 5;
		if (MPU6050_GYRO_FS == MPU6050_GYRO_FS_2000)
			return 4;
		return 1; // should never happen
	}

	uint16_t accToG() {
		if (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2)
			return 16384;
		if (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4)
			return 8192;
		if (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8)
			return 4096;
		if (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16)
			return 2048;
		return 1; // should never happen
	}

	// Divide raw values by 1<<this to get g's
	uint8_t logAccToG() {
		if (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2)
			return 14;
		if (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4)
			return 13;
		if (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8)
			return 12;
		if (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16)
			return 11;
		return 1; // should never happen
	}

	// Orientation of the MPU6550 chip - 3*2*4 possibilities.
	void initSensorOrientation(uint8_t majorAxis, bool reverseZ, uint8_t rotateXY);

	// Load EEPROM-stored offsets. Return true if success.
	bool loadSensorCalibration();

	// Do the calibration ritual and store result in EEPROM.
	void recalibrateSensor(void (*complain)(), uint8_t whichMotion);

	inline void resetAccCalibration() {
		sensorOffset[0] = sensorOffset[1] = sensorOffset[2] = 0;
	}

	inline void getAllSensorsAsync() {
		if (!i2c_is_async_done()) {
			i2c_shutdown();
			i2c_init();
			return;
		}

		uint8_t logicalAxis;

		for (logicalAxis=0; logicalAxis<3; logicalAxis++) {
			uint8_t physicalOffset = sensorDef.acc[logicalAxis].idx;
			uint8_t x2 = physicalOffset<<1;
			acc[logicalAxis] = (((int16_t)i2c_buffer[x2]) << 8) | i2c_buffer[(x2)+1];
			acc[logicalAxis] -= sensorOffset[physicalOffset];
			if (sensorDef.acc[logicalAxis].dir == -1)
				acc[logicalAxis] = -acc[logicalAxis];
		}

		for (logicalAxis=0; logicalAxis<3; logicalAxis++) {
			uint8_t physicalOffset = (sensorDef.gyro[logicalAxis].idx) + 4; // TODO: A hack to get to the gyro data.
			uint8_t x2 = physicalOffset<<1;
			int16_t result = (((int16_t)i2c_buffer[x2]) << 8) | i2c_buffer[x2+1];
			result -= sensorOffset[physicalOffset];
			if (sensorDef.gyro[logicalAxis].dir == -1)
				result = -result;
			gyro[logicalAxis] = result;
	    	gyroSum[logicalAxis] += result;
		}
	}

	inline void startAllSensorsAsync() {
		i2c_read_regs_async(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14);
	}

	inline void resetGyroSums() {
		gyroSum[X] = 0;
		gyroSum[Y] = 0;
		gyroSum[Z] = 0;
	}

	// gyro calibration value. Only public for debug's sake.
	// 3 acc, 1 thermometer (not used), 3 gyro, 1 CRC
	int16_t sensorOffset[3+1+3+1];

private:
	void initialOrientation();

	uint16_t CRC();
	//void getSensor(int16_t* result, uint8_t which);
	void saveSensorCalibration();

	void rotateMajorAxis();

	// swap two char items
	inline void swap_uint8(uint8_t * a, uint8_t * b) {
		uint8_t tmp = *a;
	  *a = *b;
	  *b = tmp;
	}

	// swap two int items
	inline void swap_int(int * a, int * b) {
	  int tmp = *a;
	  *a = *b;
	  *b = tmp;
	}
	SensorOrientationDef sensorDef;
};

#endif
