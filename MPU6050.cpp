#include "MPU6050.h"
#include "Util.h"
#include "Board.h"
#include "Globals.h"
#include <avr/wdt.h>
#include <avr/eeprom.h>

int16_t gyroOffsetInEEPROM[4] EEMEM;

void MPU6050::unjam() {
	// The famous dongfang anti lockup from MultiWii.
	i2c_writeReg(devAddr, 0x6B, 0x80); 	//PWR_MGMT_1    -- DEVICE_RESET 1
	_delay_ms(100);
	i2c_writeReg(devAddr, 0x68, 0x7);   //PWR_MGMT_1    -- RESETS
	_delay_ms(100);
}

void MPU6050::init() {
	setClockSource(MPU6050_CLOCK_PLL_ZGYRO); // Set Clock to ZGyro

	// This seems not to have any f'n effect at all! I read a zero back.
	setGyroRange(MPU6050_GYRO_FS); 		// Set Gyro Sensitivity. Unfortunately the chip just ignores us.
	setAccelRange(MPU6050_ACCEL_FS); 	//+- 2G
	setRate(7); // 0=8kHz, 1=4kHz, 2=2.66Hz, 3=2kHz, 4=1.6kHz, 5=1.33kHz, 6=1.14Hz, 7=1kHz
	setSleepEnabled(false);
}

/** Verify the I2C connection.
* Make sure the device is connected and responds as expected.
* @return True if connection is valid, false otherwise
*/
bool MPU6050::testConnection() {
	i2c_read_regs(devAddr, MPU6050_RA_WHO_AM_I, 1);
	uint8_t result = selectBits(i2c_buffer[0], MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH);
    return result == 0x34;
}

/*
 * Do not use! Counterproductive.
void MPU6050::setDLPFMode(uint8_t mode) {
	i2c_read_regs(devAddr, MPU6050_RA_CONFIG, 1);
	setBits(&i2c_buffer[0], MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
	i2c_writeReg(devAddr, MPU6050_RA_CONFIG, i2c_buffer[0]);
}
*/

void MPU6050::setClockSource(uint8_t source) {
	i2c_read_regs(devAddr, MPU6050_RA_PWR_MGMT_1, 1);
	setBits(&i2c_buffer[0], MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
	i2c_writeReg(devAddr, MPU6050_RA_PWR_MGMT_1, i2c_buffer[0]);
}

void MPU6050::setGyroRange(uint8_t range) {
	i2c_read_regs(devAddr, MPU6050_RA_GYRO_CONFIG, 1);
	setBits(&i2c_buffer[0], MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
	i2c_writeReg(devAddr, MPU6050_RA_GYRO_CONFIG, i2c_buffer[0]);
}

void MPU6050::setAccelRange(uint8_t range) {
	i2c_read_regs(devAddr, MPU6050_RA_ACCEL_CONFIG, 1);
	setBits(&i2c_buffer[0], MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
	i2c_writeReg(devAddr, MPU6050_RA_ACCEL_CONFIG, i2c_buffer[0]);
}

void MPU6050::setRate(uint8_t rate) {
   i2c_writeReg(devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}

void MPU6050::setSleepEnabled(bool enabled) {
	i2c_read_regs(devAddr, MPU6050_RA_PWR_MGMT_1, 1);
	setBits(&i2c_buffer[0], MPU6050_PWR1_SLEEP_BIT, 1, enabled ? 1 : 0);
	i2c_writeReg(devAddr, MPU6050_RA_PWR_MGMT_1, i2c_buffer[0]);
}


struct SensorAxisDef {
  uint8_t idx;		// index of physical sensor
  int  dir;			// sign of logical sensor
} SensorAxisDef_t;

struct SensorOrientationDef {
  SensorAxisDef acc[3];
  SensorAxisDef gyro[3];
};

const SensorOrientationDef _initialOrientation PROGMEM = {{{0,-1},{1,1},{2,1}}, {{1,1},{0,1},{2,1}}};

void MPU6050::initialOrientation() {
	uint8_t i;
	for (i=0; i<sizeof(_initialOrientation); i++)
		((uint8_t*)&sensorDef)[i] = pgm_read_byte((uint8_t*)&_initialOrientation+i);
}

void MPU6050::rotateMajorAxis() {
	uint8_t tmpAxis = sensorDef.gyro[ROLL].idx;
	int8_t tmpDir	= sensorDef.gyro[ROLL].dir;
	sensorDef.gyro[ROLL].idx = sensorDef.gyro[PITCH].idx;
	sensorDef.gyro[ROLL].dir = sensorDef.gyro[PITCH].dir;
	sensorDef.gyro[PITCH].idx = sensorDef.gyro[YAW].idx;
	sensorDef.gyro[PITCH].dir = sensorDef.gyro[YAW].dir;
	sensorDef.gyro[YAW].idx = tmpAxis;
	sensorDef.gyro[YAW].dir = tmpDir;

	tmpAxis = sensorDef.acc[X].idx;
	tmpDir	= sensorDef.acc[X].dir;
	sensorDef.acc[X].idx = sensorDef.acc[Z].idx;
	sensorDef.acc[X].dir = -sensorDef.acc[Z].dir;
	sensorDef.acc[Z].idx = sensorDef.acc[Y].idx;
	sensorDef.acc[Z].dir = sensorDef.acc[Y].dir;
	sensorDef.acc[Y].idx = tmpAxis;
	sensorDef.acc[Y].dir = -tmpDir;
}

void MPU6050::initSensorOrientation(uint8_t majorAxis, bool reverseZ, uint8_t rotateXY) {
	uint8_t i;

	initialOrientation();

	for(i=0; i<majorAxis; i++) {
		rotateMajorAxis();
	}

	if (reverseZ) {
		// flip over roll
		sensorDef.acc[Z].dir *= -1;
		sensorDef.acc[X].dir *= -1;
		sensorDef.gyro[PITCH].dir *= -1;
		sensorDef.gyro[YAW].dir *= -1;
	}

	for(i=0; i<rotateXY; i++) {
		// swap gyro axis
		swap_uint8(&sensorDef.gyro[ROLL].idx, &sensorDef.gyro[PITCH].idx);
		swap_int(&sensorDef.gyro[ROLL].dir, &sensorDef.gyro[PITCH].dir);
		sensorDef.gyro[PITCH].dir *= -1; // try and error ;-)
		// swap acc axis
		swap_uint8(&sensorDef.acc[X].idx, &sensorDef.acc[Y].idx);
		swap_int(&sensorDef.acc[X].dir, &sensorDef.acc[Y].dir);
		sensorDef.acc[Y].dir *= -1;
	}
}

uint16_t MPU6050::CRC() {
	return ::crc16((uint8_t*)sensorOffset, sizeof(sensorOffset)-2);
}

void MPU6050::saveSensorCalibration() {
	wdt_reset();
	sensorOffset[sizeof(sensorOffset)/2-1] = CRC();
	eeprom_write_block(sensorOffset, gyroOffsetInEEPROM, sizeof(sensorOffset));
}

bool MPU6050::loadSensorCalibration() {
	wdt_reset();
	eeprom_read_block(sensorOffset, gyroOffsetInEEPROM, sizeof(sensorOffset));
	bool ok = (uint16_t)sensorOffset[sizeof(sensorOffset)/2-1] == CRC();
	// printf_P(PSTR("Reused old gyrocal: %d\r\n"), ok);
	return ok;
}

// This functions performs an initial gyro offset calibration
// INCLUDING motion detection
// Board should be still for some seconds
void MPU6050::recalibrateSensor(void (*complain)(), uint8_t whichMotion) {
	uint8_t logicalAxis;
	uint8_t wait;
	uint16_t tolerance = whichMotion == GYRO ? 64 : 280                                                                                                                                            ;
#define SENSOR_ITERATIONS 1000
	int16_t prevSensor[3];
	int16_t* sensorSrc = whichMotion==GYRO ? gyro : acc;
	int16_t sensor[3];
	int32_t sensorSums[3];
	int calibGCounter = SENSOR_ITERATIONS;

	while (calibGCounter > 0) {
		bool motionDetected = false;
		//wdt_reset();
		if (calibGCounter == SENSOR_ITERATIONS) {
			for (wait = 0; wait < 70; wait++) { // wait 0.7sec if calibration failed
				_delay_ms(10);
			}

			cli();
			memcpy(sensor, sensorSrc, 6);
			sei();
			for (logicalAxis = 0; logicalAxis < 3; logicalAxis++) {
				sensorSums[logicalAxis] = 0;
				prevSensor[logicalAxis] = sensor[logicalAxis];
			}
		}

		cli();
		memcpy(sensor, sensorSrc, 6);
		sei();

		//wdt_reset();
		for (logicalAxis = 0; logicalAxis < 3; logicalAxis++) {
			if (abs16(prevSensor[logicalAxis] - sensor[logicalAxis]) > tolerance) {
				motionDetected = true;
				break;
			}
			sensorSums[logicalAxis] += sensor[logicalAxis];
			prevSensor[logicalAxis] = sensor[logicalAxis];
		}

		calibGCounter--;
		if (motionDetected) {
			//wdt_reset();
			complain();
			calibGCounter = SENSOR_ITERATIONS;
		}
	}

	// put result into integer
	for (logicalAxis=0; logicalAxis<3; logicalAxis++) {
		int8_t sign;
		// We don't want to offset away gravity on Z.
		uint8_t physicalOffset;
		if (whichMotion == ACC) {
			sign = sensorDef.acc[logicalAxis].dir;
			physicalOffset = sensorDef.acc[logicalAxis].idx;
		} else {
			sign = sensorDef.gyro[logicalAxis].dir;
			physicalOffset = sensorDef.gyro[logicalAxis].idx + 4;
		}
		if (whichMotion == ACC && logicalAxis == sensorDef.acc[Z].idx) sensorOffset[physicalOffset] = 0;
		else sensorOffset[physicalOffset] += sign * (sensorSums[logicalAxis] + SENSOR_ITERATIONS/2) / SENSOR_ITERATIONS;
	}

	saveSensorCalibration();
}
