
Arduino -> Tools -> Serial Monitor Settings
  Baudrate: 11200 Baud, NL and CR

######################################
List of serial commands:
######################################

these are the preferred commands, use them for new GUIs !!

sd    (Set Defaults)
we    (Writes active config to eeprom)
re    (Restores values from eeprom to active config)
cal   (Recalibrates the gyro Offsets)
level (Recalibrates the accelerometer Offsets)
par <parName> <parValue>   (general parameter read/set command)
    example usage:
       par                     ... list all config parameters
       par gyroPitchKi         ... list gyroPitchKi
       par gyroPitchKi 12000   ... set gyroPitchKi to 12000

perf	Print performance info (if enabled in build)
debug	Debug various stuff: acc, gyro, estg (estimated gravity vector), att (attitude)
help   (print this output)

######################################
Config Parameters (list with "par")
######################################
All printout is prefixed by the "par" command.
To save parameters, simply run the "par" command and save output to a file.
To replay, send the contents of the file.

vers
  firmware version
versEEPROM
  eeprom version

pitchKp/rollKp
  pid controller P-value
pitchKi/rollKi
  pid controller I-value
pitchKd/RollKd
  pid controller D-value
accTimeConst
  time constant of ACC complementary filter.
  controls how fast the gimbal follows ACC.
  Set low to allow fast recovery after being knocked off lock.
  Set high to make insensitive against airframe acceleration.
  unit = 1 sec, e.g. 7 = 7 seconds
mpuFilter
  low pass filter of gyro (DLPFMode)
  legal values are 0...6, 0=fastest 6=slowest
  use slow values if high frequency oscillations occur (still experimental)
  (only 0 seems useful)

pitchOffset/rollOffset
  offset of gimbal zero position
  unit = 0.01 deg, e.g. 500 = 5.00 deg

pitchDir/rollDir
  motor direction
  1 = normal, 0 = disable motor, -1 = reverse direction
  (normal or reverse seems to not matter at all!)
pitchMotor/rollMotor
  assign motor output for pitch and roll, legal values are 0 or 1
pitchPower/rollPower
  motor power, legal range 0 to 255

minRCPitch/minRCRoll
  RC minimum set point angle, unit = 1 deg
maxRCPitch/maxRCRoll
  RC maximum set point angle, unit = 1 deg
rcGain
  RC gain in Relative mode, specifies speed of gimbal movement
rcFilter
  RC low pass filter in Absolute mode, specified speed of gimbal movement
  unit = 0.1 sec, e.g. 20 = 2.0 seconds

rcModePPM
  0 ... use two RC PWM inputs on A0 and A1
  1 ... use PPM sum input on A0

rcChannelPitch
   RC channel assignment for RC pitch, legal values are 0 to 7 in PPM mode
rcChannelRoll
   RC channel assignment for RC roll, legal values are 0 to 7 in PPM mode

rcMid
   RC center position, unit = 1 msec, default=1500
rcAbsolute
   0 ... RC Relative Mode, gimbal position is incremented/decremented by RC
   1 ... RC Absolute Mode, RC controls gimbal directly

majorAxis
	Sensor board orientation.
	0 = MPU6050 chip face up or down
	1 = MPU6050 chip standing on side (text on chip standing on end)
	2 = MPU6050 chip standing on end ((text on chip normal, like a roadsign)

reverseZ
   0 ... If pitch reads near 180 or -180 degrees, set to 1 instead
   0 ... If pitch reads near 180 or -180 degrees, set to 0 instead
swapXY
   0 ... standard X/Y sensor orientation
   1 ... swap X/Y, exchange roll/pitch function, when sensor is rotated 90 degrees

   