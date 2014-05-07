Any terminal can be used. 115200 N 8 1. CR/LF settings are not critical. Enable local echo
for convenience, but remember the backspace key does not do as expected.

Neither commands nor parameter names are case sensitive.
Boolean parameter values can be given as 0, 1, true or false.

######################################
List of serial commands:
######################################
sd    (Set Defaults)
we    (Writes active config to eeprom)
re    (Restores values from eeprom to active config)
cal   (Recalibrates the gyro offsets)
level (Recalibrates the accelerometer offsets)
par <parName> <parValue>   (general parameter read/set command)
    example usage:
       par                     ... list all config parameters (output can be saved and replayed)
       par pitchKi  	       ... list pitchKi
       par pitchKi 500   	   ... set pitchKi to 500
help   (print this output)

osc <n> Makes gimbal oscillate (like a fan) with speed n (0 to stop)
trans <pitch|roll|both> <n>	Makes gimbal jump back and forth on axes, helps detect instability
debug <category> 	Debug various stuff. Type debug help to see categories available
perf	Print performance info (if enabled in build)

+ some undocumented commands will be added for a UI tool (but they willnot do anything that could not
have beed done with the documented ones)

######################################
Config Parameters (list with "par")
######################################
All printout is prefixed by the "par" command.
To save parameters, simply run the "par" command and save output to a file.
To replay, send the contents of the file.

vers
  firmware version

pitchKp/rollKp
  pid controller P-value
pitchKi/rollKi
  pid controller I-value
pitchKd/RollKd
  pid controller D-value
ILimit
  maximum error to integrate (this is kind of a PID expert thing. Just leave default if you are not one)
pitchRateLimit/rollRateLimit
  Limits on how fast the controller should attempt to turn the motors. Intended to prevent out-of-sync runaways.

accTime
  time constant of ACC complementary filter.
  controls how fast the gimbal follows ACC.
  Set low to allow fast recovery after being knocked off lock.
  Set high to make insensitive against airframe acceleration.
  unit = 1 sec, e.g. 7 = 7 seconds
pitchPower/rollPower
  How much power to apply to pitch and roll motors (0..255).

pitchMin/rollMin
  RC minimum set point angle, unit = Nerd Degrees (65536/360 degrees, eg. 16384 = 90 degrees) (will be changed to degrees some time)
pitchMax/rollMax
  RC maximum set point angle, unit = Nerd Degrees (65536/360 degrees, eg. 16384 = 90 degrees) (will be changed to degrees some time)
pitchDefault/rollDefault
  Set point angles for use without RC
pitchSpeed/rollSpeed
  Speed or camera rotation in RC Relative Mode
  
rcAbsolute
  false ... RC Relative Mode, gimbal position is incremented/decremented by RC
  true  ... RC Absolute Mode, RC controls gimbal directly

majorAxis
  Sensor board orientation.
  0 = MPU6050 chip face up or down
  1 = MPU6050 chip standing on side (text on chip standing on end)
  2 = MPU6050 chip standing on end ((text on chip normal, like a roadsign)
reverseZ
  If pitch reads near 180 or -180 degrees, set this to opposite value
swapXY
  0 ... standard X/Y sensor orientation
  1 ... swap X/Y, exchange roll/pitch function, when sensor is rotated 90 degrees
LEDMask
  Function of on-board LED (if any):
  1:   Indicate ILimit overrun
  2:   CPU overload (too many tasks to do)
  4:   pitchRateLimit/rollRateLimit reached
  8:   N/A
  16:  I2C error
  32:  N/A
  64:  N/A
  128: Heartbeat blink (default)
