/*************************/
/* Definitions           */
/*************************/
// FOR CHANGES PLEASE READ: ReleaseHistory.txt
#define VERSION_STATUS B // A = Alpha; B = Beta , N = Normal Release
#define VERSION 49
#define VERSION_EEPROM 2 // change this number when eeprom data strcuture has changed

// MPU Address Settings
#define MPU6050_ADDRESS_AD0_LOW     0x68 // default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // Drotek MPU breakout board
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_HIGH

// Define Brushless PWM Mode, uncomment ONE setting
#define PWM_32KHZ_PHASE  // Resolution 8 bit for PWM
//#define PWM_8KHZ_FAST    // Resolution 8 bit for PWM
//#define PWM_4KHZ_PHASE   // Resolution 8 bit for PWM
//#define NO_PWM_LOOP

#define MOTORUPDATE_FREQ 500                // in Hz, 1000 is default // 1,2,4,8 for 32kHz, 1,2,4 for 4kHz
#define LOOPUPDATE_FREQ MOTORUPDATE_FREQ    // loop control sample rate equals motor update rate
#define DT_FLOAT (1.0/LOOPUPDATE_FREQ)      // loop controller sample period dT
#define DT_INT_MS (1000/MOTORUPDATE_FREQ)

#define POUT_FREQ 4      // rate of ACC print output in Hz, 25 Hz is default
#define LOCK_TIME_SEC 0  // gimbal fast lock time at startup

// LP filter coefficient
#define LOWPASS_K_FLOAT(TAU) (DT_FLOAT/(TAU+DT_FLOAT))

// Number of sinus values for full 360 deg.
// NOW FIXED TO 256 !!!
// Reason: Fast Motor Routine using uint8_t overflow for stepping
#define N_SIN 256

#define SCALE_ACC 10000.0
#define SCALE_PID_PARAMS 1000.0f

// RC data size and channel assigment
#define RC_DATA_SIZE  3
#define RC_DATA_PITCH 0
#define RC_DATA_ROLL  1
#define RC_DATA_SWITCH  2

// RC PPM pin A0, A1 or A2
// #define RC_PIN_PPM_A2
// #define RC_PIN_PPM_A1
// #define RC_PIN_PPM_A0

#define MIN_RC 950
#define MID_RC 1500
#define MAX_RC 2050
#define RC_DEADBAND 50U
#define RC_TIMEOUT 100000

// PPM Decoder
// #define RC_PPM_GUARD_TIME 4000
// #define RC_PPM_RX_MAX_CHANNELS 32


// Hardware Abstraction for Motor connectors,
// DO NOT CHANGE UNLES YOU KNOW WHAT YOU ARE DOING !!!
#define PWM_A_MOTOR1 OCR2A
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B

#ifdef PWM_32KHZ_PHASE
  #define CC_FACTOR 32
#endif
#ifdef PWM_4KHZ_PHASE
  #define CC_FACTOR 4
#endif
#ifdef PWM_8KHZ_FAST
  #define CC_FACTOR 8
#endif
#ifdef NO_PWM_LOOP
  #define CC_FACTOR 1
#endif

// enable stack and heapsize check (use just for debugging)
// #define STACKHEAPCHECK_ENABLE

#define ROLL 	0
#define PITCH 	1
#define YAW		2

#define X		0
#define Y		1
#define Z		2

#define GIMBAL_OFF 0
#define GIMBAL_FROZEN 1
#define GIMBAL_RUNNING 2

// Just a binary switch for retract.
#define SW_UNKNOWN 0
#define SW_RETRACTED -1
#define SW_EXTENDED 1

#define CONFIGNAME_MAX_LEN 17

#define WDT_TIMEOUT WDTO_250MS

#define ANGLE_SCALING 1000L
#define LOG_ANGLE_SCALING 3
