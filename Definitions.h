/*************************/
/* Definitions           */
/*************************/
// FOR CHANGES PLEASE READ: ReleaseHistory.txt
#define VERSION_STATUS B // A = Alpha; B = Beta , N = Normal Release
#define VERSION 100
#define VERSION_EEPROM 0 // change this number when eeprom data strcuture has changed

// MPU Address Settings
// TODO: Not supported right now.
#define MPU6050_ADDRESS_AD0_LOW     0x68 // default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // Drotek MPU breakout board
//#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_HIGH

#define FASTLOOP_FREQ 1050
#define FASTLOOP_DT_F_S  (1.0/FASTLOOP_FREQ)   	    // loop controller sample period dT
#define FASTLOOP_DT_I_MS (1000/FASTLOOP_FREQ)

// Must be an integral fraction of FASTLOOP_FREQ
#define MEDIUMLOOP_FREQ 525
#define MEDIUMLOOP_DT_F_S  (1.0/MEDIUMLOOP_FREQ)       // loop controller sample period dT
#define MEDIUMLOOP_DT_I_MS (1000/MEDIUMLOOP_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define RC_FREQ 100
#define RC_LATCH (MEDIUMLOOP_FREQ/RC_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define SOFTSTART_FREQ 80
#define SOFTSTART_LATCH (MEDIUMLOOP_FREQ/SOFTSTART_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define HUMAN_DEBUG_FREQ 5
#define HUMAN_DEBUG_LATCH (MEDIUMLOOP_FREQ/HUMAN_DEBUG_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define ACCMAG_FREQ 5
#define ACCMAG_LATCH (MEDIUMLOOP_FREQ/ACCMAG_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define UI_DEBUG_FREQ 20

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define LED_FREQ 10
#define LED_LATCH (MEDIUMLOOP_FREQ/LED_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define OSCILLATION_FREQ 50
#define OSCILLATION_LATCH (OSCILLATION_FREQ/SOFTSTART_FREQ)

#define POUT_FREQ 4      // rate of ACC print output in Hz, 25 Hz is default
#define LOCK_TIME_SEC 0  // gimbal fast lock time at startup

// LP filter coefficient
#define LOWPASS_K_FLOAT(TAU) (MEDIUMLOOP_DT_F_S/(TAU+MEDIUMLOOP_DT_F_S))

// Number of sinus values for full 360 deg.
// NOW FIXED TO 256 !!!
// Reason: Fast Motor Routine using uint8_t overflow for stepping
#define N_SIN 256

// RC data size and channel assigment
#define RC_DATA_SIZE  3
#define RC_DATA_PITCH 0
#define RC_DATA_ROLL  1
#define RC_DATA_SWITCH  2

// RC PPM pin A0, A1 or A2
// #define RC_PIN_PPM_A2
// #define RC_PIN_PPM_A1
// #define RC_PIN_PPM_A0

#define MIN_RC 1050
#define MID_RC 1500
#define MAX_RC 1950
#define RC_DEADBAND 50U
#define RC_TIMEOUT 1000000UL

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

#define GIMBAL_ON_DELAY 10
#define DEBUG_DELAY 255

// Just a binary switch for retract.
#define SW_UNKNOWN 0
#define SW_RETRACTED -1
#define SW_EXTENDED 1

#define CONFIGNAME_MAX_LEN 17

#define WDT_TIMEOUT WDTO_1S

// Let's use int16-degrees.
// -pi-->-(1<<15) to pi->(1<<15)-1
#define ANGLE_SCALING (32768.0/M_PI)
