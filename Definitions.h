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
#define RC_FREQ 250
#define RC_LATCH (MEDIUMLOOP_FREQ/RC_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define SOFTSTART_FREQ 100
#define SOFTSTART_LATCH (MEDIUMLOOP_FREQ/SOFTSTART_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define HUMAN_DEBUG_FREQ 5
#define HUMAN_DEBUG_LATCH (MEDIUMLOOP_FREQ/HUMAN_DEBUG_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
// We can send about 10k chars / sec, that is 250 lines/sec.
// However we are almost out of steam.
#define GUI_DEBUG_FREQ 10
#define GUI_DEBUG_LATCH (MEDIUMLOOP_FREQ/GUI_DEBUG_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define ACCMAG_FREQ 5
#define ACCMAG_LATCH (MEDIUMLOOP_FREQ/ACCMAG_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define UI_DEBUG_FREQ 20

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define LED_FREQ 10
#define LED_LATCH (MEDIUMLOOP_FREQ/LED_FREQ)

// Must be an integral fraction of MEDIUMLOOP_FREQ
#define OSCILLATION_FREQ 250
#define OSCILLATION_LATCH (MEDIUMLOOP_FREQ/OSCILLATION_FREQ)

#define POUT_FREQ 4      // rate of ACC print output in Hz, 25 Hz is default
#define LOCK_TIME_SEC 0  // gimbal fast lock time at startup

// LP filter coefficient
// #define LOWPASS_K_FLOAT(TAU) (MEDIUMLOOP_DT_F_S/(TAU+MEDIUMLOOP_DT_F_S))

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

#define MIN_RC (1100*16)
#define MID_RC (1500*16)
#define MAX_RC (1900*16)
#define RC_DEADBAND (50*16)

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

// Some LED event masks. They are supposed to be set by event, and automatically be reset after some time by LED driver.
#define LED_I_LIMIT_MASK 1
#define LED_SCHEDULER_OVERLOAD_MASK 2
#define LED_SPEED_LIMIT_MASK 4
#define LED_SERIAL_RX 8
#define LED_I2C_TIMEOUT_MASK 16
#define LED_RC_MASK 32
#define LED_RC_MISSED 64
#define LED_HEARTBEAT_MASK 128

#define TO_NERD_DEGREES(a) ((int16_t)(a) * 65536L / 360L)
#define FROM_NERD_DEGREES(a) ((a) * 360L / 65536L)

// Whether the motors should be turned on or off. This should get a power ramp routine in the
// medium task to ramp up or down the softstart.
// When changing this, always clear POWER_RAMPING_COMPLETE and wait for it to come true again.
// If medium loop does not run, this will not happen automatically.
#define MOTORS_POWERED 1
// Set to get the synced interrupt handler to output to hardware.
// Should be set after the pwm-out table was updated. The timer1 interrupt reads, outputs and resets.
// #define MOTORS_NEW_DATA 2
// Whether softstart ramping has completed
#define POWER_RAMPING_COMPLETE 2
// Whether PIDs are output to the PWM-out table
#define PIDS_ARE_OUTPUT 4
// Whether fast and medium loops should run (reason not to: They use I2C and expect being exclusive)
#define BACKGROUND_TASKS_RUN 8
// When BACKGROUND_TASKS_RUN is cleared, setting this will cause another routine to run in the interrupt.
#define SETUP_TASK_RUNS 16
#define SETUP_AXIS 32
#define SETUP_RESET 64

#define GIMBAL_STATE_RUN 	(MOTORS_POWERED | PIDS_ARE_OUTPUT | BACKGROUND_TASKS_RUN)
#define GIMBAL_STATE_FREEZE (MOTORS_POWERED | BACKGROUND_TASKS_RUN)
#define GIMBAL_STATE_SENSORCAL 0
#define GIMBAL_STATE_AUTOCAL (MOTORS_POWERED | AUTOCONF_TASK_RUNS)

// Start gimbal: Set MOTORS_POWERED, PIDS_ARE_OUTPUT and BACKGROUND_TASKS_RUN.
// Freeze gimbal: Set MOTORS_POWERED and BACKGROUND_TASKS_RUN. clear PIDS_ARE_OUTPUT
// Calibrate gimbal etc: Clear MOTORS_POWERED, wait till POWER_RAMPING_COMPLETE set, then clear BACKGROUND_TASKS_RUN.
//   I2C is now available
// Test-run for autoconf: clear BACKGROUND_TASKS_RUN. From test-run routine, set MOTORS_NEW_DATA whenever
//   a new output value from the test run was output. Possible refinement: In timer1 interrupt handler, when
//   BACKGROUND_TASKS_RUN is false. check for another flag to run other tasks instead.

#define TARGET_SOURCE_RC 0
#define TARGET_SOURCE_OSC 1
#define TARGET_SOURCE_AUTOCAL 2

#define INTERFACE_STATE_CONSOLE 0
#define INTERFACE_STATE_AUTOSETUP 1
#define INTERFACE_STATE_MAVLINK 2

// maybe about 10 degrees/s. That is 70 (or 72) e-degrees a sec, or 72*256/1000 microsteps per millisecond. About 18
#define SETUP_MOVE_DIVIDER 20
// 15 degrees, or about 100 e-degrees, or about (wtf?) 71 microsteps
#define SETUP_MOVE_LIMIT 80

#define SUPPORT_AUTOSETUP 1
#define SUPPORT_MAVLINK 1
