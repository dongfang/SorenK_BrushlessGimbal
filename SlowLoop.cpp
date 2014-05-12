#include "Globals.h"
#include "Definitions.h"
#include "Board.h"
#include "RCdecode.h"

uint8_t interfaceState;

// This is supposed to read the switch.
void updateGimbalState() {
	// if (switchPos < 0) balahblah .. implement switch to state logic here.
	// except when in autosetup, oops.
	if (interfaceState != INTERFACE_STATE_AUTOSETUP)
		gimbalState = GIMBAL_STATE_RUN;
}

inline bool checkMediumLoop() {
	uint8_t sreg = SREG;
	cli();
	bool result = mediumTaskHasRun;
	mediumTaskHasRun = false;
	SREG = sreg;
	return result;
}

extern void transientTask();
extern void oscillationTask();
extern void runAutosetup();

void slowLoop() {
	static uint8_t humanDebugDivider;
	static uint8_t GUIDebugDivider;
	//static uint8_t RCDivider;
	//static uint8_t softstartDivider;
	static uint8_t LEDDivider;
	static uint8_t heartbeatDivider;
	//static uint8_t accMagDivider;
	static uint8_t oscDivider;
	static uint16_t transientsDivider;

	while (true) {
		bool ticked = checkMediumLoop();

		/* No need for this, it did more harm than good.
		 if (ticked && !accMagDivider) {
		 accMagDivider = ACCMAG_LATCH;
		 imu.updateAccMagnitude();
		 }
		 */
		if (ticked) {
			// RCDivider = RC_LATCH;
			// Evaluate RC-Signals
			evaluateRCControl();
			evaluateRCSwitch();
			updateGimbalState();
		}

		/*
		int c = serial0.peek();
		if (c >= 0)
			printf_P(PSTR("Peek %d\r\n"), c);
			*/

		if (interfaceState == INTERFACE_STATE_CONSOLE)
			sCmd.readSerial();
		else if (SUPPORT_AUTOSETUP && interfaceState == INTERFACE_STATE_AUTOSETUP)
			runAutosetup();
		else if (interfaceState == INTERFACE_STATE_MAVLINK) ;

		/*
		if (ticked && !softstartDivider) {
			softstartDivider = SOFTSTART_LATCH;
			updateSoftStart();
		}
		*/

		if (ticked && !humanDebugDivider) {
			humanDebugDivider = HUMAN_DEBUG_LATCH;
			debug();
		}

		if (ticked && !GUIDebugDivider) {
			GUIDebugDivider = GUI_DEBUG_LATCH;
			GUIDebug();
		}

		if (ticked && !heartbeatDivider) {
			LEDEvent(LED_HEARTBEAT_MASK);
			heartbeatDivider = LED_LATCH*2;
		}

		if (ticked && !LEDDivider) {
			LEDDivider = LED_LATCH;
			if (LEDFlags & config.LEDMask) {
				LED_PORT |= (1<<LED_BIT);
				LEDFlags &= ~config.LEDMask;
			} else {
				LED_PORT &= ~(1<<LED_BIT);
			}
		}

		if (ticked && !oscDivider) {
			oscDivider = OSCILLATION_LATCH;
			oscillationTask();
		}

		if (ticked && !transientsDivider) {
			transientsDivider = MEDIUMLOOP_FREQ; // 1Hz
			transientTask();
		}
#ifdef STACKHEAPCHECK_ENABLE
		stackHeapEval(false);
#endif

		if (ticked) {
			// --accMagDivider;
			--humanDebugDivider;
			--GUIDebugDivider;
			//--softstartDivider;
			--LEDDivider;
			--heartbeatDivider;
			--oscDivider;
			--transientsDivider;
		}

		doubleFault = false; // If we have run the mainloop successfully, we reset the double WDT fault status.
	}
}
