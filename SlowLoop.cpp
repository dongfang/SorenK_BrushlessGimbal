#include "Globals.h"
#include "Definitions.h"
#include "Board.h"
#include "RCdecode.h"
#include "Mavlink.h"
#include "Commands.h"

uint8_t interfaceState;
uint8_t LEDFlags;

// This is supposed to read the switch.
// This pretty outdated by now.. find another way
void updateSwitchEffect() {
	static int8_t prevSwitch = SW_UNKNOWN;
	// if (switchPos < 0) balahblah .. implement switch to state logic here.
	// except when in autosetup, oops.
	if (prevSwitch == SW_UNKNOWN) {
		// switch was never defined, default to running
		// TODO - this is not desirable in normal operation. Will extend on ground.
		run();
		prevSwitch = 0;
	}
	if (switchPos != SW_UNKNOWN && switchPos != prevSwitch) {
		if (switchPos == SW_UP && interfaceState != INTERFACE_STATE_AUTOSETUP) {
			run(); // implies extend
		} else if (switchPos == SW_DOWN && interfaceState != INTERFACE_STATE_AUTOSETUP) {
			retract();
		} else if (switchPos == SW_DOWN && interfaceState != INTERFACE_STATE_AUTOSETUP) {
			freeze();
		}
		prevSwitch = switchPos;
	}
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
extern void debug();

void slowLoop() {
	static uint8_t rcDivider;
	static uint8_t humanDebugDivider;
	static uint8_t GUIDebugDivider;
	//static uint8_t RCDivider;
	//static uint8_t softstartDivider;
	static uint8_t LEDDivider;
	static uint16_t heartbeatDivider;
	//static uint8_t accMagDivider;
	static uint8_t oscDivider;
	static uint16_t transientsDivider;
	static uint8_t mavlinkMountStatusSubdivider;
	static uint8_t mavlinkTrackingDivider;

	while (true) {
		bool ticked = checkMediumLoop();

		/* No need for this, it did more harm than good.
		 if (ticked && !accMagDivider) {
		 accMagDivider = ACCMAG_LATCH;
		 imu.updateAccMagnitude();
		 }
		 */
		if (ticked && !rcDivider) {
			// Evaluate RC-Signals. Out of laziness, we ignore these if MAVLink is controlling.
			// if (interfaceState != INTERFACE_STATE_MAVLINK) {
			rcDivider = RC_LATCH;
			evaluateRCControl();
			evaluateRCSwitch();
			// }
			updateSwitchEffect();
		}

#ifdef SUPPORT_MAVLINK
		if (mavlink_parse()) {
			LEDEvent(LED_MAVLINK_RX);
		}

		// Apply autodetect
		if (mavlinkDetected) goMavlink();
#endif

		// These are not synced with medium task (ticked is not checked)
		if (interfaceState == INTERFACE_STATE_CONSOLE)
			sCmd.readSerial();
#ifdef SUPPORT_AUTOSETUP
		else if (interfaceState == INTERFACE_STATE_AUTOSETUP)
			runAutosetup();
#endif

		if (interfaceState == INTERFACE_STATE_CONSOLE && ticked && !humanDebugDivider) {
			humanDebugDivider = HUMAN_DEBUG_LATCH;
			debug();
		}

		if (interfaceState == INTERFACE_STATE_GUI && ticked && !GUIDebugDivider) {
			GUIDebugDivider = GUI_DEBUG_LATCH;
			GUIDebug();
		}

		if (ticked && !heartbeatDivider) {
			heartbeatDivider = HEARTBEAT_LATCH;
			LEDEvent(LED_HEARTBEAT_MASK);
			if (interfaceState == INTERFACE_STATE_MAVLINK) {
				mavlink_sendHeartbeat();
				if (mavlinkMountStatusSubdivider == 5) {
					mavlink_sendStatus();
					mavlinkMountStatusSubdivider = 0;
				} else {
					mavlinkMountStatusSubdivider++;
				}
			}
		}

		// Let mavlink receive a very high prio.
		if (ticked /* && !mavlinkTrackingDivider */) {
			mavlinkTrackingDivider = MAVLINK_TRACKING_LATCH;
			if (interfaceState == INTERFACE_STATE_MAVLINK) {
				mavlink_update();
			}
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
			--rcDivider;
			--humanDebugDivider;
			--GUIDebugDivider;
			--LEDDivider;
			--heartbeatDivider;
			--oscDivider;
			--transientsDivider;
			--mavlinkTrackingDivider;
		}

		doubleFault = false; // If we have run the mainloop successfully, we reset the double WDT fault status.
	}
}
