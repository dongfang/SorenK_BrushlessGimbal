#include "Globals.h"
#include <avr/interrupt.h>

//******************************************
// PPM Decoder
//******************************************
/*
 inline void intDecodePPM() {
 uint32_t ticksNow = _timer();

 static int32_t ticksPPMLastEdge = 0;
 uint16_t pulseInPPM;

 static uint8_t channel_idx = 0;

 pulseInPPM = (ticksNow - ticksPPMLastEdge)/CC_FACTOR;
 ticksPPMLastEdge = ticksNow;

 if (pulseInPPM > RC_PPM_GUARD_TIME) {
 channel_idx = 0;
 } else if (channel_idx < RC_PPM_RX_MAX_CHANNELS) {
 rcData_t* data = 0;
 if (channel_idx == config.rcChannelPitch)
 data = &rcData[RC_DATA_PITCH];
 else if (channel_idx == config.rcChannelRoll)
 data = &rcData[RC_DATA_ROLL];
 if (data) {
 data->microsLastUpdate = ticksNow;
 if ((pulseInPPM >= MIN_RC) && (pulseInPPM <= MAX_RC)) {
 data->rx     = pulseInPPM;
 data->isValid  = true;
 data->isFresh = true;
 }
 }
 channel_idx++;
 }
 }
 */

//******************************************
// Interrupts
//******************************************
ISR(PCINT1_vect) {
	static uint8_t lastPin;
	uint16_t frozenTime = time();
	uint8_t pin = PINC;
	uint8_t change = pin ^ lastPin;
	lastPin = pin;
	// If PPM is desired, make an check and branch here.
	if (change & 1) {
		if (pin & 1) {
			rcData[ROLL].m_16 = frozenTime;
			rcData[ROLL].pulseComplete = false;
		} else {
			rcData[ROLL].m_16 = frozenTime - rcData[ROLL].m_16;
			rcData[ROLL].pulseComplete = true;
		}
	}
	if (change & 2) {
		if (pin & 2) {
			rcData[PITCH].m_16 = frozenTime;
			rcData[PITCH].pulseComplete = false;
		} else {
			rcData[PITCH].m_16 = frozenTime - rcData[PITCH].m_16;
			rcData[PITCH].pulseComplete = true;
		}
	}
	if (change & 4) {
		if (pin & 4) {
			rcData[RC_DATA_SWITCH].m_16 = frozenTime;
			rcData[RC_DATA_SWITCH].pulseComplete = false;
		} else {
			rcData[RC_DATA_SWITCH].m_16 = frozenTime - rcData[RC_DATA_SWITCH].m_16;
			rcData[RC_DATA_SWITCH].pulseComplete = true;
		}
	}
}

// initialize RC Pin mode
void initRC() {
	// TODO: We need a pin identity abstraction (which is not Arduino :) )
	//pinMode(A2, INPUT); digitalWrite(A2, HIGH);
	//pinMode(A1, INPUT); digitalWrite(A1, HIGH);
	//pinMode(A0, INPUT); digitalWrite(A0, HIGH);
	DDRC &= ~(1 << 0 | 1 << 1 | 1 << 2);
	PORTC |= 1 << 0 | 1 << 1 | 1 << 2;

	PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10);
	PCICR |= (1 << PCIE1);

	for (uint8_t id = 0; id < RC_DATA_SIZE; id++) {
		cli();
		rcData[id].timeout = 0;
		sei();
	}
}

//******************************************
// Integrating
//******************************************
void evalRCChannelIntegrating(uint8_t ch) {
	// RCData_t* rcData = rcData + ch;
	LiveControlAxisDef* def = liveControlDefs + ch;

	cli();
	uint16_t rx = rcData->m_16;
	sei();
	int16_t live = rx - MID_RC - RC_DEADBAND;
	if (live <= 0) { // below top of deadband
		live = rx - MID_RC + RC_DEADBAND;
		if (live >= 0) // in deadband
			return;
	}

	int16_t result = targetSources[TARGET_SOURCE_RC][ch] + (int32_t) def->maxSlewRate * live / (RC_RANGE / 2);

	if (result > def->maxAngleND)
		result = def->maxAngleND;
	if (result < def->minAngleND)
		result = def->minAngleND;

	// live is in the order [-500*16..500*16]
	targetSources[TARGET_SOURCE_RC][ch] = result;
}

//******************************************
// Absolute
// This tracks quite slowly but there is
// more jitter in RC than in anything else
// in this system. Got to find a way to
// eliminate that.
//******************************************
void evalRCChannelAbsolute(uint8_t ch) {
	static int16_t pladder;

	// RCData_t* rcData = rcData + ch;
	// LiveControlAxisDef* def = liveControlDefs + ch;LiveControlAxisDef* def = liveControlDefs + ch;

	cli();
	uint16_t rx = rcData[ch].m_16;
	sei();
	int16_t result = rx - MID_RC;
	result = (result + (int32_t)pladder * 1023L) / 1024L;
	pladder = result;

	result += liveControlDefs[ch].midAngleND;
	targetSources[TARGET_SOURCE_RC][ch] = result;
}

void evaluateRCControl() {
	uint8_t ch;

	for (ch = ROLL; ch <= PITCH; ch++) {
		if (rcData[ch].pulseComplete) {
			rcData[ch].timeout = 0;
			if (config.rcAbsolute)
				evalRCChannelAbsolute(ch);
			else
				evalRCChannelIntegrating(ch);
		} else {
			if (rcData[ch].isTimedOut())
				targetSources[TARGET_SOURCE_RC][ch] = liveControlDefs[ch].defaultAngleND;
			else
				rcData[ch].timeout++;
		}
	}
}

/*
 * We don't care what the meanings are of the switch pos's here.
 * It is simply -1 or 1 after evaluation.
 */
void evaluateRCSwitch() {
	RCData_t* swData = rcData + RC_DATA_SWITCH;
	if (swData->pulseComplete) {
		static uint16_t f16;
		cli();
		uint16_t rx = swData->m_16;
		sei();
		f16 = (f16 * 255L + rx) / 256;
		// Do something predictable, such as setting it default-locked.
		uint16_t lThreshold = MIN_RC / 2 + MID_RC / 2;
		uint16_t hThreshold = MID_RC / 2 + MAX_RC / 2;
		if (switchPos == 0) {
			lThreshold -= RC_SW_DEADBAND;
			hThreshold += RC_SW_DEADBAND;
		}
		if (f16 <= lThreshold)
			switchPos = -1;
		else if (f16 >= hThreshold)
			switchPos = 1;
		else
			switchPos = 0;
	} else {
		if (swData->isTimedOut())
			switchPos = 1;
		else
			swData->timeout++;
	}
}
