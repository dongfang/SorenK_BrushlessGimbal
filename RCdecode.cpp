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
			rcData[RC_DATA_ROLL].m_16 = frozenTime;
			rcData[RC_DATA_ROLL].pulseComplete = false;
		} else {
			rcData[RC_DATA_ROLL].m_16 = frozenTime - rcData[RC_DATA_ROLL].m_16;
			rcData[RC_DATA_ROLL].pulseComplete = true;
		}
	}
	if (change & 2) {
		if (pin & 2) {
			rcData[RC_DATA_PITCH].m_16 = frozenTime;
			rcData[RC_DATA_PITCH].pulseComplete = false;
		} else {
			rcData[RC_DATA_PITCH].m_16 = frozenTime - rcData[RC_DATA_PITCH].m_16;
			rcData[RC_DATA_PITCH].pulseComplete = true;
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
void evalRCChannelIntegrating(RCData_t* rcData, RCChannelDef* def) {
	cli();
	uint16_t rx = rcData->m_16;
	sei();
	int16_t live = rx - MID_RC - RC_DEADBAND;
	if (live <= 0) {
		live = rx - MID_RC + RC_DEADBAND;
		if (live >= 0)
			return;
	}

	rcData->setpoint += (int32_t)def->speed * live / 16384L;

	if (rcData->setpoint > def->maxAngle)
		rcData->setpoint = def->maxAngle;
	if (rcData->setpoint < def->minAngle)
		rcData->setpoint = def->minAngle;
}

//******************************************
// Absolute
// This tracks quite slowly but there is
// more jitter in RC than in anything else
// in this system. Got to find a way to
// eliminate that.
//******************************************
void evalRCChannelAbsolute(RCData_t* rcData, RCChannelDef* def) {
	static int16_t pladder;

	cli();
	uint16_t rx = rcData->m_16;
	sei();
	int16_t result = rx - MID_RC;
	result = (result + pladder*1023L)/1024L;
	pladder = result;
	result += (def->maxAngle-def->minAngle)/2;

	if (result > def->maxAngle)
		result = def->maxAngle;
	if (result < def->minAngle)
		result = def->minAngle;

	rcData->setpoint = result;
}

void evaluateRCControl() {
	if (rcData[RC_DATA_ROLL].pulseComplete) {
		rcData[RC_DATA_ROLL].timeout = 0;
		if (config.rcAbsolute)
			evalRCChannelAbsolute(&rcData[RC_DATA_ROLL], &config.RCRoll);
		else
			evalRCChannelIntegrating(&rcData[RC_DATA_ROLL], &config.RCRoll);
	} else {
		if (rcData[RC_DATA_ROLL].isTimedOut())
			rcData[RC_DATA_ROLL].setpoint = config.RCRoll.defaultAngle;
		else
			rcData[RC_DATA_ROLL].timeout++;
	}

	if (rcData[RC_DATA_PITCH].pulseComplete) {
		rcData[RC_DATA_PITCH].timeout = 0;
		if (config.rcAbsolute)
			evalRCChannelAbsolute(&rcData[RC_DATA_PITCH], &config.RCPitch);
		else
			evalRCChannelIntegrating(&rcData[RC_DATA_PITCH], &config.RCPitch);
	} else {
		if (rcData[RC_DATA_PITCH].isTimedOut())
			rcData[RC_DATA_PITCH].setpoint = config.RCPitch.defaultAngle;
		else
			rcData[RC_DATA_PITCH].timeout++;
	}
}

/*
 * We don't care what the meanings are of the switch pos's here.
 * It is simply -1 or 1 after evaluation.
 */
void evaluateRCSwitch() {
	if (!rcData[RC_DATA_SWITCH].timeout > 0) {
		cli();
		uint16_t rx = rcData->m_16;
		sei();
		// Do something predictable, such as setting it default-locked.
		uint16_t lThreshold = MIN_RC/2 + MID_RC/2;
		if (switchPos < 0)
			lThreshold += RC_DEADBAND;
		uint16_t hThreshold = MID_RC/2 + MAX_RC/2;
		if (switchPos > 0)
			hThreshold -= RC_DEADBAND;
		if (rx <= lThreshold)
			switchPos = -1;
		else if (rx >= hThreshold)
			switchPos = 1;
		else
			switchPos = 0;
	}
	else if (rcData[RC_DATA_SWITCH].isTimedOut())
		switchPos = 1;
	else
		rcData[RC_DATA_SWITCH].timeout--;
}
