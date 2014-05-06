#include "Globals.h"
#include <avr/interrupt.h>

/*************************/
/* RC-Decoder            */
/*************************/

//******************************************
// PWM Decoder
//******************************************
void decodePWM(RCData_t* rcData, uint16_t microsFallingEdge) {
  uint16_t pulseInPWMtmp;
  pulseInPWMtmp = (microsFallingEdge - rcData->microsRisingEdge)/(F_CPU/1000000UL);
  // update if within expected RC range
  rcData->rx = pulseInPWMtmp;
  rcData->timeout = 0;
  if ((pulseInPWMtmp >= MIN_RC) && (pulseInPWMtmp <= MAX_RC)) {
    rcData->isValid=true;
  }
}

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
     rcData[RC_DATA_ROLL].microsRisingEdge = frozenTime;
    } else {
      decodePWM(&rcData[RC_DATA_ROLL], frozenTime);
    }
  }
  if (change & 2) {
    if (pin & 2) {
      rcData[RC_DATA_PITCH].microsRisingEdge = frozenTime;
    } else {
      decodePWM(&rcData[RC_DATA_PITCH], frozenTime);
    }
  }
  if (change & 4) {
    if (pin & 4) {
      rcData[RC_DATA_SWITCH].microsRisingEdge = frozenTime;
    } else {
      decodePWM(&rcData[RC_DATA_SWITCH], frozenTime);
    }
  }
}

//******************************************
// PPM & PWM Decoder
//******************************************
void checkRcTimeouts() {
  //uint32_t timerNow = time32();
  //uint16_t timerLastUpdate;
  for (uint8_t id = 0; id < RC_DATA_SIZE; id++) {
    if (rcData[id].timeout < 250) { // stale
    	++rcData[id].timeout;
    } else {
    	rcData[id].isValid = false;
    }
  }
}

// initialize RC Pin mode
void initRC() {
	// TODO: We need a pin identity abstraction (which is not Arduino :) )
	//pinMode(A2, INPUT); digitalWrite(A2, HIGH);
    //pinMode(A1, INPUT); digitalWrite(A1, HIGH);
    //pinMode(A0, INPUT); digitalWrite(A0, HIGH);
	DDRC &= ~( 1<<0 | 1<<1 | 1<<2);
	PORTC |= 1<<0 | 1<<1 | 1<<2;

    PCMSK1 |= (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10);
    PCICR |= (1<<PCIE1);

  for (uint8_t id = 0; id < RC_DATA_SIZE; id++) {
    cli();
    rcData[id].isValid = false;
    sei();
  }
}

//******************************************
// Integrating
//******************************************
void evalRCChannelIntegrating(RCData_t* rcData, RCChannelDef* def) {
	int16_t live = rcData->rx - MID_RC - RC_DEADBAND;
    if (live <= 0) {
    	live = rcData->rx - MID_RC + RC_DEADBAND;
        if (live >= 0) return;
    }

    rcData->setpoint += def->speed * live / ((MAX_RC-MIN_RC)/2);
    if (rcData->setpoint > def->maxAngle) rcData->setpoint = def->maxAngle;
    if (rcData->setpoint < def->minAngle) rcData->setpoint = def->minAngle;
}


//******************************************
// Absolute
//******************************************
void evalRCChannelAbsolute(RCData_t* rcData, RCChannelDef* def) {
	static int32_t pladder;
	// Typically in the range 16000/1000, about 16 for a 90 degree setting, less for less
	// Not really precise but who cares.
    // defaultAngle is our center point. NO, it makes no sense. Drop it.

	int32_t result = def->maxAngle + ((int32_t)rcData->rx-MID_RC) * ((int32_t)def->maxAngle - def->minAngle) / (MAX_RC - MIN_RC);

	result = (result + pladder * 15) / 16;
	pladder = result;

    // Check endpoints again
    if (result > def->maxAngle) result = def->maxAngle;
    if (result < def->minAngle) result = def->minAngle;

    if (result > rcData->setpoint + def->speed) rcData->setpoint += def->speed;
    else if (result < rcData->setpoint - def->speed) rcData->setpoint -= def->speed;
    else rcData->setpoint = result;
}

void evaluateRCControl() {
	if (rcData[RC_DATA_ROLL].isValid) {
		if (config.rcAbsolute)
			  evalRCChannelAbsolute(&rcData[RC_DATA_ROLL], &config.RCRoll);
		else
			  evalRCChannelIntegrating(&rcData[RC_DATA_ROLL], &config.RCRoll);
	} else rcData[RC_DATA_ROLL].setpoint = config.RCRoll.defaultAngle;

	if (rcData[RC_DATA_PITCH].isValid) {
		if (config.rcAbsolute)
			  evalRCChannelAbsolute(&rcData[RC_DATA_PITCH], &config.RCPitch);
		else
			  evalRCChannelIntegrating(&rcData[RC_DATA_PITCH], &config.RCPitch);
	} else rcData[RC_DATA_PITCH].setpoint = config.RCPitch.defaultAngle;
}

/*
 * We don't care what the meanings are of the switch pos's here.
 * It is simply -1 or 1 after evaluation.
 */
void evaluateRCSwitch() {
	if (!rcData[RC_DATA_SWITCH].isValid) {
		// Do something predictable, such as setting it default-locked.
	}
  uint16_t lThreshold = (MIN_RC + MID_RC)/2;
  if (switchPos < 0) lThreshold += RC_DEADBAND;
  uint16_t hThreshold = (MID_RC + MAX_RC)/2;
  if (switchPos > 0) hThreshold -= RC_DEADBAND;
  uint16_t sp = rcData[RC_DATA_SWITCH].rx;
  if (sp <= lThreshold)
    switchPos = -1;
  else if (sp >= hThreshold)
    switchPos = 1;
  else 
    switchPos = 0;
}
