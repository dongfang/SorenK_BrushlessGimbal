#include "Globals.h"
#include <avr/interrupt.h>

float rcLPF_tc = 1.0;

/*************************/
/* RC-Decoder            */
/*************************/

//******************************************
// PWM Decoder
//******************************************
inline void decodePWM(RCData_t* rcData) {
  uint16_t pulseInPWMtmp;
  pulseInPWMtmp = (rcData->microsLastUpdate - rcData->microsRisingEdge)/16;
  // update if within expected RC range
  rcData->rx = pulseInPWMtmp;
  rcData->isFresh=true;
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
      rcData[RC_DATA_ROLL].microsLastUpdate = frozenTime;
      decodePWM(&rcData[RC_DATA_ROLL]);
    }
  }
  if (change & 2) {
    if (pin & 2) {
      rcData[RC_DATA_PITCH].microsRisingEdge = frozenTime;
    } else {
      rcData[RC_DATA_PITCH].microsLastUpdate = frozenTime;
      decodePWM(&rcData[RC_DATA_PITCH]);
    }
  }
  if (change & 4) {
    if (pin & 4) {
      rcData[RC_DATA_SWITCH].microsRisingEdge = frozenTime;
    } else {
      rcData[RC_DATA_SWITCH].microsLastUpdate = frozenTime;
      decodePWM(&rcData[RC_DATA_SWITCH]);
    }
  }
}

//******************************************
// PPM & PWM Decoder
//******************************************

// check for RC timout

void checkRcTimeouts() {
  uint16_t timerNow = time();
  uint16_t timerLastUpdate;
  for (uint8_t id = 0; id < RC_DATA_SIZE; id++) {
    cli();
    timerLastUpdate = rcData[id].microsLastUpdate;
    sei();
    if (rcData[id].isValid && ((timerNow - timerLastUpdate)) > RC_TIMEOUT){
      rcData[id].rx = config.rcMid;
      rcData[id].isValid = false;
      rcData[id].isFresh = true;
    }
  }
}

void initRCFilter() {
 rcLPF_tc = LOWPASS_K_FLOAT(config.rcLPF * 0.1);
}

// initialize RC Pin mode
void initRC() {
//  static bool first = true;
//  if (first) {

	// TODO: We need a pin identity abstraction (which is not Arduino :) )
	//pinMode(A2, INPUT); digitalWrite(A2, HIGH);
    //pinMode(A1, INPUT); digitalWrite(A1, HIGH);
    //pinMode(A0, INPUT); digitalWrite(A0, HIGH);
	DDRC |= 1<<0 | 1<<1 | 1<<2;
	PORTC |= 1<<0 | 1<<1 | 1<<2;

    PCMSK1 |= (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10);
    PCICR |= (1<<PCIE1);
    
    //first = false;
  //}

  for (uint8_t id = 0; id < RC_DATA_SIZE; id++) {
    cli();
    rcData[id].microsRisingEdge = 0;
    rcData[id].microsLastUpdate = 0;
    rcData[id].rx               = MID_RC;
    rcData[id].isFresh          = true;
    rcData[id].isValid          = true;
    rcData[id].rcSpeed          = 0.0;
    rcData[id].setpoint         = 0.0;
    sei();
  }
}

//******************************************
// Integrating
//******************************************
void evalRCChannelIntegrating(RCData_t* rcData, int16_t rcGain, uint16_t rcMid) {
  if(rcData->isFresh) {
    if(rcData->rx >= rcMid + RC_DEADBAND) {
      rcData->rcSpeed = rcGain * (float)(rcData->rx - (rcMid + RC_DEADBAND))/ (float)(MAX_RC - (rcMid + RC_DEADBAND)) + 0.9 * rcData->rcSpeed;
    } else if(rcData->rx <= rcMid-RC_DEADBAND){
      rcData->rcSpeed = -rcGain * (float)((rcMid - RC_DEADBAND) - rcData->rx)/ (float)((rcMid - RC_DEADBAND)-MIN_RC) + 0.9 * rcData->rcSpeed;
    } else {
      rcData->rcSpeed = 0.0;
    }
    rcData->rcSpeed = constrain_f(rcData->rcSpeed, -200, +200);  // constrain for max speed
    rcData->isFresh = false;
  }
}

// Integrating RC control
void evaluateRCIntegrating() {
  evalRCChannelIntegrating(&rcData[RC_DATA_PITCH], config.rcGain, config.rcMid);
  evalRCChannelIntegrating(&rcData[RC_DATA_ROLL ], config.rcGain, config.rcMid);
}

//******************************************
// Absolute
//******************************************

inline void evalRCChannelAbsolute(RCData_t* rcData, int16_t rcMin, int16_t rcMax, int16_t rcMid) {
  float k;
  float y0;
  int16_t rx;
  
  if(rcData->isFresh) {
    k = (float)(rcMax - rcMin)/(MAX_RC - MIN_RC);
    y0 = rcMin + k * (MID_RC - MIN_RC);
    rx = rcData->rx - rcMid;
    //utilLP_float(&rcData->setpoint, y0 + k * rx, 0.05);
    rcData->isFresh = false;
  }
}

// Absolute RC control
void evaluateRCAbsolute() {
  evalRCChannelAbsolute(&rcData[RC_DATA_PITCH], config.minRCPitch, config.maxRCPitch, config.rcMid);
  evalRCChannelAbsolute(&rcData[RC_DATA_ROLL ], config.minRCRoll , config.maxRCRoll,  config.rcMid);
}

/*
 * We don't care what the meanings are of the switch pos's here.
 * It is simply -1 or 1 after evaluation.
 */
void evaluateRCSwitch() {
  if (!rcData[RC_DATA_SWITCH].isFresh)
    return;
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
    
  rcData[RC_DATA_SWITCH].isFresh = false;
}
