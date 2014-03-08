/*************************/
/* RC-Decoder            */
/*************************/

// init RC config variables
void initRC() {
  rcLPF_tc = LOWPASS_K_FLOAT(config.rcLPF*0.1);
}

//******************************************
// PWM Decoder
//******************************************
inline void decodePWM(rcData_t* rcData) {
  uint16_t pulseInPWMtmp;
  pulseInPWMtmp = (rcData->microsLastUpdate - rcData->microsRisingEdge)/CC_FACTOR;
  if ((pulseInPWMtmp >= MIN_RC) && (pulseInPWMtmp <= MAX_RC)) {
    // update if within expected RC range
    rcData->rx = pulseInPWMtmp;
    rcData->isValid=true;
    rcData->isFresh=true;
  }
}

//******************************************
// PPM Decoder
//******************************************

inline void intDecodePPM() { 
  uint32_t ticksNow = micros();
  
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

//******************************************
// Interrupts
//******************************************
ISR(PCINT1_vect) {
  static uint8_t lastPin;
  uint16_t frozenTime = micros();
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
  else if (change & 2) {
    if (pin & 2) {
      rcData[RC_DATA_PITCH].microsRisingEdge = frozenTime;
    } else {
      rcData[RC_DATA_PITCH].microsLastUpdate = frozenTime;
      decodePWM(&rcData[RC_DATA_PITCH]);
    }
  }
  else if (change & 4) {
    if (pin & 4) {
      rcData[RC_DATA_SWITCH].microsRisingEdge = frozenTime;
    } else {
      rcData[RC_DATA_PITCH].microsLastUpdate = frozenTime;
      decodePWM(&rcData[RC_DATA_SWITCH]);
    }
  }
}

//******************************************
// PPM & PWM Decoder
//******************************************

// check for RC timout

void checkRcTimeouts() {
  int32_t microsNow = micros();
  int32_t microsLastUpdate;
  for (uint8_t id = 0; id < RC_DATA_SIZE; id++) {
    cli();
    microsLastUpdate = rcData[id].microsLastUpdate;
    sei();
    if (rcData[id].isValid && ((microsNow - microsLastUpdate)/CC_FACTOR) > RC_TIMEOUT){
      rcData[id].rx     = config.rcMid;
      rcData[id].isValid  = false;
      rcData[id].isFresh = true;
    }
  }
}

// initialize RC Pin mode
void initRCPins() {
  static bool first = true;
  if (first) {
    pinMode(A2, INPUT); digitalWrite(A2, HIGH);
    pinMode(A1, INPUT); digitalWrite(A1, HIGH);
    pinMode(A0, INPUT); digitalWrite(A0, HIGH);

    PCMSK1 |= (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10);
    PCICR |= (1<<PCIE1);
    
    first = false;
  }

  for (uint8_t id = 0; id < RC_DATA_SIZE; id++) {
    cli();
    rcData[id].microsRisingEdge = 0;
    rcData[id].microsLastUpdate = 0;
    rcData[id].rx               = 1500;
    rcData[id].isFresh          = true;
    rcData[id].isValid          = true;
    rcData[id].rcSpeed          = 0.0;
    rcData[id].setpoint         = 0.0;
    sei();
  }

  if (!config.rcModePPM) {
    if (config.rcChannelRoll  > 2 || config.rcChannelPitch > 2 || config.rcChannelRoll == config.rcChannelPitch) {
      config.rcChannelRoll  = 0;
      config.rcChannelPitch = 1;
    }
  }
}

//******************************************
// Integrating
//******************************************
void evalRCChannelIntegrating(rcData_t* rcData, int16_t rcGain, uint16_t rcMid) {
  if(rcData->isFresh) {
    if(rcData->rx >= rcMid + RC_DEADBAND) {
      rcData->rcSpeed = rcGain * (float)(rcData->rx - (rcMid + RC_DEADBAND))/ (float)(MAX_RC - (rcMid + RC_DEADBAND)) + 0.9 * rcData->rcSpeed;
    } else if(rcData->rx <= rcMid-RC_DEADBAND){
      rcData->rcSpeed = -rcGain * (float)((rcMid - RC_DEADBAND) - rcData->rx)/ (float)((rcMid - RC_DEADBAND)-MIN_RC) + 0.9 * rcData->rcSpeed;
    } else {
      rcData->rcSpeed = 0.0;
    }
    rcData->rcSpeed = constrain(rcData->rcSpeed, -200, +200);  // constrain for max speed
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

inline void evalRCChannelAbsolute(rcData_t* rcData, int16_t rcMin, int16_t rcMax, int16_t rcMid) {
  float k;
  float y0;
  int16_t rx;
  
  if(rcData->isFresh) {
    k = (float)(rcMax - rcMin)/(MAX_RC - MIN_RC);
    y0 = rcMin + k * (MID_RC - MIN_RC);
    rx = rcData->rx - rcMid;
    utilLP_float(&rcData->setpoint, y0 + k * rx, 0.05);
    rcData->isFresh = false;
  }
}

// Absolute RC control

void evaluateRCAbsolute() {
  evalRCChannelAbsolute(&rcData[RC_DATA_PITCH], config.minRCPitch, config.maxRCPitch, config.rcMid);
  evalRCChannelAbsolute(&rcData[RC_DATA_ROLL ], config.minRCRoll , config.maxRCRoll,  config.rcMid);
}

void evaluateRCSwitch() {
  if (!rcData[RC_DATA_SWITCH].isFresh)
    return;
  uint16_t lThreshold = (MID_RC + MID_RC)/2;
  if (switchPos < 0) lThreshold += 50; else if (switchPos == 0) lThreshold -= 50;
  uint16_t hThreshold = (MID_RC + MAX_RC)/2;
  if (switchPos > 0) lThreshold -= 50; else if (switchPos == 0) lThreshold += 50;
  uint16_t sp = rcData[RC_DATA_SWITCH].setpoint;
  if (sp <= lThreshold)
    switchPos = -1;
  else if (sp >= hThreshold)
    switchPos = 1;
  else 
    switchPos = 0;
    
  switchPos = 0;
}
