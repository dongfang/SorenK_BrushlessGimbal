#ifndef __RCDECODE_H
#define __RCDECODE_H
#include "Globals.h"

// init RC config variables
void initRC();
void initRCFilter();
void evaluateRCAbsolute();
void evaluateRCIntegrating();
void evaluateRCSwitch();
void checkRcTimeouts();

#endif
