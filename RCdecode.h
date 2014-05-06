#ifndef __RCDECODE_H
#define __RCDECODE_H
#include "Globals.h"

// init RC config variables
void initRC();
void checkRcTimeouts();
void evaluateRCControl();
void evaluateRCSwitch();

#endif
