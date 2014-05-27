#include "Board.h"

void reset();
void calibrateGyro();
void calibrateAcc();
void run();
void stop();

#ifdef SUPPORT_RETRACT
void retract();
void extend();
#endif

void freeze();
void motorTest();

#ifdef SUPPORT_MAVLINK
void goMavlink();
#endif

void startAutosetup();
