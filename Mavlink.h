#ifndef __MAVLINK_H
#define __MAVLINK_H

void mavlink_init();
bool mavlink_parse();
void mavlink_update();
void mavlink_updateTarget();
void mavlink_sendStatus();
void mavlink_sendHeartbeat();

extern int16_t mavlinkTargetPitch;
extern bool mavlinkDetected;

#endif
