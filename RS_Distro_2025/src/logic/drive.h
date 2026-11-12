/***************************************************
 CSC C85 RoboSoccer drive layer

 The ONLY place that calls BT_*. Skills and states coomunicaate in (v, w)
***************************************************/

#ifndef LOGIC_DRIVE_H
#define LOGIC_DRIVE_H

#include "world.h"

// Send a motion command. It is safe to call every frame with the same value.
// it dedupes internally and only talks to BT_* when something changed.
// Deduping is good (we don't want to send the same arguments every frame)
void drive_set(Motion m);

// Halt. brake=1 to hold position, 0 to coast.
void drive_stop(int brake);

#endif
