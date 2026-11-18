/***************************************************
 CSC C85 RoboSoccer drive layer

 The ONLY place that calls BT_*. Skills and states coomunicaate in (v, w)
***************************************************/

#ifndef LOGIC_DRIVE_H
#define LOGIC_DRIVE_H

#include "world.h"

// Motion -> (left, right) motor power, before rate limiting/clamping/dedupe.
//
// w > 0 means "angle_of(heading) increases", or clockwise on
// screen (camera y points down), which is a physical RIGHT turn, so left wheel
// faster (Left on screen != Left irl).
void drive_shape_power(Motion m, int use_regulated, double *pl_out, double *pr_out);

// Send a motion command. It is safe to call every frame with the same value.
// it dedupes internally and only communicates with BT_* when something changed.
// Deduping is bonne (we don't want to send the same arguments every frame)
// use_regulated: 0 = open-loop BT_turn (POWER_TO_CMPS/POWER_TO_RADPS,
// voltage-dependent, 1 = regulated BT_turn_speed
// (SPEED_API_TO_CMPS_GAIN/SPEED_API_TO_DEGPS_GAIN, voltage-independent).
// Stick with 0 because while 1 in theory sounds good, we haven't run into many
// issues with 0 yet and 1 is mostly untested
void drive_set(Motion m, int use_regulated);

// Runtime toggle read by the adapters to decide what to pass as drive_set()'s
// use_regulated argument. Defaults to 0.
void drive_set_regulated_speed(int enabled);
int drive_get_regulated_speed(void);

// The wheel powers most recently handed to the motors (including shaping pulses,
// minimum-power floor, per-frame ramp limit, clamping).
void drive_get_last_power(double *left, double *right);

// Halt. brake=1 to hold position, 0 to coast.
void drive_stop(int brake);

// BT delivery diagnostics since the previous call (then reset). Number of BT
// calls completed, commands overwritten before being sent, avg/max call time.
void drive_get_bt_stats(unsigned *calls, unsigned *overwritten, double *avg_ms, double *max_ms);

#endif
