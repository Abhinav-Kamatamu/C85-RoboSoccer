#include "drive.h"
#include "../roboAI.h"
#include "ai_config.h"
#include "skills.h"
#include <math.h>

// Kinematics (v, w, WHEEL_TRACK_CM) give each wheel's target speed in
// cm/s, and then POWER_TO_CMPS converts that to motor power.

static double g_last_l = -1000, g_last_r = -1000;

// Makes sure power isn't too low, else sets it to the minimum power we allow
double apply_min_power(double power) {
    if (power == 0)
        return 0;
    if (fabs(power) < MIN_MOVE_POWER)
        return power > 0 ? MIN_MOVE_POWER : -MIN_MOVE_POWER;
    return power;
}

// Caps how much power can change from last frame to this one, so commands
// ramp smoothly instead of jumping (which makes the wheels skid/jolt).
double limit_power_step(double target, double last) {
    // this is the first call, so there's nothing to limit against
    if (last < -999)
        return target;
    // We make sure we don't speed up or slow down too fast or else,
    // very bad things might happen
    return clampd(target, last - MAX_POWER_STEP_PER_FRAME, last + MAX_POWER_STEP_PER_FRAME);
}

void drive_set(Motion m) {
    // Kinematics
    // Each wheel is WHEEL_TRACK_CM / 2 from the robot's center, so while rotating
    // at w (rad/s) each wheel travels along a path along the circumference of the wheel.
    // linear speed = radius * angular speed (s = r*theta) [Divide by time].
    // Add that rotational speed to the forward speed v.
    // On a left rotation, the inside wheel (left, turning left) is slower,
    // the outside wheel (right) is faster. w=0 -> both wheels are turning at equal
    // rates (straight). v=0 -> the wheels are rotating equal and opposite (spin in place).
    double vl_cmps = m.v - m.w * (WHEEL_TRACK_CM / 2.0);
    double vr_cmps = m.v + m.w * (WHEEL_TRACK_CM / 2.0);

    double pl = vl_cmps / POWER_TO_CMPS + MOTOR_TRIM_L;
    double pr = vr_cmps / POWER_TO_CMPS + MOTOR_TRIM_R;

    pl = apply_min_power(pl);
    pr = apply_min_power(pr);

    pl = limit_power_step(pl, g_last_l);
    pr = limit_power_step(pr, g_last_r);

    pl = clampd(pl, -MAX_POWER, MAX_POWER);
    pr = clampd(pr, -MAX_POWER, MAX_POWER);

    // Dedupe (don't call if values are similar to last time)
    if (fabs(pl - g_last_l) < 1.0 && fabs(pr - g_last_r) < 1.0)
        return;

    BT_turn(LEFT_MOTOR, (char) lround(pl), RIGHT_MOTOR, (char) lround(pr));
    g_last_l = pl;
    g_last_r = pr;
}

void drive_stop(int brake) {
    BT_all_stop(brake);
    g_last_l = 0;
    g_last_r = 0;
}
