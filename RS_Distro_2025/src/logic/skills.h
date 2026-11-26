/***************************************************
 CSC C85 RoboSoccer skills

 Reusable behaviours shared by every mode (penalty, chase, soccer). Each
 skill reads World and writes to a desired Motion ptr. None of them call BT_*
 (drive.c does that) and none of them know what mode or state called them.

 A few skills need to remember something across frames (like a stable-alignment
 frame count, or how far we've backed off so far). That state is a static
 local inside the .c file, reset by the caller's own state transition.
 This means only ONE instance of a given
 skill can be "in flight" at a time, which is fine. the FSM only ever runs
 one state (and therefore one skill) per mode at once.
***************************************************/

#ifndef LOGIC_SKILLS_H
#define LOGIC_SKILLS_H

#include "world.h"

// clamp function for doubles
double clampd(double v, double lo, double hi);

// Drive toward `target`. DONE once within STAGE_TOL_CM. resetting (pass the
// g_entered/first-frame flag) clears the antipodal turn-direction
// commitment.
SkillStatus skill_goto(const World *w, Vec2 target, int reset, Motion *out);

// Drive at `target` and keep going, with no slowdown approaching
// it. For continuously chasing/pushing something, as opposed to
// skill_goto's arrive-and-stop.
SkillStatus skill_chase(const World *w, Vec2 target, Motion *out);

// Skill to face towards target
SkillStatus skill_face(const World *w, Vec2 target, int reset, Motion *out);

// Skill to strike ball
SkillStatus skill_strike(const World *w, Vec2 aim_target, int reset, Motion *out);

// Keep going a short distance/time after contact, then DONE. Tracks
// elapsed distance/time via a static accumulator reset on first call
// after a state change (pass reset=1 on the frame you enter this state).
SkillStatus skill_follow_through(const World *w, Vec2 aim_dir_target, int reset, Motion *out);

// Back straight off by `dist_cm`. DONE once that far. Same reset
// convention as skill_follow_through.
SkillStatus skill_backoff(const World *w, double dist_cm, int reset, Motion *out);

// Stop and hold. Always RUNNING the caller's `next()` decides when to
// leave, based on conditions outside this skill (ball at rest, timers).
SkillStatus skill_hold(const World *w, Motion *out);

// Drive away from the nearest wall. DONE once clear of DEFAULT_WALL_MARGIN_CM.
SkillStatus skill_leave_wall(const World *w, Motion *out, int reset);

// Wiggle free. reverse briefly, then turn briefly, repeat. Always RUNNING
// the caller's `next()` checks World.self.vel to decide we're free.
// reset=1 on the frame you enter UNSTICK.
SkillStatus skill_unstick(const World *w, int reset, Motion *out);

// Steer at where the ball will be shortly (extrapolated from its current
// velocity), not where it is right now for cutting off a moving ball
// instead of always chasing one step behind it. Always RUNNING; holds
// still if the ball isn't currently visible.
SkillStatus skill_intercept(const World *w, int reset, Motion *out);

// Cheap, low-precision "get the ball away from our own goal right now."
// Unlike skill_strike this doesn't need to be carefully aimed first,
// it aims directly away from own_goal through the ball, with a much more
// forgiving abort tolerance, prioritizing speed over where it ends up.
// FAILED if the ball isn't visible or the angle's too far off; DONE once
// the ball's visibly moved.
SkillStatus skill_clear(const World *w, int reset, Motion *out);

// Defensive blocking; stand between the opponent and our own goal
// (plan_defend_point()) instead of going to meet them. Always RUNNING;
// holds still if the opponent isn't currently visible.
SkillStatus skill_mark(const World *w, int reset, Motion *out);

#endif
