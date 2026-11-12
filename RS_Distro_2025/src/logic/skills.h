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

// Drive toward `target`. DONE once within STAGE_TOL_CM.
SkillStatus skill_goto(const World *w, Vec2 target, Motion *out);

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
SkillStatus skill_leave_wall(const World *w, Motion *out);

// Wiggle free. reverse briefly, then turn briefly, repeat. Always RUNNING
// the caller's `next()` checks World.self.vel to decide we're free.
// reset=1 on the frame you enter UNSTICK.
SkillStatus skill_unstick(const World *w, int reset, Motion *out);

#endif
