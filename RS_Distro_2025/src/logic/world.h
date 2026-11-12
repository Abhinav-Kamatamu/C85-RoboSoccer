/***************************************************
 CSC C85 RoboSoccer - world

 Core types used by every layer above drive.c. They never invoke BT_*

 Angle convention: atan2(dy, dx) in
 field centimetres, y DOWN, and a
 positive angle should look clockwise on screen.
***************************************************/

#ifndef LOGIC_WORLD_H
#define LOGIC_WORLD_H

typedef struct {
    double x, y;
} Vec2;

typedef struct {
    int    valid;        // seen this frame
    int    lost_frames;  // consecutive frames NOT seen (0 while valid)
    Vec2   pos;           // cm
    Vec2   vel;           // cm/s
    Vec2   heading;        // unit vector WITH sign (robots only, (0,0) for the ball cuz we don't need it)
} Agent;

// Snapshot of everything a skill or state needs, in field units (cm, s).
// Built fresh each frame by perception.c from struct AI_data; nothing
// downstream of World touches ai->st or blob pointers directly.
typedef struct {
    Agent  self, opp, ball;
    Vec2   field_min, field_max;   // field bounding box, cm
    Vec2   own_goal, their_goal;    // goal centre points, cm
    double goal_width_cm;
    double now;                     // seconds, monotonic, for timeouts/dwell
} World;

// Desired motion out of a skill. drive.c is the only place that turns
// this into motor power -- rebuild the robot and only drive.c/ai_config.h
// change.
typedef struct {
    double v;  // forward speed, cm/s (negative = reverse)
    double w;  // turn rate, rad/s (+ = clockwise, see angle convention above)
} Motion;

typedef enum { SKILL_RUNNING, SKILL_DONE, SKILL_FAILED } SkillStatus;

#endif
