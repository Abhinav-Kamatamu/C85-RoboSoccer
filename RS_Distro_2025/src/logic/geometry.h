/***************************************************
 CSC C85 RoboSoccer - pure geometry (layer 3)

 No BT_*, no World mutation.
***************************************************/

#ifndef LOGIC_GEOMETRY_H
#define LOGIC_GEOMETRY_H

#include "world.h"

Vec2   vec2_add(Vec2 a, Vec2 b);
Vec2   vec2_sub(Vec2 a, Vec2 b);
Vec2   vec2_scale(Vec2 a, double s);
double vec2_len(Vec2 a);
Vec2   vec2_normalize(Vec2 a);
double vec2_dot(Vec2 a, Vec2 b);
double vec2_dist(Vec2 a, Vec2 b);

double angle_of(Vec2 v);               // atan2(v.y, v.x), radians
double angle_diff_rad(double a, double b);  // a-b wrapped to (-pi, pi]
double rad_to_deg(double r);
double deg_to_rad(double d);

// Clamp a point to stay at least `margin` inside [field_min, field_max].
Vec2 clamp_to_field(Vec2 p, Vec2 field_min, Vec2 field_max, double margin);

// Plan a staging point, a point `stage_dist` cm behind the ball on the line from
// `aim_target` through `ball_pos`, while being aware of walls
Vec2 plan_waypoint(Vec2 ball_pos, Vec2 aim_target, double stage_dist,
                    Vec2 field_min, Vec2 field_max, double margin);

// True if `pos` is within `margin` of any field edge.
int near_wall(Vec2 pos, Vec2 field_min, Vec2 field_max, double margin);

// Which of the 4 walls is closest to `pos`, and which direction to drive
// to get away from it. A unit vector pointing from that wall back into
// the field (e.g. near the left wall -> (1,0), drive in +x). In a corner,
// this picks whichever single wall is minutely closer, NOT a diagonal
// blend of both.
Vec2 direction_away_from_nearest_wall(Vec2 pos, Vec2 field_min, Vec2 field_max);

// Distance from `pos` to whichever of the 4 walls is nearest.
double dist_to_nearest_wall(Vec2 pos, Vec2 field_min, Vec2 field_max);

// When the ball is near a wall, `plan_waypoint` can't put a staging point
// behind it on the goal line without landing outside the field. Instead, stage
// on the WALL side of the ball (clamped to the field bounds) and push it
// inward, toward *aim_target_out, before switching back to normal
// plan_waypoint() once it fails near_wall() (we don't want to be near a wall).
Vec2 plan_wall_push_staging(Vec2 ball_pos, Vec2 field_min, Vec2 field_max,
                             double stage_dist, double min_stage_cm,
                             Vec2 *aim_target_out);

// True if `ball_pos` is past `goal_x` (in the direction away from
// `field_center_x`) and within `goal_width`/2 of `goal_y`.
int ball_in_goal(Vec2 ball_pos, Vec2 goal, double goal_width, double field_center_x);

// Perpendicular distance from `p` to the infinite line through `a` and `b`.
double perp_dist_to_line(Vec2 p, Vec2 a, Vec2 b);

// Per-frame fix for the blob-shape heading vector's 180deg switching. it's
// the long axis of a rectangle, which can't tell front from back, and can
// flip sign on any given frame.
// id_bot() corrects this ONCE at startup by comparing against the motion
// vector, but that correction doesn't persist.
Vec2 fix_heading_sign(Vec2 raw, Vec2 vel, double trust_speed_cmps, Vec2 *last_good);

#endif
