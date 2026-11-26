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
// If the straight segment from->to passes within block_cm of obstacle (and
// the obstacle is actually between the two points), return a waypoint that
// goes clear_cm to the side of the obstacle (the side with more room
// inside the field); otherwise return `to` unchanged. Used so driving to a
// staging point behind the ball doesn't go through the ball on the way.
Vec2 plan_avoid_point(Vec2 from, Vec2 to, Vec2 obstacle, double block_cm,
                      double clear_cm, Vec2 field_min, Vec2 field_max);

// Clamb vectors to be inside the field within a certain margin                      
Vec2 clamp_to_field(Vec2 p, Vec2 field_min, Vec2 field_max, double margin);

// Plan a staging point, a point `stage_dist` cm behind the ball on the line from
// `aim_target` through `ball_pos`, while being aware of walls
Vec2 plan_waypoint(Vec2 ball_pos, Vec2 aim_target, double stage_dist,
                    Vec2 field_min, Vec2 field_max, double margin);

// A point `stand_dist` cm from own_goal, toward whatever threat_pos is
// (the ball, or the opponent) i.e. standing between the threat and our
// own goal instead of going to meet it directly. Used for defensive
// positioning (skill_mark and friends), mirrors plan_waypoint()'s
// clamp-to-field safety net.
Vec2 plan_defend_point(Vec2 threat_pos, Vec2 own_goal, double stand_dist,
                        Vec2 field_min, Vec2 field_max, double margin);

// True if `pos` is within `margin` of any field edge.
int near_wall(Vec2 pos, Vec2 field_min, Vec2 field_max, double margin);

// Finds whichever of the 4 walls pos is closest to, and returns the unit
// vector pointing away from that wall, back toward the field's interior.
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
// `field_center_x`) and within `goal_width`/2 of `goal_y`. "Past" is
// evaluated against the ball's center reach (goal_x -+ ball_radius), instead of
// goal_x itself. The ball's leading edge is what actually crosses the
// line, and its center physically cannot reach goal_x before the edge
// does. Pass in 0 for ball_radius for a "literal" center-crossing test,
// but you probably never will
int ball_in_goal(Vec2 ball_pos, Vec2 goal, double goal_width, double field_center_x,
                  double ball_radius);

// Perpendicular distance from `p` to the infinite line through `a` and `b`.
double perp_dist_to_line(Vec2 p, Vec2 a, Vec2 b);

// Per-frame fix for the blob-shape heading vector's 180deg switching. it's
// the long axis of a rectangle, which can't tell front from back, and can
// flip sign on any given frame.
// id_bot() corrects this once at startup by comparing against the motion
// vector, but that correction doesn't persist.
//
// raw is resolved against *last_good first (whichever of raw/-raw is
// closer to the already-committed direction), so a fleeting one-time flip in raw
// itself can never show up in the output. Actually changing the
// committed direction requires HEADING_FLIP_CONFIRM_FRAMES consecutive
// frames of vel disagreeing with it. A single frame's vel, which comes from from
// one ~30ms position delta, isn't reliable enough in DIRECTION to trust
// immediately even when its magnitude passes trust_speed_cmps
Vec2 fix_heading_sign(Vec2 raw, Vec2 vel, double trust_speed_cmps, Vec2 *last_good,
                       int *flip_streak, int confirm_frames);

#endif
