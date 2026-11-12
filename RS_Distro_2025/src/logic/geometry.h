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

/***************************************************
 Segment-based boundary model.

 NOT USED YET: World/perception.c/mode_penalty.c still use the plain
 field_min/field_max rectangle above. This is the general replacement for
 when we actually have boundry data
***************************************************/

typedef struct {
    int    is_arc;           
    Vec2   a, b;             
    Vec2   arc_center;       
    double radius_cm;        
    double angle_start_rad, angle_end_rad;
} BoundarySegment;

typedef struct {
    const BoundarySegment *segments;
    int n_segments;
    Vec2 bbox_min, bbox_max; 
} FieldModel;


void field_model_finalize(FieldModel *field);

BoundarySegment make_straight_segment(Vec2 a, Vec2 b);
// a, b: the two measured points where the curve starts and ends (e.g. the
// straightedge-tangent-point method). center/radius_cm as measured or
// looked up (e.g. a standard track piece's radius). Computes
// angle_start/end from a and b so callers never hand-compute angles.
BoundarySegment make_arc_segment(Vec2 center, double radius_cm, Vec2 a, Vec2 b);

typedef struct {
    double distance;  // to the nearest boundary feature
    Vec2   direction;  // unit vector pointing away from it, into the field;
                        // (0,0) when `distance` is large enough that a
                        // meaningful direction wasn't computed (see
                        // nearest_wall()'s fast-reject path)
} WallQuery;

// Distance + direction to the nearest point on the boundary, in one pass
// (replaces separately calling a distance function and a direction
// function against the same position).
//
// `safe_margin_cm`: if `pos` is at least this far inside the field's
// bounding box on every side, skip the real search and report that bbox
// distance directly -- cheap, but only a lower bound for a non-convex
// shape, so pick something comfortably larger than any margin you intend
// to compare `distance` against (e.g. 2x your largest WALL_MARGIN).
// Assumes the boundary loop is reasonably convex (true for a field-shaped
// track); this is a performance shortcut, not a correctness guarantee.
//
// `hint`: optional (NULL is fine) in/out cache of "which segment index was
// nearest last time," owned by the caller (one per agent you're tracking,
// same static-local convention as the rest of this codebase, e.g.
// skills.c's `reset` pattern). When valid, only that segment and its two
// immediate neighbours are checked -- the nearest segment essentially
// never jumps further than that between consecutive frames of a slowly
// moving agent. Falls back to a full scan over every segment only when
// there's no usable hint yet (first call, or the caller opted out).
//
// Adding real spatial binning later (skipped for now -- not needed at the
// segment counts a track-piece field produces) only means replacing the
// full-scan loop inside this function with a grid-cell lookup that yields
// a short candidate list instead of every segment. Nothing in this
// function's signature, or any caller, needs to change.
WallQuery nearest_wall(Vec2 pos, const FieldModel *field, double safe_margin_cm, int *hint);

#endif
