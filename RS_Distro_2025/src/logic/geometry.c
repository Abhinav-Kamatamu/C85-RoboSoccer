#include "geometry.h"
#include <math.h>

Vec2 vec2_add(Vec2 a, Vec2 b) {
    Vec2 r = {a.x + b.x, a.y + b.y};
    return r;
}

Vec2 vec2_sub(Vec2 a, Vec2 b) {
    Vec2 r = {a.x - b.x, a.y - b.y};
    return r;
}

Vec2 vec2_scale(Vec2 a, double s) {
    Vec2 r = {a.x * s, a.y * s};
    return r;
}

double vec2_dot(Vec2 a, Vec2 b) {
    return a.x * b.x + a.y * b.y;
}

double vec2_len(Vec2 a) {
    return sqrt(vec2_dot(a, a));
}

double vec2_dist(Vec2 a, Vec2 b) {
    return vec2_len(vec2_sub(a, b));
}

Vec2 vec2_normalize(Vec2 a) {
    double l = vec2_len(a);
    if (l < 1e-9) {
        Vec2 z = {0, 0};
        return z;
    }
    return vec2_scale(a, 1.0 / l);
}

double angle_of(Vec2 v) {
    return atan2(v.y, v.x);
}

double angle_diff_rad(double a, double b) {
    double d = fmod(a - b + M_PI, 2.0 * M_PI);
    if (d < 0.0)
        d += 2.0 * M_PI;
    return d - M_PI;
}

double rad_to_deg(double r) {
    return r * 180.0 / M_PI;
}

double deg_to_rad(double d) {
    return d * M_PI / 180.0;
}

double clampd(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

Vec2 clamp_to_field(Vec2 p, Vec2 field_min, Vec2 field_max, double margin) {
    Vec2 r;
    r.x = clampd(p.x, field_min.x + margin, field_max.x - margin);
    r.y = clampd(p.y, field_min.y + margin, field_max.y - margin);
    return r;
}

int near_wall(Vec2 pos, Vec2 field_min, Vec2 field_max, double margin) {
    return pos.x < field_min.x + margin || pos.x > field_max.x - margin ||
           pos.y < field_min.y + margin || pos.y > field_max.y - margin;
}

typedef struct {
    double left, right, top, bottom;
} WallDists;

WallDists wall_distances(Vec2 pos, Vec2 field_min, Vec2 field_max) {
    WallDists d;
    d.left = pos.x - field_min.x; // distance to the left wall (x = field_min.x)
    d.right = field_max.x - pos.x;     // distance to the right wall (x = field_max.x)
    d.top = pos.y - field_min.y; // distance to the top wall (y = field_min.y)
    d.bottom = field_max.y - pos.y; // distance to the bottom wall (y = field_max.y)
    return d;
}

// Finds whichever of the 4 walls pos is closest to, and returns the unit
// vector pointing away from that wall, back toward the field's interior.
Vec2 direction_away_from_nearest_wall(Vec2 pos, Vec2 field_min, Vec2 field_max) {
    WallDists d = wall_distances(pos, field_min, field_max);
    double closest_dist = d.left; // closest wall distance seen so far
    Vec2 n = {1, 0}; // and the direction away from that closest wall
    if (d.right < closest_dist) {
        closest_dist = d.right;
        n.x = -1;
        n.y = 0;
    }
    if (d.top < closest_dist) {
        closest_dist = d.top;
        n.x = 0;
        n.y = 1;
    }
    if (d.bottom < closest_dist) {
        closest_dist = d.bottom;
        n.x = 0;
        n.y = -1;
    }
    return n;
}

// Distance from pos to whichever of the 4 walls is nearest.
double dist_to_nearest_wall(Vec2 pos, Vec2 field_min, Vec2 field_max) {
    WallDists d = wall_distances(pos, field_min, field_max);
    double m = d.left;
    if (d.right < m)
        m = d.right;
    if (d.top < m)
        m = d.top;
    if (d.bottom < m)
        m = d.bottom;
    return m;
}

Vec2 plan_waypoint(
    Vec2 ball_pos, Vec2 aim_target, double stage_dist,
    Vec2 field_min, Vec2 field_max, double margin
) {
    Vec2 away = vec2_normalize(vec2_sub(ball_pos, aim_target));
    // If the ball is at the main target.
    if (away.x == 0 && away.y == 0)
        away.x = 1;
    // The spot the robot drives to before it tries to kick.
    Vec2 staging = vec2_add(ball_pos, vec2_scale(away, stage_dist));
    // Make sure where we're going to is actually in the field. This clamp is
    // only a safety net for minor overshoot in the normal case. If the
    // ball itself is near a wall, YOU should use
    // plan_wall_push_staging() instead of rely on this clamp
    return clamp_to_field(staging, field_min, field_max, margin);
}

// A staging point is always "push the ball further away from wherever
// we're aiming." If the ball is already sitting on the boundary, that
// pushes the staging point beyond the wall no matter which interior
// point you aim at (goal or field centre). This is a completely seperate
// maneuver from plan_waypoint. get on the WALL side of the ball and push it
// inward first, allowing an imperfect approach.
//
// Returns the staging point (on the wall side of the ball, clamped to
// whatever room actually exists between the ball and the wall), and sets
// *aim_target_out to the point to aim/drive toward once there, a point
// far out along the inward direction, so skill_face/skill_strike can use
// it exactly like any other aim target.
Vec2 plan_wall_push_staging(
    Vec2 ball_pos, Vec2 field_min, Vec2 field_max,
    double stage_dist, double min_stage_cm,
    Vec2 *aim_target_out
) {
    Vec2 in = direction_away_from_nearest_wall(ball_pos, field_min, field_max);
    double wall_dist = dist_to_nearest_wall(ball_pos, field_min, field_max);

    // How far we can actually stand on the wall side of the ball without
    // trying to occupy space beyond the wall itself. If the ball is right
    // against the wall there may be almost no room. min_stage_cm is the
    // smallest gap we'll still attempt, instead of refusing to move.
    double room = wall_dist - min_stage_cm;
    double actual_dist = (room > min_stage_cm) ? fmin(stage_dist, room) : min_stage_cm;
    // We want to move the ball away from the wall, so we go to the wall side of
    // the ball
    Vec2 staging = vec2_sub(ball_pos, vec2_scale(in, actual_dist));

    if (aim_target_out)
        *aim_target_out = vec2_add(ball_pos, vec2_scale(in, 1000.0));

    // Safety-net clamp with a SMALL margin, not the normal wall margin.
    // the normal margin is the zone this manouver has to enter.
    return clamp_to_field(staging, field_min, field_max, min_stage_cm);
}

int ball_in_goal(Vec2 ball_pos, Vec2 goal, double goal_width, double field_center_x) {
    int past_line = (goal.x < field_center_x) ? (ball_pos.x <= goal.x)
                                              : (ball_pos.x >= goal.x);
    return past_line && fabs(ball_pos.y - goal.y) < goal_width / 2.0;
}

double perp_dist_to_line(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = vec2_sub(b, a);
    double len = vec2_len(ab);
    // a and b are approximately equal
    if (len < 1e-9)
        return vec2_dist(p, a);
    Vec2 ap = vec2_sub(p, a);
    double cross = ab.x * ap.y - ab.y * ap.x;
    return fabs(cross) / len;
}

/***************************************************
 Segment-based boundary model. This is for the next phase where we
 actually have a board representation that isn't purely rectangular.

 THIS IS STILL EXPERIMENTAL! I'm still trying to figure out if this is
 the best way to represent the field!
***************************************************/

BoundarySegment make_straight_segment(Vec2 a, Vec2 b) {
    BoundarySegment s = {0};
    s.is_arc = 0;
    s.a = a;
    s.b = b;
    return s;
}

BoundarySegment make_arc_segment(Vec2 center, double radius_cm, Vec2 a, Vec2 b) {
    BoundarySegment s = {0};
    s.is_arc = 1;
    s.a = a;
    s.b = b;
    s.arc_center = center;
    s.radius_cm = radius_cm;
    s.angle_start_rad = angle_of(vec2_sub(a, center));
    s.angle_end_rad = angle_of(vec2_sub(b, center));
    return s;
}

void field_model_finalize(FieldModel *field) {
    Vec2 lo = field->segments[0].a, hi = field->segments[0].a;
    for (int i = 0; i < field->n_segments; i++) {
        Vec2 pts[2] = {field->segments[i].a, field->segments[i].b};
        for (int k = 0; k < 2; k++) {
            if (pts[k].x < lo.x)
                lo.x = pts[k].x;
            if (pts[k].y < lo.y)
                lo.y = pts[k].y;
            if (pts[k].x > hi.x)
                hi.x = pts[k].x;
            if (pts[k].y > hi.y)
                hi.y = pts[k].y;
        }
    }
    field->bbox_min = lo;
    field->bbox_max = hi;
}

Vec2 closest_point_on_straight(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = vec2_sub(b, a);
    double len2 = vec2_dot(ab, ab);
    if (len2 < 1e-9)
        return a;
    double t = vec2_dot(vec2_sub(p, a), ab) / len2;
    t = clampd(t, 0.0, 1.0);
    return vec2_add(a, vec2_scale(ab, t));
}

// Is `angle` within the arc's span, going from angle_start to angle_end?
int angle_in_arc_span(double angle, double angle_start, double angle_end) {
    double span = angle_end - angle_start;
    while (span < 0)
        span += 2.0 * M_PI;
    double rel = angle - angle_start;
    while (rel < 0)
        rel += 2.0 * M_PI;
    while (rel >= 2.0 * M_PI)
        rel -= 2.0 * M_PI;
    return rel <= span;
}

Vec2 closest_point_on_arc(Vec2 p, const BoundarySegment *seg) {
    double ang = angle_of(vec2_sub(p, seg->arc_center));
    if (angle_in_arc_span(ang, seg->angle_start_rad, seg->angle_end_rad)) {
        // Project radially onto the curve itself
        Vec2 dir = vec2_normalize(vec2_sub(p, seg->arc_center));
        if (dir.x == 0 && dir.y == 0)
            dir.x = 1;
        return vec2_add(seg->arc_center, vec2_scale(dir, seg->radius_cm));
    }
    return (vec2_dist(p, seg->a) < vec2_dist(p, seg->b)) ? seg->a : seg->b;
}

Vec2 closest_point_on_segment(Vec2 p, const BoundarySegment *seg) {
    return seg->is_arc ? closest_point_on_arc(p, seg)
                       : closest_point_on_straight(p, seg->a, seg->b);
}

WallQuery make_wall_query(Vec2 pos, Vec2 closest_pt, double dist) {
    WallQuery q;
    q.distance = dist;
    q.direction = vec2_normalize(vec2_sub(pos, closest_pt));
    return q;
}

void consider_segment(
    Vec2 pos, const FieldModel *field, int i, int *best,
    Vec2 *best_pt, double *best_dist, int *have_best
) {
    Vec2 pt = closest_point_on_segment(pos, &field->segments[i]);
    double dist = vec2_dist(pos, pt);
    if (!*have_best || dist < *best_dist) {
        *best = i;
        *best_pt = pt;
        *best_dist = dist;
        *have_best = 1;
    }
}

WallQuery nearest_wall(
    Vec2 pos, const FieldModel *field,
    double safe_margin_cm, int *hint
) {
    double safe = fmin(pos.x - field->bbox_min.x, field->bbox_max.x - pos.x);
    safe = fmin(safe, fmin(pos.y - field->bbox_min.y, field->bbox_max.y - pos.y));
    if (safe > safe_margin_cm) {
        WallQuery q = {safe, {0, 0}};
        return q;
    }

    int best = -1, have_best = 0;
    Vec2 best_pt = {0, 0};
    double best_dist = 0;
    int have_hint = hint && *hint >= 0 && *hint < field->n_segments;

    if (have_hint) {
        int n = field->n_segments;
        int prev = *hint;
        consider_segment(
            pos, field, (prev - 1 + n) % n, &best,
            &best_pt, &best_dist, &have_best
        );
        consider_segment(
            pos, field, prev, &best, &best_pt,
            &best_dist, &have_best
        );
        consider_segment(
            pos, field, (prev + 1) % n, &best,
            &best_pt, &best_dist, &have_best
        );
    } else {
        for (int i = 0; i < field->n_segments; i++)
            consider_segment(
                pos, field, i, &best, &best_pt,
                &best_dist, &have_best
            );
    }

    if (hint)
        *hint = best;
    return make_wall_query(pos, best_pt, best_dist);
}
