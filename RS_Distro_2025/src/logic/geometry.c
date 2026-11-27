#include "geometry.h"
#include "ai_config.h"
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

// Both goals sit ON the end walls, so driving in to score necessarily
// means being right at the x-boundary near_wall() flags - this is the
// one place on a wall where heading "outward" is correct, not a hazard,
// and no state should wall-recover there. Checks x is near either end
// wall AND y is within the goal mouth at that end - a wall-trigger
// elsewhere along the same end wall (outside the goal width) still
// counts as a real wall.
int near_goal_mouth(const Vec2 pos, double wall_margin, Vec2 field_min,
                     Vec2 field_max, Vec2 own_goal, Vec2 their_goal,
                     double goal_width_cm) {
    int at_end_wall = pos.x < field_min.x + wall_margin ||
                       pos.x > field_max.x - wall_margin;
    if (!at_end_wall)
        return 0;
    double half = goal_width_cm / 2.0;
    return fabs(pos.y - own_goal.y) < half || fabs(pos.y - their_goal.y) < half;
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

double footprint_wall_dist(Vec2 pos, Vec2 heading, Vec2 field_min, Vec2 field_max,
                           Vec2 *inward_out) {
    // The bot is a full width block plus a narrower prong box in front.
    // Against straight walls the closest point of that shape is always one of
    // its outer corners, so checking these 6 is exact. The prong box's back
    // corners sit on the block's front edge, so they never win.
    Vec2 side = {-heading.y, heading.x};
    double fwd[6] = {BOT_FRONT_CM, BOT_FRONT_CM,
                     BOT_BODY_FRONT_CM, BOT_BODY_FRONT_CM,
                     -BOT_REAR_CM, -BOT_REAR_CM};
    double lat[6] = {BOT_PRONG_HALF_WIDTH_CM, -BOT_PRONG_HALF_WIDTH_CM,
                     BOT_HALF_WIDTH_CM, -BOT_HALF_WIDTH_CM,
                     BOT_HALF_WIDTH_CM, -BOT_HALF_WIDTH_CM};
    double best = 1e9;
    for (int i = 0; i < 6; i++) {
        Vec2 c = vec2_add(pos, vec2_add(vec2_scale(heading, fwd[i]), vec2_scale(side, lat[i])));
        double d = dist_to_nearest_wall(c, field_min, field_max);
        if (d < best) {
            best = d;
            if (inward_out)
                *inward_out = direction_away_from_nearest_wall(c, field_min, field_max);
        }
    }
    return best;
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

Vec2 plan_defend_point(
    Vec2 threat_pos, Vec2 own_goal, double stand_dist,
    Vec2 field_min, Vec2 field_max, double margin
) {
    Vec2 toward = vec2_normalize(vec2_sub(threat_pos, own_goal));
    // If the threat is sitting right on our own goal.
    if (toward.x == 0 && toward.y == 0)
        toward.x = 1;
    Vec2 stand = vec2_add(own_goal, vec2_scale(toward, stand_dist));
    return clamp_to_field(stand, field_min, field_max, margin);
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

int ball_in_goal(
    Vec2 ball_pos, Vec2 goal, double goal_width,
    double field_center_x, double ball_radius
) {
    int past_line = (goal.x < field_center_x) ? (ball_pos.x <= goal.x + ball_radius)
                                              : (ball_pos.x >= goal.x - ball_radius);
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

Vec2 fix_heading_sign(
    Vec2 raw, Vec2 vel, double trust_speed_cmps, Vec2 *last_good,
    int *flip_streak, int confirm_frames
) {
    // Resolve raw's 180deg ambiguity against the committed direction
    // first instead of vel.
    Vec2 aligned = raw;
    // If the angle between raw and *last_good > 90, but only check if
    // last_good was actually set (it starts out as 0 for the first frame).
    if (vec2_len(*last_good) > 1e-6 && vec2_dot(raw, *last_good) < 0)
        aligned = vec2_scale(raw, -1);

    // Only sustained (HEADING_FLIP_CONFIRM_FRAMES consecutive) disagreement
    // from vel is allowed to actually change the committed direction. A
    // single frame's vel is too noisy in DIRECTION to trust on its own,
    // even past trust_speed_cmps in magnitude
    if (vec2_len(vel) > trust_speed_cmps && vec2_dot(aligned, vel) < 0) {
        (*flip_streak)++;
        if (*flip_streak >= confirm_frames) {
            aligned = vec2_scale(aligned, -1);
            *flip_streak = 0;
        }
    } else {
        *flip_streak = 0;
    }

    *last_good = aligned;
    return aligned;
}

// distance from point p to the segment a-b
static double seg_point_dist(Vec2 a, Vec2 b, Vec2 p) {
    Vec2 ab = vec2_sub(b, a);
    double l2 = vec2_dot(ab, ab);;
    double t = (l2 < 1e-9) ? 0.0 : vec2_dot(vec2_sub(p, a), ab) / l2;
    t = fmax(0.0, fmin(1.0, t));
    return vec2_dist(p, vec2_add(a, vec2_scale(ab, t)));
}

Vec2 plan_avoid_point(
    Vec2 from, Vec2 to, Vec2 obstacle, double block_cm,
    double clear_cm, Vec2 field_min, Vec2 field_max
) {
    // If self drives straight from `from` to `to`, does that
    // line pass too close to an obstacle (for instance, the ball)?
    // If so, return a detour waypoint. Used in P_GOTO_STAGING, so that
    // the bot doesn't drive over the ball on its way to the staging point
    // and accidentally knock it out of position.

    // Check if we're even close to said obstacle to begin with
    // We do `perp_dist_to_line` by hand instead of calling the function
    // beause can reuse len here instrad of `perp_dist_to_line` having to
    // recalculate it each time. Small optimization I suppose.
    Vec2 seg = vec2_sub(to, from);
    double len = vec2_len(seg);
    if (len < 1e-6)
        return to;
    Vec2 d = vec2_scale(seg, 1.0 / len);
    Vec2 rel = vec2_sub(obstacle, from);
    double along = vec2_dot(rel, d);
    // signed perpendicular distance
    double lateral = rel.x * d.y - rel.y * d.x;
    // Only skip the detour if the destination is practically on the obstacle.
    // obstacle must lie between the endpoints
    if (
        vec2_dist(to, obstacle) < 5.0 ||
        (along < 0 || along > len) ||
        fabs(lateral) >= block_cm
    )
        return to;

    // Two candidate waypoints, one on each side of the obstacle->destination
    // line. Each is clamped inside the field with room beyond the wall-recover
    // trigger margin. Clamping can squash a candidate back next to the obstacle,
    // so judge each candidate by the clearance its whole two-leg PATH has from
    // the obstacle, prefer the side the bot is already on, and take the other
    // side when that one is not clear enough.
    Vec2 od = vec2_sub(to, obstacle);
    double odl = vec2_len(od);
    Vec2 odn = (odl > 1e-6) ? vec2_scale(od, 1.0 / odl) : d;
    Vec2 p = {-odn.y, odn.x};
    double via_margin = DEFAULT_WALL_MARGIN_CM + 6.0;
    Vec2 cand[2];
    double clr[2]; // store clearances
    cand[0] = clamp_to_field(vec2_add(obstacle, vec2_scale(p, clear_cm)), field_min, field_max, via_margin);
    cand[1] = clamp_to_field(vec2_sub(obstacle, vec2_scale(p, clear_cm)), field_min, field_max, via_margin);
    for (int i = 0; i < 2; i++)
        clr[i] = fmin(seg_point_dist(from, cand[i], obstacle), seg_point_dist(cand[i], to, obstacle));
    // Pick a side
    double on_p = vec2_dot(vec2_sub(from, obstacle), p);
    int pref = (fabs(on_p) > 3.0) ? (on_p > 0 ? 0 : 1) : (clr[0] >= clr[1] ? 0 : 1);
    int other = 1 - pref;
    if (clr[pref] >= block_cm)
        return cand[pref];
    if (clr[other] >= block_cm)
        return cand[other];
    return (clr[pref] >= clr[other]) ? cand[pref] : cand[other];
}
