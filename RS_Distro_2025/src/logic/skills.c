#include "skills.h"
#include "ai_config.h"
#include "geometry.h"
#include <math.h>

double heading_err_rad(const World *w, Vec2 target) {
    double desired = angle_of(vec2_sub(target, w->self.pos));
    return angle_diff_rad(desired, angle_of(w->self.heading));
}

SkillStatus skill_goto(const World *w, Vec2 target, Motion *out) {
    double dist = vec2_dist(w->self.pos, target);
    if (dist < STAGE_TOL_CM) {
        out->v = 0;
        out->w = 0;
        return SKILL_DONE;
    }
    // The error is the target theta - our current theta
    double err_rad = heading_err_rad(w, target);
    // The angular velocity we want to enact to correct the error in angle
    double w_cmd = clampd(HEADING_TURN_GAIN * err_rad, -MAX_W_RADPS, MAX_W_RADPS);
    // Slowing factor for when we're near where we want to go to
    double slow = dist < GOTO_SLOWDOWN_CM ? dist / GOTO_SLOWDOWN_CM : 1.0;\
    // The linear velocity we want to enact this frame
    double v_cmd = GOTO_V_CMPS * slow * fmax(0.0, cos(err_rad));
    out->v = v_cmd;
    out->w = w_cmd;
    return SKILL_RUNNING;
}

SkillStatus skill_face(
    const World *w, Vec2 target, int reset, Motion *out
) {
    static int stable_count = 0;

    if (reset)
        stable_count = 0;

    // The error of our direction is the target theta - our current theta
    double err_rad = heading_err_rad(w, target);
    double err_deg = rad_to_deg(err_rad);

    // rotate in place
    out->v = 0;
    out->w = clampd(HEADING_TURN_GAIN * err_rad, -MAX_W_RADPS, MAX_W_RADPS);

    // If we're withing a small margin
    if (fabs(err_deg) < ALIGN_TOL_DEG) {
        // Basically, once we are within range, we rotate just a tiny bit
        // more, for a set amount of frames, which increases the accuracy of
        // rotation.
        stable_count++;
        if (stable_count >= ALIGN_STABLE_FRAMES) {
            stable_count = 0;
            out->w = 0;
            return SKILL_DONE;
        }
    } else {
        stable_count = 0;
    }
    return SKILL_RUNNING;
}

SkillStatus skill_strike(
    const World *w, Vec2 aim_target, int reset, Motion *out
) {
    static double last_ball_dist = 1e9;

    if (reset)
        last_ball_dist = 1e9;

    // The error of our direction is the target theta - our current theta
    double err_deg = rad_to_deg(heading_err_rad(w, aim_target));
    // If we're trying to strike and the error is too great, then don't
    // attempt it
    if (fabs(err_deg) > ABORT_TOL_DEG) {
        out->v = 0;
        out->w = 0;
        return SKILL_FAILED;
    }

    if (!w->ball.valid) {
        // The ball vanished mid-strike. If the ball was close ahead in front of us,
        // then that could potentially be contact and not a tracking failure. This state
        // is exempt from the global ball-lost pre-emptive action for this reason
        if (last_ball_dist < CLOSE_AHEAD_CM) {
            out->v = 0;
            out->w = 0;
            last_ball_dist = 1e9;
            return SKILL_DONE;
        }
        // Otherwise, we lost the ball and it was far away. We keep driving straight rather
        // than flailing on a n-frame glitch, where n is (hopefully) small; this is why we
        // have a timeout when setting an active skill. The skills own timeout will trigger
        // a real failure if we don't strike soon enough.
        out->v = STRIKE_V_CMPS;
        out->w = 0;
        return SKILL_RUNNING;
    }

    // We calculate how far we are from the ball using the last distance
    // we recorded
    last_ball_dist = vec2_dist(w->self.pos, w->ball.pos);
    // We get the (unit vector) direction in which we want to aim the ball
    Vec2 aim_dir = vec2_normalize(vec2_sub(aim_target, w->ball.pos));
    // Get the speed of the ball (magnitude), assuming it will follow a straightish
    // line to our target.
    double speed_along = vec2_dot(w->ball.vel, aim_dir);

    // If the ball is moving this fast already, then we consider ourselves
    // to have striked it already
    if (speed_along > STRIKE_V_MIN_CMPS) {
        out->v = 0;
        out->w = 0;
        last_ball_dist = 1e9;
        return SKILL_DONE;
    }

    // We want to strike the ball at this velocity
    out->v = STRIKE_V_CMPS;
    // Whatever rotation we need to do to fix the error in heading
    out->w = clampd(HEADING_TURN_GAIN * deg_to_rad(err_deg), -MAX_W_RADPS, MAX_W_RADPS);
    return SKILL_RUNNING;
}

SkillStatus skill_follow_through(
    const World *w, Vec2 aim_dir_target, int reset, Motion *out
) {
    static double traveled_cm = 0, elapsed_s = 0, last_t = -1;

    if (reset) {
        traveled_cm = 0;
        elapsed_s = 0;
        last_t = w->now;
    }
    // dt is the # of frames since we started our "follow_through" skill.
    double dt = last_t < 0 ? 0 : (w->now - last_t);
    last_t = w->now;
    // Distance travelled = velocity * time 
    traveled_cm += vec2_len(w->self.vel) * dt;
    elapsed_s += dt;

    // If we've followed through for long enough in distance or in time, or
    // we're near a wall, then we stop following through 
    if (
        traveled_cm >= FT_DIST_CM || elapsed_s >= FT_T_S ||
        near_wall(
            w->self.pos, w->field_min, w->field_max,
            DEFAULT_WALL_MARGIN_CM
        )
    ) {
        out->v = 0;
        out->w = 0;
        return SKILL_DONE;
    }

    // The error of our direction is the target theta - our current theta
    double err_rad = heading_err_rad(w, aim_dir_target);
    // We want to keep folowing through with this linear speed
    out->v = FT_V_CMPS;
    // We keep adjusting our rotation to fix whatever error there might be]
    // in our rotation
    out->w = clampd(HEADING_TURN_GAIN * err_rad, -MAX_W_RADPS, MAX_W_RADPS);
    return SKILL_RUNNING;
}

SkillStatus skill_backoff(
    const World *w, double dist_cm, int reset, Motion *out
) {
    static double traveled_cm = 0, last_t = -1;

    if (reset) {
        traveled_cm = 0;
        last_t = w->now;
    }
    // dt is the # of frames since we started our "backoff" skill.
    double dt = last_t < 0 ? 0 : (w->now - last_t);
    last_t = w->now;
    // Distance travelled = velocity * time 
    traveled_cm += vec2_len(w->self.vel) * dt;

    // If we've travelled further than how much we've requested to,
    // then stop
    if (traveled_cm >= dist_cm) {
        out->v = 0;
        out->w = 0;
        return SKILL_DONE;
    }

    // In this case, we assume that we are already facing the direction that
    // we want to move in, so there is presumably no error in our orientation
    // to correct. We want to keep backing off at this speed.
    out->v = REPO_V_CMPS;
    out->w = 0;
    return SKILL_RUNNING;
}

SkillStatus skill_hold(const World *w, Motion *out) {
    // For the intellisense
    (void)w;
    // The hold skill does literally nothing. This is the default
    // active skill. We change to another skill when there's something to do
    // and return if we're done and there's nothing else pending
    out->v = 0;
    out->w = 0;
    return SKILL_RUNNING;
}

SkillStatus skill_leave_wall(const World *w, Motion *out) {
    // If we're not near a wall, then there's no nearby wall to leave
    // from, so we're done
    if (!near_wall(
        w->self.pos, w->field_min, w->field_max,
        DEFAULT_WALL_MARGIN_CM
    )) {
        out->v = 0;
        out->w = 0;
        return SKILL_DONE;
    }
    Vec2 n = direction_away_from_nearest_wall(
        w->self.pos, w->field_min, w->field_max
    );
    // The error of our direction is the target theta - our current theta
    double err_rad = angle_diff_rad(angle_of(n), angle_of(w->self.heading));
    // We want to leave the wall with this speed
    out->v = WALL_RECOVER_V_CMPS;
    // We keep adjusting our rotation to fix whatever error there might be]
    // in our rotation
    out->w = clampd(HEADING_TURN_GAIN * err_rad, -MAX_W_RADPS, MAX_W_RADPS);
    return SKILL_RUNNING;
}

SkillStatus skill_unstick(const World *w, int reset, Motion *out) {
    static int phase = 0;
    static double phase_t = 0, last_t = -1;

    if (reset) {
        phase = 0;
        phase_t = 0;
        last_t = w->now;
    }
    // dt is the # of frames since we started our "unstick" skill.
    double dt = last_t < 0 ? 0 : (w->now - last_t);
    last_t = w->now;
    phase_t += dt;

    // We alternate between two phases for unsticking: Reverse, and turn
    if (phase == 0) {
        out->v = UNSTICK_V_CMPS;
        out->w = 0;
        // If we've reversed for long enough, then go into turn phase
        if (phase_t >= UNSTICK_REVERSE_S) {
            phase = 1;
            phase_t = 0;
        }
    } else {
        out->v = 0;
        out->w = UNSTICK_W_RADPS;
        // If we've tuned long enough, then go into reverse phase
        if (phase_t >= UNSTICK_TURN_S) {
            phase = 0;
            phase_t = 0;
        }
    }
    return SKILL_RUNNING;
}
