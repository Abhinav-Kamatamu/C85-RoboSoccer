#include "skills.h"
#include "ai_config.h"
#include "geometry.h"
#include <math.h>

double heading_err_rad(const World *w, Vec2 target) {
    double desired = angle_of(vec2_sub(target, w->self.pos));
    return angle_diff_rad(desired, angle_of(w->self.heading));
}

// Once confidently turning one way (|err_rad| outside the deadzone), *committed_sign
// keeps track of that direction. Inside the deadzone, this returns err_rad with
// its sign made to match the last confident choice instead of
// whatever a almost-noisy reading says this frame, so a skill can't
// flip its commanded turn direction every frame right at the
// discontinuity.
double stabilize_turn_sign(double err_rad, double *committed_sign) {
    if (fabs(err_rad) > M_PI - deg_to_rad(ANTIPODAL_DEADZONE_DEG))
        return copysign(fabs(err_rad), *committed_sign);
    *committed_sign = (err_rad >= 0) ? 1.0 : -1.0;
    return err_rad;
}

// **** struct for step-and-probe in-place turning****
typedef struct {
    int phase;            // 0 idle, 1 pulse, 2 settle
    double t0;            // phase start time
    double dur;           // pulse length
    double sign;          // +1 / -1 turn direction of this pulse
    double start_hdg;     // heading at pulse start
    double rate;          // calculated achieved turn rate (rad/s) per unit pulse
    double omega;         // filtered measured angular rate (rad/s)
} TurnBurst;

#define OMEGA_BASELINE_S 0.2
#define OMEGA_HIST 12
// History of angular velocityes {timestamps, heading, n = current count}
static double g_oh_t[OMEGA_HIST], g_oh_h[OMEGA_HIST];
static int g_oh_n = 0;
static TurnBurst g_tb = {0, 0, 0, 1, 0, TURN_RATE_INIT_RADPS, 0};

void turn_burst_reset(void) {
    g_tb.phase = 0;
    g_oh_n = 0;
    g_tb.omega = 0;
}

// Track the measured turn rate. Call once per frame whenever turning is active.
// Differencing consecutive frames is useless at 30fps. 3deg of heading noise
// is +/-1.6 rad/s of fake "rotation". Use the heading change over a ~0.2 s
// baseline instead (noise / 0.2 s: ~0.25 rad/s, thanks CLT).
void turn_omega_update(const World *w) {
    double hdg = angle_of(w->self.heading);
    if (g_oh_n > 0 && w->now - g_oh_t[g_oh_n - 1] > 0.25)
        g_oh_n = 0; // There's a gap, start over
    if (g_oh_n == OMEGA_HIST) {
        for (int i = 1; i < OMEGA_HIST; i++) {
            g_oh_t[i - 1] = g_oh_t[i];
            g_oh_h[i - 1] = g_oh_h[i];
        }
        g_oh_n--;
    }
    g_oh_t[g_oh_n] = w->now;
    g_oh_h[g_oh_n] = hdg;
    g_oh_n++;
    // oldest sample at least OMEGA_BASELINE_S back (or the oldest we have)
    int k = 0;
    for (int i = g_oh_n - 1; i >= 0; i--) {
        if (w->now - g_oh_t[i] >= OMEGA_BASELINE_S) {
            k = i;
            break;
        }
    }
    double dt = w->now - g_oh_t[k];
    g_tb.omega = (dt >= 0.1) ? angle_diff_rad(hdg, g_oh_h[k]) / dt : 0.0;
}

// Step-and-probe in-place turn, fires one pulse whose size is proportiona to err_rad and
// the learned turn rate, waits for the bot to actually stop moving, then
// measures how far that pulse turned it to refine the rate estimate
// before planning the next one. err_rad is the signed heading error
// (target - current). Sets v=0 and sets w.
void turn_burst(const World *w, double err_rad, Motion *out) {
    double now = w->now;
    double hdg = angle_of(w->self.heading);
    out->v = 0;
    out->w = 0;
    if (g_tb.phase == 0) {
        g_tb.sign = (err_rad >= 0) ? 1.0 : -1.0;
        g_tb.dur = clampd(TURN_UNDERSHOOT * fabs(err_rad) / g_tb.rate,
                          TURN_MIN_PULSE_S, TURN_MAX_PULSE_S);
        g_tb.t0 = now;
        g_tb.start_hdg = hdg;
        g_tb.phase = 1;
    }
    if (g_tb.phase == 1) {
        // stop early if we already crossed the target
        int crossed = (err_rad * g_tb.sign) <= 0;
        if (now - g_tb.t0 >= g_tb.dur || crossed) {
            g_tb.phase = 2;
            g_tb.t0 = now;
        } else {
            out->w = g_tb.sign * TURN_PULSE_W_RADPS;
            return;
        }
    }
    // Probe. hold still until the bot has really stopped, then calculate how far
    // that pulse actually turned us
    double waited = now - g_tb.t0;
    if ((waited >= TURN_SETTLE_S && fabs(g_tb.omega) < TURN_STILL_RADPS) ||
        waited >= TURN_SETTLE_MAX_S) {
        double achieved = g_tb.sign * angle_diff_rad(hdg, g_tb.start_hdg);
        if (achieved > 0.02) {
            double r = achieved / g_tb.dur;
            g_tb.rate = clampd(0.6 * g_tb.rate + 0.4 * r, 0.5, TURN_RATE_MAX_RADPS);
        } else {
            g_tb.rate = fmax(0.5, g_tb.rate * 0.75); // pulse did nothing (static friction)
        }
        g_tb.phase = 0;
    }
}

SkillStatus skill_goto(const World *w, Vec2 target, int reset, Motion *out) {
    static double committed_sign = 1.0;
    static double closest = 1e9;
    static Vec2 last_target = {1e9, 1e9};
    if (reset || vec2_dist(target, last_target) > 1.0) {
        if (reset) committed_sign = 1.0;
        closest = 1e9;
    }
    last_target = target;

    turn_omega_update(w);
    double dist = vec2_dist(w->self.pos, target);
    if (dist < closest)
        closest = dist;
    // Arrived, we are inside the tolerance, or got close and we're now moving away
    if (dist < STAGE_TOL_CM || (closest < PASSBY_CM && dist > closest + 1.5)) {
        out->v = 0;
        out->w = 0;
        return SKILL_DONE;
    }
    // The error is the target theta - our current theta
    double err_rad = stabilize_turn_sign(heading_err_rad(w, target), &committed_sign);
    // The angular velocity we want to enact to correct the error in angle
    double w_cmd = clampd(HEADING_TURN_GAIN * err_rad, -MAX_W_RADPS, MAX_W_RADPS);
    // Slowing factor for when we're near where we want to go to
    double slow = dist < GOTO_SLOWDOWN_CM ? dist / GOTO_SLOWDOWN_CM : 1.0;
    // The linear velocity we want to enact this frame
    double v_cmd = GOTO_V_CMPS * slow * fmax(0.0, cos(err_rad));
    // Close and badly misaligned. Turn first, and don't orbit (WHY does it do ts).
    if (dist < ORBIT_ZONE_CM && fabs(rad_to_deg(err_rad)) > ORBIT_ERR_DEG) {
        turn_burst(w, err_rad, out);
        return SKILL_RUNNING;
    }
    turn_burst_reset();
    out->v = v_cmd;
    out->w = w_cmd;
    return SKILL_RUNNING;
}

SkillStatus skill_chase(const World *w, Vec2 target, Motion *out) {
    static double committed_sign = 1.0;
    // The error is the target theta - our current theta
    double err_rad = stabilize_turn_sign(heading_err_rad(w, target), &committed_sign);
    // The angular velocity we want to enact to correct the error in angle
    double w_cmd = clampd(HEADING_TURN_GAIN * err_rad, -MAX_W_RADPS, MAX_W_RADPS);
    // Ball in the pouch. Steer GENTLY, ignore wobble.
    // TODO: In an actual game with a real opponent, we prob can't afford to move slowly.
    // Maybe consider adding a physics engine so we can probabalistically calculate the balls
    // spread of movements at different speeds and settle on a blend of fast enough and precise
    // enough
    if (vec2_dist(w->self.pos, target) < CARRY_DIST_CM &&
        fabs(rad_to_deg(err_rad)) < 35.0) {
        if (fabs(rad_to_deg(err_rad)) < CARRY_DEADBAND_DEG)
            w_cmd = 0;
        else
            w_cmd *= CARRY_GAIN_SCALE;
    }
    // The linear velocity we want to enact this frame
    double v_cmd = GOTO_V_CMPS * (0.5 + 0.5 * cos(err_rad));
    out->v = v_cmd;
    out->w = w_cmd;
    return SKILL_RUNNING;
}

SkillStatus skill_face(
    const World *w, Vec2 target, int reset, Motion *out
) {
    static int stable_count = 0;
    static double committed_sign = 1.0;

    if (reset) {
        stable_count = 0;
        committed_sign = 1.0;
        turn_burst_reset();
    }
    turn_omega_update(w);

    // The error of our direction is the target theta - our current theta
    double err_rad = stabilize_turn_sign(heading_err_rad(w, target), &committed_sign);
    double err_deg = rad_to_deg(err_rad);
    int within = fabs(err_deg) < ALIGN_TOL_DEG;

    out->v = 0;
    out->w = 0;
    // An active pulse-n-probe cycle always runs to completion. The heading
    // sweeping through the tolerance band mid-coast must NOT cancel the probing.
    // Only cut a pulse short once it has reached the band.
    if (g_tb.phase != 0) {
        if (g_tb.phase == 1 && within) {
            g_tb.phase = 2;
            g_tb.t0 = w->now;
        }
        turn_burst(w, err_rad, out);
        stable_count = 0;
        return SKILL_RUNNING;
    }
    if (within) {
        // Idle, inside the band and not rotating. We consider ourselves DONE.
        if (fabs(g_tb.omega) < TURN_STILL_RADPS) {
            stable_count++;
            if (stable_count >= ALIGN_STABLE_FRAMES)
                return SKILL_DONE;
        } else {
            stable_count = 0;
        }
        return SKILL_RUNNING;
    }
    stable_count = 0;
    turn_burst(w, err_rad, out);
    return SKILL_RUNNING;
}

SkillStatus skill_strike(
    const World *w, Vec2 aim_target, int reset, Motion *out
) {
    static double last_ball_dist = 1e9;
    static Vec2 ball_start = {0, 0};
    static int have_start = 0;

    if (reset) {
        last_ball_dist = 1e9;
        have_start = 0;
    }
    // Where the ball was sitting when the strike began, to detect that WE
    // moved it.
    if (!have_start && w->ball.valid) {
        ball_start = w->ball.pos;
        have_start = 1;
    }

    // Steer at a point just beyond the ball on the ball->aim line
    // Falls back to aim_target itself if the ball is
    // not visible (no line to project onto).
    Vec2 steer_to = aim_target;
    if (w->ball.valid) {
        Vec2 line = vec2_normalize(vec2_sub(aim_target, w->ball.pos));
        steer_to = vec2_add(w->ball.pos, vec2_scale(line, STRIKE_CARROT_CM));
    }
    // The error of our direction is the target theta - our current theta
    double err_deg = rad_to_deg(heading_err_rad(w, steer_to));
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

    // Struck = the ball has really moved from where it sat when we began. The
    // old test was "ball velocity along the aim > STRIKE_V_MIN_CMPS", but the
    // ball velocity is a frame-to-frame position difference. Again, at 30fps, half a
    // cm of blob jitter is 15 cm/s, so the strike was being declared done
    // after ~0.17 s with the bot still 27 cm from the ball. Mucho Mal
    if (vec2_dist(w->ball.pos, ball_start) > STRIKE_MOVED_CM) {
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

// 1 if the bot's nose (BOT_FRONT_CM ahead of the tracked point along its
// heading, plus a little) is within FT_EDGE_STOP_CM of leaving the field.
// follow-through used to stop when the tracked point was within
// DEFAULT_WALL_MARGIN_CM (18) of ANY wall, which ended it after a couple frames
// whenever the shot started near a side wallm even though the bot was
// driving away from that wall.
static int nose_near_edge(const World *w) {
    Vec2 nose = vec2_add(w->self.pos, vec2_scale(w->self.heading, BOT_FRONT_CM));
    return nose.x < w->field_min.x + FT_EDGE_STOP_CM || nose.x > w->field_max.x - FT_EDGE_STOP_CM ||
           nose.y < w->field_min.y + FT_EDGE_STOP_CM || nose.y > w->field_max.y - FT_EDGE_STOP_CM;
}

SkillStatus skill_follow_through(
    const World *w, Vec2 aim_dir_target, int reset, Motion *out
) {
    static Vec2 start = {0, 0};
    static double t_start = 0;

    if (reset) {
        start = w->self.pos;
        t_start = w->now;
    }
    // Displacement from the start of the follow-through (We do not integrate
    // |vel| because it is extremely noisy).
    double traveled_cm = vec2_dist(w->self.pos, start);
    double elapsed_s = w->now - t_start;

    // If we've followed through for long enough in distance or in time, or
    // we're near a wall, then we stop following through 
    if (
        traveled_cm >= FT_DIST_CM || elapsed_s >= FT_T_S ||
        nose_near_edge(w)
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
    static Vec2 start = {0, 0};
    static double t_start = 0, dir = -1.0; // direction, -1 reverse, +1 forward

    if (reset) {
        start = w->self.pos;
        t_start = w->now;
        // Back off away from the ball. Blindly reversing drove the bot's
        // rear into the ball whenever the ball was behind it (it was
        // booting the ball with its back after an align timeout). If the
        // ball is in the rear-half plane, drive forward instead (away
        // from it); otherwise reverse as before.
        dir = -1.0;
        if (w->ball.valid) {
            Vec2 to_ball = vec2_sub(w->ball.pos, w->self.pos);
            if (vec2_dot(w->self.heading, to_ball) < 0)
                dir = 1.0;
        }
    }

    // Distance moved = displacement from where we started. Integrating
    // |vel| over time (the old way) counts position jitter (3-9 cm/s on a
    // stationary bot) as travel and could finish before the bot had moved.
    double moved = vec2_dist(w->self.pos, start);
    if (moved >= dist_cm || (w->now - t_start) > BACKOFF_TIMEOUT_S) {
        out->v = 0;
        out->w = 0;
        return SKILL_DONE;
    }

    // No rotation. We move straight along the line we are already facing.
    out->v = dir * fabs(REPO_V_CMPS);
    out->w = 0;
    return SKILL_RUNNING;
}

SkillStatus skill_hold(const World *w, Motion *out) {
    (void)w;
    // The hold skill does literally nothing. This is the default
    // active skill. We change to another skill when there's something to do
    // and return if we're done and there's nothing else pending
    out->v = 0;
    out->w = 0;
    return SKILL_RUNNING;
}

SkillStatus skill_leave_wall(const World *w, Motion *out, int reset) {
    static double committed_sign = 1.0;
    if (reset) {
        committed_sign = 1.0;
        turn_burst_reset();
    }
    if (!near_wall(
        w->self.pos, w->field_min, w->field_max,
        WALL_RECOVER_CLEAR_MARGIN_CM
    )) {
        out->v = 0;
        out->w = 0;
        turn_burst_reset();
        return SKILL_DONE;
    }
    Vec2 n = direction_away_from_nearest_wall(
        w->self.pos, w->field_min, w->field_max
    );
    double hdg = angle_of(w->self.heading);
    // Error to face away from the wall (drive forward out) and error to face
    // into it (drive backward out). Reversing means the tail is in front, so a bot that is
    // already facing the wall leaves by backing up, and there is no turning needed.
    double err_fwd = angle_diff_rad(angle_of(n), hdg);
    double err_rev = angle_diff_rad(angle_of(vec2_scale(n, -1.0)), hdg);

    double a_fwd = fabs(err_fwd), a_rev = fabs(err_rev);
    double lim = deg_to_rad(WALL_DRIVE_CONE_DEG);
    if (a_fwd <= lim || a_rev <= lim) {
        // Roughly facing away from (or into) the wall => just drive out.
        turn_burst_reset();
        // Never reverse into the ball. if it is close behind us, the tail
        // would boot it Turn to face away from the wall instead.
        int ball_behind = 0;
        if (w->ball.valid) {
            Vec2 tb = vec2_sub(w->ball.pos, w->self.pos);
            ball_behind = vec2_len(tb) < (BOT_REAR_CM + BALL_RADIUS_CM + 12.0) &&
                          vec2_dot(w->self.heading, tb) < 0;
        }
        if (a_fwd > a_rev && ball_behind) {
            turn_burst(w, err_fwd, out);
            return SKILL_RUNNING;
        }
        if (a_fwd <= a_rev) {
            double e = stabilize_turn_sign(err_fwd, &committed_sign);
            out->v = WALL_RECOVER_V_CMPS * fmax(0.6, cos(e));
            out->w = clampd(HEADING_TURN_GAIN * e, -MAX_W_RADPS, MAX_W_RADPS);
        } else {
            out->v = -WALL_RECOVER_V_CMPS * fmax(0.6, cos(err_rev));
            out->w = clampd(HEADING_TURN_GAIN * err_rev, -MAX_W_RADPS, MAX_W_RADPS);
        }
        return SKILL_RUNNING;
    }
    // Heading is roughly parallel to the wall. Neither driving nor backing
    // gets us out. Turn in place, to whichever of "face away" / "face the wall"
    // is nearer, then drive.
    turn_burst(w, (a_fwd <= a_rev) ? err_fwd : err_rev, out);
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
