/***************************************************
 CSC C85 RoboSoccer Mode 0: Soccer (the real combined mode)
***************************************************/

#include "mode_soccer.h"
#include "ai_config.h"
#include "fsm.h"
#include "geometry.h"
#include "skills.h"
#include "stall.h"
#include "world.h"
#include <math.h>
#include <stdio.h>

// Set by mode_soccer_step() right before calling the active state's run().
static int g_entered = 0;
// Forward speed commanded last frame, for the stall detector.
static double g_last_cmd_v = 0;

// **** attack/defend decision, re-evaluated every tick ****
//
// Defend our own half,
// attack theirs. No opponent-distance-to-ball comparison yet (we don't
// have real data on what opponents actually do).
static int should_defend(const World *w) {
    if (!w->ball.valid)
        return 0; // nothing to defend against; let ATTACK's own P_READY wait for it
    return vec2_dist(w->ball.pos, w->own_goal) < vec2_dist(w->ball.pos, w->their_goal);
}

// **** S_READY ****
static SkillStatus run_ready(World *w, Motion *out) {
    return skill_hold(w, out);
}

static int next_ready(World *w, SkillStatus s, double t) {
    (void)s;
    (void)t;
    static int frames = 0;
    if (w->self.valid && w->ball.valid) {
        frames++;
    } else {
        frames--;
        if (frames < 0) frames = 0;
    }
    if (frames < READY_FRAMES)
        return S_READY;
    frames = 0;
    return should_defend(w) ? S_ATTACK : S_ATTACK;
}

// **** S_ATTACK ****
// Used to delegate wholesale to mode_penalty_fsm_step() (stage behind the
// ball, stop and spin in place to align, then strike). That is the right
// shape for an uncontested penalty kick, but with an opponent on the field
// and no time pressure in our favour, stopping to align just looks like
// the bot freezing and spinning instead of going for the ball - every other
// bot we saw just drives straight at it. So this is the same continuous,
// always-turning-while-driving shape as mode_chase's skill_chase, just
// steering through the ball at their_goal instead of at the ball itself,
// so it still generally aims instead of only bumping the ball sideways.
// No staging, no align phase, no abort-on-bad-angle: it reacts every frame
// from whatever angle it's actually at, the same way chase does.
static SkillStatus run_attack(World *w, Motion *out) {
    Vec2 steer_to = w->their_goal;
    // Tried twice to make S_ATTACK aware of "ball near a wall" and both
    // attempts were wrong in ways only caught by writing an actual
    // deterministic test (simulatte/test_wall_push.c) - not something
    // worth guessing a third time with no real-robot testing left before
    // the demo. Reverted to the simple version: always aim through the
    // ball at their_goal. Doesn't solve the wall-fling problem, but it's
    // the last version that was actually validated, and "unsolved" beats
    // "confidently wrong" the day of a demo. mode_penalty's
    // plan_wall_push_staging() already solves this correctly for penalty
    // mode (it stages to the wall side FIRST, as a discrete step, which
    // is what a continuous version needs to replicate correctly and
    // didn't) - worth porting properly after the demo, with time to test.
    if (w->ball.valid) {
        // Aiming STRIKE_CARROT_CM beyond the ball (toward goal) is fine
        // from a distance - the difference from aiming at the ball itself
        // is negligible at range. Close in it's not: if self isn't
        // already sitting exactly on the ball->goal line (it usually
        // isn't), aiming past the ball instead of at it steers the final
        // approach to clip the ball off-centre rather than centre it in
        // the pouch - with the front prong extending much further than
        // the back, a small aim error here becomes a much bigger lateral
        // miss at the actual contact point than at the tracked centroid.
        // Confirmed in a real log: repeated glancing contacts at a
        // near-constant self heading, ball deflected at a different,
        // inconsistent angle each time - a flick, not a capture. Shrink
        // the carrot to 0 (aim AT the ball) as we close in, so the final
        // approach centres the hit instead of clipping it.
        double ball_d = vec2_dist(w->self.pos, w->ball.pos);
        double carrot = STRIKE_CARROT_CM;
        if (ball_d < BALL_CAPTURE_DIST_CM)
            carrot *= ball_d / BALL_CAPTURE_DIST_CM;
        Vec2 line = vec2_normalize(vec2_sub(w->their_goal, w->ball.pos));
        steer_to = vec2_add(w->ball.pos, vec2_scale(line, carrot));
    }
    // Detour around the opponent if the straight line to steer_to runs
    // through it, same plan_avoid_point() P_GOTO_STAGING uses for the
    // ball. Re-evaluated fresh every frame (no detour_done tracking like
    // P_GOTO_STAGING has) because both self and the opponent keep moving
    // here, unlike the stationary-ball case that tracking was for.
    //
    // BUT: skip the detour entirely when the opponent is right on top of
    // steer_to itself (e.g. marking the ball) - confirmed by testing
    // against a stationary opponent that plan_avoid_point's side choice
    // then has nothing stable to anchor to and flips frame to frame
    // (no committed-side memory, unlike P_GOTO_STAGING), so self just
    // juddered in place for 15+ seconds instead of closing in. The
    // OPP_DANGER_CM speed cap below still guarantees we don't ram it -
    // this only gives up on steering AROUND it in this one tight case.
    if (w->opp.valid && vec2_dist(steer_to, w->opp.pos) >= OPP_AVOID_CLEAR_CM) {
        steer_to = plan_avoid_point(w->self.pos, steer_to, w->opp.pos,
                                    OPP_AVOID_BLOCK_CM, OPP_AVOID_CLEAR_CM,
                                    w->field_min, w->field_max);
    }
    SkillStatus s = skill_chase(w, steer_to, out);

    // skill_chase has no distance-based slowdown of its own (see
    // ai_config.h's BALL_CAPTURE_DIST_CM comment) - taper OUR speed once
    // close to the actual ball (not steer_to, which sits STRIKE_CARROT_CM
    // beyond it) so contact is a controlled touch into the pouch instead
    // of a full-speed kick, without slowing down the long approach.
    if (w->ball.valid) {
        double ball_d = vec2_dist(w->self.pos, w->ball.pos);
        if (ball_d < BALL_CAPTURE_DIST_CM) {
            double frac = BALL_CAPTURE_MIN_V_FRAC +
                (1.0 - BALL_CAPTURE_MIN_V_FRAC) * (ball_d / BALL_CAPTURE_DIST_CM);
            out->v *= frac;
        }
    }
    return s;
}

static int next_attack(World *w, SkillStatus s, double t) {
    (void)s;
    (void)t;
    return should_defend(w) ? S_ATTACK : S_ATTACK;
}

// **** S_DEFEND, skill_mark / skill_intercept / skill_clear ****
static SkillStatus run_defend(World *w, Motion *out) {
    if (!w->ball.valid)
        return skill_mark(w, g_entered, out); // nothing to clear/intercept, just block
    if (vec2_dist(w->ball.pos, w->own_goal) < DEFEND_DANGER_DIST_CM)
        return skill_clear(w, g_entered, out);
    double toward_goal = vec2_len(w->ball.vel) > 1e-6
        ? vec2_dot(w->ball.vel, vec2_normalize(vec2_sub(w->own_goal, w->ball.pos)))
        : 0.0;
    if (toward_goal > BALL_APPROACH_SPEED_CMPS)
        return skill_intercept(w, g_entered, out);
    return skill_mark(w, g_entered, out);
}

static int next_defend(World *w, SkillStatus s, double t) {
    (void)s;
    (void)t;
    return should_defend(w) ? S_ATTACK : S_ATTACK;
}

// **** S_WALL_RECOVER / S_UNSTICK (only reachable from S_DEFEND/S_READY) ****
static SkillStatus run_wall_recover(World *w, Motion *out) {
    return skill_leave_wall(w, out, g_entered);
}

static int next_wall_recover(World *w, SkillStatus s, double t) {
    (void)w;
    (void)t;
    return (s == SKILL_DONE) ? S_ATTACK : S_WALL_RECOVER;
}

static SkillStatus run_unstick(World *w, Motion *out) {
    return skill_unstick(w, g_entered, out);
}

static int next_unstick(World *w, SkillStatus s, double t) {
    (void)w;
    (void)s;
    if (t >= UNSTICK_REVERSE_S + UNSTICK_TURN_S)
        return S_ATTACK;
    return S_UNSTICK;
}

// **** S_LOST ****
static SkillStatus run_lost(World *w, Motion *out) {
    return skill_hold(w, out);
}

static int next_lost(World *w, SkillStatus s, double t) {
    (void)s;
    (void)t;
    static int frames = 0;
    if (w->self.valid && w->ball.valid) {
        frames++;
    } else {
        frames--;
        if (frames < 0)
            frames = 0;
    }
    if (frames >= READY_FRAMES) {
        frames = 0;
        return S_READY;
    }
    return S_LOST;
}

// ****************************************************************************
static const StateDef g_soccer_states[] = {
    {S_READY, "S_READY", run_ready, next_ready, 0},
    {S_ATTACK, "S_ATTACK", run_attack, next_attack, 0},
    {S_DEFEND, "S_DEFEND", run_defend, next_defend, 0},
    {S_WALL_RECOVER, "S_WALL_RECOVER", run_wall_recover, next_wall_recover, 0},
    {S_UNSTICK, "S_UNSTICK", run_unstick, next_unstick, 0},
    {S_LOST, "S_LOST", run_lost, next_lost, 0},
};
#define N_SOCCER_STATES (int)(sizeof(g_soccer_states) / sizeof(g_soccer_states[0]))

// **** global preemptive moves, checked before the state table every frame.
// S_ATTACK used to skip all of this because it delegated wholesale to
// mode_penalty_fsm_step(), which ran its own wall-recover/unstick/lost
// internally. Now that S_ATTACK is a plain skill_chase() call with no such
// protection of its own, it needs these checks like every other state.
static int soccer_check_preempt(int state, const World *w, double now) {
    if (state != S_WALL_RECOVER && w->self.valid &&
        !near_goal_mouth(w->self.pos, WALL_TRIGGER_MARGIN_CM, w->field_min,
                         w->field_max, w->own_goal, w->their_goal, w->goal_width_cm)) {
        if (near_wall(w->self.pos, w->field_min, w->field_max, WALL_TRIGGER_MARGIN_CM)) {
            Vec2 in = direction_away_from_nearest_wall(w->self.pos, w->field_min, w->field_max);
            if (vec2_dot(w->self.heading, in) < -WALL_OUTWARD_DOT)
                return S_WALL_RECOVER;
        }
    }

    // Detect stalls
    static StallDetector stall;
    int stationary_state = (state == S_READY || state == S_LOST || state == S_UNSTICK);
    if (stationary_state)
        stall_reset(&stall);
    int stalled = stall_update(&stall, now, w->self.pos,
                               w->self.valid && !stationary_state, g_last_cmd_v);
    if (stalled && state != S_UNSTICK) {
        stall_reset(&stall);
        return S_UNSTICK;
    }

    if (state != S_LOST &&
        ((w->self.valid == 0 && w->self.lost_frames > LOST_FRAMES) ||
         (w->ball.valid == 0 && w->ball.lost_frames > BALL_LOST_FRAMES))) {
        return S_LOST;
    }

    return -1;
}

static int prev_state = -1;
static double t_entered = 0;

int mode_soccer_fsm_step(World *world, int state, double now, Motion *out_motion) {
    int target = soccer_check_preempt(state, world, now);
    if (target >= 0 && target != state) {
        fsm_log_transition(
            g_soccer_states, N_SOCCER_STATES,
            state, target, now, "pre-empt"
        );
        state = target;
    }

    if (state != prev_state) {
        prev_state = state;
        t_entered = now;
        g_entered = 1;
    } else {
        g_entered = 0;
    }
    double t_in_state = now - t_entered;

    const StateDef *row = fsm_find(g_soccer_states, N_SOCCER_STATES, state);
    if (!row) {
        fprintf(stderr, "mode_soccer_fsm_step(): unknown state %d, holding\n",
                state);
        out_motion->v = 0;
        out_motion->w = 0;
        return state;
    }

    SkillStatus s = row->run(world, out_motion);

    // Unconditional safety net: never accelerate into the opponent,
    // regardless of which state is active or what it decided to do (see
    // ai_config.h's OPP_DANGER_CM comment). Applied after every state's
    // run(), not just S_ATTACK's own detour, since that detour only
    // covers where S_ATTACK chooses to steer, not a sudden close
    // encounter in S_DEFEND or anywhere else.
    if (world->opp.valid && world->self.valid) {
        double opp_dist = vec2_dist(world->self.pos, world->opp.pos);
        if (opp_dist < OPP_DANGER_CM) {
            double err_deg = fabs(rad_to_deg(heading_err_rad(world, world->opp.pos)));
            if (err_deg < OPP_DANGER_DEG && out_motion->v > OPP_BACKOFF_V_CMPS)
                out_motion->v = OPP_BACKOFF_V_CMPS;
        }
    }
    g_last_cmd_v = out_motion->v;

    int next_id = row->next(world, s, t_in_state);
    if (next_id != state) {
        fsm_log_transition(g_soccer_states, N_SOCCER_STATES, state, next_id,
                           now, "state");
    }
    return next_id;
}
