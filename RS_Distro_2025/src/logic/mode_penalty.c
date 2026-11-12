/***************************************************
 CSC C85 RoboSoccer Mode 1: Penalty shot
***************************************************/

/***************************************************
 Strictly logic. World in, Motion out, no BT_*, no struct RoboAI/blob, no
 track_agents()/world_from_ai(). This is what makes it linkable (and
 testable) without the image-capture/SDL/Bluetooth chain -- see
 src/simulatte. mode_penalty_adapter.c is the other half: the real-world
 wrapper that actually touches the robot.
***************************************************/

#include "mode_penalty.h"
#include "ai_config.h"
#include "fsm.h"
#include "geometry.h"
#include "skills.h"
#include "world.h"
#include <stdio.h>

// **** multifaceted (shared) staging/aim plan, set by replan() and read by several states ****
static Vec2 g_aim_target;
static Vec2 g_staging_point;
static Vec2 g_ball_at_plan;

// Used by replan() and next_align()'s "did the plan go stale" recheck, so
// the two never disagree about what the staging point should be. Ball
// away from any wall: stage behind it on the line to their_goal, same as
// always. Ball near a wall: can't do that, push it off the wall first.
// once it's clear, the next planning stage will fall back to the normal
// branch for sCORING GOALS.
Vec2 compute_staging(const World *w, Vec2 *aim_out) {
    // We have the separate staging for id we're near a wall
    if (near_wall(w->ball.pos, w->field_min, w->field_max, DEFAULT_WALL_MARGIN_CM))
        return plan_wall_push_staging(
            w->ball.pos, w->field_min, w->field_max,
            STAGE_DIST_CM, MIN_WALL_STAGE_CM, aim_out
        );
    // WE ARE AIMING TOWARDS THEIR GOAL!!!
    // PULVERIZE THOSE WHO STAND BEFORE US RAHHHHHHHHHHHHHHH
    *aim_out = w->their_goal;
    return plan_waypoint(
        w->ball.pos, w->their_goal, STAGE_DIST_CM,
        w->field_min, w->field_max, DEFAULT_WALL_MARGIN_CM
    );
}

void replan(const World *w) {
    // Our replan cycle. Store the position of the ball at the time
    // the new plan is concocted
    g_staging_point = compute_staging(w, &g_aim_target);
    g_ball_at_plan = w->ball.pos;
}

// Set by mode_penalty_step() right before calling the active state's run().
// This is set to true on precisely on the first frame of that state. Basically,
// whenever we enter a new state, this is set to 1 so we reset, and after that,
// it is set to 0.
static int g_entered = 0;

// *********************** P_READY *****************************************
SkillStatus run_ready(World *w, Motion *out) {
    // This just stalls until the ball and the bot are visible
    return skill_hold(w, out);
}

int next_ready(World *w, SkillStatus s, double t) {
    (void)s;
    (void)t;
    static int frames = 0;
    if (w->self.valid && w->ball.valid)
        frames++;
    else
        frames = 0;
    if (frames >= READY_FRAMES) {
        frames = 0;
        replan(w);
        return P_GOTO_STAGING;
    }
    return P_READY;
}

// ***** P_GOTO_STAGING *****
SkillStatus run_goto_staging(World *w, Motion *out) {
    return skill_goto(w, g_staging_point, out);
}

static int g_align_attempts = 0;

int next_goto_staging(World *w, SkillStatus s, double t) {
    if (vec2_dist(w->ball.pos, g_ball_at_plan) > MOVE_TOL_CM) {
        replan(w);
        return P_GOTO_STAGING;
    }
    if (s == SKILL_DONE) {
        g_align_attempts++;
        return P_ALIGN;
    }
    if (t > GOTO_STAGING_TIMEOUT_S) {
        g_align_attempts = 0;
        return P_REPOSITION;
    }
    return P_GOTO_STAGING;
}

// ***** P_ALIGN *****
SkillStatus run_align(World *w, Motion *out) {
    return skill_face(w, g_aim_target, g_entered, out);
}

int next_align(World *w, SkillStatus s, double t) {
    // Was: recompute the "ideal" staging point fresh every frame and compare
    // it to self.pos (> 1.5*STAGE_TOL_CM -> replan). That compounds TWO
    // independent noisy quantities (self-position noise AND the recomputed
    // geometry's own sensitivity to ball-position noise), so it could stay
    // just over threshold from noise alone with the ball never actually
    // moving -- bouncing GOTO_STAGING<->ALIGN forever. Use the same,
    // simpler, more robust criterion GOTO_STAGING itself uses: did the ball
    // actually move, not "does self.pos look slightly off from a fresh
    // recompute."
    if (vec2_dist(w->ball.pos, g_ball_at_plan) > MOVE_TOL_CM) {
        replan(w);
        return P_GOTO_STAGING;
    }
    if (s == SKILL_DONE) {
        double lat = perp_dist_to_line(w->self.pos, w->ball.pos, g_aim_target);
        if (lat < LAT_TOL_CM) {
            g_align_attempts = 0; // real progress
            return P_STRIKE;
        }
        // Heading's stable but we drifted sideways of the aim line more than
        // plan_waypoint() should ever allow -- stay and let ALIGN_TIMEOUT_S
        // catch it if this somehow doesn't resolve itself.
    }
    // t_in_state resets every time we re-enter ALIGN, so ALIGN_TIMEOUT_S
    // alone never accumulates across repeated short GOTO_STAGING<->ALIGN
    // bounces -- that loop could otherwise run forever with no escape at
    // all. g_align_attempts is the escape valve that survives the bounce.
    if (t > ALIGN_TIMEOUT_S || g_align_attempts > ALIGN_ATTEMPT_LIMIT) {
        g_align_attempts = 0;
        return P_REPOSITION;
    }
    return P_ALIGN;
}

// ************************************************************--- P_STRIKE --
SkillStatus run_strike(World *w, Motion *out) {
    return skill_strike(w, g_aim_target, g_entered, out);
}

int next_strike(World *w, SkillStatus s, double t) {
    (void)w;
    if (s == SKILL_DONE)
        return P_FOLLOW_THROUGH;
    if (s == SKILL_FAILED || t > STRIKE_TIMEOUT_S)
        return P_REPOSITION;
    return P_STRIKE;
}

// ******************************************************** P_FOLLOW_THROUGH --
SkillStatus run_follow_through(World *w, Motion *out) {
    return skill_follow_through(w, g_aim_target, g_entered, out);
}

int next_follow_through(World *w, SkillStatus s, double t) {
    (void)w;
    (void)t;
    if (s == SKILL_DONE)
        return P_SHOT_DONE;
    return P_FOLLOW_THROUGH;
}

// ************************************************************ P_REPOSITION --
SkillStatus run_reposition(World *w, Motion *out) {
    return skill_backoff(w, REPO_DIST_CM, g_entered, out);
}

int next_reposition(World *w, SkillStatus s, double t) {
    (void)t;
    if (s == SKILL_DONE) {
        replan(w);
        return P_GOTO_STAGING;
    }
    return P_REPOSITION;
}

// ********************************************************-- P_WALL_RECOVER --
SkillStatus run_wall_recover(World *w, Motion *out) {
    return skill_leave_wall(w, out);
}

int next_wall_recover(World *w, SkillStatus s, double t) {
    (void)t;
    if (s == SKILL_DONE) {
        replan(w);
        return P_GOTO_STAGING;
    }
    return P_WALL_RECOVER;
}

// ************************************************************--- P_UNSTICK --
SkillStatus run_unstick(World *w, Motion *out) {
    return skill_unstick(w, g_entered, out);
}

int next_unstick(World *w, SkillStatus s, double t) {
    (void)s;
    (void)t;
    if (w->self.valid && vec2_len(w->self.vel) > STALL_EPS_CMPS) {
        replan(w);
        return P_GOTO_STAGING;
    }
    return P_UNSTICK;
}

// ************************************************************- P_SHOT_DONE --
SkillStatus run_shot_done(World *w, Motion *out) {
    return skill_hold(w, out);
}

int next_shot_done(World *w, SkillStatus s, double t) {
    (void)s;
    double field_center_x = (w->field_min.x + w->field_max.x) / 2.0;
    int at_rest = !w->ball.valid || vec2_len(w->ball.vel) < STALL_EPS_CMPS;
    int in_goal =
        w->ball.valid && ball_in_goal(w->ball.pos, w->their_goal,
                                      w->goal_width_cm, field_center_x);
    if (w->ball.valid && at_rest && !in_goal && t > RETRY_WAIT_S) {
        replan(w);
        return P_GOTO_STAGING;
    }
    return P_SHOT_DONE;
}

// ****************************************************************- P_LOST --
SkillStatus run_lost(World *w, Motion *out) {
    return skill_hold(w, out);
}

int next_lost(World *w, SkillStatus s, double t) {
    (void)s;
    (void)t;
    static int frames = 0;
    if (w->self.valid && w->ball.valid)
        frames++;
    else
        frames = 0;
    if (frames >= READY_FRAMES) {
        frames = 0;
        return P_READY;
    }
    return P_LOST;
}

// ************************************************************************---

static const StateDef g_penalty_states[] = {
    {P_READY, "P_READY", run_ready, next_ready, 0},
    {P_GOTO_STAGING, "P_GOTO_STAGING", run_goto_staging, next_goto_staging, GOTO_STAGING_TIMEOUT_S},
    {P_ALIGN, "P_ALIGN", run_align, next_align, ALIGN_TIMEOUT_S},
    {P_STRIKE, "P_STRIKE", run_strike, next_strike, STRIKE_TIMEOUT_S},
    {P_FOLLOW_THROUGH, "P_FOLLOW_THROUGH", run_follow_through,
     next_follow_through, 0},
    {P_REPOSITION, "P_REPOSITION", run_reposition, next_reposition, 0},
    {P_WALL_RECOVER, "P_WALL_RECOVER", run_wall_recover, next_wall_recover, 0},
    {P_UNSTICK, "P_UNSTICK", run_unstick, next_unstick, 0},
    {P_SHOT_DONE, "P_SHOT_DONE", run_shot_done, next_shot_done, 0},
    {P_LOST, "P_LOST", run_lost, next_lost, 0},
};
#define N_PENALTY_STATES                                                       \
    (int)(sizeof(g_penalty_states) / sizeof(g_penalty_states[0]))

// **** global preemptive moves, checked before the state table every frame.
// Priority when more than one fires: wall_recover > unstick > lost > goal
int check_preempt(int state, const World *w, double now) {
    if (state != P_WALL_RECOVER && state != P_FOLLOW_THROUGH && w->self.valid) {
        double wall_margin = near_wall(w->ball.pos, w->field_min, w->field_max, DEFAULT_WALL_MARGIN_CM)
                ? MIN_WALL_STAGE_CM
                : DEFAULT_WALL_MARGIN_CM;
        if (near_wall(w->self.pos, w->field_min, w->field_max, wall_margin)) {
            Vec2 in = direction_away_from_nearest_wall(w->self.pos, w->field_min, w->field_max);
            if (vec2_dot(w->self.heading, in) < 0)
                return P_WALL_RECOVER; // heading outward
        }
    }

    static double stall_t = 0, stall_last_now = -1;
    double dt = stall_last_now < 0 ? 0 : (now - stall_last_now);
    stall_last_now = now;
    int stationary_state =
        (state == P_READY || state == P_LOST || state == P_SHOT_DONE);
    if (!w->self.valid || stationary_state ||
        vec2_len(w->self.vel) > STALL_EPS_CMPS) {
        stall_t = 0;
    } else {
        stall_t += dt;
    }
    if (stall_t >= UNSTICK_STALL_S && state != P_UNSTICK)
        return P_UNSTICK;

    if (state != P_STRIKE && state != P_FOLLOW_THROUGH &&
        state != P_SHOT_DONE && state != P_LOST &&
        ((w->self.valid == 0 && w->self.lost_frames > LOST_FRAMES) ||
         (w->ball.valid == 0 && w->ball.lost_frames > BALL_LOST_FRAMES))) {
        return P_LOST;
    }

    if (state != P_SHOT_DONE && w->ball.valid) {
        double field_center_x = (w->field_min.x + w->field_max.x) / 2.0;
        if (ball_in_goal(w->ball.pos, w->their_goal, w->goal_width_cm,
                         field_center_x))
            return P_SHOT_DONE;
    }

    return -1;
}

static int prev_state = -1;
static double t_entered = 0;

int mode_penalty_fsm_step(World *world, int state, double now, Motion *out_motion) {
    int target = check_preempt(state, world, now);
    if (target >= 0 && target != state) {
        fsm_log_transition(
            g_penalty_states, N_PENALTY_STATES,
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

    const StateDef *row = fsm_find(g_penalty_states, N_PENALTY_STATES, state);
    if (!row) {
        fprintf(stderr, "mode_penalty_fsm_step(): unknown state %d, holding\n",
                state);
        out_motion->v = 0;
        out_motion->w = 0;
        return state;
    }

    SkillStatus s = row->run(world, out_motion);

    int next_id = row->next(world, s, t_in_state);
    if (next_id != state) {
        fsm_log_transition(g_penalty_states, N_PENALTY_STATES, state, next_id,
                           now, "state");
    }
    return next_id;
}
