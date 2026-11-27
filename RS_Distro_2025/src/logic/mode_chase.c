/***************************************************
 CSC C85 RoboSoccer Mode 2: Chase
***************************************************/

#include "mode_chase.h"
#include "ai_config.h"
#include "fsm.h"
#include "geometry.h"
#include "skills.h"
#include "stall.h"
#include "world.h"
#include <math.h>
#include <stdio.h>

// Set by mode_chase_step() right before calling the active state's run().
// Set to true on the first frame of a new state
static int g_entered = 0;
// Forward speed commanded last frame, for the stall detector.
static double g_last_cmd_v = 0;

// **** C_READY ****
SkillStatus chase_run_ready(World *w, Motion *out) {
    return skill_hold(w, out);
}

int chase_next_ready(World *w, SkillStatus s, double t) {
    (void)s;
    (void)t;
    static int frames = 0;
    if (w->self.valid && w->ball.valid) {
        frames++;
    } else {
        frames--;
        if (frames < 0) frames = 0;
    }
    if (frames >= READY_FRAMES) {
        frames = 0;
        return C_CHASE;
    }
    return C_READY;
}

// **** C_CHASE ****
SkillStatus run_chase(World *w, Motion *out) {
    // An invalid ball has pos (0,0), i.e. the top-left
    // CORNER. Steering at that during a dropout drove the bot across the
    // field and into the walls Keep heading for where the ball was last seen instead.
    static Vec2 last_ball = {0, 0};
    static int have_ball = 0;
    if (w->ball.valid) {
        last_ball = w->ball.pos;
        have_ball = 1;
    }
    if (!have_ball) {
        out->v = 0;
        out->w = 0;
        return SKILL_RUNNING;
    }
    Vec2 target = clamp_to_field(last_ball, w->field_min, w->field_max, CHASE_EDGE_MARGIN_CM);
    return skill_chase(w, target, out);
}

static int next_chase(World *w, SkillStatus s, double t) {
    (void)w;
    (void)s;
    (void)t;
    return C_CHASE;
}

// **** C_WALL_RECOVER ****
static SkillStatus run_wall_recover(World *w, Motion *out) {
    return skill_leave_wall(w, out, g_entered);
}

static int next_wall_recover(World *w, SkillStatus s, double t) {
    (void)w;
    (void)t;
    if (s == SKILL_DONE)
        return C_CHASE;
    return C_WALL_RECOVER;
}

// **** C_UNSTICK ****
SkillStatus chase_run_unstick(World *w, Motion *out) {
    return skill_unstick(w, g_entered, out);
}

int chase_next_unstick(World *w, SkillStatus s, double t) {
    (void)s;
    (void)w;
    // One full reverse+turn cycle.
    if (t >= UNSTICK_REVERSE_S + UNSTICK_TURN_S)
        return C_CHASE;
    return C_UNSTICK;
}

// ****C_LOST ****
SkillStatus chase_run_lost(World *w, Motion *out) {
    return skill_hold(w, out);
}

int chase_next_lost(World *w, SkillStatus s, double t) {
    (void)s;
    (void)t;
    static int frames = 0;
    if (w->self.valid && w->ball.valid) {
        frames++;
    } else {
        frames--;
        if (frames < 0) frames = 0;
    }
    if (frames >= READY_FRAMES) {
        frames = 0;
        return C_READY;
    }
    return C_LOST;
}

// ****************************************************************************
static const StateDef g_chase_states[] = {
    {C_READY, "C_READY", chase_run_ready, chase_next_ready, 0},
    {C_CHASE, "C_CHASE", run_chase, next_chase, 0},
    {C_WALL_RECOVER, "C_WALL_RECOVER", run_wall_recover, next_wall_recover, 0},
    {C_UNSTICK, "C_UNSTICK", chase_run_unstick, chase_next_unstick, 0},
    {C_LOST, "C_LOST", chase_run_lost, chase_next_lost, 0},
};
#define N_CHASE_STATES (int)(sizeof(g_chase_states) / sizeof(g_chase_states[0]))

// **** global preemptive moves, checked before the state table every frame.
// Priority when more than one triggers is wall_recover > unstick > lost.
int chase_check_preempt(int state, const World *w, double now) {
    // While the ball is right in front of us we are mid-contact. Wall-recover
    // pivots the bot in place, and pivoting with the ball at the nose
    // knocks it sideways, along the wall and off the table. So don't
    // recover while closing on a ball that is ahead, unless we're about to
    // go over the edge ourselves (HARD margin).
    int engaging_ball = 0;
    if (w->self.valid && w->ball.valid) {
        double d = vec2_dist(w->self.pos, w->ball.pos);
        double err = fabs(angle_diff_rad(angle_of(vec2_sub(w->ball.pos, w->self.pos)),
                                         angle_of(w->self.heading)));
        engaging_ball = d < ENGAGE_BALL_CM && err < deg_to_rad(ENGAGE_BALL_DEG) &&
                        !near_wall(w->self.pos, w->field_min, w->field_max,
                                   HARD_WALL_MARGIN_CM);
    }
    static StallDetector wall_stall;
    if (state == C_WALL_RECOVER || engaging_ball) {
        stall_reset(&wall_stall);
    } else if (wall_stuck_update(&wall_stall, w, g_last_cmd_v)) {
        stall_reset(&wall_stall);
        return C_WALL_RECOVER;
    }

    // It was REALLY hard to figure out how to not STALL OMG, but now we have a StallDetector
    // that can DETECT stalls.
    static StallDetector stall;
    int stationary_state = (state == C_READY || state == C_LOST || state == C_UNSTICK);
    if (stationary_state)
        stall_reset(&stall);
    int stalled = stall_update(&stall, now, w->self.pos,
                               w->self.valid && !stationary_state, g_last_cmd_v);
    if (stalled && state != C_UNSTICK) {
        stall_reset(&stall);
        return C_UNSTICK;
    }

    // TODO: We have a tall robot, which can actually hide the ball. Additionally, RoboDictador's
    // bot is REALLY wide, and that might also hide the ball, so maybe we should disambiguate betweem
    // self-is-lost, ball-is-lost, and opponent-is-lost
    if (
        state != C_LOST &&
        ((w->self.valid == 0 && w->self.lost_frames > LOST_FRAMES) ||
         (w->ball.valid == 0 && w->ball.lost_frames > BALL_LOST_FRAMES))
    ) {
        return C_LOST;
    }

    return -1;
}

static int prev_state = -1;
static double t_entered = 0;

int mode_chase_fsm_step(World *world, int state, double now, Motion *out_motion) {
    int target = chase_check_preempt(state, world, now);
    if (target >= 0 && target != state) {
        fsm_log_transition(
            g_chase_states, N_CHASE_STATES,
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

    const StateDef *row = fsm_find(g_chase_states, N_CHASE_STATES, state);
    if (!row) {
        fprintf(stderr, "mode_chase_fsm_step(): unknown state %d, holding\n",
                state);
        out_motion->v = 0;
        out_motion->w = 0;
        return state;
    }

    SkillStatus s = row->run(world, out_motion);
    g_last_cmd_v = out_motion->v;

    int next_id = row->next(world, s, t_in_state);
    if (next_id != state) {
        fsm_log_transition(g_chase_states, N_CHASE_STATES, state, next_id,
                           now, "state");
    }
    return next_id;
}
