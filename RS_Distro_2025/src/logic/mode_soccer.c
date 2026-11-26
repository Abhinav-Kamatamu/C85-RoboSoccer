/***************************************************
 CSC C85 RoboSoccer Mode 0: Soccer (the real combined mode)
***************************************************/

#include "mode_soccer.h"
#include "mode_penalty.h"
#include "ai_config.h"
#include "fsm.h"
#include "geometry.h"
#include "skills.h"
#include "stall.h"
#include "world.h"
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
    return should_defend(w) ? S_DEFEND : S_ATTACK;
}

// **** S_ATTACK is a sub-FSM, mode_penalty_fsm_step() wholesale ****
// penalty_sub_state is private for this file. mode_soccer_fsm_step() must
// never return it directly as it lives in the 101-199 range, which
// roboAI.c's dispatch would then route to mode_penalty_step() instead of
// mode_soccer_step() next frame. Only S_ATTACK itself (1-99) is ever
// returned upward.
static int penalty_sub_state = P_READY;

static SkillStatus run_attack(World *w, Motion *out) {
    if (g_entered)
        penalty_sub_state = P_READY; // initiate a fresh sequence every time we (re-)enter ATTACK
    penalty_sub_state = mode_penalty_fsm_step(w, penalty_sub_state, w->now, out);
    // mode_penalty_fsm_step() already drives everything about this state
    // (including its own wall-recover/unstick/lost/shot-done), so there is
    // nothing for next_attack() to react to beyond "should we even still
    // be attacking", so this is always RUNNING as far as mode_soccer is
    // concerned.
    return SKILL_RUNNING;
}

static int next_attack(World *w, SkillStatus s, double t) {
    (void)s;
    (void)t;
    // Don't abandon an active shot attempt just because the ball
    // looks like it's back in our half mid-strike (position
    // noise, or the ball bouncing off something during the shot itself).
    if (penalty_sub_state == P_STRIKE || penalty_sub_state == P_FOLLOW_THROUGH)
        return S_ATTACK;
    return should_defend(w) ? S_DEFEND : S_ATTACK;
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
    return should_defend(w) ? S_DEFEND : S_ATTACK;
}

// **** S_WALL_RECOVER / S_UNSTICK (only reachable from S_DEFEND/S_READY) ****
static SkillStatus run_wall_recover(World *w, Motion *out) {
    return skill_leave_wall(w, out, g_entered);
}

static int next_wall_recover(World *w, SkillStatus s, double t) {
    (void)w;
    (void)t;
    return (s == SKILL_DONE) ? S_DEFEND : S_WALL_RECOVER;
}

static SkillStatus run_unstick(World *w, Motion *out) {
    return skill_unstick(w, g_entered, out);
}

static int next_unstick(World *w, SkillStatus s, double t) {
    (void)w;
    (void)s;
    if (t >= UNSTICK_REVERSE_S + UNSTICK_TURN_S)
        return S_DEFEND;
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
// Deliberately skipped while S_ATTACK is active. mode_penalty_fsm_step()
// already runs its own wall-recover/unstick/lost internally, and this
// would just be fighting it for state ownership having not delegated the
// motion itself.
static int soccer_check_preempt(int state, const World *w, double now) {
    if (state == S_ATTACK)
        return -1;

    if (state != S_WALL_RECOVER && w->self.valid) {
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
    g_last_cmd_v = out_motion->v;

    int next_id = row->next(world, s, t_in_state);
    if (next_id != state) {
        fsm_log_transition(g_soccer_states, N_SOCCER_STATES, state, next_id,
                           now, "state");
    }
    return next_id;
}
