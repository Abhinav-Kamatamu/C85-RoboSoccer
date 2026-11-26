/***************************************************
 CSC C85 RoboSoccer Mode 1: Penalty shot
***************************************************/

#include "mode_penalty.h"
#include "ai_config.h"
#include "fsm.h"
#include "geometry.h"
#include "skills.h"
#include "stall.h"
#include "world.h"
#include <stdio.h>

// **** multifaceted (shared) staging/aim plan, set by replan() and read by several states ****
static Vec2 g_aim_target;
static Vec2 g_staging_point;
static Vec2 g_ball_at_plan;

// Where self should park (and which way to aim) before approaching the
// ball to shoot. Normal case: STAGE_DIST_CM directly behind the ball on
// the line to their_goal and clamped off the walls. Ball pinned against a
// wall: that spot would be off the table, so fall back to
// plan_wall_push_staging() instead. Get beside it and push it off the
// wall first. Once clear, the next replan() falls back to the normal
// goal-scoring branch above.
// Also used by replan() and next_align()'s recheck for plan staleness,
// so the two don't disagree about what the staging point should be.
Vec2 compute_staging(const World *w, Vec2 *aim_out) {
    if (near_wall(w->ball.pos, w->field_min, w->field_max, WALL_PUSH_BAND_CM))
        return plan_wall_push_staging(
            w->ball.pos, w->field_min, w->field_max,
            STAGE_DIST_CM, MIN_WALL_STAGE_CM, aim_out
        );
    // WE ARE AIMING TOWARDS THEIR GOAL!!!
    // PULVERIZE THOSE WHO STAND BEFORE US RAHHHHHHHHHHHHHHH
    *aim_out = w->their_goal;
    Vec2 face = vec2_normalize(vec2_sub(w->their_goal, w->ball.pos));
    Vec2 staging = vec2_sub(w->ball.pos, vec2_scale(face, STAGE_DIST_CM));
    // Margins per wall. The wall behind the staging point (facing inward)
    // only has to clear the tail; the wall in front has to clear the nose.
    double m_left  = (face.x >  0.5) ? WALL_TAIL_MARGIN_CM : (face.x < -0.5 ? WALL_NOSE_MARGIN_CM : WALL_TAIL_MARGIN_CM);
    double m_right = (face.x < -0.5) ? WALL_TAIL_MARGIN_CM : (face.x >  0.5 ? WALL_NOSE_MARGIN_CM : WALL_TAIL_MARGIN_CM);
    double m_top   = (face.y >  0.5) ? WALL_TAIL_MARGIN_CM : (face.y < -0.5 ? WALL_NOSE_MARGIN_CM : WALL_TAIL_MARGIN_CM);
    double m_bot   = (face.y < -0.5) ? WALL_TAIL_MARGIN_CM : (face.y >  0.5 ? WALL_NOSE_MARGIN_CM : WALL_TAIL_MARGIN_CM);
    staging.x = clampd(staging.x, w->field_min.x + m_left, w->field_max.x - m_right);
    staging.y = clampd(staging.y, w->field_min.y + m_top,  w->field_max.y - m_bot);
    // If the wall margin pulled the staging point off the ball->goal line
    // (like if the ball within STAGE_DIST of a wall), the line ball->goal no longer passes
    // through it. In testing, P_ALIGN's lateral check against that line NEVER pass
    // and the bot KEPT cycling GOTO -> ALIGN -> GOTO until a timeout. So we instead aim along the
    // line from the (clamped) staging point THROUGH the ball instead. It still
    // points at the far side of the field, just a few degrees off the goal.
    Vec2 ideal = vec2_sub(w->ball.pos,
        vec2_scale(vec2_normalize(vec2_sub(w->their_goal, w->ball.pos)), STAGE_DIST_CM));
    if (vec2_dist(staging, ideal) > 2.0) {
        Vec2 dir = vec2_sub(w->ball.pos, staging);
        if (vec2_len(dir) > 1.0)
            *aim_out = vec2_add(w->ball.pos, vec2_scale(vec2_normalize(dir), 1000.0));
    }
    return staging;
}

void replan(const World *w) {
    // An invalid ball is represented as (0,0) (the top-left corner). Replanning on
    // that built staging points in the corner and, via the MOVE_TOL_CM
    // check in next_goto_staging/next_align, re-triggered every frame of a
    // dropout. Keep the existing plan until the ball is seen again.
    if (!w->ball.valid)
        return;
    // Our replan cycle. Store the position of the ball at the time
    // the new plan is created
    g_staging_point = compute_staging(w, &g_aim_target);
    g_ball_at_plan = w->ball.pos;
}

Vec2 mode_penalty_debug_staging(void) { return g_staging_point; }
Vec2 mode_penalty_debug_aim(void) { return g_aim_target; }

// Set by mode_penalty_step() right before calling the active state's run().
// This is set to true on precisely on the first frame of that state.
// Whenever we enter a new state, this is set to 1 so we reset, and after that,
// it is set to 0.
static int g_entered = 0;
// Forward speed commanded last frame, for the stall detector.
static double g_last_cmd_v = 0;

// **** P_READY ****
SkillStatus penalty_run_ready(World *w, Motion *out) {
    // This just stalls until the ball and the bot are visible
    return skill_hold(w, out);
}

int penalty_next_ready(World *w, SkillStatus s, double t) {
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
        replan(w);
        return P_GOTO_STAGING;
    }
    return P_READY;
}

// **** P_GOTO_STAGING ****
SkillStatus run_goto_staging(World *w, Motion *out) {
    // Detour around the ball if the straight path to staging runs through it,
    // Once the detour point has been reached, drive
    // straight to staging for the rest of this plan. Re-evaluating from
    // the detour point could re-block and keep the bot there forever.
    static int detour_done = 0;
    static Vec2 planned_for = {1e9, 1e9};
    if (g_entered || vec2_dist(g_staging_point, planned_for) > 3.0) {
        detour_done = 0;
        planned_for = g_staging_point;
    }
    if (!detour_done && w->ball.valid) {
        Vec2 via = plan_avoid_point(w->self.pos, g_staging_point, w->ball.pos,
                                    AVOID_BLOCK_CM, AVOID_CLEAR_CM,
                                    w->field_min, w->field_max);
        if (vec2_dist(via, g_staging_point) > 1e-6) {
            SkillStatus s = skill_goto(w, via, g_entered, out);
            if (s != SKILL_DONE) return SKILL_RUNNING;
            detour_done = 1;
        }
    }
    return skill_goto(w, g_staging_point, g_entered, out);
}

static int g_align_attempts = 0;
// Escape latch for the P_GOTO_STAGING<->P_WALL_RECOVER thrash.
// Records numbe of attemps for wall recovery
static int g_wall_recover_attempts = 0;

int next_goto_staging(World *w, SkillStatus s, double t) {
    if (w->ball.valid && vec2_dist(w->ball.pos, g_ball_at_plan) > MOVE_TOL_CM) {
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

// **** P_ALIGN ****
SkillStatus run_align(World *w, Motion *out) {
    return skill_face(w, g_aim_target, g_entered, out);
}

int next_align(World *w, SkillStatus s, double t) {
    if (w->ball.valid && vec2_dist(w->ball.pos, g_ball_at_plan) > MOVE_TOL_CM) {
        replan(w);
        return P_GOTO_STAGING;
    }
    if (s == SKILL_DONE) {
        double lat = perp_dist_to_line(w->self.pos, w->ball.pos, g_aim_target);
        if (lat < LAT_TOL_CM) {
            g_align_attempts = 0; // real progress
            g_wall_recover_attempts = 0;
            return P_STRIKE;
        }
        // Heading's stable but we drifted sideways of the aim line more
        // than plan_waypoint() should allow. P_ALIGN only rotates
        // (skill_face commands v=0), and pure rotation can never fix a
        // lateral/perpendicular offset, only translation can. Staying in
        // P_ALIGN here used to just retry, re-achieve heading-stable,
        // fail this same check, and cycle, until I moved the ball
        //
        // Replanning and going back to P_GOTO_STAGING fixes that, but
        // returning immediately here also skips the g_align_attempts
        // escape below entirely, each time. If the exact same
        // lateral miss keeps happening after every replan, this
        // path never counts toward giving up, so it never would give up.
        // We count it the same way the regular case does.
        g_align_attempts++;
        if (g_align_attempts > ALIGN_ATTEMPT_LIMIT) {
            g_align_attempts = 0;
            return P_REPOSITION;
        }
        replan(w);
        return P_GOTO_STAGING;
    }
    // t_in_state resets every time we re-enter ALIGN, so ALIGN_TIMEOUT_S
    // never accumulates across repeated short GOTO_STAGING <-> ALIGN
    // bounce. That loop could otherwise run forever with no escape at
    // all. g_align_attempts is the escape that survives the bounce.
    if (t > ALIGN_TIMEOUT_S || g_align_attempts > ALIGN_ATTEMPT_LIMIT) {
        g_align_attempts = 0;
        return P_REPOSITION;
    }
    return P_ALIGN;
}

// **** P_STRIKE ****
SkillStatus run_strike(World *w, Motion *out) {
    return skill_strike(w, g_aim_target, g_entered, out);
}

int next_strike(World *w, SkillStatus s, double t) {
    (void)w;
    // One kick, one contact - no P_FOLLOW_THROUGH. P_LOST was tried here
    // first (it also just holds still), but it skips P_SHOT_DONE
    // entirely, which means ball_in_goal() never gets checked and a
    // missed kick never replans another attempt - it just sits there
    // until self/ball happen to drop out of tracking long enough to
    // reset. P_SHOT_DONE's run() is the same skill_hold() stop, but its
    // next() still confirms the goal and replans via P_GOTO_STAGING if
    // the ball came to rest outside it - exactly "one kick" without
    // losing goal detection.
    if (s == SKILL_DONE)
        return P_SHOT_DONE;
    if (s == SKILL_FAILED || t > STRIKE_TIMEOUT_S)
        return P_REPOSITION;
    return P_STRIKE;
}

// **** P_FOLLOW_THROUGH ****
SkillStatus run_follow_through(World *w, Motion *out) {
    return skill_follow_through(w, g_aim_target, g_entered, out);
}

int next_follow_through(World *w, SkillStatus s, double t) {
    (void)t;
    if (s == SKILL_DONE) {
        // "Shot done" must mean a shot was actually taken. If the ball is
        // still essentially where it sat when this plan was made, the strike
        // never moved it (or it was cut short), so just try again rather than
        // sitting in SHOT_DONE waiting for something that already failed.
        if (w->ball.valid && vec2_dist(w->ball.pos, g_ball_at_plan) < NO_SHOT_MOVED_CM) {
            replan(w);
            return P_GOTO_STAGING;
        }
        return P_SHOT_DONE;
    }
    return P_FOLLOW_THROUGH;
}

// **** P_REPOSITION ****
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

// **** P_WALL_RECOVER ****
SkillStatus run_wall_recover(World *w, Motion *out) {
    return skill_leave_wall(w, out, g_entered);
}

int next_wall_recover(World *w, SkillStatus s, double t) {
    (void)t;
    if (s == SKILL_DONE) {
        g_wall_recover_attempts++;
        if (g_wall_recover_attempts > WALL_RECOVER_ATTEMPT_LIMIT) {
            g_wall_recover_attempts = 0;
            replan(w);
            return P_REPOSITION;
        }
        replan(w);
        return P_GOTO_STAGING;
    }
    return P_WALL_RECOVER;
}

// **** P_UNSTICK ****
SkillStatus penalty_run_unstick(World *w, Motion *out) {
    return skill_unstick(w, g_entered, out);
}

int penalty_next_unstick(World *w, SkillStatus s, double t) {
    (void)s;
    // Run one full reverse+turn cycle. The old exit
    if (t >= UNSTICK_REVERSE_S + UNSTICK_TURN_S) {
        replan(w);
        return P_GOTO_STAGING;
    }
    return P_UNSTICK;
}

// **** P_SHOT_DONE ****
SkillStatus run_shot_done(World *w, Motion *out) {
    return skill_hold(w, out);
}

// Ball "at rest" = it has not moved more than REST_SPREAD_CM over the last
// REST_WINDOW_S. The old test was |ball.vel| < 2 cm/s, but ball.vel is a
// frame-to-frame position difference and at 30fps ~0.5cm of noise explodes to
// 15 cm/s.
int ball_is_resting(const World *w) {
    static double ht[REST_MAX_SAMPLES];
    static Vec2 hp[REST_MAX_SAMPLES];
    static int n = 0;
    if (!w->ball.valid) { n = 0; return 0; }
    // drop samples older than the window
    int k = 0;
    while (k < n && w->now - ht[k] > REST_WINDOW_S)
        k++;
    if (k > 0) {
        for (int i = k; i < n; i++) {
            ht[i - k] = ht[i];
            hp[i - k] = hp[i];
        }
        n -= k;
    }
    if (n == REST_MAX_SAMPLES) {
        for (int i = 1; i < n; i++) {
            ht[i - 1] = ht[i];
            hp[i - 1] = hp[i];
        }
        n--;
    }
    ht[n] = w->now;
    hp[n] = w->ball.pos; n++;
    if (n < 8 || w->now - ht[0] < REST_WINDOW_S * 0.8)
        return 0;
    double minx = hp[0].x, maxx = minx, miny = hp[0].y, maxy = miny;
    for (int i = 1; i < n; i++) {
        if (hp[i].x < minx)
            minx = hp[i].x;
        if (hp[i].x > maxx)
            maxx = hp[i].x;
        if (hp[i].y < miny)
            miny = hp[i].y;
        if (hp[i].y > maxy)
            maxy = hp[i].y;
    }
    return hypot(maxx - minx, maxy - miny) < REST_SPREAD_CM;
}

int next_shot_done(World *w, SkillStatus s, double t) {
    (void)s;
    double field_center_x = (w->field_min.x + w->field_max.x) / 2.0;
    int at_rest = ball_is_resting(w);
    Vec2 confirm_goal = w->their_goal;
    confirm_goal.x += (w->their_goal.x < field_center_x)
        ? -GOAL_CONFIRM_MARGIN_CM : GOAL_CONFIRM_MARGIN_CM;
    int in_goal = w->ball.valid && ball_in_goal(w->ball.pos, confirm_goal,
                                      w->goal_width_cm, field_center_x,
                                      BALL_RADIUS_CM);
    // Require GOAL_LEAVE_CONFIRM_FRAMES consecutive not-in-goal reportings
    // before actually leaving.
    // A ball resting right at the confirm boundary can flicker in/out of
    // in_goal purely from reported-position noise.
    static int not_in_goal_frames = 0;
    if (w->ball.valid && at_rest && !in_goal) {
        not_in_goal_frames++;
    } else {
        not_in_goal_frames = 0;
    }
    if (not_in_goal_frames >= GOAL_LEAVE_CONFIRM_FRAMES && t > RETRY_WAIT_S) {
        not_in_goal_frames = 0;
        replan(w);
        return P_GOTO_STAGING;
    }
    return P_SHOT_DONE;
}

// **** P_LOST ****
SkillStatus penalty_run_lost(World *w, Motion *out) {
    return skill_hold(w, out);
}

int penalty_next_lost(World *w, SkillStatus s, double t) {
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
        return P_READY;
    }
    return P_LOST;
}

// ************************************************************************

static const StateDef g_penalty_states[] = {
    {P_READY, "P_READY", penalty_run_ready, penalty_next_ready, 0},
    {P_GOTO_STAGING, "P_GOTO_STAGING", run_goto_staging, next_goto_staging, GOTO_STAGING_TIMEOUT_S},
    {P_ALIGN, "P_ALIGN", run_align, next_align, ALIGN_TIMEOUT_S},
    {P_STRIKE, "P_STRIKE", run_strike, next_strike, STRIKE_TIMEOUT_S},
    {P_FOLLOW_THROUGH, "P_FOLLOW_THROUGH", run_follow_through,
     next_follow_through, 0},
    {P_REPOSITION, "P_REPOSITION", run_reposition, next_reposition, 0},
    {P_WALL_RECOVER, "P_WALL_RECOVER", run_wall_recover, next_wall_recover, 0},
    {P_UNSTICK, "P_UNSTICK", penalty_run_unstick, penalty_next_unstick, 0},
    {P_SHOT_DONE, "P_SHOT_DONE", run_shot_done, next_shot_done, 0},
    {P_LOST, "P_LOST", penalty_run_lost, penalty_next_lost, 0},
};
#define N_PENALTY_STATES                                                       \
    (int)(sizeof(g_penalty_states) / sizeof(g_penalty_states[0]))

// **** global preemptive moves, checked before the state table every frame.
// Priority when more than one fires is wall_recover > unstick > lost > goal
int penalty_check_preempt(int state, const World *w, double now) {
    if (state != P_WALL_RECOVER && state != P_FOLLOW_THROUGH && w->self.valid) {
        static double last_wall_margin = WALL_TRIGGER_MARGIN_CM;
        double wall_margin;
        if (w->ball.valid) {
            wall_margin = near_wall(w->ball.pos, w->field_min, w->field_max, DEFAULT_WALL_MARGIN_CM)
                    ? WALL_TRIGGER_MARGIN_BALLWALL_CM
                    : WALL_TRIGGER_MARGIN_CM;
            if (state == P_GOTO_STAGING || state == P_ALIGN)
                wall_margin = fmin(wall_margin, WALL_TRIGGER_MARGIN_STAGING_CM);
            last_wall_margin = wall_margin;
        } else {
            wall_margin = last_wall_margin;
        }
        // Both goals sit ON the end walls, so driving in to score (P_STRIKE
        // isn't excluded above the way P_FOLLOW_THROUGH is) necessarily
        // looks like "near wall, heading outward" - skip the trigger at
        // the goal mouth specifically, same margin as the real check below.
        if (near_wall(w->self.pos, w->field_min, w->field_max, wall_margin) &&
            !near_goal_mouth(w->self.pos, wall_margin, w->field_min,
                             w->field_max, w->own_goal, w->their_goal, w->goal_width_cm)) {
            Vec2 in = direction_away_from_nearest_wall(w->self.pos, w->field_min, w->field_max);
            if (vec2_dot(w->self.heading, in) < -WALL_OUTWARD_DOT)
                return P_WALL_RECOVER; // heading clearly outward
        }
    }

    // Finally have a way to detect stalls now
    static StallDetector stall;
    int stationary_state =
        (state == P_READY || state == P_LOST || state == P_SHOT_DONE ||
         state == P_ALIGN || state == P_UNSTICK);
    if (stationary_state)
        stall_reset(&stall);
    int stalled = stall_update(&stall, now, w->self.pos,
                               w->self.valid && !stationary_state, g_last_cmd_v);
    if (stalled && state != P_UNSTICK) {
        stall_reset(&stall);
        return P_UNSTICK;
    }

    if (state != P_STRIKE && state != P_FOLLOW_THROUGH &&
        state != P_SHOT_DONE && state != P_LOST &&
        ((w->self.valid == 0 && w->self.lost_frames > LOST_FRAMES) ||
         (w->ball.valid == 0 && w->ball.lost_frames > BALL_LOST_FRAMES))) {
        return P_LOST;
    }

    if (state != P_SHOT_DONE && w->ball.valid) {
        double field_center_x = (w->field_min.x + w->field_max.x) / 2.0;
        Vec2 confirm_goal = w->their_goal;
        confirm_goal.x += (w->their_goal.x < field_center_x)
            ? -GOAL_CONFIRM_MARGIN_CM : GOAL_CONFIRM_MARGIN_CM;
        if (ball_in_goal(w->ball.pos, confirm_goal, w->goal_width_cm,
                         field_center_x, BALL_RADIUS_CM))
            return P_SHOT_DONE;
    }

    return -1;
}

static int prev_state = -1;
static double t_entered = 0;

int mode_penalty_fsm_step(World *world, int state, double now, Motion *out_motion) {
    int target = penalty_check_preempt(state, world, now);
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
    g_last_cmd_v = out_motion->v;

    int next_id = row->next(world, s, t_in_state);
    if (next_id != state) {
        fsm_log_transition(g_penalty_states, N_PENALTY_STATES, state, next_id,
                           now, "state");
    }
    return next_id;
}
