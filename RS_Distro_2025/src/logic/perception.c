#include "perception.h"
#include "../roboAI.h"
#include "ai_config.h"
#include "geometry.h"
#include "gyro.h"
#include "drive.h"
#include <math.h>
#include <stdio.h>

static double g_last_cmd_v = 0;
void perception_set_last_cmd_v(double v) { g_last_cmd_v = v; }

void fill_agent(
    Agent *a, int seen, double cx, double cy, double vx_px,
    double vy_px, double dx, double dy, double now, int *lost_counter,
    double *last_seen_t, Vec2 *last_good_heading, int *heading_flip_streak,
    double heading_vel_scale
) {
    if (seen && *last_seen_t >= 0) {
        double dt_check = now - *last_seen_t;
        if (dt_check > 0) {
            double implied_speed =
                vec2_len((Vec2){vx_px / PX_PER_CM_X, vy_px / PX_PER_CM_Y}) / dt_check;
            if (implied_speed > MAX_PLAUSIBLE_SPEED_CMPS) seen = 0;
        }
    }

    if (seen) {
        // We've seen it, so this object is not lost
        *lost_counter = 0;
        a->lost_frames = 0;
        // We know where we are for this frame, so +1 to the counter
        a->valid = 1;
        // update our state variables using physics
        a->pos.x = cx / PX_PER_CM_X;
        a->pos.y = cy / PX_PER_CM_Y;
        // vx_px/vy_px are the raw pixel delta since THIS agent was last
        // seen, which can span more than one frame if it was briefly lost. So
        // dt has to be "time since we last actually saw this agent," not
        // "time since the last call to world_from_ai"
        double dt = (*last_seen_t < 0) ? (1.0 / 15.0) : (now - *last_seen_t);
        if (dt <= 0) dt = 1.0 / 15.0; // bad/repeated timestamp guard
        *last_seen_t = now;
        // TODO: We might not always be able to trust this. Maybe we should
        // have some sort of small physics sim that predicts actions but accepts
        // corrections
        a->vel.x = (vx_px / PX_PER_CM_X) / dt;
        a->vel.y = (vy_px / PX_PER_CM_Y) / dt;
        if (last_good_heading) {
            Vec2 raw = {dx, dy};
            // heading_vel_scale => +1 trust vel as-is, -1 we are reversing (vel
            // should be OPPOSITE heading), 0 don't trust vel at all.
            Vec2 vel_for_heading = vec2_scale(a->vel, heading_vel_scale);
            a->heading = fix_heading_sign(raw, vel_for_heading, HEADING_SIGN_TRUST_CMPS,
                                           last_good_heading, heading_flip_streak,
                                           HEADING_FLIP_CONFIRM_FRAMES);
        } else {
            a->heading.x = 0;
            a->heading.y = 0;
        }
    } else {
        // We are in fact, lost
        (*lost_counter)++;
        a->lost_frames = *lost_counter;
        // So we have no valid frames where we know where we are
        a->valid = 0;
        // We know absolutely nothing about our bot this frame. Do not pretend to
        a->pos.x = 0;
        a->pos.y = 0;
        a->vel.x = 0;
        a->vel.y = 0;
        // Persist the last known heading through a dropout instead of
        // zeroing it.
        a->heading = last_good_heading ? *last_good_heading : (Vec2){0, 0};
    }
}

void world_from_ai(struct RoboAI *ai, int sx, int sy, double now, World *out) {
    static int self_lost = 0, opp_lost = 0, ball_lost = 0;
    static double self_last_seen = -1, opp_last_seen = -1, ball_last_seen = -1;
    static Vec2 self_last_heading = {0, 0}, opp_last_heading = {0, 0};
    static int self_flip_streak = 0, opp_flip_streak = 0;
    static double self_last_tick = -1;
    (void)sx;
    (void)sy;

    out->now = now;

    int self_seen = ai->st.selfID && ai->st.self;
    double tick_dt = (self_last_tick < 0) ? 0 : (now - self_last_tick);
    self_last_tick = now;

    // Fill out the fields of our agents for each different object
    fill_agent(
        &out->self, self_seen,
        ai->st.self ? ai->st.self->cx : 0,
        ai->st.self ? ai->st.self->cy : 0,
        ai->st.svx, ai->st.svy, ai->st.sdx,
        ai->st.sdy, now, &self_lost, &self_last_seen, &self_last_heading,
        &self_flip_streak,
        (fabs(g_last_cmd_v) >= HEADING_CMD_V_MIN_CMPS) ? (g_last_cmd_v > 0 ? 1.0 : -1.0) : 0.0
    );

    if (!self_seen && tick_dt > 0) {
        double angle_s;
        double rate_deg_s;
        if (gyro_read_rate(&angle_s, &rate_deg_s)) {
            double ang = angle_of(out->self.heading) + deg_to_rad(rate_deg_s) * tick_dt;
            // TODO: compare with actual angle reading
            out->self.heading = (Vec2){cos(ang), sin(ang)};
            self_last_heading = out->self.heading; // keep fix_heading_sign's prior in sync
            self_flip_streak = 0; // streak was counted against the now-stale prior
        }
    }

    fill_agent(
        &out->opp, ai->st.oppID && ai->st.opp,
        ai->st.opp ? ai->st.opp->cx : 0,
        ai->st.opp ? ai->st.opp->cy : 0,
        ai->st.ovx, ai->st.ovy, ai->st.odx,
        ai->st.ody, now, &opp_lost, &opp_last_seen, &opp_last_heading,
        &opp_flip_streak, 1.0
    );

    fill_agent(
        &out->ball, ai->st.ballID && ai->st.ball,
        ai->st.ball ? ai->st.ball->cx : 0,
        ai->st.ball ? ai->st.ball->cy : 0,
        ai->st.bvx, ai->st.bvy, ai->st.bdx,
        ai->st.bdy, now, &ball_lost, &ball_last_seen, NULL, NULL, 1.0
    );

    // This is our model of the field
    out->field_min.x = 0;
    out->field_min.y = 0;
    out->field_max.x = DEFAULT_FIELD_W_CM;
    out->field_max.y = DEFAULT_FIELD_H_CM;
    out->goal_width_cm = DEFAULT_GOAL_WIDTH_CM;

    double mid_y = DEFAULT_FIELD_H_CM / 2.0;
    out->own_goal.y = mid_y;
    out->their_goal.y = mid_y;
    // Store where the position of our goal and the opposing teams goal is
    // based on what side of the field our goal is on
    if (ai->st.side == 0) {
        out->own_goal.x = 0;
        out->their_goal.x = DEFAULT_FIELD_W_CM;
    } else {
        out->own_goal.x = DEFAULT_FIELD_W_CM;
        out->their_goal.x = 0;
    }
}

// Raw blob data (shape-axis direction dx/dy, motion-vector
// direction mx/my, pixel position/size/bbox/colour) for one agent, raw
// relative to fix_heading_sign(), which is what turns dx/dy into World's
// heading. "null" if this agent has no current blob (not seen this frame).
void fprint_blob(FILE *f, const char *tag, struct blob *b) {
    if (b) {
        fprintf(f,
            "%s[cx=%.1f cy=%.1f dx=%.3f dy=%.3f mx=%.3f my=%.3f size=%d "
            "bbox=(%d,%d,%d,%d) HSV=(%.1f,%.3f,%.3f)] ",
            tag, b->cx, b->cy, b->dx, b->dy, b->mx, b->my, b->size,
            b->x1, b->y1, b->x2, b->y2, b->H, b->S, b->V);
    } else {
        fprintf(f, "%s[null] ", tag);
    }
}

void world_log_frame(struct RoboAI *ai, int own_col, int ai_mode, int state,
                      const World *w, Motion motion, Vec2 staging, Vec2 aim) {
    static FILE *f = NULL;
    static int opened = 0;
    if (!opened) {
        opened = 1;
        char fname[64];
        snprintf(fname, sizeof(fname), "positions_%d_%d.log", own_col, ai_mode);
        f = fopen(fname, "w");
        if (!f) {
            fprintf(stderr, "world_log_frame(): could not open %s for writing\n", fname);
            return;
        }
        fprintf(stderr, "Logging per-frame object positions to %s\n", fname);
    }
    if (!f) return;

    // Wheel powers actually sent for the previous frame (this frame's are not
    // decided until drive_set(), which runs right after this call).
    double sent_l, sent_r;
    drive_get_last_power(&sent_l, &sent_r);

    fprintf(f,
        "t=%.3f state=%d "
        "self[valid=%d lost=%d pos=(%.2f,%.2f) vel=(%.2f,%.2f) heading=(%.3f,%.3f)] "
        "opp[valid=%d lost=%d pos=(%.2f,%.2f) vel=(%.2f,%.2f) heading=(%.3f,%.3f)] "
        "ball[valid=%d lost=%d pos=(%.2f,%.2f) vel=(%.2f,%.2f) heading=(%.3f,%.3f)] "
        "raw_self[sdx=%.3f sdy=%.3f smx=%.3f smy=%.3f] "
        "raw_opp[odx=%.3f ody=%.3f omx=%.3f omy=%.3f] "
        "raw_ball[bdx=%.3f bdy=%.3f bmx=%.3f bmy=%.3f] "
        "motion[v=%.2f w=%.3f] plan[staging=(%.2f,%.2f) aim=(%.2f,%.2f)] sent[prev_l=%.0f prev_r=%.0f] ",
        w->now, state,
        w->self.valid, w->self.lost_frames, w->self.pos.x, w->self.pos.y,
        w->self.vel.x, w->self.vel.y, w->self.heading.x, w->self.heading.y,
        w->opp.valid, w->opp.lost_frames, w->opp.pos.x, w->opp.pos.y,
        w->opp.vel.x, w->opp.vel.y, w->opp.heading.x, w->opp.heading.y,
        w->ball.valid, w->ball.lost_frames, w->ball.pos.x, w->ball.pos.y,
        w->ball.vel.x, w->ball.vel.y, w->ball.heading.x, w->ball.heading.y,
        ai->st.sdx, ai->st.sdy, ai->st.smx, ai->st.smy,
        ai->st.odx, ai->st.ody, ai->st.omx, ai->st.omy,
        ai->st.bdx, ai->st.bdy, ai->st.bmx, ai->st.bmy,
        motion.v, motion.w, staging.x, staging.y, aim.x, aim.y,
        sent_l, sent_r
    );
    fprint_blob(f, "blob_self", ai->st.self);
    fprint_blob(f, "blob_opp", ai->st.opp);
    fprint_blob(f, "blob_ball", ai->st.ball);
    fprintf(f, "\n");
    fflush(f);
}
