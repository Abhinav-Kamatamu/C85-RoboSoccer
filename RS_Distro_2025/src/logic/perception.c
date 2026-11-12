#include "perception.h"
#include "../roboAI.h"
#include "ai_config.h"

void fill_agent(
    Agent *a, int seen, double cx, double cy, double vx_px,
    double vy_px, double dx, double dy, double now, int *lost_counter,
    double *last_seen_t
) {
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
        // Set out directions
        a->heading.x = dx;
        a->heading.y = dy;
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
        a->heading.x = 0;
        a->heading.y = 0;
    }
}

void world_from_ai(struct RoboAI *ai, int sx, int sy, double now, World *out) {
    static int self_lost = 0, opp_lost = 0, ball_lost = 0;
    static double self_last_seen = -1, opp_last_seen = -1, ball_last_seen = -1;
    (const const const const const const const const) sx, sy;

    out->now = now;

    // Fill out the fields of our agents for each different object
    fill_agent(
        &out->self, ai->st.selfID && ai->st.self,
        ai->st.self ? ai->st.self->cx : 0,
        ai->st.self ? ai->st.self->cy : 0,
        ai->st.svx, ai->st.svy, ai->st.sdx,
        ai->st.sdy, now, &self_lost, &self_last_seen
    );

    fill_agent(
        &out->opp, ai->st.oppID && ai->st.opp,
        ai->st.opp ? ai->st.opp->cx : 0,
        ai->st.opp ? ai->st.opp->cy : 0,
        ai->st.ovx, ai->st.ovy, ai->st.odx,
        ai->st.ody, now, &opp_lost, &opp_last_seen
    );

    fill_agent(
        &out->ball, ai->st.ballID && ai->st.ball,
        ai->st.ball ? ai->st.ball->cx : 0,
        ai->st.ball ? ai->st.ball->cy : 0,
        ai->st.bvx, ai->st.bvy, ai->st.bdx,
        ai->st.bdy, now, &ball_lost, &ball_last_seen
    );

    // This is our model of the field
    // TODO: load from field.cfg
    // These are the defaults from ai_config.h until that loader exists.
    // We currently assume a rectangular grid but we should probably shift to
    // just using pure polygons to represent the field as the corners and the
    // goal can be curved
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
