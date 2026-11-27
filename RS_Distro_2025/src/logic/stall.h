/***************************************************
 CSC C85 RoboSoccer stall detector
***************************************************/

#ifndef LOGIC_STALL_H
#define LOGIC_STALL_H

#include <math.h>
#include "ai_config.h"
#include "geometry.h"
#include "world.h"

#define STALL_MAX_SAMPLES 96

typedef struct {
    double t[STALL_MAX_SAMPLES];
    Vec2   p[STALL_MAX_SAMPLES];
    double v[STALL_MAX_SAMPLES]; // |commanded forward speed| that frame
    int    n;                     // samples stored (oldest first)
} StallDetector;

static void stall_reset(StallDetector *s) {
    s->n = 0;
}

// Input one frame. Returns 1 if the bot has been commanded at least
// STALL_CMD_V_CMPS on average for window_s yet moved less than
// STALL_SPREAD_CM in total over that time.
static int stall_update_window(StallDetector *s, double now, Vec2 pos, int valid,
                               double cmd_v, double window_s) {
    if (!valid) {
        s->n = 0;
        return 0;
    }
    // drop samples older than the window
    int keep_from = 0;
    while (keep_from < s->n && now - s->t[keep_from] > window_s)
        keep_from++;
    if (keep_from > 0) {
        for (int i = keep_from; i < s->n; i++) {
            s->t[i - keep_from] = s->t[i];
            s->p[i - keep_from] = s->p[i];
            s->v[i - keep_from] = s->v[i];
        }
        s->n -= keep_from;
    }
    if (s->n == STALL_MAX_SAMPLES) {
        // full, so drop the oldest
        // Maybe I should just use a ring buffer?
        for (int i = 1; i < s->n; i++) {
            s->t[i - 1] = s->t[i];
            s->p[i - 1] = s->p[i];
            s->v[i - 1] = s->v[i];
        }
        s->n--;
    }
    s->t[s->n] = now;
    s->p[s->n] = pos;
    s->v[s->n] = fabs(cmd_v);
    s->n++;

    // need a (nearly) full window of history
    if (s->n < 8 || now - s->t[0] < window_s * 0.9)
        return 0;
    double minx = s->p[0].x, maxx = minx, miny = s->p[0].y, maxy = miny, vsum = 0;
    for (int i = 0; i < s->n; i++) {
        if (s->p[i].x < minx) minx = s->p[i].x;
        if (s->p[i].x > maxx) maxx = s->p[i].x;
        if (s->p[i].y < miny) miny = s->p[i].y;
        if (s->p[i].y > maxy) maxy = s->p[i].y;
        vsum += s->v[i];
    }
    double spread = hypot(maxx - minx, maxy - miny);
    return (vsum / s->n >= STALL_CMD_V_CMPS) && spread < STALL_SPREAD_CM;
}

static int stall_update(StallDetector *s, double now, Vec2 pos, int valid, double cmd_v) {
    return stall_update_window(s, now, pos, valid, cmd_v, UNSTICK_STALL_S);
}

// Stuck on a wall = a corner of the body is touching a wall, we are driving
// into that wall, and we have not moved for WALL_STALL_S. The window only
// fills while all of that holds, so it is also the timer. Touching or
// sliding along a wall is fine.
static int wall_stuck_update(StallDetector *s, const World *w, double cmd_v) {
    if (!w->self.valid) {
        stall_reset(s);
        return 0;
    }
    Vec2 in;
    double d = footprint_wall_dist(w->self.pos, w->self.heading,
                                   w->field_min, w->field_max, &in);
    // Driving forward pushes with the nose, reversing pushes with the tail
    double out_dot = -vec2_dot(w->self.heading, in);
    if (cmd_v < 0)
        out_dot = -out_dot;
    if (d > WALL_CONTACT_CM || out_dot < WALL_OUTWARD_DOT) {
        stall_reset(s);
        return 0;
    }
    return stall_update_window(s, w->now, w->self.pos, 1, cmd_v, WALL_STALL_S);
}

#endif
