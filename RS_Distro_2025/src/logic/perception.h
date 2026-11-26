/***************************************************
 CSC C85 RoboSoccer, perception

 Builds a World out of struct AI_data for one frame. Pixels -> cm here
 (PX_PER_CM_X/Y, ai_config.h)

 struct RoboAI is forward-declared, not included, so this header stays
 cheap to include from geometry/skills callers that only need World.
***************************************************/

#ifndef LOGIC_PERCEPTION_H
#define LOGIC_PERCEPTION_H

#include "world.h"
#include "ai_config.h"

struct RoboAI;

// `now` is a seconds timestamp (use maintenant()), used simultaneously
// to turn per-frame pixel deltas into cm/s and as World.now for
// timeouts/dwell elsewhere.
// Tell perception what forward speed (cm/s, signed) was last commanded, so
// heading-sign disambiguation only trusts measured velocity while the bot is
// actually driving. Call from each adapter after the FSM step.
void perception_set_last_cmd_v(double v);

void world_from_ai(struct RoboAI *ai, int sx, int sy, double now, World *out);

// Appends one line per frame to "positions_<own_col>_<ai_mode>.log" (whuich is opened
// lazily on first call, in the process's current directory, and flushed every
// line) with EVERYTHING that might matter for diagnosing bad steering.
// Call once per frame from each mode's adapter, right after
// world_from_ai() and after the FSM step. own_col/
// ai_mode just tag the filename so different
// `roboSoccer <dev> <own_col> <ai_mode>` runs don't overwrite each other.
// Do note, MEGABYTES have been lost here, but it is what helped me see why
// we just kept rotationg previously. BUT the logs ARE a pain to read, so only
// use it as a LAST RESORT
void world_log_frame(struct RoboAI *ai, int own_col, int ai_mode, int state,
                      const World *w, Motion motion, Vec2 staging, Vec2 aim);

// Use this in the adapters instead of calling world_log_frame() directly.
// The human-readable positions log is written when ENABLE_FRAME_LOG
// (ai_config.h) is on OR the run log is on (logItems bit0, see runlog.h);
// the run log's frames.jsonl is fed from the same call. With both off it
// returns straight away.
#define WORLD_LOG_FRAME(...) world_log_frame(__VA_ARGS__)

#endif
