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

struct RoboAI;

// `now` is a seconds timestamp (use maintenant()), used simultaneously
// to turn per-frame pixel deltas into cm/s and as World.now for
// timeouts/dwell elsewhere.
void world_from_ai(struct RoboAI *ai, int sx, int sy, double now, World *out);

#endif
