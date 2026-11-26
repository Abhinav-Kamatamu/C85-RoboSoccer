/***************************************************
 CSC C85 RoboSoccer; Mode 0 adapter
***************************************************/

#include "mode_soccer.h"
#include "mode_penalty.h"
#include "../roboAI.h"
#include "drive.h"
#include "perception.h"
#include "clock.h"

extern int sx;
extern int sy;
extern int botCol;
extern int AIMode;

void mode_soccer_step(struct RoboAI *ai, struct blob *blobs) {
    // refresh ai->st.* for this frame (perception's input)
    track_agents(ai, blobs);

    World world;
    double now = maintenant();
    world_from_ai(ai, sx, sy, now, &world);

    Motion motion = {0, 0};
    int next_id = mode_soccer_fsm_step(&world, ai->st.state, now, &motion);
    // The logs for mode soccer are only really useful when we're mid-S_ATTACK
    // {0,0} the rest of the time, same convention mode_chase_adapter.c
    // uses for "not a concept this mode has right now".
    WORLD_LOG_FRAME(ai, botCol, AIMode, ai->st.state, &world, motion,
                     mode_penalty_debug_staging(), mode_penalty_debug_aim());
    perception_set_last_cmd_v(motion.v);
    drive_set(motion, drive_get_regulated_speed());
    ai->st.state = next_id;
}
