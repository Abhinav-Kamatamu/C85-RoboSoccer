/***************************************************
 CSC C85 RoboSoccer; Mode 1 adapter
***************************************************/

#include "mode_penalty.h"
#include "../roboAI.h"
#include "drive.h"
#include "perception.h"
#include "clock.h"

extern int sx;
extern int sy;
extern int botCol;
extern int AIMode;

void mode_penalty_step(struct RoboAI *ai, struct blob *blobs) {
    // refresh ai->st.* for this frame (perception's input)
    track_agents(ai, blobs);

    World world;
    double now = maintenant();
    world_from_ai(ai, sx, sy, now, &world);

    Motion motion = {0, 0};
    int next_id = mode_penalty_fsm_step(&world, ai->st.state, now, &motion);
    WORLD_LOG_FRAME(ai, botCol, AIMode, ai->st.state, &world, motion,
                     mode_penalty_debug_staging(), mode_penalty_debug_aim());
    perception_set_last_cmd_v(motion.v);
    drive_set(motion, drive_get_regulated_speed());
    ai->st.state = next_id;
}
