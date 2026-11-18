/***************************************************
 CSC C85 RoboSoccer; Mode 2 adapter
***************************************************/

#include "mode_chase.h"
#include "../roboAI.h"
#include "drive.h"
#include "perception.h"
#include "clock.h"

extern int sx;
extern int sy;
extern int botCol;
extern int AIMode;

void mode_chase_step(struct RoboAI *ai, struct blob *blobs) {
    // refresh ai->st.* for this frame (perception's input)
    track_agents(ai, blobs);

    World world;
    double now = maintenant();
    world_from_ai(ai, sx, sy, now, &world);

    Motion motion = {0, 0};
    int next_id = mode_chase_fsm_step(&world, ai->st.state, now, &motion);
    // Chase has no staging/aim plan (mode_penalty only). Log the ball's
    // position as "aim" since that's what skill_chase actually drives
    // toward every frame, and {0,0} for "staging" (not a chase concept).
    WORLD_LOG_FRAME(ai, botCol, AIMode, ai->st.state, &world, motion,
                     (Vec2){0, 0}, world.ball.pos);
    perception_set_last_cmd_v(motion.v);
    drive_set(motion, drive_get_regulated_speed());
    ai->st.state = next_id;
}
