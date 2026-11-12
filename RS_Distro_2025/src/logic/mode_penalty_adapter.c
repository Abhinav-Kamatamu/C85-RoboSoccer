/***************************************************
 CSC C85 RoboSoccer - Mode 1 real-world adapter

 mode_penalty.c is pure logic (World in, Motion out, no I/O) specifically
 so it can be linked and tested without the image-capture/SDL/Bluetooth
 chain (good for simulatte). This tiny file is the other half. it's the
 only place in mode 1 that touches struct RoboAI/blob, track_agents(),
 and drive_set().
***************************************************/

#include "mode_penalty.h"
#include "../roboAI.h"
#include "drive.h"
#include "perception.h"
#include <sys/time.h>

extern int sx;
extern int sy;

static double maintenant(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
}

void mode_penalty_step(struct RoboAI *ai, struct blob *blobs) {
    // refresh ai->st.* for this frame (perception's input)
    track_agents(ai, blobs);

    World world;
    double now = maintenant();
    world_from_ai(ai, sx, sy, now, &world);

    Motion motion = {0, 0};
    int next_id = mode_penalty_fsm_step(&world, ai->st.state, now, &motion);
    drive_set(motion);
    ai->st.state = next_id;
}
