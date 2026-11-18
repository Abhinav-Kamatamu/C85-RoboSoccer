/***************************************************
 CSC C85 RoboSoccer Chase mode (states 200-299)

 "Kick the ball and chase it around the field" (roboAI.c). Continuously drive at
 wherever the ball currently is. We consider contact to be the kick.
***************************************************/

#ifndef LOGIC_MODE_CHASE_H
#define LOGIC_MODE_CHASE_H

#include "world.h"

struct RoboAI;
struct blob;

enum ChaseState {
    C_IDENTIFY     = 200, // provided (id_bot(), handled in roboAI.c, do not touch)
    C_READY        = 201,
    C_CHASE        = 210,
    C_WALL_RECOVER = 260,
    C_UNSTICK      = 270,
    C_LOST         = 295,
};

// Call once per frame from AI_main() whenever ai->st.state is 201-299.
// Calls track_agents() itself, builds a World, runs the chase FSM, and
// calls drive_set(). Does nothing with state 200, that's id_bot()'s job.
void mode_chase_step(struct RoboAI *ai, struct blob *blobs);

// Step the FSM
int mode_chase_fsm_step(World *world, int state, double now, Motion *out_motion);

#endif
