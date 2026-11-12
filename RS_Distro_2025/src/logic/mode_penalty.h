/***************************************************
 CSC C85 RoboSoccer Penalty shot (states 100-199)

***************************************************/

#ifndef LOGIC_MODE_PENALTY_H
#define LOGIC_MODE_PENALTY_H

#include "world.h"

struct RoboAI;
struct blob;

enum PenaltyState {
    P_IDENTIFY        = 100, // provided (id_bot(), handled in roboAI.c, do not touch)
    P_READY           = 101,
    P_GOTO_STAGING    = 110,
    P_ALIGN           = 120,
    P_STRIKE          = 130,
    P_FOLLOW_THROUGH  = 140,
    P_REPOSITION      = 150,
    P_WALL_RECOVER    = 160,
    P_UNSTICK         = 170,
    P_SHOT_DONE       = 190,
    P_LOST            = 195,
};

// Call once per frame from AI_main() whenever ai->st.state is 101-199.
// Calls track_agents() itself, builds a World, runs the penalty FSM, and
// calls drive_set(). Does nothing with state 100, that's id_bot()'s job.
void mode_penalty_step(struct RoboAI *ai, struct blob *blobs);

// Step the FSM
int mode_penalty_fsm_step(World *world, int state, double now, Motion *out_motion);

#endif
