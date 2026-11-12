/***************************************************
 CSC C85 RoboSoccer - Mode 1: Penalty shot (states 100-199)

 Source of truth for behaviour is notes/fsm/penalty.dot; this enum and the
 state table in mode_penalty.c must mirror it number-for-number (see
 notes/fsm/README.md, "Adding a state or transition", step 4).
***************************************************/

#ifndef LOGIC_MODE_PENALTY_H
#define LOGIC_MODE_PENALTY_H

struct RoboAI;
struct blob;

enum PenaltyState {
    P_IDENTIFY        = 100, // provided (id_bot(), handled in roboAI.c -- do not touch)
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
// calls drive_set(). Does nothing with state 100 -- that's id_bot()'s job.
void mode_penalty_step(struct RoboAI *ai, struct blob *blobs);

#endif
