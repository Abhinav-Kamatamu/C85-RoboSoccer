/***************************************************
 CSC C85 RoboSoccer Soccer mode (states 1-99)

 The real combined mode. Decide attack vs defend every tick and run the
 correspondinig behaviour. Attack reuses mode_penalty's entire FSM as
 a private sub-FSM (it already handles its own staging/align/strike/
 wall-recover/unstick/lost internally) mode_soccer doesn't duplicates
 any of that. Defend picks between skill_mark/skill_intercept/skill_clear,
 plus its own wall-recover/unstick/lost handling (those three skills have
 no built-in wall-safety of their own, unlike the attack sub-FSM).
***************************************************/

#ifndef LOGIC_MODE_SOCCER_H
#define LOGIC_MODE_SOCCER_H

#include "world.h"

struct RoboAI;
struct blob;

enum SoccerState {
    S_IDENTIFY     = 0,  // provided (id_bot(), handled in roboAI.c, do not touch)
    S_READY        = 1,
    S_ATTACK       = 10, // delegates to mode_penalty_fsm_step() as a private sub-FSM
    S_DEFEND       = 20, // skill_mark / skill_intercept / skill_clear
    S_WALL_RECOVER = 90, // only reachable from S_DEFEND/S_READY. S_ATTACK handles its own
    S_UNSTICK      = 91, // same
    S_LOST         = 95,
};

// Call once per frame from AI_main() whenever ai->st.state is 1-99.
// Calls track_agents() itself, builds a World, runs the soccer FSM, and
// calls drive_set(). Does nothing with state 0, that's id_bot()'s job.
void mode_soccer_step(struct RoboAI *ai, struct blob *blobs);

// Step the FSM
int mode_soccer_fsm_step(World *world, int state, double now, Motion *out_motion);

#endif
