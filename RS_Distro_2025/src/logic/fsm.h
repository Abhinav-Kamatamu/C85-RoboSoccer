/***************************************************
 CSC C85 RoboSoccer FSM stuff

 One StateDef table per mode (see mode_penalty.h); this file just knows
 how to look a state up by id and print a transition. Tracking
 (how long we've been in the current state) stays in the mode's own
 step function as two locals.
***************************************************/

#ifndef LOGIC_FSM_H
#define LOGIC_FSM_H

#include "world.h"

typedef struct StateDef {
    // The id of this state
    int id;
    // The name of this state
    const char *name;
    // The function to run every frame we're in this state.
    // Motion is the action we want to take this frame. SkillStatus
    // is the status that we will pass into the next function
    SkillStatus (*run)(World *w, Motion *out);
    // The function to run after we have called the run function.
    // Looks at the status from run, and our time in this state, to
    // decide which sytate id we should be in next
    int (*next)(World *w, SkillStatus s, double t_in_state);
    // the maximum amount of time we can be in this state
    // 0 = no timeout, the `next` function is responsible for acting on it
    double timeout_s;
} StateDef;

const StateDef *fsm_find(const StateDef *table, int count, int id);

// Logs "t=%.2f NAME -> NAME (reason)" to stderr. Call this when next_id
// differs from the current id, for self information, and to ensure that we
// actually follow the fsm arrows.
void fsm_log_transition(const StateDef *table, int count, int from_id, int to_id,
                         double now, const char *reason);

// Say WHY the FSM is about to move to `to_id`. Put it on its own line right
// before the `return to_id;` (or before a one-line `if (...) return to_id;`):
//     fsm_reason(P_REPOSITION, "align timeout %.1fs > %.1fs", t, ALIGN_TIMEOUT_S);
// fsm_log_transition() uses it if the transition it logs goes to `to_id`;
// otherwise it falls back to the generic "state"/"pre-empt". The latest call
// wins, so a reason set before an `if` that doesn't fire is overwritten by the
// next return's own reason. The reason also lands in the run log
// (transitions.jsonl, see runlog.h) so a human or an AI can see the cause.
void fsm_reason(int to_id, const char *fmt, ...);

// Forget any pending reason. Each mode's fsm_step calls this once per frame
// (top of the step, and again right before `next()`).
void fsm_reason_clear(void);

#endif
