/***************************************************
 CSC C85 RoboSoccer FSM replay shim

 Built by analyze_run.py (`replay` command) into a shared library together
 with the real logic/*.c files, so a recorded run's World snapshots can be
 fed back through the CURRENT FSM code without the camera or the robot.
 Not part of the robot build.
***************************************************/

#include "logic/mode_chase.h"
#include "logic/mode_penalty.h"
#include "logic/mode_soccer.h"

// mode: 0 soccer, 1 penalty, 2 chase (same as the roboSoccer command line)
int replay_step(int mode, World *w, int state, double now, Motion *out) {
    switch (mode) {
    case 0:
        return mode_soccer_fsm_step(w, state, now, out);
    case 1:
        return mode_penalty_fsm_step(w, state, now, out);
    case 2:
        return mode_chase_fsm_step(w, state, now, out);
    }
    return state;
}

int replay_world_size(void) {
    return (int)sizeof(World);
}
