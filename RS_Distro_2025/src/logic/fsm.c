#include "fsm.h"
#include <stdio.h>

// find the fsm state associated with id up to count
const StateDef *fsm_find(const StateDef *table, int count, int id) {
    for (int i = 0; i < count; i++)
        if (table[i].id == id)
            return &table[i];
    return NULL;
}

// Log the transition between two states
void fsm_log_transition(
    const StateDef *table, int count, int from_id,
    int to_id, double now, const char *reason
) {
    const StateDef *f = fsm_find(table, count, from_id);
    const StateDef *t = fsm_find(table, count, to_id);
    fprintf(
        stderr, "t=%.2f %d %s -> %d %s (%s)\n",
        now, from_id, f ? f->name : "?", to_id,
        t ? t->name : "?", reason
    );
}
