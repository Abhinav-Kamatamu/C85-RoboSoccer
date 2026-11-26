#include "fsm.h"
#include "runlog.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// Pending "why" for the next transition, see fsm_reason() in fsm.h
static int g_reason_to = -1;
static char g_reason[160];

// Table lookup without telling the run log (used for the transition printout)
static const StateDef *find_row(const StateDef *table, int count, int id) {
    for (int i = 0; i < count; i++)
        if (table[i].id == id)
            return &table[i];
    return NULL;
}

// find the fsm state associated with id up to count
const StateDef *fsm_find(const StateDef *table, int count, int id) {
    // Every mode's step looks its current row up through here exactly once per
    // frame, so this doubles as "which state (and sub-state) ran this frame"
    // for the run log, and teaches it every state's name.
    runlog_note_states(table, count);
    const StateDef *row = find_row(table, count, id);
    if (row)
        runlog_fsm_visit(row->id);
    return row;
}

void fsm_reason(int to_id, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_reason, sizeof(g_reason), fmt, ap);
    va_end(ap);
    g_reason_to = to_id;
}

void fsm_reason_clear(void) {
    g_reason_to = -1;
    g_reason[0] = 0;
}

// Log the transition between two states
void fsm_log_transition(
    const StateDef *table, int count, int from_id,
    int to_id, double now, const char *reason
) {
    const StateDef *f = find_row(table, count, from_id);
    const StateDef *t = find_row(table, count, to_id);
    const char *why = (g_reason_to == to_id && g_reason[0]) ? g_reason : NULL;
    if (why)
        fprintf(
            stderr, "t=%.2f %d %s -> %d %s (%s: %s)\n",
            now, from_id, f ? f->name : "?", to_id,
            t ? t->name : "?", reason, why
        );
    else
        fprintf(
            stderr, "t=%.2f %d %s -> %d %s (%s)\n",
            now, from_id, f ? f->name : "?", to_id,
            t ? t->name : "?", reason
        );
    runlog_transition(from_id, f ? f->name : "?", to_id, t ? t->name : "?",
                      now, reason, why ? why : "");
    fsm_reason_clear();
}
