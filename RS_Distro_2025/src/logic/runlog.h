/***************************************************
 CSC C85 RoboSoccer run log (the "flight recorder")

 One folder per run under .local/logs/ (or $ROBOSOCCER_LOG_DIR):

   .local/logs/
     README_FOR_AI.md                  how to read everything below
     0042_2026-11-25_14-03-22_penalty_red/
       manifest.json                   who/what/when, config, calibration, stats
       frames.jsonl                    one JSON object per processed frame
       transitions.jsonl               one JSON object per FSM state change
       positions.log                   the old human-readable per-frame line
       config/                         ai_config.h + calibration files as run
       frames/f000123.png              5 fps snapshots   (framelog.c)
       events/E003_f000456_P_UNSTICK/  replay-buffer dumps (framelog.c)

 Which parts are written is the logItems bitmask from the command line:
 bit0 text logs, bit1 display frames, bit2 raw camera/field frames.

 Everything here is plain stdio on the main thread and cheap (one fprintf'd
 line per frame). Images are framelog.c's job, on its own thread.
***************************************************/

#ifndef LOGIC_RUNLOG_H
#define LOGIC_RUNLOG_H

#include "world.h"
#include <stddef.h>

#define RUNLOG_TEXT    1  // frames.jsonl, transitions.jsonl, positions.log, manifest
#define RUNLOG_DISPLAY 2  // what the window shows (blobs + overlays), 5 fps + events
#define RUNLOG_RAW     4  // raw camera (1 fps) + unwarped field (events only)

struct StateDef;

// Create the run folder, prune old runs to the disk budget, write the
// manifest. Returns the bits that are actually on (images get switched off
// when the disk is nearly full). 0 = logging off.
int runlog_start(int log_items, int own_col, int ai_mode, int sx, int sy,
                 const char *colours_file);
// Rewrites the manifest with end-of-run stats and closes files. Safe to call
// twice, registered with atexit() by runlog_start().
void runlog_stop(void);

int runlog_items(void);          // bits that are on, 0 if not started
const char *runlog_dir(void);    // run folder, NULL if none

// ---- per frame, all called from the main loop in this order ----
void runlog_frame_begin(int frame_no, double t);
void runlog_frame_blob(int blob_id, int idtype, double cx, double cy, int size,
                       int x1, int y1, int x2, int y2,
                       double R, double G, double B,
                       double H, double S, double V);
// From world_log_frame() (perception.c), i.e. only while the AI is running.
// raw[12] = self dx,dy,mx,my  opp dx,dy,mx,my  ball dx,dy,mx,my (pixels/unit)
void runlog_frame_ai(int state, const World *w, Motion m, Vec2 staging, Vec2 aim,
                     double sent_l, double sent_r, const double raw[12],
                     int self_blob, int opp_blob, int ball_blob);
void runlog_frame_image(const char *key, const char *relpath);
void runlog_frame_end(void);

// ---- FSM hooks (fsm.c) ----
void runlog_note_states(const struct StateDef *table, int count);
void runlog_fsm_visit(int state_id);
void runlog_transition(int from_id, const char *from_name, int to_id,
                       const char *to_name, double t, const char *kind,
                       const char *reason);

// Manual bookmark ('b' key): logged and treated as an event.
void runlog_mark(const char *note);

// Returns 1 (and the event name) once per new event worth an image dump:
// entering a wall-recover/unstick/lost/strike/shot-done state, any pre-empt,
// a manual mark, or quitting. framelog.c polls this every frame.
int runlog_take_event(char *name, size_t n);
void runlog_request_event(const char *name);

// For drawing the robot's intent on the display (imageCapture.c). Returns 0
// when the AI hasn't produced anything this frame.
typedef struct {
    int  self_valid, ball_valid;
    Vec2 self_pos, self_heading, ball_pos, staging, aim;
    int  have_plan;
} RunlogOverlay;
int runlog_overlay(RunlogOverlay *o);

// Current state name for labelling images ("" if unknown).
const char *runlog_state_name(void);

// Image stats from framelog.c, written into the manifest at the end.
void runlog_image_stats(long snapshots, long events, long dropped, long long bytes);

#endif
