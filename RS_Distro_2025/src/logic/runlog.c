/***************************************************
 CSC C85 RoboSoccer run log, see runlog.h for the folder layout.

 Written to be read by an AI as much as by us: every number is in field
 units (cm, s, deg), every frame is one self-contained JSON object, and the
 error terms that the FSM guards compare against are computed here so
 nobody has to re-derive them from positions.
***************************************************/

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "runlog.h"
#include "fsm.h"
#include "geometry.h"
#include "ai_config.h"
#include "clock.h"

#include <dirent.h>
#include <errno.h>
#include <ftw.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <time.h>
#include <unistd.h>

#define RUNLOG_SCHEMA_VERSION 1
#define MAX_BLOBS        64
#define MAX_NAMES        128
#define MAX_PATH_IDS     4
#define MAX_FRAME_TRANS  4
#define MAX_FRAME_IMAGES 4
#define MAX_PENDING_EV   16
#define DEFAULT_BUDGET_GB     5.0
#define RUN_RESERVE_BYTES     (1024.0 * 1024 * 1024)       // room left for this run
#define MIN_FREE_IMAGES_BYTES (2.0 * 1024 * 1024 * 1024)   // below: no images
#define MIN_FREE_TEXT_BYTES   (200.0 * 1024 * 1024)        // below: nothing

// **** run-wide state ****
static int g_items = 0;
static int g_stopped = 0;
static char g_root[1100];
static char g_dir[1400];
static char g_repo[1024];
static char g_run_name[256];
static char g_started_at[64];
static char g_colours_file[256];
static double g_t_start = 0;
static int g_mode = 0, g_col = 0, g_sx = 0, g_sy = 0;
static FILE *g_frames = NULL;
static FILE *g_trans = NULL;
static long g_n_frames = 0, g_n_ai_frames = 0, g_n_trans = 0, g_n_events = 0, g_n_marks = 0;
static long g_img_snapshots = 0, g_img_events = 0, g_img_dropped = 0;
static long long g_img_bytes = 0;
static char g_disabled_note[256] = "";

// first World we saw, for the manifest
static int g_have_field = 0;
static World g_field;

// state id -> name, learned from every StateDef table fsm_find() sees
static int g_name_id[MAX_NAMES];
static const char *g_name_str[MAX_NAMES];
static int g_n_names = 0;
static const void *g_seen_tables[16];
static int g_n_seen_tables = 0;

// pending image-dump events
static char g_ev[MAX_PENDING_EV][64];
static int g_ev_head = 0, g_ev_n = 0;

// **** the frame being built ****
typedef struct {
    int id, idtype, size, x1, y1, x2, y2;
    double cx, cy, R, G, B, H, S, V;
} BlobRec;

typedef struct {
    int from, to;
    char kind[16];
    char reason[160];
} TransRec;

static struct {
    int open;
    int frame_no;
    double t;
    int n_blobs;
    BlobRec blobs[MAX_BLOBS];
    int have_ai;
    int state;
    World w;
    Motion m;
    Vec2 staging, aim;
    double sent_l, sent_r;
    double raw[12];
    int self_blob, opp_blob, ball_blob;
    int n_path;
    int path[MAX_PATH_IDS];
    int n_trans;
    TransRec trans[MAX_FRAME_TRANS];
    int n_images;
    char img_key[MAX_FRAME_IMAGES][16];
    char img_path[MAX_FRAME_IMAGES][96];
    char mark[128];
} g_fr;

static double g_last_frame_t = -1;
static int g_cur_state = -1;        // the state that actually ran last (top level)
static double g_cur_state_since = 0;
static RunlogOverlay g_overlay;
static int g_overlay_valid = 0;

// ============================================================ helpers ====

static const char *mode_name(int m) {
    return m == 0 ? "soccer" : m == 1 ? "penalty" : m == 2 ? "chase" : "unknown";
}

static const char *col_name(int c) {
    return c == 0 ? "blue" : c == 1 ? "red" : "unknown";
}

static const char *state_name(int id) {
    for (int i = 0; i < g_n_names; i++)
        if (g_name_id[i] == id)
            return g_name_str[i];
    return NULL;
}

static int mkdir_p(const char *path) {
    char tmp[1200];
    snprintf(tmp, sizeof(tmp), "%s", path);
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = 0;
            if (mkdir(tmp, 0755) != 0 && errno != EEXIST)
                return -1;
            *p = '/';
        }
    }
    if (mkdir(tmp, 0755) != 0 && errno != EEXIST)
        return -1;
    return 0;
}

static int copy_file(const char *src, const char *dst) {
    FILE *in = fopen(src, "rb");
    if (!in)
        return -1;
    FILE *out = fopen(dst, "wb");
    if (!out) {
        fclose(in);
        return -1;
    }
    char buf[65536];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), in)) > 0)
        fwrite(buf, 1, n, out);
    fclose(in);
    fclose(out);
    return 0;
}

// Walk up from the current directory to the folder holding .git
static int find_repo_root(char *out, size_t n) {
    char cwd[1024];
    if (!getcwd(cwd, sizeof(cwd)))
        return 0;
    for (;;) {
        char probe[1100];
        snprintf(probe, sizeof(probe), "%s/.git", cwd);
        if (access(probe, F_OK) == 0) {
            snprintf(out, n, "%s", cwd);
            return 1;
        }
        char *slash = strrchr(cwd, '/');
        if (!slash || slash == cwd)
            return 0;
        *slash = 0;
    }
}

// Run a shell command and keep the first line of its output
static void cmd_line(const char *cmd, char *out, size_t n) {
    out[0] = 0;
    FILE *p = popen(cmd, "r");
    if (!p)
        return;
    if (fgets(out, (int)n, p)) {
        size_t l = strlen(out);
        while (l && (out[l - 1] == '\n' || out[l - 1] == '\r'))
            out[--l] = 0;
    }
    pclose(p);
}

// JSON string escaping into a FILE
static void fjstr(FILE *f, const char *s) {
    fputc('"', f);
    for (; s && *s; s++) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') {
            fputc('\\', f);
            fputc(c, f);
        } else if (c < 0x20) {
            fprintf(f, "\\u%04x", c);
        } else {
            fputc(c, f);
        }
    }
    fputc('"', f);
}

// **** a tiny appender for building one frame line ****
typedef struct {
    char *buf;
    size_t cap, len;
} Sb;

static void sb_add(Sb *b, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void sb_add(Sb *b, const char *fmt, ...) {
    if (b->len >= b->cap)
        return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b->buf + b->len, b->cap - b->len, fmt, ap);
    va_end(ap);
    if (n > 0)
        b->len += (size_t)n < b->cap - b->len ? (size_t)n : b->cap - b->len - 1;
}

static void sb_num(Sb *b, double v, int prec) {
    if (isfinite(v))
        sb_add(b, "%.*f", prec, v);
    else
        sb_add(b, "null");
}

static void sb_vec(Sb *b, Vec2 v, int prec) {
    sb_add(b, "[");
    sb_num(b, v.x, prec);
    sb_add(b, ",");
    sb_num(b, v.y, prec);
    sb_add(b, "]");
}

static void sb_str(Sb *b, const char *s) {
    sb_add(b, "\"");
    for (; s && *s; s++) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\')
            sb_add(b, "\\%c", c);
        else if (c < 0x20)
            sb_add(b, "\\u%04x", c);
        else
            sb_add(b, "%c", c);
    }
    sb_add(b, "\"");
}

static int vec_is_zero(Vec2 v) {
    return v.x == 0 && v.y == 0;
}

// ======================================================== old-run cleanup ==

static long long g_du_total;
static int du_cb(const char *p, const struct stat *st, int flag, struct FTW *ftw) {
    (void)p;
    (void)ftw;
    if (flag == FTW_F)
        g_du_total += st->st_size;
    return 0;
}

static long long dir_bytes(const char *path) {
    g_du_total = 0;
    nftw(path, du_cb, 16, FTW_PHYS);
    return g_du_total;
}

static int rm_cb(const char *p, const struct stat *st, int flag, struct FTW *ftw) {
    (void)st;
    (void)flag;
    (void)ftw;
    return remove(p);
}

static int is_run_dir_name(const char *n) {
    for (int i = 0; i < 4; i++)
        if (n[i] < '0' || n[i] > '9')
            return 0;
    return n[4] == '_';
}

typedef struct {
    char name[256];
    long long bytes;
    int keep;
} RunDir;

static int rundir_cmp(const void *a, const void *b) {
    return strcmp(((const RunDir *)a)->name, ((const RunDir *)b)->name);
}

// Deletes the oldest runs (never ones holding a KEEP file) until everything
// plus a reserve for this run fits in the budget. Returns the next run number.
static int prune_old_runs(double budget_bytes) {
    DIR *d = opendir(g_root);
    if (!d)
        return 1;
    RunDir *runs = NULL;
    int n = 0, cap = 0, next_id = 1;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (!is_run_dir_name(e->d_name))
            continue;
        if (n == cap) {
            cap = cap ? cap * 2 : 64;
            runs = (RunDir *)realloc(runs, sizeof(RunDir) * cap);
        }
        snprintf(runs[n].name, sizeof(runs[n].name), "%s", e->d_name);
        char p[1700];
        snprintf(p, sizeof(p), "%s/%s", g_root, e->d_name);
        runs[n].bytes = dir_bytes(p);
        snprintf(p, sizeof(p), "%s/%s/KEEP", g_root, e->d_name);
        runs[n].keep = access(p, F_OK) == 0;
        int id = atoi(e->d_name);
        if (id >= next_id)
            next_id = id + 1;
        n++;
    }
    closedir(d);
    qsort(runs, n, sizeof(RunDir), rundir_cmp);

    double total = 0;
    for (int i = 0; i < n; i++)
        total += (double)runs[i].bytes;
    for (int i = 0; i < n && total + RUN_RESERVE_BYTES > budget_bytes; i++) {
        if (runs[i].keep)
            continue;
        char p[1700];
        snprintf(p, sizeof(p), "%s/%s", g_root, runs[i].name);
        if (nftw(p, rm_cb, 16, FTW_DEPTH | FTW_PHYS) == 0) {
            fprintf(stderr, "runlog: deleted old run %s (%.1f MB) to stay under the %.1f GB log budget\n",
                    runs[i].name, runs[i].bytes / 1e6, budget_bytes / 1e9);
            total -= (double)runs[i].bytes;
        }
    }
    free(runs);
    return next_id;
}

// =============================================================== manifest ==

static void write_manifest(int final) {
    if (!g_dir[0])
        return;
    char p[1700], tmp[1700];
    snprintf(p, sizeof(p), "%s/manifest.json", g_dir);
    snprintf(tmp, sizeof(tmp), "%s/manifest.json.tmp", g_dir);
    FILE *f = fopen(tmp, "w");
    if (!f)
        return;

    char commit[64], branch[128], dirty[8];
    commit[0] = branch[0] = dirty[0] = 0;
    if (g_repo[0] && !strchr(g_repo, '\'')) {
        char cmd[1400];
        snprintf(cmd, sizeof(cmd), "git -C '%s' rev-parse --short=12 HEAD 2>/dev/null", g_repo);
        cmd_line(cmd, commit, sizeof(commit));
        snprintf(cmd, sizeof(cmd), "git -C '%s' rev-parse --abbrev-ref HEAD 2>/dev/null", g_repo);
        cmd_line(cmd, branch, sizeof(branch));
        snprintf(cmd, sizeof(cmd),
                 "git -C '%s' status --porcelain --untracked-files=no 2>/dev/null | head -c1", g_repo);
        cmd_line(cmd, dirty, sizeof(dirty));
    }

    double dur = maintenant() - g_t_start;
    fprintf(f, "{\n");
    fprintf(f, "  \"schema_version\": %d,\n", RUNLOG_SCHEMA_VERSION);
    fprintf(f, "  \"run\": ");
    fjstr(f, g_run_name);
    fprintf(f, ",\n  \"complete\": %s,\n", final ? "true" : "false");
    fprintf(f, "  \"started_at\": ");
    fjstr(f, g_started_at);
    fprintf(f, ",\n  \"mode\": {\"id\": %d, \"name\": \"%s\"},\n", g_mode, mode_name(g_mode));
    fprintf(f, "  \"own_colour\": {\"id\": %d, \"name\": \"%s\"},\n", g_col, col_name(g_col));
    fprintf(f, "  \"git\": {\"commit\": ");
    fjstr(f, commit);
    fprintf(f, ", \"branch\": ");
    fjstr(f, branch);
    fprintf(f, ", \"uncommitted_changes\": %s},\n", dirty[0] ? "true" : "false");
    fprintf(f, "  \"log_items\": {\"bits\": %d, \"text\": %s, \"display_frames\": %s, \"raw_frames\": %s},\n",
            g_items, (g_items & RUNLOG_TEXT) ? "true" : "false",
            (g_items & RUNLOG_DISPLAY) ? "true" : "false",
            (g_items & RUNLOG_RAW) ? "true" : "false");
    fprintf(f, "  \"logging_note\": ");
    fjstr(f, g_disabled_note);
    fprintf(f, ",\n  \"camera\": {\"width_px\": %d, \"height_px\": %d, \"nominal_fps\": 30},\n", g_sx, g_sy);
    fprintf(f, "  \"images\": {\"display_size_px\": [1024, 768], \"event_display_size_px\": [512, 384],"
               " \"snapshot_fps\": 5, \"raw_snapshot_fps\": 1, \"event_pre_s\": 2.0, \"event_post_s\": 1.0},\n");
    fprintf(f, "  \"colours_file\": ");
    fjstr(f, g_colours_file);
    fprintf(f, ",\n");

    // Compiled-in values (what this binary actually used), not ai_config.h on
    // disk, which may have been edited since. config/ai_config.h is the
    // whole file as it was when the run started.
    fprintf(f, "  \"config\": {\n");
    fprintf(f, "    \"PX_PER_CM_X\": %g, \"PX_PER_CM_Y\": %g,\n", (double)PX_PER_CM_X, (double)PX_PER_CM_Y);
    fprintf(f, "    \"STAGE_TOL_CM\": %g, \"MOVE_TOL_CM\": %g, \"LAT_TOL_CM\": %g, \"STAGE_DIST_CM\": %g,\n",
            (double)STAGE_TOL_CM, (double)MOVE_TOL_CM, (double)LAT_TOL_CM, (double)STAGE_DIST_CM);
    fprintf(f, "    \"ALIGN_TOL_DEG\": %g, \"ABORT_TOL_DEG\": %g, \"ENGAGE_BALL_DEG\": %g, \"ENGAGE_BALL_CM\": %g,\n",
            (double)ALIGN_TOL_DEG, (double)ABORT_TOL_DEG, (double)ENGAGE_BALL_DEG, (double)ENGAGE_BALL_CM);
    fprintf(f, "    \"GOTO_STAGING_TIMEOUT_S\": %g, \"ALIGN_TIMEOUT_S\": %g, \"STRIKE_TIMEOUT_S\": %g, \"RETRY_WAIT_S\": %g,\n",
            (double)GOTO_STAGING_TIMEOUT_S, (double)ALIGN_TIMEOUT_S, (double)STRIKE_TIMEOUT_S, (double)RETRY_WAIT_S);
    fprintf(f, "    \"ALIGN_ATTEMPT_LIMIT\": %g, \"WALL_RECOVER_ATTEMPT_LIMIT\": %g, \"NO_SHOT_MOVED_CM\": %g,\n",
            (double)ALIGN_ATTEMPT_LIMIT, (double)WALL_RECOVER_ATTEMPT_LIMIT, (double)NO_SHOT_MOVED_CM);
    fprintf(f, "    \"LOST_FRAMES\": %g, \"BALL_LOST_FRAMES\": %g, \"READY_FRAMES\": %g,\n",
            (double)LOST_FRAMES, (double)BALL_LOST_FRAMES, (double)READY_FRAMES);
    fprintf(f, "    \"WALL_TRIGGER_MARGIN_CM\": %g, \"DEFAULT_WALL_MARGIN_CM\": %g, \"WALL_PUSH_BAND_CM\": %g, \"WALL_OUTWARD_DOT\": %g,\n",
            (double)WALL_TRIGGER_MARGIN_CM, (double)DEFAULT_WALL_MARGIN_CM, (double)WALL_PUSH_BAND_CM, (double)WALL_OUTWARD_DOT);
    fprintf(f, "    \"UNSTICK_STALL_S\": %g, \"UNSTICK_REVERSE_S\": %g, \"UNSTICK_TURN_S\": %g,\n",
            (double)UNSTICK_STALL_S, (double)UNSTICK_REVERSE_S, (double)UNSTICK_TURN_S);
    fprintf(f, "    \"GOTO_V_CMPS\": %g, \"STRIKE_V_CMPS\": %g, \"DEFEND_DANGER_DIST_CM\": %g, \"GOAL_CONFIRM_MARGIN_CM\": %g,\n",
            (double)GOTO_V_CMPS, (double)STRIKE_V_CMPS, (double)DEFEND_DANGER_DIST_CM, (double)GOAL_CONFIRM_MARGIN_CM);
    fprintf(f, "    \"DEFAULT_FIELD_W_CM\": %g, \"DEFAULT_FIELD_H_CM\": %g, \"DEFAULT_GOAL_WIDTH_CM\": %g\n",
            (double)DEFAULT_FIELD_W_CM, (double)DEFAULT_FIELD_H_CM, (double)DEFAULT_GOAL_WIDTH_CM);
    fprintf(f, "  },\n");

    if (g_have_field) {
        fprintf(f, "  \"field\": {\"min_cm\": [%.1f, %.1f], \"max_cm\": [%.1f, %.1f], "
                   "\"own_goal_cm\": [%.1f, %.1f], \"their_goal_cm\": [%.1f, %.1f], \"goal_width_cm\": %.1f},\n",
                g_field.field_min.x, g_field.field_min.y, g_field.field_max.x, g_field.field_max.y,
                g_field.own_goal.x, g_field.own_goal.y, g_field.their_goal.x, g_field.their_goal.y,
                g_field.goal_width_cm);
    } else {
        fprintf(f, "  \"field\": null,\n");
    }

    fprintf(f, "  \"states\": {");
    for (int i = 0; i < g_n_names; i++) {
        fprintf(f, "%s\"%d\": ", i ? ", " : "", g_name_id[i]);
        fjstr(f, g_name_str[i]);
    }
    fprintf(f, "},\n");

    fprintf(f, "  \"stats\": {\"duration_s\": %.2f, \"frames\": %ld, \"ai_frames\": %ld, \"avg_fps\": %.2f, "
               "\"transitions\": %ld, \"events\": %ld, \"marks\": %ld, \"snapshots\": %ld, "
               "\"event_dumps\": %ld, \"images_dropped\": %ld, \"image_bytes\": %lld}\n",
            dur, g_n_frames, g_n_ai_frames, dur > 0 ? g_n_frames / dur : 0.0,
            g_n_trans, g_n_events, g_n_marks, g_img_snapshots, g_img_events, g_img_dropped, g_img_bytes);
    fprintf(f, "}\n");
    fclose(f);
    rename(tmp, p);
}

// ================================================================ startup ==

int runlog_start(int log_items, int own_col, int ai_mode, int sx, int sy,
                 const char *colours_file) {
    if (g_items || log_items <= 0)
        return g_items;
    g_mode = ai_mode;
    g_col = own_col;
    g_sx = sx;
    g_sy = sy;
    g_t_start = maintenant();
    snprintf(g_colours_file, sizeof(g_colours_file), "%s", colours_file ? colours_file : "");

    const char *env_dir = getenv("ROBOSOCCER_LOG_DIR");
    int have_repo = find_repo_root(g_repo, sizeof(g_repo));
    if (!have_repo)
        g_repo[0] = 0;
    if (env_dir && env_dir[0])
        snprintf(g_root, sizeof(g_root), "%s", env_dir);
    else if (have_repo)
        snprintf(g_root, sizeof(g_root), "%s/.local/logs", g_repo);
    else
        snprintf(g_root, sizeof(g_root), ".local/logs");
    if (mkdir_p(g_root) != 0) {
        fprintf(stderr, "runlog: cannot create %s, logging OFF\n", g_root);
        return 0;
    }

    double budget_gb = DEFAULT_BUDGET_GB;
    const char *env_budget = getenv("ROBOSOCCER_LOG_BUDGET_GB");
    if (env_budget && atof(env_budget) > 0)
        budget_gb = atof(env_budget);
    int run_id = prune_old_runs(budget_gb * 1e9);

    // Not enough disk: drop images first, then everything
    struct statvfs vfs;
    if (statvfs(g_root, &vfs) == 0) {
        double free_b = (double)vfs.f_bavail * (double)vfs.f_frsize;
        if (free_b < MIN_FREE_TEXT_BYTES) {
            fprintf(stderr, "runlog: only %.0f MB free on disk, logging OFF\n", free_b / 1e6);
            return 0;
        }
        if (free_b < MIN_FREE_IMAGES_BYTES && (log_items & (RUNLOG_DISPLAY | RUNLOG_RAW))) {
            log_items &= ~(RUNLOG_DISPLAY | RUNLOG_RAW);
            snprintf(g_disabled_note, sizeof(g_disabled_note),
                     "image logging switched off at start: only %.1f GB free on disk", free_b / 1e9);
            fprintf(stderr, "runlog: %s\n", g_disabled_note);
        }
    }
    // Images and events are only useful next to the text log, so text is always on
    log_items |= RUNLOG_TEXT;

    time_t now = time(NULL);
    struct tm lt;
    localtime_r(&now, &lt);
    char stamp[64];
    strftime(stamp, sizeof(stamp), "%Y-%m-%d_%H-%M-%S", &lt);
    strftime(g_started_at, sizeof(g_started_at), "%Y-%m-%dT%H:%M:%S%z", &lt);
    snprintf(g_run_name, sizeof(g_run_name), "%04d_%s_%s_%s", run_id % 10000, stamp,
             mode_name(ai_mode), col_name(own_col));
    snprintf(g_dir, sizeof(g_dir), "%s/%s", g_root, g_run_name);
    char p[1700];
    snprintf(p, sizeof(p), "%s/config", g_dir);
    if (mkdir_p(p) != 0) {
        fprintf(stderr, "runlog: cannot create %s, logging OFF\n", g_dir);
        g_dir[0] = 0;
        return 0;
    }

    // Calibration + config exactly as this run used them
    const char *cal[] = {"Homography.dat", "offsets.dat", "colours.dat"};
    for (size_t i = 0; i < sizeof(cal) / sizeof(cal[0]); i++) {
        snprintf(p, sizeof(p), "%s/config/%s", g_dir, cal[i]);
        copy_file(cal[i], p);
    }
    if (g_colours_file[0]) {
        const char *base = strrchr(g_colours_file, '/');
        snprintf(p, sizeof(p), "%s/config/%s", g_dir, base ? base + 1 : g_colours_file);
        copy_file(g_colours_file, p);
    }
    if (g_repo[0]) {
        char src[1700];
        snprintf(src, sizeof(src), "%s/RS_Distro_2025/src/logic/ai_config.h", g_repo);
        snprintf(p, sizeof(p), "%s/config/ai_config.h", g_dir);
        copy_file(src, p);
        // Keep the guide next to the runs so an AI pointed at .local/logs finds it
        snprintf(src, sizeof(src), "%s/RS_Distro_2025/tools/runlog/README_FOR_AI.md", g_repo);
        snprintf(p, sizeof(p), "%s/README_FOR_AI.md", g_root);
        copy_file(src, p);
    }

    snprintf(p, sizeof(p), "%s/frames.jsonl", g_dir);
    g_frames = fopen(p, "w");
    snprintf(p, sizeof(p), "%s/transitions.jsonl", g_dir);
    g_trans = fopen(p, "w");
    if (!g_frames || !g_trans) {
        fprintf(stderr, "runlog: cannot open log files in %s, logging OFF\n", g_dir);
        if (g_frames)
            fclose(g_frames);
        if (g_trans)
            fclose(g_trans);
        g_frames = g_trans = NULL;
        g_dir[0] = 0;
        return 0;
    }

    g_items = log_items;
    write_manifest(0);
    atexit(runlog_stop);
    fprintf(stderr, "runlog: logging this run to %s (text%s%s)\n", g_dir,
            (g_items & RUNLOG_DISPLAY) ? " + display frames" : "",
            (g_items & RUNLOG_RAW) ? " + raw frames" : "");
    return g_items;
}

void runlog_stop(void) {
    if (!g_items || g_stopped)
        return;
    g_stopped = 1;
    if (g_fr.open)
        runlog_frame_end();
    if (g_frames)
        fclose(g_frames);
    if (g_trans)
        fclose(g_trans);
    g_frames = g_trans = NULL;
    write_manifest(1);
    fprintf(stderr, "runlog: run saved in %s\n", g_dir);
}

int runlog_items(void) {
    return g_stopped ? 0 : g_items;
}

const char *runlog_dir(void) {
    return g_dir[0] ? g_dir : NULL;
}

// ============================================================ per frame ====

void runlog_frame_begin(int frame_no, double t) {
    if (!runlog_items())
        return;
    if (g_fr.open)
        runlog_frame_end();
    g_fr.open = 1;
    g_fr.frame_no = frame_no;
    g_fr.t = t;
    g_fr.n_blobs = 0;
    g_fr.have_ai = 0;
    g_fr.n_path = 0;
    g_fr.n_trans = 0;
    g_fr.n_images = 0;
    g_fr.mark[0] = 0;
    g_overlay_valid = 0;
}

void runlog_frame_blob(int blob_id, int idtype, double cx, double cy, int size,
                       int x1, int y1, int x2, int y2,
                       double R, double G, double B,
                       double H, double S, double V) {
    if (!g_fr.open || g_fr.n_blobs >= MAX_BLOBS)
        return;
    BlobRec *b = &g_fr.blobs[g_fr.n_blobs++];
    b->id = blob_id;
    b->idtype = idtype;
    b->cx = cx;
    b->cy = cy;
    b->size = size;
    b->x1 = x1;
    b->y1 = y1;
    b->x2 = x2;
    b->y2 = y2;
    b->R = R;
    b->G = G;
    b->B = B;
    b->H = H;
    b->S = S;
    b->V = V;
}

void runlog_frame_ai(int state, const World *w, Motion m, Vec2 staging, Vec2 aim,
                     double sent_l, double sent_r, const double raw[12],
                     int self_blob, int opp_blob, int ball_blob) {
    if (!g_fr.open)
        return;
    g_fr.have_ai = 1;
    g_fr.state = state;
    g_fr.w = *w;
    g_fr.m = m;
    g_fr.staging = staging;
    g_fr.aim = aim;
    g_fr.sent_l = sent_l;
    g_fr.sent_r = sent_r;
    memcpy(g_fr.raw, raw, sizeof(g_fr.raw));
    g_fr.self_blob = self_blob;
    g_fr.opp_blob = opp_blob;
    g_fr.ball_blob = ball_blob;
    if (!g_have_field) {
        g_have_field = 1;
        g_field = *w;
    }

    g_overlay.self_valid = w->self.valid;
    g_overlay.ball_valid = w->ball.valid;
    g_overlay.self_pos = w->self.pos;
    g_overlay.self_heading = w->self.heading;
    g_overlay.ball_pos = w->ball.pos;
    g_overlay.staging = staging;
    g_overlay.aim = aim;
    g_overlay.have_plan = !vec_is_zero(staging) || !vec_is_zero(aim);
    g_overlay_valid = 1;
}

void runlog_frame_image(const char *key, const char *relpath) {
    if (!g_fr.open || g_fr.n_images >= MAX_FRAME_IMAGES)
        return;
    snprintf(g_fr.img_key[g_fr.n_images], sizeof(g_fr.img_key[0]), "%s", key);
    snprintf(g_fr.img_path[g_fr.n_images], sizeof(g_fr.img_path[0]), "%s", relpath);
    g_fr.n_images++;
}

static void sb_agent(Sb *b, const char *name, const Agent *a) {
    sb_add(b, "\"%s\":{\"valid\":%d,\"lost_frames\":%d,\"pos\":", name, a->valid, a->lost_frames);
    sb_vec(b, a->pos, 2);
    sb_add(b, ",\"vel\":");
    sb_vec(b, a->vel, 2);
    sb_add(b, ",\"heading\":");
    sb_vec(b, a->heading, 3);
    sb_add(b, ",\"heading_deg\":");
    if (vec_is_zero(a->heading))
        sb_add(b, "null");
    else
        sb_num(b, rad_to_deg(angle_of(a->heading)), 1);
    sb_add(b, ",\"speed\":");
    sb_num(b, vec2_len(a->vel), 2);
    sb_add(b, "}");
}

// The numbers the FSM guards compare against, so they can be read off
// directly instead of re-derived (null when they don't apply this frame).
static void sb_errors(Sb *b) {
    const World *w = &g_fr.w;
    int sv = w->self.valid, bv = w->ball.valid, ov = w->opp.valid;
    int have_stage = !vec_is_zero(g_fr.staging), have_aim = !vec_is_zero(g_fr.aim);
    int have_head = !vec_is_zero(w->self.heading);

    sb_add(b, "\"err\":{\"self_to_staging_cm\":");
    if (sv && have_stage) sb_num(b, vec2_dist(w->self.pos, g_fr.staging), 2); else sb_add(b, "null");
    sb_add(b, ",\"heading_err_to_aim_deg\":");
    if (sv && have_aim && have_head)
        sb_num(b, rad_to_deg(angle_diff_rad(angle_of(vec2_sub(g_fr.aim, w->self.pos)),
                                            angle_of(w->self.heading))), 1);
    else sb_add(b, "null");
    sb_add(b, ",\"heading_err_to_ball_deg\":");
    if (sv && bv && have_head)
        sb_num(b, rad_to_deg(angle_diff_rad(angle_of(vec2_sub(w->ball.pos, w->self.pos)),
                                            angle_of(w->self.heading))), 1);
    else sb_add(b, "null");
    sb_add(b, ",\"lateral_to_aim_line_cm\":");
    if (sv && bv && have_aim) sb_num(b, perp_dist_to_line(w->self.pos, w->ball.pos, g_fr.aim), 2);
    else sb_add(b, "null");
    sb_add(b, ",\"self_to_ball_cm\":");
    if (sv && bv) sb_num(b, vec2_dist(w->self.pos, w->ball.pos), 2); else sb_add(b, "null");
    sb_add(b, ",\"opp_to_ball_cm\":");
    if (ov && bv) sb_num(b, vec2_dist(w->opp.pos, w->ball.pos), 2); else sb_add(b, "null");
    sb_add(b, ",\"ball_to_their_goal_cm\":");
    if (bv) sb_num(b, vec2_dist(w->ball.pos, w->their_goal), 2); else sb_add(b, "null");
    sb_add(b, ",\"ball_to_own_goal_cm\":");
    if (bv) sb_num(b, vec2_dist(w->ball.pos, w->own_goal), 2); else sb_add(b, "null");
    sb_add(b, ",\"self_to_wall_cm\":");
    if (sv) sb_num(b, dist_to_nearest_wall(w->self.pos, w->field_min, w->field_max), 2); else sb_add(b, "null");
    sb_add(b, ",\"ball_to_wall_cm\":");
    if (bv) sb_num(b, dist_to_nearest_wall(w->ball.pos, w->field_min, w->field_max), 2); else sb_add(b, "null");
    sb_add(b, ",\"heading_dot_wall_inward\":");
    if (sv && have_head)
        sb_num(b, vec2_dot(w->self.heading,
                           direction_away_from_nearest_wall(w->self.pos, w->field_min, w->field_max)), 3);
    else sb_add(b, "null");
    sb_add(b, "}");
}

static const char *blob_role(int id) {
    if (!g_fr.have_ai)
        return NULL;
    if (id == g_fr.self_blob) return "self";
    if (id == g_fr.opp_blob) return "opp";
    if (id == g_fr.ball_blob) return "ball";
    return NULL;
}

void runlog_frame_end(void) {
    if (!g_fr.open)
        return;
    g_fr.open = 0;
    if (!g_frames)
        return;
    g_n_frames++;

    static char buf[40000];
    Sb b = {buf, sizeof(buf), 0};
    double dt_ms = g_last_frame_t < 0 ? 0 : (g_fr.t - g_last_frame_t) * 1000.0;
    g_last_frame_t = g_fr.t;

    sb_add(&b, "{\"f\":%d,\"t\":", g_fr.frame_no);
    sb_num(&b, g_fr.t - g_t_start, 3);
    sb_add(&b, ",\"dt_ms\":");
    sb_num(&b, dt_ms, 1);
    sb_add(&b, ",\"ai\":%s", g_fr.have_ai ? "true" : "false");

    if (g_fr.have_ai) {
        g_n_ai_frames++;
        const char *sn = state_name(g_fr.state);
        sb_add(&b, ",\"state\":%d,\"state_name\":", g_fr.state);
        sb_str(&b, sn ? sn : "?");
        sb_add(&b, ",\"ran\":[");
        for (int i = 0; i < g_fr.n_path; i++) {
            const char *pn = state_name(g_fr.path[i]);
            if (i) sb_add(&b, ",");
            sb_str(&b, pn ? pn : "?");
        }
        sb_add(&b, "]");
        // time in the (top-level) state that actually ran this frame
        int ran = g_fr.n_path ? g_fr.path[0] : g_fr.state;
        if (ran != g_cur_state) {
            g_cur_state = ran;
            g_cur_state_since = g_fr.t;
        }
        sb_add(&b, ",\"t_in_state\":");
        sb_num(&b, g_fr.t - g_cur_state_since, 2);

        sb_add(&b, ",");
        sb_agent(&b, "self", &g_fr.w.self);
        sb_add(&b, ",");
        sb_agent(&b, "opp", &g_fr.w.opp);
        sb_add(&b, ",");
        sb_agent(&b, "ball", &g_fr.w.ball);
        sb_add(&b, ",\"plan\":{\"staging\":");
        if (vec_is_zero(g_fr.staging)) sb_add(&b, "null"); else sb_vec(&b, g_fr.staging, 2);
        sb_add(&b, ",\"aim\":");
        if (vec_is_zero(g_fr.aim)) sb_add(&b, "null"); else sb_vec(&b, g_fr.aim, 2);
        sb_add(&b, "},");
        sb_errors(&b);
        sb_add(&b, ",\"cmd\":{\"v_cmps\":");
        sb_num(&b, g_fr.m.v, 2);
        sb_add(&b, ",\"w_radps\":");
        sb_num(&b, g_fr.m.w, 3);
        sb_add(&b, "},\"sent_prev\":{\"l\":");
        sb_num(&b, g_fr.sent_l, 0);
        sb_add(&b, ",\"r\":");
        sb_num(&b, g_fr.sent_r, 0);
        sb_add(&b, "},\"raw_px\":{");
        const char *rn[3] = {"self", "opp", "ball"};
        for (int i = 0; i < 3; i++) {
            sb_add(&b, "%s\"%s\":{\"shape_dir\":[", i ? "," : "", rn[i]);
            sb_num(&b, g_fr.raw[i * 4 + 0], 3);
            sb_add(&b, ",");
            sb_num(&b, g_fr.raw[i * 4 + 1], 3);
            sb_add(&b, "],\"motion_dir\":[");
            sb_num(&b, g_fr.raw[i * 4 + 2], 3);
            sb_add(&b, ",");
            sb_num(&b, g_fr.raw[i * 4 + 3], 3);
            sb_add(&b, "]}");
        }
        sb_add(&b, "}");
    }

    sb_add(&b, ",\"blobs\":[");
    for (int i = 0; i < g_fr.n_blobs; i++) {
        BlobRec *bl = &g_fr.blobs[i];
        const char *role = blob_role(bl->id);
        sb_add(&b, "%s{\"id\":%d,\"idtype\":%d,\"role\":", i ? "," : "", bl->id, bl->idtype);
        if (role) sb_str(&b, role); else sb_add(&b, "null");
        sb_add(&b, ",\"c_px\":[%.1f,%.1f],\"c_cm\":[%.1f,%.1f],\"size\":%d,\"bbox_px\":[%d,%d,%d,%d],"
                   "\"rgb\":[%.0f,%.0f,%.0f],\"hsv\":[%.1f,%.3f,%.3f]}",
               bl->cx, bl->cy, bl->cx / PX_PER_CM_X, bl->cy / PX_PER_CM_Y, bl->size,
               bl->x1, bl->y1, bl->x2, bl->y2, bl->R, bl->G, bl->B, bl->H, bl->S, bl->V);
    }
    sb_add(&b, "]");

    if (g_fr.n_trans) {
        sb_add(&b, ",\"transitions\":[");
        for (int i = 0; i < g_fr.n_trans; i++) {
            TransRec *tr = &g_fr.trans[i];
            const char *fn = state_name(tr->from), *tn = state_name(tr->to);
            sb_add(&b, "%s{\"from\":", i ? "," : "");
            sb_str(&b, fn ? fn : "?");
            sb_add(&b, ",\"to\":");
            sb_str(&b, tn ? tn : "?");
            sb_add(&b, ",\"kind\":");
            sb_str(&b, tr->kind);
            sb_add(&b, ",\"reason\":");
            sb_str(&b, tr->reason);
            sb_add(&b, "}");
        }
        sb_add(&b, "]");
    }
    if (g_fr.mark[0]) {
        sb_add(&b, ",\"mark\":");
        sb_str(&b, g_fr.mark);
    }
    if (g_fr.n_images) {
        sb_add(&b, ",\"images\":{");
        for (int i = 0; i < g_fr.n_images; i++) {
            sb_add(&b, "%s\"%s\":", i ? "," : "", g_fr.img_key[i]);
            sb_str(&b, g_fr.img_path[i]);
        }
        sb_add(&b, "}");
    }
    sb_add(&b, "}\n");
    fputs(buf, g_frames);
    fflush(g_frames);
}

// ============================================================ FSM hooks ====

void runlog_note_states(const struct StateDef *table, int count) {
    for (int i = 0; i < g_n_seen_tables; i++)
        if (g_seen_tables[i] == (const void *)table)
            return;
    if (g_n_seen_tables < 16)
        g_seen_tables[g_n_seen_tables++] = table;
    for (int i = 0; i < count && g_n_names < MAX_NAMES; i++) {
        if (state_name(table[i].id))
            continue;
        g_name_id[g_n_names] = table[i].id;
        g_name_str[g_n_names] = table[i].name;
        g_n_names++;
    }
}

void runlog_fsm_visit(int state_id) {
    if (g_fr.open && g_fr.n_path < MAX_PATH_IDS)
        g_fr.path[g_fr.n_path++] = state_id;
}

static void push_event(const char *name) {
    if (g_ev_n == MAX_PENDING_EV)
        return;
    int slot = (g_ev_head + g_ev_n) % MAX_PENDING_EV;
    snprintf(g_ev[slot], sizeof(g_ev[0]), "%s", name);
    g_ev_n++;
    g_n_events++;
}

static int is_event_state(const char *name) {
    static const char *keys[] = {"WALL_RECOVER", "UNSTICK", "LOST", "STRIKE", "SHOT_DONE"};
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++)
        if (strstr(name, keys[i]))
            return 1;
    return 0;
}

void runlog_transition(int from_id, const char *from_name, int to_id,
                       const char *to_name, double t, const char *kind,
                       const char *reason) {
    if (!runlog_items())
        return;
    g_n_trans++;
    if (g_trans) {
        fprintf(g_trans, "{\"f\":%d,\"t\":%.3f,\"from\":%d,\"from_name\":",
                g_fr.open ? g_fr.frame_no : -1, t - g_t_start, from_id);
        fjstr(g_trans, from_name);
        fprintf(g_trans, ",\"to\":%d,\"to_name\":", to_id);
        fjstr(g_trans, to_name);
        fprintf(g_trans, ",\"kind\":");
        fjstr(g_trans, kind);
        fprintf(g_trans, ",\"reason\":");
        fjstr(g_trans, reason);
        fprintf(g_trans, "}\n");
        fflush(g_trans);
    }
    if (g_fr.open && g_fr.n_trans < MAX_FRAME_TRANS) {
        TransRec *tr = &g_fr.trans[g_fr.n_trans++];
        tr->from = from_id;
        tr->to = to_id;
        snprintf(tr->kind, sizeof(tr->kind), "%s", kind);
        snprintf(tr->reason, sizeof(tr->reason), "%s", reason);
    }
    if (strcmp(kind, "pre-empt") == 0 || is_event_state(to_name))
        push_event(to_name);
}

void runlog_mark(const char *note) {
    if (!runlog_items())
        return;
    g_n_marks++;
    if (g_fr.open)
        snprintf(g_fr.mark, sizeof(g_fr.mark), "%s", note);
    if (g_trans) {
        fprintf(g_trans, "{\"f\":%d,\"t\":%.3f,\"mark\":", g_fr.open ? g_fr.frame_no : -1,
                maintenant() - g_t_start);
        fjstr(g_trans, note);
        fprintf(g_trans, "}\n");
        fflush(g_trans);
    }
    push_event("MARK");
}

void runlog_request_event(const char *name) {
    if (runlog_items())
        push_event(name);
}

int runlog_take_event(char *name, size_t n) {
    if (!g_ev_n)
        return 0;
    snprintf(name, n, "%s", g_ev[g_ev_head]);
    g_ev_head = (g_ev_head + 1) % MAX_PENDING_EV;
    g_ev_n--;
    return 1;
}

int runlog_overlay(RunlogOverlay *o) {
    if (!g_overlay_valid)
        return 0;
    *o = g_overlay;
    return 1;
}

const char *runlog_state_name(void) {
    if (!g_fr.have_ai)
        return "";
    int id = g_fr.n_path ? g_fr.path[g_fr.n_path - 1] : g_fr.state;
    const char *s = state_name(id);
    return s ? s : "";
}

void runlog_image_stats(long snapshots, long events, long dropped, long long bytes) {
    g_img_snapshots = snapshots;
    g_img_events = events;
    g_img_dropped = dropped;
    g_img_bytes = bytes;
}
