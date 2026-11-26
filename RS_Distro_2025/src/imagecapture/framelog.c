/***************************************************
 CSC C85 RoboSoccer frame recorder, see framelog.h

 Main thread: copy pixels into a ring buffer (+ a full-size copy every
 0.2 s for snapshots) and hand finished copies to the writer thread.
 Writer thread: PNG-encode with zlib and write. If the writer falls behind
 the queue is capped and new images are DROPPED (counted in the manifest)
 rather than ever making the camera loop wait.
***************************************************/

#include "framelog.h"
#include "imageCapture.h"
#include "../logic/runlog.h"
#include "../logic/ai_config.h"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <zlib.h>

#define DISP_W 1024
#define DISP_H 768
#define EV_W   512                 // event frames are half size (display / 2)
#define EV_H   384
#define SNAP_PERIOD_S   0.2        // 5 fps display snapshots
#define RAW_PERIOD_S    1.0        // 1 fps raw camera snapshots
#define FIELD_PERIOD_S  (1.0 / 15) // field frames in the event ring
#define EV_PRE_S        2.0
#define EV_POST_S       1.0
#define DISP_RING       120        // > 3 s at 30 fps
#define FIELD_RING      50         // > 3 s at 15 fps
#define MAX_QUEUE_BYTES (512LL * 1024 * 1024)
#define MAX_RUN_IMAGE_BYTES (3LL * 1024 * 1024 * 1024)
#define MAX_EVENTS      200
#define PNG_LEVEL       1          // fast; blob images are mostly flat colour anyway

// ==================================================================== PNG ==

static void put32(unsigned char *p, unsigned v) {
    p[0] = (unsigned char)(v >> 24);
    p[1] = (unsigned char)(v >> 16);
    p[2] = (unsigned char)(v >> 8);
    p[3] = (unsigned char)v;
}

static void png_chunk(FILE *f, const char *type, const unsigned char *data, unsigned len) {
    unsigned char hdr[8];
    put32(hdr, len);
    memcpy(hdr + 4, type, 4);
    fwrite(hdr, 1, 8, f);
    if (len)
        fwrite(data, 1, len, f);
    uLong crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, (const Bytef *)type, 4);
    if (len)
        crc = crc32(crc, data, len);
    unsigned char c[4];
    put32(c, (unsigned)crc);
    fwrite(c, 1, 4, f);
}

// 8-bit RGB PNG, "Sub" filter on every row. Returns bytes written, 0 on error.
static long png_write(const char *path, int w, int h, const unsigned char *px, const char *text) {
    size_t row = (size_t)w * 3;
    size_t raw_len = (row + 1) * (size_t)h;
    unsigned char *raw = (unsigned char *)malloc(raw_len);
    if (!raw)
        return 0;
    for (int y = 0; y < h; y++) {
        unsigned char *dst = raw + y * (row + 1);
        const unsigned char *src = px + y * row;
        dst[0] = 1; // Sub
        for (size_t i = 0; i < row; i++)
            dst[1 + i] = (unsigned char)(src[i] - (i >= 3 ? src[i - 3] : 0));
    }
    uLongf zlen = compressBound(raw_len);
    unsigned char *z = (unsigned char *)malloc(zlen);
    if (!z || compress2(z, &zlen, raw, raw_len, PNG_LEVEL) != Z_OK) {
        free(raw);
        free(z);
        return 0;
    }
    free(raw);

    FILE *f = fopen(path, "wb");
    if (!f) {
        free(z);
        return 0;
    }
    static const unsigned char sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    fwrite(sig, 1, 8, f);
    unsigned char ihdr[13];
    put32(ihdr, (unsigned)w);
    put32(ihdr + 4, (unsigned)h);
    ihdr[8] = 8;  // bit depth
    ihdr[9] = 2;  // RGB
    ihdr[10] = 0;
    ihdr[11] = 0;
    ihdr[12] = 0;
    png_chunk(f, "IHDR", ihdr, 13);
    if (text && text[0]) {
        // tEXt: keyword \0 text. Lets a viewer see frame/state without the log.
        char t[300];
        int n = snprintf(t, sizeof(t), "robosoccer%c%s", 0, text);
        if (n > (int)sizeof(t))
            n = sizeof(t);
        png_chunk(f, "tEXt", (const unsigned char *)t, (unsigned)n);
    }
    png_chunk(f, "IDAT", z, (unsigned)zlen);
    png_chunk(f, "IEND", NULL, 0);
    long bytes = ftell(f);
    fclose(f);
    free(z);
    return bytes;
}

static int mkdir_p(const char *path) {
    char tmp[1400];
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

// ========================================================= writer thread ==

typedef struct Job {
    char path[1400];
    char text[200];
    int w, h;
    unsigned char *px;
    long long bytes;
    int kind;              // 0 snapshot, 1 event frame
    struct Job *next;
} Job;

static pthread_mutex_t g_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_cv = PTHREAD_COND_INITIALIZER;
static Job *g_q_head = NULL, *g_q_tail = NULL;
static long long g_q_bytes = 0;
static int g_quit = 0;
static pthread_t g_thread;
static int g_thread_up = 0;
// stats (under g_mx)
static long g_n_snap = 0, g_n_dropped = 0;
static long long g_written = 0;
static int g_over_budget = 0;

static void *writer_main(void *arg) {
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&g_mx);
        while (!g_q_head && !g_quit)
            pthread_cond_wait(&g_cv, &g_mx);
        Job *j = g_q_head;
        if (!j) {
            pthread_mutex_unlock(&g_mx);
            break; // quit and drained
        }
        g_q_head = j->next;
        if (!g_q_head)
            g_q_tail = NULL;
        g_q_bytes -= j->bytes;
        pthread_mutex_unlock(&g_mx);

        long n = png_write(j->path, j->w, j->h, j->px, j->text);
        pthread_mutex_lock(&g_mx);
        if (n > 0) {
            g_written += n;
            if (j->kind == 0)
                g_n_snap++;
        } else {
            g_n_dropped++;
        }
        pthread_mutex_unlock(&g_mx);
        free(j->px);
        free(j);
    }
    return NULL;
}

// Takes ownership of px. Returns 1 if queued, 0 if dropped.
static int enqueue(const char *path, int w, int h, unsigned char *px, const char *text, int kind) {
    long long bytes = (long long)w * h * 3;
    pthread_mutex_lock(&g_mx);
    if (g_over_budget || g_written > MAX_RUN_IMAGE_BYTES) {
        if (!g_over_budget)
            fprintf(stderr, "framelog: this run hit %.1f GB of images, no more images will be saved\n",
                    MAX_RUN_IMAGE_BYTES / 1e9);
        g_over_budget = 1;
        g_n_dropped++;
        pthread_mutex_unlock(&g_mx);
        free(px);
        return 0;
    }
    if (g_q_bytes + bytes > MAX_QUEUE_BYTES) {
        g_n_dropped++;
        pthread_mutex_unlock(&g_mx);
        free(px);
        return 0;
    }
    Job *j = (Job *)calloc(1, sizeof(Job));
    if (!j) {
        g_n_dropped++;
        pthread_mutex_unlock(&g_mx);
        free(px);
        return 0;
    }
    snprintf(j->path, sizeof(j->path), "%s", path);
    snprintf(j->text, sizeof(j->text), "%s", text ? text : "");
    j->w = w;
    j->h = h;
    j->px = px;
    j->bytes = bytes;
    j->kind = kind;
    if (g_q_tail)
        g_q_tail->next = j;
    else
        g_q_head = j;
    g_q_tail = j;
    g_q_bytes += bytes;
    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mx);
    return 1;
}

// ============================================================ ring + state ==

typedef struct {
    int frame_no;    // -1 = empty
    double t;
    char state[40];
    int kind;
    unsigned char *px;
} Slot;

static int g_items = 0;
static int g_stopped = 0;
static char g_dir[1200];
static int g_fw = 0, g_fh = 0;
static Slot g_disp[DISP_RING];
static Slot g_field[FIELD_RING];
static int g_disp_head = 0, g_field_head = 0;
static double g_last_snap = -1e9, g_last_raw = -1e9, g_last_field = -1e9;
static double g_last_t = 0;
static int g_last_frame = 0;
static long g_n_events = 0;
static char g_prev_rel[200] = "";

// one event waiting for its post-trigger second to pass
static struct {
    int active;
    int frame_no;
    double t;
    char names[128];
    char rel[200];   // events/E003_f000456_P_UNSTICK
} g_pend;

static const char *kind_name(int k) {
    return k == 2 ? "blobs" : k == 1 ? "field" : "camera";
}

static void sanitize(char *s) {
    for (; *s; s++)
        if (!((*s >= 'A' && *s <= 'Z') || (*s >= 'a' && *s <= 'z') ||
              (*s >= '0' && *s <= '9') || *s == '_' || *s == '+' || *s == '-'))
            *s = '_';
}

void framelog_start(int items, const char *run_dir, int field_w, int field_h) {
    if (g_items || !run_dir || !(items & (RUNLOG_DISPLAY | RUNLOG_RAW)))
        return;
    snprintf(g_dir, sizeof(g_dir), "%s", run_dir);
    g_fw = field_w;
    g_fh = field_h;
    for (int i = 0; i < DISP_RING; i++) {
        g_disp[i].frame_no = -1;
        g_disp[i].px = (items & RUNLOG_DISPLAY) ? (unsigned char *)malloc(EV_W * EV_H * 3) : NULL;
    }
    for (int i = 0; i < FIELD_RING; i++) {
        g_field[i].frame_no = -1;
        g_field[i].px = (items & RUNLOG_RAW) ? (unsigned char *)malloc((size_t)g_fw * g_fh * 3) : NULL;
    }
    char p[1400];
    if (items & RUNLOG_DISPLAY) {
        snprintf(p, sizeof(p), "%s/frames", g_dir);
        mkdir_p(p);
    }
    if (items & RUNLOG_RAW) {
        snprintf(p, sizeof(p), "%s/raw", g_dir);
        mkdir_p(p);
    }
    if (pthread_create(&g_thread, NULL, writer_main, NULL) != 0) {
        fprintf(stderr, "framelog: could not start writer thread, image logging OFF\n");
        return;
    }
    g_thread_up = 1;
    g_items = items;
    atexit(framelog_stop);
}

// Write event.json and queue every ring frame inside the event's window.
static void dump_event(void) {
    if (!g_pend.active)
        return;
    g_pend.active = 0;
    char dir[1400], p[1500];
    snprintf(dir, sizeof(dir), "%s/%s", g_dir, g_pend.rel);
    if (g_items & RUNLOG_DISPLAY) {
        snprintf(p, sizeof(p), "%s/display", dir);
        mkdir_p(p);
    }
    if (g_items & RUNLOG_RAW) {
        snprintf(p, sizeof(p), "%s/field", dir);
        mkdir_p(p);
    }
    double t0 = g_pend.t - EV_PRE_S, t1 = g_pend.t + EV_POST_S;

    snprintf(p, sizeof(p), "%s/event.json", dir);
    FILE *f = fopen(p, "w");
    if (f) {
        fprintf(f, "{\n  \"event\": \"%s\",\n  \"trigger_frame\": %d,\n  \"window_s\": [%.1f, %.1f],\n",
                g_pend.names, g_pend.frame_no, -EV_PRE_S, EV_POST_S);
        fprintf(f, "  \"note\": \"display/ = what the window showed (blobs + intent overlay), half size. "
                   "field/ = unwarped field image that blob detection ran on (only with logItems bit2). "
                   "Match frame numbers with frames.jsonl. A frame is saved in one event only, so if this "
                   "event follows closely on previous_event, its first frames are in that folder.\",\n");
        fprintf(f, "  \"previous_event\": \"%s\",\n  \"frames\": [\n", g_prev_rel);
    }
    int first = 1;
    // oldest -> newest
    for (int k = 0; k < DISP_RING; k++) {
        Slot *s = &g_disp[(g_disp_head + k) % DISP_RING];
        if (s->frame_no < 0 || s->t < t0 || s->t > t1 || !s->px)
            continue;
        // Hand the buffer itself to the writer (no copy: copying ~3 s of
        // frames at once stalled the loop for tens of ms). The ring
        // re-allocates the slot the next time it comes round.
        unsigned char *copy = s->px;
        int fno = s->frame_no;
        s->px = NULL;
        s->frame_no = -1;   // each frame goes to one event only
        char path[1500], text[200];
        snprintf(path, sizeof(path), "%s/display/f%06d.png", dir, fno);
        snprintf(text, sizeof(text), "frame=%d t_rel_event=%.3f state=%s kind=%s",
                 fno, s->t - g_pend.t, s->state, kind_name(s->kind));
        enqueue(path, EV_W, EV_H, copy, text, 1);
        if (f) {
            fprintf(f, "%s    {\"f\": %d, \"t_rel_event\": %.3f, \"state\": \"%s\", \"display\": \"display/f%06d.png\"}",
                    first ? "" : ",\n", fno, s->t - g_pend.t, s->state, fno);
            first = 0;
        }
    }
    for (int k = 0; k < FIELD_RING; k++) {
        Slot *s = &g_field[(g_field_head + k) % FIELD_RING];
        if (s->frame_no < 0 || s->t < t0 || s->t > t1 || !s->px)
            continue;
        unsigned char *copy = s->px;   // handed over, see above
        int fno = s->frame_no;
        s->px = NULL;
        s->frame_no = -1;
        char path[1500], text[200];
        snprintf(path, sizeof(path), "%s/field/f%06d.png", dir, fno);
        snprintf(text, sizeof(text), "frame=%d t_rel_event=%.3f state=%s kind=field",
                 fno, s->t - g_pend.t, s->state);
        enqueue(path, g_fw, g_fh, copy, text, 1);
        if (f) {
            fprintf(f, "%s    {\"f\": %d, \"t_rel_event\": %.3f, \"state\": \"%s\", \"field\": \"field/f%06d.png\"}",
                    first ? "" : ",\n", fno, s->t - g_pend.t, s->state, fno);
            first = 0;
        }
    }
    if (f) {
        fprintf(f, "\n  ]\n}\n");
        fclose(f);
    }
    snprintf(g_prev_rel, sizeof(g_prev_rel), "%s", g_pend.rel);
}

static void start_event(const char *name, int frame_no, double t) {
    if (g_pend.active) {
        // Inside the current event's window: same incident, just note it
        if (strlen(g_pend.names) + strlen(name) + 2 < sizeof(g_pend.names)) {
            strcat(g_pend.names, "+");
            strcat(g_pend.names, name);
        }
        return;
    }
    if (g_n_events >= MAX_EVENTS) {
        if (g_n_events == MAX_EVENTS)
            fprintf(stderr, "framelog: %d events this run, not dumping any more\n", MAX_EVENTS);
        g_n_events++;
        return;
    }
    g_n_events++;
    g_pend.active = 1;
    g_pend.frame_no = frame_no;
    g_pend.t = t;
    snprintf(g_pend.names, sizeof(g_pend.names), "%s", name);
    char clean[64];
    snprintf(clean, sizeof(clean), "%s", name);
    sanitize(clean);
    snprintf(g_pend.rel, sizeof(g_pend.rel), "events/E%03ld_f%06d_%s", g_n_events, frame_no, clean);
    runlog_frame_image("event", g_pend.rel);
}

void framelog_frame(int frame_no, double t, const unsigned char *display, int kind,
                    const unsigned char *field, const unsigned char *raw) {
    if (!g_items || g_stopped)
        return;
    g_last_t = t;
    g_last_frame = frame_no;
    const char *state = runlog_state_name();

    if ((g_items & RUNLOG_DISPLAY) && display) {
        // ring: half size, nearest neighbour
        Slot *s = &g_disp[g_disp_head];
        g_disp_head = (g_disp_head + 1) % DISP_RING;
        if (!s->px)
            s->px = (unsigned char *)malloc(EV_W * EV_H * 3);
        if (!s->px)
            return;
        s->frame_no = frame_no;
        s->t = t;
        s->kind = kind;
        snprintf(s->state, sizeof(s->state), "%s", state);
        for (int y = 0; y < EV_H; y++) {
            const unsigned char *src = display + (size_t)(2 * y) * DISP_W * 3;
            unsigned char *dst = s->px + (size_t)y * EV_W * 3;
            for (int x = 0; x < EV_W; x++) {
                dst[x * 3 + 0] = src[x * 6 + 0];
                dst[x * 3 + 1] = src[x * 6 + 1];
                dst[x * 3 + 2] = src[x * 6 + 2];
            }
        }
        // 5 fps full-size snapshot
        if (t - g_last_snap >= SNAP_PERIOD_S - 0.005) {
            g_last_snap = t;
            unsigned char *copy = (unsigned char *)malloc(DISP_W * DISP_H * 3);
            if (copy) {
                memcpy(copy, display, DISP_W * DISP_H * 3);
                char path[1400], rel[64], text[200];
                snprintf(rel, sizeof(rel), "frames/f%06d.png", frame_no);
                snprintf(path, sizeof(path), "%s/%s", g_dir, rel);
                snprintf(text, sizeof(text), "frame=%d state=%s kind=%s", frame_no, state, kind_name(kind));
                if (enqueue(path, DISP_W, DISP_H, copy, text, 0))
                    runlog_frame_image("snapshot", rel);
            }
        }
    }

    if (g_items & RUNLOG_RAW) {
        if (field && t - g_last_field >= FIELD_PERIOD_S - 0.005) {
            g_last_field = t;
            Slot *s = &g_field[g_field_head];
            g_field_head = (g_field_head + 1) % FIELD_RING;
            if (!s->px)
                s->px = (unsigned char *)malloc((size_t)g_fw * g_fh * 3);
            if (!s->px)
                return;
            s->frame_no = frame_no;
            s->t = t;
            s->kind = 1;
            snprintf(s->state, sizeof(s->state), "%s", state);
            memcpy(s->px, field, (size_t)g_fw * g_fh * 3);
        }
        if (raw && t - g_last_raw >= RAW_PERIOD_S - 0.005) {
            g_last_raw = t;
            size_t n = (size_t)g_fw * g_fh * 3;
            unsigned char *copy = (unsigned char *)malloc(n);
            if (copy) {
                memcpy(copy, raw, n);
                char path[1400], rel[64], text[200];
                snprintf(rel, sizeof(rel), "raw/f%06d.png", frame_no);
                snprintf(path, sizeof(path), "%s/%s", g_dir, rel);
                snprintf(text, sizeof(text), "frame=%d state=%s kind=camera", frame_no, state);
                if (enqueue(path, g_fw, g_fh, copy, text, 0))
                    runlog_frame_image("raw", rel);
            }
        }
    }

    char name[64];
    while (runlog_take_event(name, sizeof(name)))
        start_event(name, frame_no, t);
    if (g_pend.active && t >= g_pend.t + EV_POST_S)
        dump_event();
}

void framelog_log_blobs(struct blob *list) {
    if (!runlog_items())
        return;
    for (struct blob *b = list; b; b = b->next)
        runlog_frame_blob(b->blobId, b->idtype, b->cx, b->cy, b->size,
                          b->x1, b->y1, b->x2, b->y2, b->R, b->G, b->B,
                          b->H, b->S, b->V);
}

// ============================================================== overlay ==

static void put_px(struct image *im, int x, int y, double R, double G, double B) {
    if (x < 0 || y < 0 || x >= im->sx || y >= im->sy)
        return;
    *(im->layers[0] + x + y * im->sx) = R;
    *(im->layers[1] + x + y * im->sx) = G;
    *(im->layers[2] + x + y * im->sx) = B;
}

static void line_px(struct image *im, double x0, double y0, double x1, double y1,
                    double R, double G, double B, int dashed) {
    double len = hypot(x1 - x0, y1 - y0);
    int n = (int)len + 1;
    for (int i = 0; i <= n; i++) {
        if (dashed && (i / 6) % 2)
            continue;
        double a = (double)i / n;
        int x = (int)(x0 + (x1 - x0) * a), y = (int)(y0 + (y1 - y0) * a);
        put_px(im, x, y, R, G, B);
        put_px(im, x + 1, y, R, G, B);
        put_px(im, x, y + 1, R, G, B);
    }
}

static void box_px(struct image *im, double cx, double cy, int r, double R, double G, double B) {
    line_px(im, cx - r, cy - r, cx + r, cy - r, R, G, B, 0);
    line_px(im, cx - r, cy + r, cx + r, cy + r, R, G, B, 0);
    line_px(im, cx - r, cy - r, cx - r, cy + r, R, G, B, 0);
    line_px(im, cx + r, cy - r, cx + r, cy + r, R, G, B, 0);
}

void framelog_overlay(struct image *blob_im) {
    if (!(g_items & RUNLOG_DISPLAY) || !blob_im)
        return;
    RunlogOverlay o;
    if (!runlog_overlay(&o))
        return;
    // World is cm, the blob image is (unwarped) pixels
    double sx = o.self_pos.x * PX_PER_CM_X, sy = o.self_pos.y * PX_PER_CM_Y;
    double bx = o.ball_pos.x * PX_PER_CM_X, by = o.ball_pos.y * PX_PER_CM_Y;
    if (o.have_plan) {
        if (o.staging.x != 0 || o.staging.y != 0) {
            double gx = o.staging.x * PX_PER_CM_X, gy = o.staging.y * PX_PER_CM_Y;
            box_px(blob_im, gx, gy, 8, 0, 255, 255);                 // cyan: staging point
            if (o.self_valid)
                line_px(blob_im, sx, sy, gx, gy, 0, 255, 255, 1);   // dashed: path to it
        }
        if ((o.aim.x != 0 || o.aim.y != 0) && o.ball_valid) {
            double ax = o.aim.x * PX_PER_CM_X, ay = o.aim.y * PX_PER_CM_Y;
            double d = hypot(ax - bx, ay - by);
            if (d > 1) {
                double L = fmin(d, 300.0);
                line_px(blob_im, bx, by, bx + (ax - bx) / d * L, by + (ay - by) / d * L,
                        0, 255, 0, 0);                                // green: aim line
            }
        }
    }
    if (o.self_valid && (o.self_heading.x != 0 || o.self_heading.y != 0)) {
        double hx = o.self_heading.x * PX_PER_CM_X, hy = o.self_heading.y * PX_PER_CM_Y;
        double hl = hypot(hx, hy);
        if (hl > 1e-6) {
            double ex = sx + hx / hl * 70, ey = sy + hy / hl * 70;
            line_px(blob_im, sx, sy, ex, ey, 255, 255, 255, 0);       // white: heading
            box_px(blob_im, ex, ey, 3, 255, 255, 255);
        }
    }
}

void framelog_stop(void) {
    if (!g_items || g_stopped)
        return;
    // last seconds before quitting are usually the interesting ones
    if (!g_pend.active)
        start_event("QUIT", g_last_frame, g_last_t);
    dump_event();
    g_stopped = 1;

    pthread_mutex_lock(&g_mx);
    long long queued = g_q_bytes;
    g_quit = 1;
    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mx);
    if (queued > 0)
        fprintf(stderr, "framelog: saving the last %.0f MB of images, give it a moment...\n", queued / 1e6);
    if (g_thread_up)
        pthread_join(g_thread, NULL);
    g_thread_up = 0;

    pthread_mutex_lock(&g_mx);
    long events = g_n_events < MAX_EVENTS ? g_n_events : MAX_EVENTS;
    runlog_image_stats(g_n_snap, events, g_n_dropped, g_written);
    if (g_n_dropped)
        fprintf(stderr, "framelog: %ld images dropped (writer could not keep up or budget hit)\n", g_n_dropped);
    pthread_mutex_unlock(&g_mx);

    for (int i = 0; i < DISP_RING; i++) {
        free(g_disp[i].px);
        g_disp[i].px = NULL;
    }
    for (int i = 0; i < FIELD_RING; i++) {
        free(g_field[i].px);
        g_field[i].px = NULL;
    }
}
