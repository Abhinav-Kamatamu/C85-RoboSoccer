#include "drive.h"
#include "../roboAI.h"
#include "ai_config.h"
#include "skills.h"
#include "clock.h"
#include <math.h>
#include <pthread.h>
#include <stdlib.h>

// Kinematics, v (forward) and w (rotation) are converted to a motor
// command separately, each through its own measured gain, then added.
// Straight-line driving and in-place spinning were measured as having
// somewhat different gains (turning saturates sooner than driving),
// so one combined gain applied to an already-summed wheel speed would get
// both "pure" cases slightly wrong in opposite directions. Converting v and
// w through their own gains first and adding the results gets both "pure"
// cases right and treats combined v+w motion as a linear interpolation between
// them which should be better than reusing one gain for everything.

#define RESEND_S 0.5
static double g_last_l = -1000, g_last_r = -1000;
// Use the open-loop path unless the ROBO_REGULATED
// environment variable is set to 1
static int g_use_regulated_speed = -1;

void drive_set_regulated_speed(int enabled) {
    g_use_regulated_speed = enabled;
}

int drive_get_regulated_speed(void) {
    if (g_use_regulated_speed < 0) {
        const char *e = getenv("ROBO_REGULATED");
        g_use_regulated_speed = (e && e[0] == '1') ? 1 : 0;
    }
    return g_use_regulated_speed;
}

// Omg, is this... B09?????
typedef struct {
    pthread_mutex_t lock;
    pthread_cond_t cond;
    int have_cmd;     // 1 once a command is waiting to be sent
    int is_stop;       // 0 = turn, 1 = all_stop
    char lpower, rpower;
    int brake;
    int use_regulated; // which BT_* call to issue for THIS command
} BTCommand;

static BTCommand g_cmd = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .cond = PTHREAD_COND_INITIALIZER,
    .have_cmd = 0
};
static pthread_t g_bt_thread;
static int g_bt_thread_started = 0;

// ** BT delivery diagnostics (printed by the 'f' output in imageCapture.c) ***
// The command slot above holds one command and the newest overwrites an unsent
// one, so a short pulse (spin, then stop 2 frames later) relies on the speed of
// BT round trip. If the thread is mid-call the spin is overwritten by the stop
// and never reaches the bot.
static unsigned g_bt_calls = 0, g_bt_overwritten = 0;
static double g_bt_ms_sum = 0, g_bt_ms_max = 0;

void drive_get_bt_stats(unsigned *calls, unsigned *overwritten, double *avg_ms, double *max_ms) {
    pthread_mutex_lock(&g_cmd.lock);
    *calls = g_bt_calls;
    *overwritten = g_bt_overwritten;
    *avg_ms = g_bt_calls ? g_bt_ms_sum / g_bt_calls : 0;
    *max_ms = g_bt_ms_max;
    g_bt_ms_sum = 0;
    g_bt_ms_max = 0;
    g_bt_calls = 0;
    g_bt_overwritten = 0; // reset per readout
    pthread_mutex_unlock(&g_cmd.lock);
}

// You know what would be really nice here? The never type `!` from Rust.
// It just got stablized in 1.100.0
// Lowk can we RIIR this entire project lol
void *bt_thread_fn(void *arg) {
    (void)arg;
    while (1) {
        pthread_mutex_lock(&g_cmd.lock);
        while (!g_cmd.have_cmd)
            pthread_cond_wait(&g_cmd.cond, &g_cmd.lock);
        int is_stop = g_cmd.is_stop;
        char lp = g_cmd.lpower, rp = g_cmd.rpower;
        int brake = g_cmd.brake;
        int use_regulated = g_cmd.use_regulated;
        g_cmd.have_cmd = 0;
        pthread_mutex_unlock(&g_cmd.lock);

        // The actual blocking BT_* call happens with the lock released, so
        // drive_set()/drive_stop() aren't stuck waiting on this.
        double bt_t0 = maintenant() * 1e3;
        if (is_stop)
            BT_all_stop(brake);
        else if (use_regulated)
            BT_turn_speed(LEFT_MOTOR, -lp, RIGHT_MOTOR, -rp);
        else
            BT_turn(LEFT_MOTOR, -lp, RIGHT_MOTOR, -rp);
        double bt_dt = maintenant() * 1e3 - bt_t0;
        pthread_mutex_lock(&g_cmd.lock);
        g_bt_calls++;
        g_bt_ms_sum += bt_dt;
        if (bt_dt > g_bt_ms_max) g_bt_ms_max = bt_dt;
        pthread_mutex_unlock(&g_cmd.lock);
    }
    return NULL;
}

void ensure_bt_thread_started(void) {
    if (g_bt_thread_started)
        return;
    g_bt_thread_started = 1;
    pthread_create(&g_bt_thread, NULL, bt_thread_fn, NULL);
}

// Makes sure power isn't too high/low, else sets it to the min/max power we allow
double apply_min_power(double power) {
    if (power == 0)
        return 0;
    if (fabs(power) < MIN_MOVE_POWER)
        return power > 0 ? MIN_MOVE_POWER : -MIN_MOVE_POWER;
    return power;
}

// Caps how much power can change from last frame to this one, so commands
// ramp smoothly instead of jumping (which makes the wheels skid/jolt).
double limit_power_step(double target, double last) {
    // this is the first call, so there's nothing to limit against
    if (last < -999)
        return target;
    // We make sure we don't speed up or slow down too fast or else,
    // very bad things might happen
    return clampd(target, last - MAX_POWER_STEP_PER_FRAME, last + MAX_POWER_STEP_PER_FRAME);
}

void drive_shape_power(Motion m, int use_regulated, double *pl_out, double *pr_out) {
    static double pulse_acc = 0;
    double translate, rotate;
    if (use_regulated) {
        translate = m.v / SPEED_API_TO_CMPS_GAIN;
        rotate = (m.w * 180.0 / M_PI) / SPEED_API_TO_DEGPS_GAIN;
    } else {
        translate = m.v / POWER_TO_CMPS;
        rotate = m.w / POWER_TO_RADPS;
        // Static friction compensation for in-place spins (open loop only. The
        // regulated mode closes the loop on the encoders itself).
        if (MIN_SPIN_POWER > 0 && fabs(translate) < SPIN_TRANSLATE_MAX_POWER &&
            fabs(rotate) > 0.5 && fabs(rotate) < MIN_SPIN_POWER) {
            pulse_acc += fabs(rotate) / MIN_SPIN_POWER;
            if (pulse_acc >= 1.0) {
                pulse_acc -= 1.0;
                rotate = (rotate > 0) ? MIN_SPIN_POWER : -MIN_SPIN_POWER;
            } else {
                rotate = 0;
            }
        } else {
            pulse_acc = 0;
        }
    }
    *pl_out = translate + rotate;
    *pr_out = translate - rotate;
}

void drive_set(Motion m, int use_regulated) {
    // See drive_shape_power() in drive.h for the v/w -> power mapping, the
    // w sign convention (w>0 = angle_of(heading) increases = clockwise on
    // screen = physical RIGHT turn. The old mapping turned every command
    // AWAY from its target) and the in-place-spin static friction compensation.
    double pl, pr;
    drive_shape_power(m, use_regulated, &pl, &pr);
    pl += MOTOR_TRIM_L;
    pr += MOTOR_TRIM_R;

    pl = apply_min_power(pl);
    pr = apply_min_power(pr);

    // The per-frame power ramp would split a static friction pulse (0 -> MIN_SPIN_POWER
    // in one frame) into a weak n-frame ramp that never breaks static
    // friction, so in-place spins skip it.
    if (fabs((pl + pr) / 2.0) >= SPIN_TRANSLATE_MAX_POWER) {
        pl = limit_power_step(pl, g_last_l);
        pr = limit_power_step(pr, g_last_r);
    }

    // Clamp pl/pr as a PAIR, not independently. pl=translate+rotate,
    // pr=translate-rotate, so whenever a turn is big enough that one of
    // them would exceed MAX_POWER, independent clamping only chops that
    // one wheel - which (since a big rotate pushes one wheel positive and
    // the other negative) silently eats the translate contribution out of
    // the clipped wheel while leaving the other wheel's rotate-heavy
    // value untouched, distorting the actual v/w ratio toward pure
    // rotation. Scaling both wheels by the same factor instead preserves
    // the commanded ratio - this is "why forward is slow, rotation is
    // fast": simultaneous turn+drive commands were quietly losing most of
    // their forward component whenever the turn alone was already large.
    double peak = fmax(fabs(pl), fabs(pr));
    if (peak > MAX_POWER) {
        double scale = MAX_POWER / peak;
        pl *= scale;
        pr *= scale;
    }

    // Dedupe (don't call if values are similar to last time), but re-send
    // the identical command every RESEND_S anyway. A dropped/failed
    // BT write would otherwise never be retried, since the dedupe thinks
    // the bot already has it.
    static double last_send_t = -1;
    double now_t = maintenant();
    if (fabs(pl - g_last_l) < 1.0 && fabs(pr - g_last_r) < 1.0 &&
        now_t - last_send_t < RESEND_S)
        return;
    last_send_t = now_t;

    g_last_l = pl;
    g_last_r = pr;

    ensure_bt_thread_started();
    pthread_mutex_lock(&g_cmd.lock);
    if (g_cmd.have_cmd) g_bt_overwritten++;
    g_cmd.is_stop = 0;
    g_cmd.lpower = (char) lround(pl);
    g_cmd.rpower = (char) lround(pr);
    g_cmd.use_regulated = use_regulated;
    g_cmd.have_cmd = 1;
    pthread_cond_signal(&g_cmd.cond);
    pthread_mutex_unlock(&g_cmd.lock);
}

void drive_get_last_power(double *left, double *right) {
    *left = g_last_l < -999 ? 0 : g_last_l;
    *right = g_last_r < -999 ? 0 : g_last_r;
}

void drive_stop(int brake) {
    g_last_l = 0;
    g_last_r = 0;

    ensure_bt_thread_started();
    pthread_mutex_lock(&g_cmd.lock);
    if (g_cmd.have_cmd) g_bt_overwritten++;
    g_cmd.is_stop = 1;
    g_cmd.brake = brake;
    g_cmd.have_cmd = 1;
    pthread_cond_signal(&g_cmd.cond);
    pthread_mutex_unlock(&g_cmd.lock);
}
