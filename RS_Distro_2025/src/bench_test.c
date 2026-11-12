/***************************************************
 CSC C85 RoboSoccer - standalone bench-test harness

 Runs the raw motor primitives from notes/05-robot-and-bench-tests.md
 (tests 1-5) WITHOUT the camera -- roboSoccer proper needs a webcam just to
 start up, this doesn't. Uses the exact same BT_turn/BT_all_stop calls
 src/logic/drive.c uses, so whatever you measure here actually matches
 what the real control code will do (unlike, say, the old project's
 BT_move_sync, which behaves differently on repeated calls -- see
 .local/notes/EV3_NOTES.md).

 Always stops the robot on exit, including Ctrl+C, so killing this
 mid-test doesn't leave it running blind (notes/05, bench test 9).

 Build (from src/):
   gcc -O2 -g bench_test.c API/btcomm.c -lbluetooth -o bench_test
 Run:
   ./bench_test                              # prints usage
   ./bench_test AA:BB:CC:DD:EE:FF drive 30 3  # drive straight, power 30, 3s
   ./bench_test AA:BB:CC:DD:EE:FF turn 30 3   # spin in place, power 30, 3s
   ./bench_test AA:BB:CC:DD:EE:FF stop 1      # BT_all_stop(brake=1)
   ./bench_test AA:BB:CC:DD:EE:FF latency 100 # time 100 BT_drive calls (test 1)
***************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include "API/btcomm.h"

// Deliberately NOT #include "roboAI.h" -- it drags in the whole image
// capture chain (and SDL/SDL.h) just for these two defines, which defeats
// the point of this tool working without a camera. Keep these in sync
// with roboAI.h by hand if the real wiring ever changes.
#define LEFT_MOTOR  MOTOR_A
#define RIGHT_MOTOR MOTOR_D

void stop_and_exit(int sig) {
    (void)sig;
    fprintf(stderr, "\n[bench_test] stopping motors and exiting\n");
    BT_all_stop(1);
    BT_close();
    _exit(sig == 0 ? 0 : 1);
}

double now_seconds(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
}

void usage(const char *prog) {
    fprintf(stderr,
        "Usage:\n"
        "  %s <bt-mac> drive <power -100..100> <seconds>\n"
        "  %s <bt-mac> turn  <power 0..100>     <seconds>   (right wheel +power, left -power)\n"
        "  %s <bt-mac> stop  <brake 0|1>\n"
        "  %s <bt-mac> latency <N calls>\n"
        "\n"
        "Always stops the robot on exit (including Ctrl+C). Time the drive/turn\n"
        "commands yourself with a stopwatch + tape marks on the floor -- this\n"
        "just fires the motor command and holds it for the given duration; the\n"
        "'GO'/'STOP' lines below are your cue, same idea as bench tests 2-4 in\n"
        "notes/05-robot-and-bench-tests.md.\n",
        prog, prog, prog, prog);
}

int main(int argc, char **argv) {
    if (argc < 3) { usage(argv[0]); return 1; }

    signal(SIGINT, stop_and_exit);
    signal(SIGTERM, stop_and_exit);

    const char *mac = argv[1];
    const char *cmd = argv[2];

    // BT_open() returns 0 on SUCCESS, -1 on failure -- opposite of the usual
    // C convention, and exactly the kind of inconsistent return value
    // .local/notes/EV3_NOTES.md warns about across this whole library.
    // Checked API/btcomm.c's actual implementation to confirm, not guessed.
    if (BT_open(mac) != 0) {
        fprintf(stderr, "[bench_test] BT_open(%s) failed -- check the MAC and that the brick is on\n", mac);
        return 1;
    }
    fprintf(stderr, "[bench_test] connected to %s\n", mac);

    if (strcmp(cmd, "drive") == 0) {
        if (argc < 5) { usage(argv[0]); return 1; }
        int power = atoi(argv[3]);
        double seconds = atof(argv[4]);
        fprintf(stderr, "[bench_test] GO: driving straight, power=%d, for %.2fs\n", power, seconds);
        BT_turn(LEFT_MOTOR, (char)power, RIGHT_MOTOR, (char)power);
        usleep((useconds_t)(seconds * 1e6));
        BT_all_stop(1);
        fprintf(stderr, "[bench_test] STOP\n");

    } else if (strcmp(cmd, "turn") == 0) {
        if (argc < 5) { usage(argv[0]); return 1; }
        int power = atoi(argv[3]);
        double seconds = atof(argv[4]);
        fprintf(stderr, "[bench_test] GO: spinning in place, power=%d, for %.2fs "
                        "(right=+%d, left=-%d -- which way this actually turns is "
                        "bench test 5's question, not assumed here)\n",
                power, seconds, power, power);
        BT_turn(LEFT_MOTOR, (char)-power, RIGHT_MOTOR, (char)power);
        usleep((useconds_t)(seconds * 1e6));
        BT_all_stop(1);
        fprintf(stderr, "[bench_test] STOP\n");

    } else if (strcmp(cmd, "stop") == 0) {
        int brake = argc >= 4 ? atoi(argv[3]) : 1;
        fprintf(stderr, "[bench_test] BT_all_stop(%d)\n", brake);
        BT_all_stop(brake);

    } else if (strcmp(cmd, "latency") == 0) {
        int n = argc >= 4 ? atoi(argv[3]) : 100;
        fprintf(stderr, "[bench_test] timing %d BT_drive calls (bench test 1)...\n", n);
        double t0 = now_seconds();
        for (int i = 0; i < n; i++) BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0); // power 0: measuring round-trip, not moving
        double t1 = now_seconds();
        BT_all_stop(1);
        double total_ms = (t1 - t0) * 1000.0;
        fprintf(stderr, "[bench_test] %d calls in %.1f ms -> %.2f ms/call\n", n, total_ms, total_ms / n);

    } else {
        usage(argv[0]);
        BT_close();
        return 1;
    }

    BT_all_stop(1);
    BT_close();
    return 0;
}
