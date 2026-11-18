/***************************************************
 CSC C85 RoboSoccer; bench-test

 Build (from src/):
   gcc -O3 -g bench_test.c API/btcomm.c -lbluetooth -o bench_test
 Run:
   ./bench_test                                   # prints usage
   ./bench_test AA:BB:CC:DD:EE:FF drive 30 3       # drive straight, power 30, 3s
   ./bench_test AA:BB:CC:DD:EE:FF turn 30 3        # spin in place, power 30, 3s
   ./bench_test AA:BB:CC:DD:EE:FF drivespeed 30 3  # same as drive, but regulated speed
   ./bench_test AA:BB:CC:DD:EE:FF turnspeed 30 3   # same as turn, but regulated speed
   ./bench_test AA:BB:CC:DD:EE:FF stop 1           # BT_all_stop(brake=1)
   ./bench_test AA:BB:CC:DD:EE:FF latency 100      # time 100 BT_drive calls (test 1)
***************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>
#include <math.h>
#include "API/btcomm.h"
#include "logic/ai_config.h"
#include "clock.h"

#define LEFT_MOTOR  MOTOR_A
#define RIGHT_MOTOR MOTOR_D

static struct termios g_orig_termios;
static int g_termios_saved = 0;

void stop_and_exit(int sig) {
    (void)sig;
    fprintf(stderr, "\n[bench_test] stopping motors and exiting\n");
    if (g_termios_saved) tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
    BT_all_stop(1);
    BT_close();
    _exit(sig == 0 ? 0 : 1);
}

void wait_for_key(char key) {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    g_termios_saved = 1;
    newt = g_orig_termios;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int c;
    do {
        c = getchar();
    } while (c != (int)key && c != EOF);

    tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
    g_termios_saved = 0;
}

void usage(const char *prog) {
    fprintf(stderr,
        "Usage:\n"
        "  %s <bt-mac> drive      <power -100..100> <seconds> [brake 0|1, default 1]\n"
        "  %s <bt-mac> turn       <power 0..100>     <seconds> [brake 0|1, default 1]   (right wheel +power, left -power)\n"
        "  %s <bt-mac> drivespeed <speed -100..100>  <seconds> [brake 0|1, default 1]   (same as drive, regulated speed)\n"
        "  %s <bt-mac> turnspeed  <speed 0..100>     <seconds> [brake 0|1, default 1]   (same as turn, regulated speed)\n"
        "  %s <bt-mac> drive_until      <power -100..100> [brake 0|1, default 1]   (drives until you press 'd', prints elapsed time)\n"
        "  %s <bt-mac> drivespeed_until <speed -100..100>  [brake 0|1, default 1]   (same, regulated speed)\n"
        "  %s <bt-mac> stop  <brake 0|1>\n"
        "  %s <bt-mac> battery\n"
        "  %s <bt-mac> latency <N calls>\n"
        "\n",
        prog, prog, prog, prog, prog, prog, prog, prog, prog);
}

int main(int argc, char **argv) {
    if (argc < 3) { usage(argv[0]); return 1; }

    signal(SIGINT, stop_and_exit);
    signal(SIGTERM, stop_and_exit);

    const char *mac = argv[1];
    const char *cmd = argv[2];

    if (BT_open(mac) != 0) {
        fprintf(stderr, "[bench_test] BT_open(%s) failed. Check the MAC and that the brick is on\n", mac);
        return 1;
    }
    fprintf(stderr, "[bench_test] connected to %s\n", mac);

    if (strcmp(cmd, "drive") == 0) {
        if (argc < 5) { usage(argv[0]); return 1; }
        int power = atoi(argv[3]);
        double seconds = atof(argv[4]);
        int brake = argc >= 6 ? atoi(argv[5]) : 1;
        char lp = (char)lround(power + MOTOR_TRIM_L);
        char rp = (char)lround(power + MOTOR_TRIM_R);
        fprintf(stderr, "[bench_test] GO: driving straight, power=%d (L=%d R=%d with trim), for %.2fs\n", power, lp, rp, seconds);
        BT_turn(LEFT_MOTOR, lp, RIGHT_MOTOR, rp);
        usleep((useconds_t)(seconds * 1e6));
        BT_all_stop(brake);
        fprintf(stderr, "[bench_test] STOP (brake=%d)\n", brake);

    } else if (strcmp(cmd, "turn") == 0) {
        if (argc < 5) { usage(argv[0]); return 1; }
        int power = atoi(argv[3]);
        double seconds = atof(argv[4]);
        int brake = argc >= 6 ? atoi(argv[5]) : 1;
        fprintf(stderr, "[bench_test] GO: spinning in place, power=%d, for %.2fs "
                        "(right=+%d, left=-%d)\n",
                power, seconds, power, power);
        BT_turn(LEFT_MOTOR, (char)-power, RIGHT_MOTOR, (char)power);
        usleep((useconds_t)(seconds * 1e6));
        BT_all_stop(brake);
        fprintf(stderr, "[bench_test] STOP (brake=%d)\n", brake);

    } else if (strcmp(cmd, "drivespeed") == 0) {
        if (argc < 5) { usage(argv[0]); return 1; }
        int speed = atoi(argv[3]);
        double seconds = atof(argv[4]);
        int brake = argc >= 6 ? atoi(argv[5]) : 1;
        fprintf(stderr, "[bench_test] GO: driving straight, regulated speed=%d, for %.2fs\n", speed, seconds);
        BT_turn_speed(LEFT_MOTOR, (char)speed, RIGHT_MOTOR, (char)speed);
        usleep((useconds_t)(seconds * 1e6));
        BT_all_stop(brake);
        fprintf(stderr, "[bench_test] STOP (brake=%d)\n", brake);

    } else if (strcmp(cmd, "turnspeed") == 0) {
        if (argc < 5) { usage(argv[0]); return 1; }
        int speed = atoi(argv[3]);
        double seconds = atof(argv[4]);
        int brake = argc >= 6 ? atoi(argv[5]) : 1;
        fprintf(stderr, "[bench_test] GO: spinning in place, regulated speed=%d, for %.2fs "
                        "(right=+%d, left=-%d, same convention as turn)\n",
                speed, seconds, speed, speed);
        BT_turn_speed(LEFT_MOTOR, (char)-speed, RIGHT_MOTOR, (char)speed);
        usleep((useconds_t)(seconds * 1e6));
        BT_all_stop(brake);
        fprintf(stderr, "[bench_test] STOP (brake=%d)\n", brake);

    } else if (strcmp(cmd, "drive_until") == 0) {
        if (argc < 4) { usage(argv[0]); return 1; }
        int power = atoi(argv[3]);
        int brake = argc >= 5 ? atoi(argv[4]) : 1;
        char lp = (char)lround(power + MOTOR_TRIM_L);
        char rp = (char)lround(power + MOTOR_TRIM_R);
        fprintf(stderr, "[bench_test] GO: driving straight, power=%d (L=%d R=%d with trim). Press 'd' the instant it reaches the end...\n", power, lp, rp);
        BT_turn(LEFT_MOTOR, lp, RIGHT_MOTOR, rp);
        double t0 = maintenant();
        wait_for_key('d');
        double t1 = maintenant();
        BT_all_stop(brake);
        fprintf(stderr, "[bench_test] STOP after %.3fs (brake=%d)\n", t1 - t0, brake);

    } else if (strcmp(cmd, "drivespeed_until") == 0) {
        if (argc < 4) { usage(argv[0]); return 1; }
        int speed = atoi(argv[3]);
        int brake = argc >= 5 ? atoi(argv[4]) : 1;
        fprintf(stderr, "[bench_test] GO: driving straight, regulated speed=%d. Press 'd' the instant it reaches the end...\n", speed);
        BT_turn_speed(LEFT_MOTOR, (char)speed, RIGHT_MOTOR, (char)speed);
        double t0 = maintenant();
        wait_for_key('d');
        double t1 = maintenant();
        BT_all_stop(brake);
        fprintf(stderr, "[bench_test] STOP after %.3fs (brake=%d)\n", t1 - t0, brake);

    } else if (strcmp(cmd, "stop") == 0) {
        int brake = argc >= 4 ? atoi(argv[3]) : 1;
        fprintf(stderr, "[bench_test] BT_all_stop(%d)\n", brake);
        BT_all_stop(brake);

    } else if (strcmp(cmd, "battery") == 0) {
        double volts = 0;
        int pct = 0;
        int rv = BT_read_battery_voltage(&volts);
        int rp = BT_read_battery_pct(&pct);
        if (rv == 0) fprintf(stderr, "[bench_test] battery voltage: %.2fV\n", volts);
        else fprintf(stderr, "[bench_test] BT_read_battery_voltage() failed\n");
        if (rp == 0) fprintf(stderr, "[bench_test] battery level: %d%%\n", pct);
        else fprintf(stderr, "[bench_test] BT_read_battery_pct() failed\n");

    } else if (strcmp(cmd, "latency") == 0) {
        int n = argc >= 4 ? atoi(argv[3]) : 100;
        fprintf(stderr, "[bench_test] timing %d BT_drive calls (bench test 1)...\n", n);
        double t0 = maintenant();
        for (int i = 0; i < n; i++) BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);
        double t1 = maintenant();
        BT_all_stop(1);
        double total_ms = (t1 - t0) * 1000.0;
        fprintf(stderr, "[bench_test] %d calls in %.1f ms -> %.2f ms/call\n", n, total_ms, total_ms / n);

    } else {
        usage(argv[0]);
        BT_close();
        return 1;
    }
    BT_close();
    return 0;
}
