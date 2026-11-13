# New EV3 APIs: Documentation

CSC C85 Localization Project, team RoboDictador

This document describes every API our team added on top of the EV3 starter code. The code for all of them is collected in [EV3_New_APIs.c](EV3_New_APIs.c), copied unchanged from the project files.

Nothing described here existed in the starter code. Section 7 lists the two small fixes we made to starter functions, which are not new APIs.

## Contents

1. [Summary of what was added](#1-summary-of-what-was-added)
2. [Conventions](#2-conventions)
3. [Extended motor API](#3-extended-motor-api)
4. [Drive and gyro-turn helpers](#4-drive-and-gyro-turn-helpers)
5. [Colour sensor API](#5-colour-sensor-api)
6. [Sensor rail API](#6-sensor-rail-api)
7. [Windows Bluetooth transport](#7-windows-bluetooth-transport)
8. [Fixes to starter functions](#8-fixes-to-starter-functions)

---

## 1. Summary of what was added

| Group | Functions | Lives in |
|---|---|---|
| Extended motor API | 20 public `BT_motor_*` functions | `EV3_RobotControl/btcomm.c`, `btcomm.h` |
| Drive and gyro-turn helpers | `P_gyro_turn` and 6 internal helpers | `EV3_Localization.c` |
| Colour sensor API | `calibrate_sensor`, `load_calibration`, `read_colour_med`, `colour_assign` | `EV3_Localization.c` |
| Sensor rail API | `rail_goto`, `read_rail_colours` | `EV3_Localization.c` |
| Windows Bluetooth transport | `bt_win_open`, `bt_win_close`, `bt_win_read`, `bt_win_write` | `EV3_RobotControl/bt_win.c`, `bt_win.h` |

### Why the motor API was needed

The starter API can only set a motor's power and stop it. Every move is open loop and timed by the PC, so distance depends on battery level and Bluetooth delay, and there is no way to read a wheel encoder.

The EV3 firmware already has bytecodes for regulated speed, moves measured in encoder degrees, two-motor synchronisation and encoder reads. The extended motor API wraps those bytecodes. With it the robot can:

- drive an exact distance and turn an exact angle, with the EV3 itself closing the loop on the encoders;
- keep two wheels in step so the robot drives straight;
- read each motor's encoder, which is how the colour sensor is positioned on its rail.

### All public functions at a glance

| Function | What it does |
|---|---|
| `BT_motor_set_type` | Tell the EV3 whether a port holds a large or medium motor |
| `BT_motor_reset` | Zero the per-move step counter |
| `BT_motor_set_power` | Set open-loop power (does not start the motor) |
| `BT_motor_set_speed` | Set regulated speed (does not start the motor) |
| `BT_motor_start` | Start motors with the power or speed already set |
| `BT_motor_set_polarity` | Set or flip a motor's forward direction |
| `BT_motor_get_reading` | Read measured speed and the per-move step counter |
| `BT_motor_is_busy` | Check whether a move is still running |
| `BT_motor_wait` | Block until a move finishes |
| `BT_motor_step_power` | Open-loop move for a number of encoder degrees |
| `BT_motor_time_power` | Open-loop move for a number of milliseconds |
| `BT_motor_step_speed` | Regulated-speed move for a number of encoder degrees |
| `BT_motor_time_speed` | Regulated-speed move for a number of milliseconds |
| `BT_motor_step_sync` | Two motors in sync for a number of encoder degrees |
| `BT_motor_time_sync` | Two motors in sync for a number of milliseconds |
| `BT_motor_clear_count` | Zero the encoder (odometer) count |
| `BT_motor_get_count` | Read one motor's encoder count |
| `BT_motor_get_count_pair` | Read two motors' encoder counts in one round trip |
| `BT_motor_get_position` | Read one motor's encoder count, selected by port mask |
| `BT_motor_program_stop` | Stop all output activity of the running program |
| `P_gyro_turn` | Turn in place by an angle, corrected with the gyro |
| `calibrate_sensor` | Sample the six map colours and save them to a file |
| `load_calibration` | Load the saved colour references |
| `read_colour_med` | Noise-filtered RGB read (median of 10) |
| `colour_assign` | Read the sensor and return the colour it sees |
| `rail_goto` | Move the colour sensor to a position on its rail |
| `read_rail_colours` | Sweep the rail and return a row of colours |

---

## 2. Conventions

### Port mask or port number

The EV3 identifies motors in two different ways depending on the bytecode, and the API follows the firmware.

| Argument name | Meaning | Values |
|---|---|---|
| `port_ids` | Bit mask, can select several motors | `MOTOR_A` (1), `MOTOR_B` (2), `MOTOR_C` (4), `MOTOR_D` (8), combined with `\|` |
| `port_no` | One port number | `MOTOR_NO_A` (0), `MOTOR_NO_B` (1), `MOTOR_NO_C` (2), `MOTOR_NO_D` (3) |

The `MOTOR_NO_*` constants are new. Only `BT_motor_set_type`, `BT_motor_get_reading`, `BT_motor_get_count` and `BT_motor_get_count_pair` take a port number. Passing a mask where a number is expected (or the reverse) addresses the wrong motor, so each function checks its range: masks must be 1 to 15, numbers 0 to 3.

### Units

- Power and speed are percentages from -100 to 100. Negative runs the motor backwards.
- Distances are encoder (tacho) degrees. 360 is one turn of the motor shaft.
- Times are milliseconds.
- `brake` is 1 to hold the motor at the end of a move and 0 to let it coast.

### Power or speed

Power is open loop: the EV3 applies a fixed drive level, and the real speed changes with load and battery. Speed is regulated: the EV3 adjusts the drive level to hold the requested speed. Use speed whenever the motion needs to be repeatable.

### Return values

Every `BT_motor_*` function returns 0 on success and -1 on failure, except `BT_motor_is_busy`, which returns 1, 0 or -1. Failure means either an argument was out of range (nothing is sent, and a message is printed to `stderr`) or the Bluetooth exchange failed.

A return of 0 from a move function means the EV3 accepted the command. The move itself is still running; see `BT_motor_is_busy` and `BT_motor_wait`.

### The two encoder counters

The EV3 keeps two separate counts for each motor, and they are easy to mix up.

| | Per-move step counter | Encoder (odometer) count |
|---|---|---|
| Read with | `BT_motor_get_reading` | `BT_motor_get_count`, `BT_motor_get_count_pair`, `BT_motor_get_position` |
| Zeroed by | `BT_motor_reset`, and by the firmware every time the motor is stopped | `BT_motor_clear_count` only |
| Use it for | Progress of the current move | Position over a whole run |

### How a command is sent

All 20 functions go through one internal routine, `BT_motor_send_command`. It builds the direct-command header, writes the full message, and reads the reply. It differs from the starter code's send path in two ways:

- It checks the message id that the EV3 echoes back. A reply with a different id belongs to an earlier command, so it is discarded and the next reply is read (up to 8 times). This stops a late reply from being handed back as the answer to a new command.
- It loops on partial reads and writes, so a reply split across Bluetooth packets is still read whole.

---

## 3. Extended motor API

Declared in `btcomm.h`, defined in `btcomm.c`.

### 3.1 Configuration

#### `int BT_motor_set_type(char port_no, int motor_type)`

Sets the motor type the EV3 associates with one output port.

- `port_no`: port number, `MOTOR_NO_A` to `MOTOR_NO_D`.
- `motor_type`: 7 for a large motor, 8 for a medium motor. Any other value is rejected.

The EV3 detects motor types by itself and overwrites this value, so the call is rarely needed.

#### `int BT_motor_set_polarity(char port_ids, int polarity)`

Sets which direction counts as forward for the selected motors.

- `polarity`: 1 for forward, -1 for backward, 0 to flip the current setting.

Useful when a motor is mounted mirrored, so that positive speed still means "robot forward".

### 3.2 Continuous running

These three functions split "choose a level" from "start", so several motors can be configured first and then started together.

#### `int BT_motor_set_power(char port_ids, int power)`

Sets the open-loop power of the selected motors. `power` is -100 to 100. The motors do not move until `BT_motor_start` is called.

#### `int BT_motor_set_speed(char port_ids, int speed)`

Sets the regulated speed of the selected motors. `speed` is -100 to 100. The motors do not move until `BT_motor_start` is called.

#### `int BT_motor_start(char port_ids)`

Starts the selected motors with the power or speed set earlier. They run until stopped with the starter function `BT_motor_port_stop` or `BT_all_stop`.

```c
BT_motor_set_speed(MOTOR_A | MOTOR_D, 40);   // regulated 40% on both wheels
BT_motor_start(MOTOR_A | MOTOR_D);           // go
// ...
BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);    // brake
```

### 3.3 Moves with a set length

Each of these runs one motor (or several, each on its own) through three phases: ramp up, run at the full level, ramp down. The total length is the sum of the three. All four take the same arguments:

- `port_ids`: mask of the motors to move.
- `power` or `speed`: -100 to 100. The sign sets the direction.
- ramp up, run, ramp down: length of each phase, 0 or more. Pass 0 for a ramp to skip it.
- `brake`: 1 to brake at the end, 0 to coast.

The functions return as soon as the EV3 accepts the command.

| Function | Level | Length measured in |
|---|---|---|
| `int BT_motor_step_power(char port_ids, int power, int ramp_up_degrees, int run_degrees, int ramp_down_degrees, int brake)` | Open-loop power | Encoder degrees |
| `int BT_motor_time_power(char port_ids, int power, int ramp_up_ms, int run_ms, int ramp_down_ms, int brake)` | Open-loop power | Milliseconds |
| `int BT_motor_step_speed(char port_ids, int speed, int ramp_up_degrees, int run_degrees, int ramp_down_degrees, int brake)` | Regulated speed | Encoder degrees |
| `int BT_motor_time_speed(char port_ids, int speed, int ramp_up_ms, int run_ms, int ramp_down_ms, int brake)` | Regulated speed | Milliseconds |

```c
// Turn motor B forward 180 degrees at 60% speed: 10 up, 160 at speed, 10 down, then brake
BT_motor_step_speed(MOTOR_B, 60, 10, 160, 10, 1);
```

`BT_motor_step_speed` is what moves the colour sensor along its rail (section 6).

### 3.4 Synchronised two-motor moves

In a sync move the EV3 watches both encoders and keeps the two motors in a fixed ratio. This is what makes the robot drive straight: if one wheel is held back, the other slows down to match.

#### `int BT_motor_step_sync(char port_ids, int speed, int turn_ratio, int degrees, int brake)`

Drives a pair of motors in sync for a number of encoder degrees.

- `port_ids`: mask with exactly two motors, for example `MOTOR_A | MOTOR_D`. Any other count is rejected.
- `speed`: -100 to 100.
- `turn_ratio`: -200 to 200. 0 drives straight. Moving away from 0 slows one motor relative to the other; at ±100 that motor is stopped and the robot pivots on it; at ±200 the motors turn at equal speed in opposite directions and the robot spins in place. The sign picks which motor is slowed.
- `degrees`: length of the move. 0 runs until the motors are stopped.
- `brake`: 1 to brake at the end, 0 to coast.

#### `int BT_motor_time_sync(char port_ids, int speed, int turn_ratio, int time_ms, int brake)`

Same as `BT_motor_step_sync`, with the length given in milliseconds. 0 runs until the motors are stopped.

```c
// Straight ahead for two wheel turns at 80% speed, then brake
BT_motor_step_sync(MOTOR_A | MOTOR_D, 80, 0, 720, 1);

// Spin in place (on our robot, a turn_ratio of -200 spins left)
BT_motor_step_sync(MOTOR_A | MOTOR_D, 50, -200, 224, 1);
```

### 3.5 Waiting for a move

#### `int BT_motor_is_busy(char port_ids)`

Returns 1 if any of the selected motors is still running a step, time or sync move, 0 if all are idle, and -1 on error.

#### `int BT_motor_wait(char port_ids)`

Blocks until the selected motors finish their current move. The EV3 does not answer any other command while it waits, and a motor that is physically blocked never finishes, so this call can hang. When a timeout is needed, poll `BT_motor_is_busy` in a loop, as our `wait_wheels` helper does (section 4).

#### `int BT_motor_program_stop(void)`

Sends the firmware's program-stop opcode, which stops all output activity started by the running program. Takes no arguments.

### 3.6 Reading the motors

See "The two encoder counters" in section 2 for which counter each function uses.

#### `int BT_motor_get_reading(char port_no, int *speed, int *tacho_count)`

Reads the measured speed and the per-move step counter of one motor.

- `port_no`: port number, `MOTOR_NO_A` to `MOTOR_NO_D`.
- `speed`: receives the measured speed, -100 to 100.
- `tacho_count`: receives the per-move step counter, in degrees.

Both pointers must be non-NULL. The step counter returns to 0 whenever the motor stops, so it cannot be used as an odometer.

#### `int BT_motor_reset(char port_ids)`

Zeroes the per-move step counter of the selected motors. It does not change the encoder count.

#### `int BT_motor_get_count(char port_no, int *tacho_count)`

Reads the encoder count of one motor, in degrees. The count is signed and keeps running across moves and stops.

- `port_no`: port number, `MOTOR_NO_A` to `MOTOR_NO_D`.
- `tacho_count`: receives the count. Must be non-NULL.

#### `int BT_motor_get_count_pair(char port_no_a, char port_no_b, int *count_a, int *count_b)`

Reads the encoder counts of two motors in one Bluetooth round trip, by packing two read opcodes into a single command. This takes half the time of two `BT_motor_get_count` calls and the two values are sampled almost at the same instant, which matters when comparing the left and right wheels inside a control loop.

- `port_no_a`, `port_no_b`: port numbers.
- `count_a`, `count_b`: receive the two counts. Both must be non-NULL.

#### `int BT_motor_get_position(char port_id, int *position)`

Reads the same encoder count as `BT_motor_get_count`, with the motor selected by mask instead of number. `port_id` must have exactly one motor bit set.

The firmware has an opcode named `opOUTPUT_POSITION`, but it is a "set position" command that was never implemented; the EV3 answers it with an error. This function therefore reads the encoder count instead.

#### `int BT_motor_clear_count(char port_ids)`

Sets the encoder count of the selected motors to 0. This is the only call that changes it.

```c
int left, right;
BT_motor_clear_count(MOTOR_A | MOTOR_D);
BT_motor_step_sync(MOTOR_A | MOTOR_D, 50, 0, 360, 1);
while (BT_motor_is_busy(MOTOR_A | MOTOR_D) == 1);
BT_motor_get_count_pair(MOTOR_NO_A, MOTOR_NO_D, &left, &right);
printf("left wheel %d deg, right wheel %d deg\n", left, right);
```

---

## 4. Drive and gyro-turn helpers

Defined in `EV3_Localization.c`. These turn the motor API into moves measured in millimetres and degrees of robot heading. They read the robot's dimensions from constants in `EV3_Localization.h`:

| Constant | Value | Meaning |
|---|---|---|
| `DRIVE_WHEELS` | `MOTOR_A \| MOTOR_D` | Left wheel on A, right wheel on D |
| `WHEEL_DIAM_MM` | 48.2 | Effective wheel diameter, found by measuring a commanded drive |
| `TRACK_MM` | 120.0 | Distance between the two wheel centres |
| `GYRO_PORT` | `PORT_1` | Gyro sensor port |
| `TURN_SPEED` / `TURN_FIX_SPEED` | 50 / 10 | Speed of the main turn and of the corrections |
| `TURN_MAX_FIXES` | 4 | Corrections allowed per turn |
| `TURN_TIMEOUT` | 10.0 s | Longest wait for one move |

### `void P_gyro_turn(int ang)`

Turns the robot in place by `ang` degrees. Positive is left, negative is right. Angles outside -360 to 360 are rejected.

How it works:

1. Reset the gyro to 0.
2. Do the whole turn as one sync spin at `TURN_SPEED`, with the wheel travel computed from the robot's geometry.
3. Wait for the robot to stop rotating, then read the gyro.
4. Compare the commanded angle with the measured one to estimate how much the tyres slip on this floor.
5. Spin by the remaining error at `TURN_FIX_SPEED`, scaled by that slip estimate. Repeat until the error is 0 degrees or `TURN_MAX_FIXES` is reached.

The encoders make the turn fast and the gyro makes it accurate: the result lands within the gyro's 1 degree resolution. It is the only function in this section declared in `EV3_Localization.h`; the rest are `static`.

### Internal helpers

| Function | What it does |
|---|---|
| `static int wait_wheels(void)` | Polls `BT_motor_is_busy` until the drive wheels finish. Returns -1 on error or after `TURN_TIMEOUT` seconds, so a stuck wheel cannot hang the program. |
| `static int sync_drive_speed(double mm, int speed)` | Drives straight by `mm` millimetres (negative reverses) at `speed` percent, then waits. Wheel degrees = `mm * 360 / (pi * WHEEL_DIAM_MM)`. |
| `static int sync_drive(double mm)` | `sync_drive_speed` at the normal speed, `DRIVE_SPEED` (80). |
| `static int sync_drive_slow(double mm)` | `sync_drive_speed` at the positioning speed, `FS_MOVE_SPEED` (15), so short moves do not overshoot. |
| `static int sync_spin(double deg, int speed)` | Spins in place by `deg` degrees (positive is left), then waits. Wheel degrees = `deg * TRACK_MM / WHEEL_DIAM_MM`. |
| `static int settled_heading(void)` | Reads the gyro until the turn rate stays at or below 2 deg/s for 3 readings in a row, then returns the heading in degrees to the left since the last gyro reset. |

All return 0 on success and -1 on error or timeout, except `settled_heading`, which returns the heading.

---

## 5. Colour sensor API

Defined in `EV3_Localization.c`. The starter API returns raw RGB numbers from the NXT colour sensor. These functions turn those numbers into one of the six map colours.

### Types

```c
typedef enum colour_t {
  COLOUR_INVALID = -1,
  COLOUR_BLACK = 1, COLOUR_BLUE = 2, COLOUR_GREEN = 3,
  COLOUR_YELLOW = 4, COLOUR_RED = 5, COLOUR_WHITE = 6
} colour_t;

typedef struct { int R, G, B; int valid; } colour_ref;   // one reference colour
```

The colour numbers match the ones the starter code uses for the map. The reference table holds one `colour_ref` per colour and starts with built-in default values, so classification works even without a calibration file.

### `void calibrate_sensor(void)`

Records what each of the six colours looks like to the sensor under the current lighting. This function was an empty stub in the starter code; the implementation is ours. It runs when the program is started with a target of `-1 -1`.

For each colour in turn (black, blue, green, yellow, red, white) it asks the user to place the sensor on that colour and press ENTER, takes a median reading with `read_colour_med`, and stores it as that colour's reference. A colour whose read fails keeps its previous value. The table is then written to `calibration.txt`, one `R G B` line per colour in enum order.

It opens and closes its own Bluetooth connection, because the starter `main()` calls it before connecting.

### `int load_calibration(void)`

Reads `calibration.txt` into the reference table. Returns 1 on success and 0 if the file is missing or malformed.

The file is parsed into a temporary table first and copied over only if all six lines are valid. A bad file therefore leaves the built-in defaults untouched instead of half-overwriting them.

### `int read_colour_med(int *R, int *G, int *B, int *A)`

Takes `CAL_READ_SAMPLES` (10) raw readings with the starter function `BT_read_colour_RGBraw_NXT` and returns the median of each channel. Returns 1 on success and -1 if any read fails.

The median removes single bad samples. The ambient value is also corrected: the firmware already removes ambient light from R, G and B, and the starter function subtracts it a second time, which can push dark readings below zero. This function adds the median ambient value back to each channel. `*A` receives that median ambient value.

### `colour_t colour_assign(void)`

Reads the sensor and returns the colour under it, or `COLOUR_INVALID` (-1) if the read fails.

How it classifies:

1. Take a median reading with `read_colour_med`.
2. White-balance it by dividing each channel by the white reference.
3. Convert to hue, saturation and value, and place the reading at the point `(s*cos h, s*sin h, v)`.
4. Do the same for each valid reference colour and return the nearest one (smallest squared distance).

Multiplying hue by saturation matters. Black and white have almost no saturation, so their hue is noise; scaling it by saturation makes that noise vanish, and they are told apart by brightness alone. The coloured tiles keep their full hue separation. Steps 2 and 3 are done by the internal helper `colour_features`.

```c
load_calibration();                // once at startup; defaults are used if it returns 0
colour_t c = colour_assign();
if (c == COLOUR_YELLOW) printf("on an intersection\n");
```

---

## 6. Sensor rail API

Defined in `EV3_Localization.c`. Our colour sensor rides on a rail across the front of the robot, moved by the medium motor on port B. One sweep gives a row of colours, which shows where the street is under the robot without moving the wheels.

Rail positions are encoder degrees of motor B, with 0 at the centre of the rail.

| Constant | Value | Meaning |
|---|---|---|
| `RAIL_MOTOR` / `RAIL_TACHO_NO` | `MOTOR_B` / `MOTOR_NO_B` | The rail motor, as a mask and as a number |
| `RAIL_LEFT_DEG` / `RAIL_RIGHT_DEG` | -140 / 140 | Ends of the travel |
| `RAIL_SPEED` | 60 | Regulated speed of every rail move |
| `RAIL_RAMP_DEG` | 10 | Longest ramp at each end of a move |
| `RAIL_TOL_DEG` | 3 | Close enough to the target |
| `RAIL_SETTLE` | 0.25 s | Stopped once the encoder is unchanged for this long |
| `RAIL_TIMEOUT` | 2.0 s | Longest wait for one move |

### `int rail_goto(int target)`

Moves the sensor to the absolute rail position `target`. Returns 0 on success and -1 if a motor command fails.

It reads the encoder with `BT_motor_get_count`, works out the distance to the target, and sends one `BT_motor_step_speed` move with a ramp at each end (each ramp is a third of the distance, at most `RAIL_RAMP_DEG`). It then polls the encoder until the count stops changing for `RAIL_SETTLE` seconds or `RAIL_TIMEOUT` passes, and brakes.

Each move is computed from the encoder position read at that moment, never from timing or from where the last move was meant to end, so small errors do not add up over many moves.

### `int read_rail_colours(int count, colour_t *res)`

Sweeps the rail once and reads `count` colours at evenly spaced positions. Returns 1 on success and 0 on failure or bad arguments.

- `count`: number of readings, at least 1. With an odd count the middle reading is at the rail centre.
- `res`: array of at least `count` entries. `res[0]` is the reading at `RAIL_LEFT_DEG` and `res[count-1]` the one at `RAIL_RIGHT_DEG`.

The sweep goes to the left end, steps to the right end calling `colour_assign` at each stop, and returns to the centre.

The first call zeroes the encoder with `BT_motor_clear_count`, so the sensor must be at the centre of the rail when the program starts.

```c
colour_t row[9];
if (read_rail_colours(9, row))
  for (int i = 0; i < 9; i++) printf("%d ", row[i]);   // e.g. 6 6 6 1 1 1 6 6 6: street under the centre
```

---

## 7. Windows Bluetooth transport

Defined in `EV3_RobotControl/bt_win.c`, declared in `bt_win.h`. The starter code only runs on Linux, because it opens the Bluetooth connection through BlueZ. These four functions do the same job with the Microsoft Bluetooth stack (Winsock2), so the project builds and runs on Windows as well. Robot code never calls them directly: `BT_open` and `BT_close` call them, and inside `btcomm.c` the POSIX `read`/`write` calls are redirected to them when compiling on Windows.

| Function | What it does |
|---|---|
| `int bt_win_open(const char *device_id)` | Opens an RFCOMM connection (channel 1) to the EV3 with address `"xx:xx:xx:xx:xx:xx"`. The EV3 must already be paired in Windows. Returns 0 on success, -1 on failure. |
| `int bt_win_close(void)` | Closes the connection. Safe to call when not connected. Returns 0. |
| `int bt_win_write(const void *buf, int n)` | Sends `n` bytes. Returns the number sent, or a negative value on error. |
| `int bt_win_read(void *buf, int n)` | Receives up to `n` bytes. Returns the number received, 0 if the EV3 closed the connection, or a negative value on error. |

The code is a separate source file for a reason: `<winsock2.h>` and `<windows.h>` define names such as `ULONG`, `RECT` and `DELETE` that clash with the Lego headers used by `btcomm.c`. `bt_win.c` never includes the Lego headers and `bt_win.h` uses no Windows types, so the two never meet.

The Linux build is unchanged. The `Makefile` we added detects the platform and links `-lbluetooth` on Linux, or compiles `bt_win.c` and links `-lws2_32` on Windows.

---

## 8. Fixes to starter functions

These are corrections to existing functions, listed for completeness. They are not in `EV3_New_APIs.c`.

- `BT_motor_port_start`: the command was marked "no reply expected", but the function then waited for a reply and could block forever. The command type is now "reply expected".
- `BT_turn`: the range check tested `lpower < -100` twice and never checked `rpower < -100`, so a right power below -100 was accepted. It now checks `rpower`.
