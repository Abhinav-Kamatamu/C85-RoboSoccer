# RoboSoccer run logs: guide for an AI (and humans)

You are looking at the flight recorder of a small two-wheeled LEGO EV3 robot
playing table soccer under an overhead camera. Each run of `roboSoccer` with a
non-zero `logItems` argument writes one folder here. Your job is usually:
**find out why the robot did something wrong, using numbers first and
pictures second.**

## Fast path

```bash
python3 RS_Distro_2025/tools/runlog/analyze_run.py list            # all runs
python3 RS_Distro_2025/tools/runlog/analyze_run.py report 42       # summary + event reports + sheets
python3 RS_Distro_2025/tools/runlog/analyze_run.py window 42 1200 1290 --every 3
python3 RS_Distro_2025/tools/runlog/analyze_run.py event 42 3      # one event, with contact sheets
python3 RS_Distro_2025/tools/runlog/analyze_run.py replay 42       # re-run the log through the CURRENT FSM code
```

Suggested order:
1. `report` → read `report/summary.md`: "Things that look wrong", ping-pong
   loops, the transition table with reasons.
2. Pick the suspicious time range or event and use `window` for the numbers.
3. Only then look at pictures: `report/sheets/*_overview.png` (16 frames over
   the 3 s event window) and `*_trigger.png` (every frame around the trigger).
   One sheet costs about as much as one image; prefer sheets over single frames.
4. If you change code, `replay` the same run to see whether the decisions change.

## Folder layout

```
.local/logs/
  README_FOR_AI.md            this file (copied here on every run)
  0042_2026-11-25_14-03-22_penalty_red/
    manifest.json             run facts; see below
    frames.jsonl              one JSON object per camera frame (~30/s)
    transitions.jsonl         one JSON object per FSM state change (+ bookmarks)
    positions.log             older human-readable per-frame line (same data)
    config/                   ai_config.h, colours/Homography/offsets .dat as used
    frames/f000123.png        5 fps: exactly what the window showed (1024x768)
    raw/f000123.png           1 fps raw camera frames (only with logItems bit2)
    events/E003_f000456_P_UNSTICK/
      event.json              trigger, window, list of frames
      display/f*.png          every frame from -2 s to +1 s, 512x384
      field/f*.png            unwarped field image = blob detector input (bit2 only, 15 fps, 1280x720)
    report/                   written by analyze_run.py (safe to delete)
    KEEP                      (optional, create by hand) never auto-delete this run
```

Old runs are deleted oldest-first at startup to keep the folder under 5 GB
(`ROBOSOCCER_LOG_BUDGET_GB`), except runs that contain a `KEEP` file.

## Conventions (read this before trusting any number)

- **Units:** cm, seconds, degrees (angles in `heading_deg`/`*_deg`), cm/s.
- **Field frame:** origin top-left of the field, **x right, y DOWN**
  (image convention). Field is `manifest.field.min_cm..max_cm`
  (default 170 x 115 cm). Goals are at the middle of the left/right walls;
  which one is ours is `manifest.field.own_goal_cm`.
- **Angles:** `atan2(dy, dx)` in that y-down frame, so a positive angle /
  positive `w_radps` turns **clockwise on screen**, which is a physical right
  turn. 0° = facing +x (right).
- **Pixels vs cm:** blob `c_px` are in the unwarped 1280x720 field image;
  `c_cm = c_px / PX_PER_CM` with **different x and y scales**
  (`manifest.config.PX_PER_CM_X/Y`). The 1024x768 display PNG is that image
  scaled by 0.8 in x and 1.0667 in y.
- **Timing:** frame N's `state` is the state *entering* frame N; `ran` is the
  state(s) whose code ran (pre-empts can change it within the frame, nested
  sub-state last, e.g. `["S_ATTACK","P_ALIGN"]`); a state change made during
  frame N shows up in frame N's `transitions` and as frame N+1's `state`.
  `cmd` is what frame N decided. The motors get it over Bluetooth tens of ms
  later, plus camera latency on top of that; `sent_prev` is
  the wheel power actually sent for the previous frame. Expect the robot's
  measured motion to lag commands by 2 to 4 frames; that alone is not a bug.
- **Validity:** when `valid` is 0 the position is meaningless (often 0,0);
  `lost_frames` counts consecutive misses. Velocities are frame differences
  and noisy (0.5 cm of noise is 15 cm/s at 30 fps).
- **Heading sign:** the blob shape gives an axis, not a direction; perception
  picks the sign using motion (`raw_px.self.shape_dir` vs `motion_dir`).
  A 180° flip in `self.heading_deg` between frames is a classic failure:
  check those first when the robot spins or drives backwards.

## manifest.json

`mode` (soccer/penalty/chase), `own_colour`, `git.commit/branch/
uncommitted_changes`, `config` (the **compiled** values of the important
`ai_config.h` constants: tolerances, timeouts, margins), `field`, `states`
(id → name for every state seen), `stats` (frames, fps, transitions, events,
images dropped), `complete` (false = the program crashed or was killed).

## frames.jsonl (one line per frame)

| key | meaning |
|---|---|
| `f`, `t`, `dt_ms` | frame number, seconds since start, time since previous frame (>66 ms = frame drop) |
| `ai` | false while the AI is off (`t` key toggles); then only `blobs` is present |
| `state`, `state_name`, `ran`, `t_in_state` | see Timing above |
| `self`, `opp`, `ball` | `valid`, `lost_frames`, `pos`, `vel`, `heading` (unit vector), `heading_deg`, `speed` |
| `plan.staging`, `plan.aim` | where the robot is trying to park / what it aims at (null when the mode has no plan) |
| `err.*` | the quantities the FSM guards compare with `manifest.config` tolerances: `self_to_staging_cm` (vs STAGE_TOL_CM), `heading_err_to_aim_deg` (vs ALIGN_TOL_DEG / ABORT_TOL_DEG), `lateral_to_aim_line_cm` (vs LAT_TOL_CM), `self_to_ball_cm`, `self_to_wall_cm` (vs WALL_TRIGGER_MARGIN_CM), `heading_dot_wall_inward` (< -WALL_OUTWARD_DOT means heading out of the field), etc. |
| `cmd.v_cmps`, `cmd.w_radps` | forward speed and turn rate decided this frame |
| `sent_prev.l/.r` | wheel powers sent for the previous frame (-100..100) |
| `raw_px` | blob shape axis and motion direction for each agent, in pixels |
| `blobs[]` | every blob found: `role` (self/opp/ball/null = unassigned noise), `c_px`, `c_cm`, `size` (pixels), `bbox_px`, `rgb`, `hsv`. Unassigned blobs or the wrong `role` point at colour calibration problems |
| `transitions[]` | state changes this frame with `kind` (state / pre-empt) and `reason` |
| `mark` | the human pressed `b` here: something looked wrong to them |
| `images` | `snapshot` / `raw` PNG saved for this frame, `event` folder triggered here |

## transitions.jsonl

`{"f","t","from","from_name","to","to_name","kind","reason"}`; `kind` is
`state` (the state's own `next()` decided) or `pre-empt` (a global safety
check: wall, stall, lost, goal). Reasons carry the numbers and the limit they
were compared with, e.g. `align gave up: 4.1s in ALIGN (ALIGN_TIMEOUT_S 4.0),
attempts 2 (ALIGN_ATTEMPT_LIMIT 3)`. Bookmark lines look like
`{"f","t","mark"}`.

## Pictures

- The display shows each blob in its average colour with crosshairs, plus
  the robot's **intent**: white line + box = heading, cyan box = staging
  point, dashed cyan = path to it, green = aim line from the ball.
- Event folders are triggered by entering WALL_RECOVER / UNSTICK / LOST /
  STRIKE / SHOT_DONE states, any pre-empt, a bookmark, and quitting. Events
  that happen within 1 s of each other share one folder (`event` lists all
  names, joined with `+`). Each frame is saved in one event only.
- PNGs carry a `robosoccer` text chunk with frame/state.
- `field/` images (bit2) are the input to blob detection. Use them to tell a
  **vision problem** (ball not where the blob says, colour merged with
  background, glare) from a **decision problem** (blobs right, choice wrong).

## Replay

`analyze_run.py replay RUN` compiles the current `logic/` FSM code into a
library, feeds every logged World back in, and reports the first frame where
the state sequence or commands differ from the recording. It is **open loop**:
the recorded robot did not react to the new commands, so after the first
divergence it shows what the new code would decide in the old situation,
not what a new run would do. Use it to check that a fix changes the decision
at the frame where things went wrong, and nothing earlier.

## Known limits

- Only state changes get a reason; staying in a state does not.
- Soccer mode's attack sub-FSM (penalty states) logs its own transitions; the
  sub-state is visible in `ran`.
- The display PNG is what the screen showed. Before field calibration it is
  the raw camera (`kind=camera` in the PNG text), not blobs.
- Image writing is skipped (never blocking the robot) if the disk is nearly
  full, a run passes 3 GB of images, or the writer falls behind; check
  `manifest.stats.images_dropped`.
