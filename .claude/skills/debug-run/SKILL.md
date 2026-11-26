---
name: debug-run
description: Diagnose what went wrong in a recorded RoboSoccer run (frames.jsonl, transitions, event images under .local/logs). Use when asked why the robot did something, to analyze/debug a run or run number, or to look at the latest run.
---

# Debug a RoboSoccer run

1. Read `RS_Distro_2025/tools/runlog/README_FOR_AI.md` once: units, the y-down
   angle convention, timing/latency, and what every field means.
2. `python3 RS_Distro_2025/tools/runlog/analyze_run.py report <run|latest> --no-video`
   and read the printed summary (also in `<run>/report/summary.md`).
   Start from "Things that look wrong", ping-pong loops and the transition reasons.
   If the user pressed `b`, the bookmarks are where they saw the problem.
3. Narrow down with numbers before pictures:
   `analyze_run.py window <run> FROM TO --every 3`. Compare `err.*` against the
   tolerances in the run's `manifest.json` `config`, check `valid`/`lost_frames`,
   heading flips, blob `role`s, and `cmd` vs measured `speed`.
4. Then look at images, cheapest first: `analyze_run.py event <run> N` makes
   `report/sheets/*_overview.png` and `*_trigger.png`; read those with the Read
   tool. Open single PNGs only for a specific frame. Use `field/` images (if
   present) to separate vision problems from decision problems.
5. Explain the cause with frame numbers, the numbers that prove it, and the
   code path (`RS_Distro_2025/src/logic/mode_*.c`; transition reasons quote the
   constant names from `ai_config.h`).
6. If you change logic code, run `analyze_run.py replay <run>` and show that the
   decision changes at the bad frame and not earlier. Say that replay is open loop.

Never delete runs. Never push; this repo's owner pushes themself.
