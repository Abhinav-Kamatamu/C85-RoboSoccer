#!/usr/bin/env python3
"""Read a RoboSoccer run log (.local/logs/NNNN_...) and turn it into
something a human or an AI can reason about quickly.

    analyze_run.py list                         every run, newest last
    analyze_run.py report [RUN]                 write RUN/report/ (summary, events, sheets, video)
    analyze_run.py window RUN FROM TO [--every N]   compact per-frame table for frames FROM..TO
    analyze_run.py event RUN N                  one event's report (+ its contact sheets)
    analyze_run.py sheet RUN FROM TO            contact sheet of the snapshots in a frame range
    analyze_run.py replay RUN [--from F] [--to T]   re-run the logged world through the CURRENT FSM code

RUN is a run folder, a run number (42) or "latest" (the default).
Logs live in <repo>/.local/logs unless $ROBOSOCCER_LOG_DIR says otherwise.
See README_FOR_AI.md (next to this script, and copied into the logs folder).

Only the standard library is needed, except Pillow for contact sheets and
ffmpeg for the run video; both are skipped with a note if missing.
"""

import argparse
import bisect
import ctypes
import glob
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
DISTRO = os.path.dirname(os.path.dirname(HERE))          # RS_Distro_2025
REPO = os.path.dirname(DISTRO)
SRC = os.path.join(DISTRO, "src")

TILE_W, TILE_H, LABEL_H = 384, 288, 18


# ----------------------------------------------------------------- loading --

def logs_root():
    env = os.environ.get("ROBOSOCCER_LOG_DIR")
    return env if env else os.path.join(REPO, ".local", "logs")


def run_dirs():
    root = logs_root()
    if not os.path.isdir(root):
        return []
    return sorted(d for d in glob.glob(os.path.join(root, "[0-9][0-9][0-9][0-9]_*")) if os.path.isdir(d))


def resolve_run(arg):
    if arg in (None, "", "latest"):
        runs = run_dirs()
        if not runs:
            sys.exit(f"no runs in {logs_root()}")
        return runs[-1]
    if os.path.isdir(arg):
        return os.path.abspath(arg)
    if arg.isdigit():
        for d in run_dirs():
            if int(os.path.basename(d)[:4]) == int(arg):
                return d
    sys.exit(f"no run matching {arg!r} in {logs_root()}")


class Run:
    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        mpath = os.path.join(path, "manifest.json")
        self.manifest = json.load(open(mpath)) if os.path.exists(mpath) else {}
        self.frames = []
        fpath = os.path.join(path, "frames.jsonl")
        if os.path.exists(fpath):
            with open(fpath) as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        self.frames.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass  # a run killed mid-write can leave half a line
        self.by_f = {fr["f"]: fr for fr in self.frames}
        self.fnums = [fr["f"] for fr in self.frames]
        self.transitions, self.marks = [], []
        tpath = os.path.join(path, "transitions.jsonl")
        if os.path.exists(tpath):
            for line in open(tpath):
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:
                    continue
                (self.marks if "mark" in r else self.transitions).append(r)
        self.events = []
        for ej in sorted(glob.glob(os.path.join(path, "events", "*", "event.json"))):
            try:
                ev = json.load(open(ej))
            except json.JSONDecodeError:
                continue
            ev["dir"] = os.path.dirname(ej)
            ev["id"] = os.path.basename(ev["dir"])
            self.events.append(ev)

    def frame_near(self, f):
        i = bisect.bisect_left(self.fnums, f)
        if i < len(self.fnums) and self.fnums[i] == f:
            return self.frames[i]
        cands = [j for j in (i - 1, i) if 0 <= j < len(self.frames)]
        return self.frames[min(cands, key=lambda j: abs(self.fnums[j] - f))] if cands else None

    def frames_between(self, a, b):
        i, j = bisect.bisect_left(self.fnums, a), bisect.bisect_right(self.fnums, b)
        return self.frames[i:j]


# -------------------------------------------------------------- formatting --

def num(v, fmt="{:.1f}"):
    return "-" if v is None else fmt.format(v)


def vec(v, fmt="{:.0f}"):
    return "-" if not v else "(" + fmt.format(v[0]) + "," + fmt.format(v[1]) + ")"


def state_of(fr):
    if not fr.get("ai"):
        return "(AI off)"
    ran = fr.get("ran") or []
    return "/".join(ran) if ran else fr.get("state_name", "?")


ROW_HDR = ("frame  t(s)    state                      self(x,y) hdg   ball(x,y)  "
           "d_stage hdgErrAim lat   d_ball wall  v     w      notes")


def row(fr, t0=None):
    t = fr["t"] - (t0 or 0)
    base = f"{fr['f']:>6} {t:>7.2f}  "
    if not fr.get("ai"):
        nb = len(fr.get("blobs", []))
        return base + f"(AI off, {nb} blobs)" + notes(fr)
    s, b, e, c = fr["self"], fr["ball"], fr["err"], fr["cmd"]
    selfs = vec(s["pos"]) + " " + num(s["heading_deg"], "{:>4.0f}") if s["valid"] else f"LOST({s['lost_frames']})"
    balls = vec(b["pos"]) if b["valid"] else f"LOST({b['lost_frames']})"
    return (base + f"{state_of(fr)[:26]:<26} {selfs:<15} {balls:<10} "
            f"{num(e['self_to_staging_cm']):>6} {num(e['heading_err_to_aim_deg']):>8} "
            f"{num(e['lateral_to_aim_line_cm']):>5} {num(e['self_to_ball_cm']):>6} "
            f"{num(e['self_to_wall_cm']):>5} {num(c['v_cmps'], '{:>5.1f}')} {num(c['w_radps'], '{:>6.2f}')}"
            + notes(fr))


def notes(fr):
    out = []
    for tr in fr.get("transitions", []):
        out.append(f"{tr['from']}->{tr['to']} [{tr['kind']}] {tr['reason']}")
    if fr.get("mark"):
        out.append("MARK " + fr["mark"])
    if fr.get("images", {}).get("event"):
        out.append("event dump: " + fr["images"]["event"])
    return ("  | " + " ; ".join(out)) if out else ""


# ------------------------------------------------------------- analysis ----

def runs_of(frames, pred, min_len):
    """Consecutive stretches where pred(frame) is true: [(first, last, n)]."""
    out, start, prev, n = [], None, None, 0
    for fr in frames:
        if pred(fr):
            if start is None:
                start, n = fr, 0
            n += 1
            prev = fr
        else:
            if start is not None and n >= min_len:
                out.append((start, prev, n))
            start = None
    if start is not None and n >= min_len:
        out.append((start, prev, n))
    return out


def find_anomalies(run):
    ai = [fr for fr in run.frames if fr.get("ai")]
    found = []
    for name, key in (("self", "self"), ("ball", "ball"), ("opponent", "opp")):
        if ai and sum(1 for fr in ai if fr[key]["valid"]) < 0.05 * len(ai):
            continue  # never really on the field (e.g. no opponent in penalty mode)
        for a, b, n in sorted(runs_of(ai, lambda fr, k=key: not fr[k]["valid"], 5), key=lambda x: -x[2])[:5]:
            found.append((a["f"], f"{name} not seen for {n} frames ({a['f']}..{b['f']}, {b['t'] - a['t']:.2f}s)"))
    prev = None
    for fr in ai:
        if prev is not None and fr["f"] - prev["f"] <= 2:
            s0, s1 = prev["self"], fr["self"]
            if s0["valid"] and s1["valid"] and s0["heading_deg"] is not None and s1["heading_deg"] is not None:
                d = abs((s1["heading_deg"] - s0["heading_deg"] + 180) % 360 - 180)
                if d > 120:
                    found.append((fr["f"], f"self heading flipped {d:.0f} deg in one frame ({s0['heading_deg']:.0f} -> {s1['heading_deg']:.0f})"))
            for k, lim in (("self", 15.0), ("ball", 25.0)):
                a, b = prev[k], fr[k]
                if a["valid"] and b["valid"]:
                    jump = math.dist(a["pos"], b["pos"])
                    if jump > lim:
                        found.append((fr["f"], f"{k} jumped {jump:.1f} cm in one frame (tracking swap or noise?)"))
        prev = fr
    for a, b, n in runs_of(ai, lambda fr: abs(fr["cmd"]["v_cmps"]) > 5 and fr["self"]["valid"]
                           and fr["self"]["speed"] < 2.0, 30):
        found.append((a["f"], f"commanded to drive but not moving for {b['t'] - a['t']:.1f}s ({a['f']}..{b['f']}) in {state_of(a)}"))
    slow = [fr for fr in run.frames if (fr.get("dt_ms") or 0) > 66]
    if slow:
        worst = max(slow, key=lambda fr: fr["dt_ms"])
        found.append((worst["f"], f"{len(slow)} slow frames (>66 ms); worst {worst['dt_ms']:.0f} ms at frame {worst['f']}"))
    return sorted(found)


def ping_pong(transitions, window_s=6.0, min_count=3):
    """A->B->A bounces: [(A, B, count)]"""
    seq = [(t["from_name"], t["to_name"], t["t"]) for t in transitions]
    hits = Counter()
    for i in range(len(seq) - 1):
        a, b, t = seq[i]
        a2, b2, t2 = seq[i + 1]
        if a == b2 and b == a2 and t2 - t <= window_s:
            hits[tuple(sorted((a, b)))] += 1
    return [(a, b, n) for (a, b), n in hits.most_common() if n >= min_count]


def time_per_state(frames):
    acc = defaultdict(float)
    for fr in frames:
        if fr.get("ai"):
            acc[state_of(fr)] += (fr.get("dt_ms") or 0) / 1000.0
    return sorted(acc.items(), key=lambda kv: -kv[1])


# ---------------------------------------------------------- contact sheets --

def make_sheet(tiles, out_path, title):
    """tiles: [(png_path, label)] -> one grid image, 4 columns."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return "Pillow not installed, no contact sheet"
    tiles = [t for t in tiles if os.path.exists(t[0])]
    if not tiles:
        return "no images for this range"
    cols = 4
    rows = (len(tiles) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * TILE_W, rows * (TILE_H + LABEL_H) + LABEL_H), (30, 30, 30))
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.load_default()
    draw.text((4, 3), title, fill=(255, 255, 0), font=font)
    for i, (p, label) in enumerate(tiles):
        im = Image.open(p).convert("RGB").resize((TILE_W, TILE_H))
        x, y = (i % cols) * TILE_W, LABEL_H + (i // cols) * (TILE_H + LABEL_H)
        draw.text((x + 3, y + 3), label, fill=(255, 255, 255), font=font)
        sheet.paste(im, (x, y + LABEL_H))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    sheet.save(out_path)
    return None


def event_sheets(run, ev, out_dir):
    disp = sorted((fr for fr in ev["frames"] if "display" in fr), key=lambda fr: fr["f"])
    if not disp:
        return []
    made = []

    def label(fr):
        rec = run.by_f.get(fr["f"])
        st = state_of(rec) if rec else fr.get("state", "")
        return f"f{fr['f']} {fr['t_rel_event']:+.2f}s {st}"[:60]

    step = max(1, len(disp) // 16)
    overview = disp[::step][:16]
    p = os.path.join(out_dir, f"{ev['id']}_overview.png")
    if make_sheet([(os.path.join(ev["dir"], fr["display"]), label(fr)) for fr in overview], p,
                  f"{ev['id']}  {ev['event']}  overview (every {step} frames, -2s..+1s)") is None:
        made.append(p)
    trig = min(range(len(disp)), key=lambda i: abs(disp[i]["t_rel_event"]))
    dense = disp[max(0, trig - 8): trig + 8]
    p = os.path.join(out_dir, f"{ev['id']}_trigger.png")
    if make_sheet([(os.path.join(ev["dir"], fr["display"]), label(fr)) for fr in dense], p,
                  f"{ev['id']}  {ev['event']}  every frame around the trigger") is None:
        made.append(p)
    return made


# -------------------------------------------------------------- commands ---

def cmd_list(args):
    for d in run_dirs():
        try:
            m = json.load(open(os.path.join(d, "manifest.json")))
            st = m.get("stats", {})
            keep = " KEEP" if os.path.exists(os.path.join(d, "KEEP")) else ""
            print(f"{os.path.basename(d)}  {st.get('duration_s', 0):6.1f}s  {st.get('frames', 0):6d} frames  "
                  f"{st.get('transitions', 0):4d} transitions  {st.get('event_dumps', 0):3d} events"
                  f"{'' if m.get('complete') else '  (incomplete: crashed or still running)'}{keep}")
        except (OSError, json.JSONDecodeError):
            print(f"{os.path.basename(d)}  (no readable manifest)")


def event_markdown(run, ev, sheets):
    lines = [f"# Event {ev['id']}: {ev['event']}", ""]
    trig = run.by_f.get(ev["trigger_frame"])
    t_trig = trig["t"] if trig else None
    lines.append(f"Trigger frame {ev['trigger_frame']}" + (f" at t={t_trig:.2f}s" if t_trig is not None else "")
                 + f". Images cover {ev['window_s'][0]}s..+{ev['window_s'][1]}s.")
    if ev.get("previous_event"):
        lines.append(f"Previous event: `{ev['previous_event']}` (frames already saved there are not repeated here).")
    lines.append("")
    for p in sheets:
        lines.append(f"![{os.path.basename(p)}]({os.path.relpath(p, os.path.join(run.path, 'report'))})")
    if sheets:
        lines.append("")
    if t_trig is not None:
        lines += ["## Transitions in the window", ""]
        for t in run.transitions:
            if t_trig - 3 <= t["t"] <= t_trig + 1.5:
                lines.append(f"- f{t['f']} t={t['t']:.2f}  {t['from_name']} -> {t['to_name']} "
                             f"[{t['kind']}] {t['reason']}")
        lines += ["", "## Frames (every 3rd, -2s..+1s)", "", "```", ROW_HDR]
        for fr in run.frames_between(ev["trigger_frame"] - 60, ev["trigger_frame"] + 30)[::3]:
            lines.append(row(fr))
        lines += ["```", ""]
    return "\n".join(lines)


def cmd_report(args):
    run = Run(resolve_run(args.run))
    m = run.manifest
    out = os.path.join(run.path, "report")
    os.makedirs(out, exist_ok=True)
    st = m.get("stats", {})
    ai = [fr for fr in run.frames if fr.get("ai")]
    L = [f"# Run {run.name}", ""]
    L.append(f"- mode **{m.get('mode', {}).get('name')}**, own colour **{m.get('own_colour', {}).get('name')}**, "
             f"git `{m.get('git', {}).get('commit')}` on `{m.get('git', {}).get('branch')}`"
             + (" (with uncommitted changes)" if m.get("git", {}).get("uncommitted_changes") else ""))
    L.append(f"- {st.get('duration_s', 0):.1f}s, {len(run.frames)} frames ({len(ai)} with the AI running), "
             f"avg {st.get('avg_fps', 0):.1f} fps; {len(run.transitions)} transitions, {len(run.events)} event dumps, "
             f"{len(run.marks)} bookmarks")
    if not m.get("complete"):
        L.append("- **manifest says the run did not finish cleanly** (crash, kill -9, or still running)")
    if st.get("images_dropped"):
        L.append(f"- {st['images_dropped']} images were dropped (writer behind or budget hit)")
    if m.get("logging_note"):
        L.append(f"- {m['logging_note']}")
    if not ai:
        L.append("- **the AI never ran in this run** (press `t` to start it); only blobs and images were logged")
    if ai:
        for k, name in (("self", "self"), ("ball", "ball"), ("opp", "opponent")):
            seen = sum(1 for fr in ai if fr[k]["valid"]) / len(ai)
            L.append(f"- {name} visible in {seen * 100:.0f}% of AI frames")
    L.append("")

    L += ["## Things that look wrong", ""]
    an = find_anomalies(run)
    pp = ping_pong(run.transitions)
    for a, b, n in pp:
        L.append(f"- **{a} <-> {b} bounced {n} times** (back and forth within 6 s); look at the reasons below")
    for f, msg in an[:40]:
        L.append(f"- f{f}: {msg}")
    if not an and not pp:
        L.append("- nothing obvious from the automatic checks")
    L.append("")

    L += ["## Time per state", ""]
    for s, secs in time_per_state(run.frames)[:15]:
        L.append(f"- {s}: {secs:.1f}s")
    L.append("")

    L += ["## Events (image dumps)", ""]
    for ev in run.events:
        sheets = event_sheets(run, ev, os.path.join(out, "sheets"))
        open(os.path.join(out, f"{ev['id']}.md"), "w").write(event_markdown(run, ev, sheets))
        L.append(f"- [{ev['id']}]({ev['id']}.md): {ev['event']} at frame {ev['trigger_frame']}")
    if not run.events:
        L.append("- none (display logging off, or nothing triggered)")
    L.append("")

    L += ["## Bookmarks", ""]
    L += [f"- f{mk['f']} t={mk['t']:.2f}: {mk['mark']}" for mk in run.marks] or ["- none"]
    L.append("")

    L += ["## State transitions", "", "| frame | t (s) | from | to | kind | reason |", "|---|---|---|---|---|---|"]
    for t in run.transitions[:400]:
        L.append(f"| {t['f']} | {t['t']:.2f} | {t['from_name']} | {t['to_name']} | {t['kind']} | {t['reason']} |")
    if len(run.transitions) > 400:
        L.append(f"| ... | | | | | {len(run.transitions) - 400} more in transitions.jsonl |")
    L.append("")

    snaps = sorted(glob.glob(os.path.join(run.path, "frames", "f*.png")))
    video = os.path.join(out, "run.mp4")
    if snaps and shutil.which("ffmpeg") and not args.no_video:
        r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "5", "-pattern_type", "glob",
                            "-i", os.path.join(run.path, "frames", "f*.png"), "-c:v", "libx264",
                            "-pix_fmt", "yuv420p", video], capture_output=True, text=True)
        L.append(f"Video of the 5 fps snapshots: [run.mp4](run.mp4)" if r.returncode == 0
                 else f"(ffmpeg failed: {r.stderr.strip()[:200]})")
    L += ["", "## Digging deeper", "", "```",
          f"python3 {os.path.relpath(__file__, REPO)} window {run.name[:4]} FROM TO",
          f"python3 {os.path.relpath(__file__, REPO)} replay {run.name[:4]}", "```", ""]
    path = os.path.join(out, "summary.md")
    open(path, "w").write("\n".join(L))
    print("\n".join(L))
    print(f"\n[report written to {out}]")


def cmd_window(args):
    run = Run(resolve_run(args.run))
    frames = run.frames_between(args.start, args.end)[::max(1, args.every)]
    print(ROW_HDR)
    for fr in frames:
        print(row(fr))
    if not frames:
        print(f"(no frames in {args.start}..{args.end}; run has {run.fnums[0] if run.fnums else '-'}.."
              f"{run.fnums[-1] if run.fnums else '-'})")


def cmd_event(args):
    run = Run(resolve_run(args.run))
    evs = [e for e in run.events if e["id"].startswith(f"E{int(args.n):03d}_")]
    if not evs:
        sys.exit(f"no event {args.n}; have: " + ", ".join(e["id"] for e in run.events))
    ev = evs[0]
    out = os.path.join(run.path, "report")
    sheets = event_sheets(run, ev, os.path.join(out, "sheets"))
    md = event_markdown(run, ev, sheets)
    os.makedirs(out, exist_ok=True)
    open(os.path.join(out, f"{ev['id']}.md"), "w").write(md)
    print(md)
    for p in sheets:
        print(f"[contact sheet: {p}]")


def cmd_sheet(args):
    run = Run(resolve_run(args.run))
    tiles = []
    for fr in run.frames_between(args.start, args.end):
        p = fr.get("images", {}).get("snapshot")
        if p:
            tiles.append((os.path.join(run.path, p), f"f{fr['f']} t={fr['t']:.2f} {state_of(fr)}"[:60]))
    tiles = tiles[:16]
    out = args.out or os.path.join(run.path, "report", "sheets", f"snapshots_{args.start}_{args.end}.png")
    err = make_sheet(tiles, out, f"{run.name} snapshots f{args.start}..f{args.end}")
    print(err or f"[contact sheet: {out}] ({len(tiles)} frames, max 16; use a smaller range for more detail)")


# ---------------------------------------------------------------- replay ---

class Vec2(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double)]


class Agent(ctypes.Structure):
    _fields_ = [("valid", ctypes.c_int), ("lost_frames", ctypes.c_int),
                ("pos", Vec2), ("vel", Vec2), ("heading", Vec2)]


class World(ctypes.Structure):
    _fields_ = [("self_", Agent), ("opp", Agent), ("ball", Agent),
                ("field_min", Vec2), ("field_max", Vec2), ("own_goal", Vec2), ("their_goal", Vec2),
                ("goal_width_cm", ctypes.c_double), ("now", ctypes.c_double)]


class Motion(ctypes.Structure):
    _fields_ = [("v", ctypes.c_double), ("w", ctypes.c_double)]


REPLAY_SOURCES = ["logic/fsm.c", "logic/runlog.c", "logic/geometry.c", "logic/skills.c",
                  "logic/mode_penalty.c", "logic/mode_chase.c", "logic/mode_soccer.c"]


def build_replay_lib():
    out_dir = os.path.join(REPO, ".local", "build")
    os.makedirs(out_dir, exist_ok=True)
    lib = os.path.join(out_dir, "libfsm_replay.so")
    srcs = [os.path.join(SRC, s) for s in REPLAY_SOURCES] + [os.path.join(HERE, "fsm_replay_shim.c")]
    deps = srcs + glob.glob(os.path.join(SRC, "logic", "*.h"))
    if not os.path.exists(lib) or os.path.getmtime(lib) < max(os.path.getmtime(p) for p in deps):
        cmd = ["gcc", "-O2", "-shared", "-fPIC", "-I", SRC, "-o", lib] + srcs + ["-lm"]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit("could not build the replay library:\n" + r.stderr[-3000:])
    return lib


def to_world(fr, field):
    w = World()
    for name, attr in (("self", "self_"), ("opp", "opp"), ("ball", "ball")):
        a, src = getattr(w, attr), fr[name]
        a.valid, a.lost_frames = src["valid"], src["lost_frames"]
        a.pos = Vec2(*src["pos"])
        a.vel = Vec2(*src["vel"])
        a.heading = Vec2(*src["heading"])
    w.field_min = Vec2(*field["min_cm"])
    w.field_max = Vec2(*field["max_cm"])
    w.own_goal = Vec2(*field["own_goal_cm"])
    w.their_goal = Vec2(*field["their_goal_cm"])
    w.goal_width_cm = field["goal_width_cm"]
    w.now = fr["t"]
    return w


def cmd_replay(args):
    run = Run(resolve_run(args.run))
    lib = ctypes.CDLL(build_replay_lib())
    if lib.replay_world_size() != ctypes.sizeof(World):
        sys.exit("World struct changed in logic/world.h; update the World class in analyze_run.py")
    lib.replay_step.restype = ctypes.c_int
    lib.replay_step.argtypes = [ctypes.c_int, ctypes.POINTER(World), ctypes.c_int, ctypes.c_double,
                                ctypes.POINTER(Motion)]
    mode = run.manifest.get("mode", {}).get("id", 1)
    field = run.manifest.get("field")
    ai = [fr for fr in run.frames if fr.get("ai") and args.start <= fr["f"] <= args.end]
    if not ai:
        sys.exit("no AI frames in that range")
    if not field:
        c = run.manifest.get("config", {})
        sys.exit("manifest has no field geometry" if not c else "manifest has no field geometry; rerun with a newer log")

    # The C code prints its own transitions (with reasons) to stderr; collect them
    err_fd = os.dup(2)
    tmp = tempfile.TemporaryFile(mode="w+")
    os.dup2(tmp.fileno(), 2)
    rows, state = [], ai[0]["state"]
    try:
        for i, fr in enumerate(ai):
            w, m = to_world(fr, field), Motion()
            nxt = lib.replay_step(mode, ctypes.byref(w), state, fr["t"], ctypes.byref(m))
            logged_next = ai[i + 1]["state"] if i + 1 < len(ai) else None
            rows.append((fr, state, nxt, logged_next, m.v, m.w))
            state = nxt
    finally:
        sys.stderr.flush()
        os.dup2(err_fd, 2)
        os.close(err_fd)
    tmp.seek(0)
    replay_log = tmp.read()

    names = {int(k): v for k, v in run.manifest.get("states", {}).items()}
    nm = lambda s: names.get(s, str(s)) if s is not None else "-"
    first_div = next((r for r in rows if r[3] is not None and r[2] != r[3]), None)
    cmd_div = [r for r in rows if abs(r[4] - r[0]["cmd"]["v_cmps"]) > 0.5 or abs(r[5] - r[0]["cmd"]["w_radps"]) > 0.05]
    print(f"Replayed {len(rows)} AI frames ({ai[0]['f']}..{ai[-1]['f']}) of {run.name} through the CURRENT code, mode {mode}.")
    print("Open loop: the recorded robot did not react to the replayed commands, so anything after the first")
    print("divergence is 'what the new code would decide in the OLD situation', not a prediction of a new run.\n")
    if first_div is None:
        print("State sequence: identical to the recording.")
    else:
        fr = first_div[0]
        print(f"State sequence first differs at frame {fr['f']} (t={fr['t']:.2f}): recorded {nm(first_div[1])} -> "
              f"{nm(first_div[3])}, replay {nm(first_div[1])} -> {nm(first_div[2])}")
    print(f"Motion commands differ (|dv|>0.5 or |dw|>0.05) on {len(cmd_div)} of {len(rows)} frames"
          + (f", first at frame {cmd_div[0][0]['f']}" if cmd_div else ""))
    print("\nReplay transitions (from the C code):")
    print(replay_log.strip() or "(none)")


# ------------------------------------------------------------------- main --

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("list")
    p = sub.add_parser("report")
    p.add_argument("run", nargs="?", default="latest")
    p.add_argument("--no-video", action="store_true")
    p = sub.add_parser("window")
    p.add_argument("run")
    p.add_argument("start", type=int)
    p.add_argument("end", type=int)
    p.add_argument("--every", type=int, default=1)
    p = sub.add_parser("event")
    p.add_argument("run")
    p.add_argument("n")
    p = sub.add_parser("sheet")
    p.add_argument("run")
    p.add_argument("start", type=int)
    p.add_argument("end", type=int)
    p.add_argument("--out")
    p = sub.add_parser("replay")
    p.add_argument("run", nargs="?", default="latest")
    p.add_argument("--from", dest="start", type=int, default=0)
    p.add_argument("--to", dest="end", type=int, default=1 << 30)
    args = ap.parse_args()
    if args.cmd is None:
        args = ap.parse_args(["report"])
    {"list": cmd_list, "report": cmd_report, "window": cmd_window, "event": cmd_event,
     "sheet": cmd_sheet, "replay": cmd_replay}[args.cmd](args)


if __name__ == "__main__":
    main()
