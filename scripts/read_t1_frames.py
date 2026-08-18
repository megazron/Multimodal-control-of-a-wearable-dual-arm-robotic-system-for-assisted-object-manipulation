#!/usr/bin/env python3
"""PULL FRAMES OUT OF A T1 CLIP SO THEY CAN BE LOOKED AT.

    python3 scripts/read_t1_frames.py recordings/verification/06_full_autonomy/T1/S1_both_arms_centre

The brief for this task ends "the frame is the evidence", and every automatic
check in this repo has at some point passed a clip that was visibly wrong --
the render check that filed a scene with no table in it at 18% ink is the
worst example. So this does not judge anything. It extracts frames at the
moments that matter and prints where they are, and a person (or a model that
can see) looks at them.

WHICH MOMENTS. Not evenly spaced: evenly spaced frames of a 30 s clip are
mostly the arm traversing, which is the part nothing goes wrong in. The
interesting instants are named by the clip's OWN event log --
scene_events.json carries a wall clock on every GRASPED and RELEASED -- so
the frames come out at the card, the observe pose, each grasp, each release,
and the final state.
"""
import json
import math
import os
import subprocess
import sys


def probe_duration(mp4):
    """Seconds, read out of ffmpeg's own banner.

    THIS BOX HAS NO ffprobe -- only ffmpeg, in ~/.local/bin -- and assuming
    the pair travel together is how a frame grab silently produces nothing.
    ffmpeg with no output file prints the duration to stderr and exits
    non-zero, which is expected and is not an error here.
    """
    try:
        txt = subprocess.run(["ffmpeg", "-i", mp4],
                             capture_output=True, text=True).stderr
    except Exception:                                         # noqa: BLE001
        return None
    for line in txt.splitlines():
        if "Duration:" in line:
            hms = line.split("Duration:")[1].split(",")[0].strip()
            try:
                h, m, sec = hms.split(":")
                return int(h) * 3600 + int(m) * 60 + float(sec)
            except ValueError:
                return None
    return None


def grab(mp4, t, dest):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", "%.2f" % t,
                    "-i", mp4, "-frames:v", "1", dest], check=False)
    return os.path.exists(dest) and os.path.getsize(dest) > 0


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    d = sys.argv[1].rstrip("/")
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.environ.get("SRL_SCRATCH", "/tmp"), "t1_frames",
        os.path.basename(os.path.dirname(d)) + "_" + os.path.basename(d))
    os.makedirs(out, exist_ok=True)

    ev_path = os.path.join(d, "scene_events.json")
    ev = {}
    if os.path.exists(ev_path):
        ev = json.load(open(ev_path))

    # WHEN did each thing happen, in SECONDS FROM THE START OF THE VIDEO.
    # scene_events stamps wall time; the clip's own t0_wall is the only thing
    # that makes the two clocks comparable, and getting this wrong is how a
    # GRASPED at t=44.1 s once landed inside a 29.1 s clip.
    t0 = ev.get("t0_wall")
    moments = [("00_opening", 0.30)]
    if t0:
        for e in ev.get("events", []):
            if not e.get("wall"):
                continue
            off = e["wall"] - t0
            moments.append(("%s_%s_%s" % (e["ev"].lower(), e.get("item", "?"),
                                          e.get("arm", "?")), off))

    made = []
    for view in ("front", "quad", "iso", "gripper", "top"):
        mp4 = os.path.join(d, "rviz_%s.mp4" % view)
        if not os.path.exists(mp4):
            continue
        dur = probe_duration(mp4) or 0.0
        # THE CARD IS THE FIRST SECONDS OF THE FRONT VIEW and the footage
        # follows it, so the opening frame of `front` is the card and the
        # opening frame of every other view is the scene.
        want = list(moments) + [("99_final", max(0.0, dur - 0.40))]
        for name, t in want:
            if t is None or t < 0 or t > dur:
                continue
            dest = os.path.join(out, "%s__%s.png" % (view, name))
            if grab(mp4, t, dest):
                made.append(dest)
        print("%-8s %5.1f s  ->  %d frames" % (view, dur, len(made)))

    print("\n%d frames in %s" % (len(made), out))
    for p in sorted(made):
        print("   " + p)
    return 0 if made else 1


if __name__ == "__main__":
    sys.exit(main())
