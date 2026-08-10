#!/usr/bin/env python3
"""Build one contact sheet per clip so a human can LOOK at the whole set.

WHY THIS EXISTS. Every automatic check in this project has, at some point,
passed something a person would have rejected on sight: a mirrored AprilTag, a
clip whose arms never moved, a gripper view that was black for 34 s. The
metrics are worth having and they are not a substitute for looking. This makes
looking cheap: six moments from a clip, side by side, at a size where a grasp
is legible.

The sheet is built from EVENLY SPACED times, not from interesting ones. Picking
frames near the grasp would flatter the clip -- the question a sheet answers is
"what does this look like throughout", and dead time should be visible as dead
time.

    python3 scripts/clip_contact_sheet.py --angle gripper [--mode 01_master_teleop]
"""
import argparse
import os
import re
import subprocess
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(WS, "recordings/verification")
FFMPEG = os.environ.get("FFMPEG") or os.path.expanduser("~/.local/bin/ffmpeg")
SHEETS = os.environ.get("SRL_SCRATCH", "/tmp")


def duration(mp4):
    r = subprocess.run([FFMPEG, "-i", mp4], capture_output=True, text=True)
    m = re.search(r"Duration:\s*(\d+):(\d+):(\d+\.\d+)", r.stderr or "")
    return int(m.group(2)) * 60 + float(m.group(3)) if m else 0.0


def sheet(mp4, dest, n=6):
    d = duration(mp4)
    if d < 1.0:
        return None
    tiles = []
    for i in range(n):
        t = d * (i + 0.5) / n
        p = os.path.join(SHEETS, "_t%d.png" % i)
        subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-ss", "%.2f" % t,
                        "-i", mp4, "-frames:v", "1", p], capture_output=True)
        if os.path.exists(p):
            tiles.append(p)
    if len(tiles) < 2:
        return None
    args = []
    for p in tiles:
        args += ["-i", p]
    half = (len(tiles) + 1) // 2
    fc = ("".join("[%d]" % i for i in range(half)) + "hstack=%d[a];" % half +
          "".join("[%d]" % i for i in range(half, len(tiles))) +
          "hstack=%d[b];" % (len(tiles) - half) +
          "[a][b]vstack=2,scale=2000:-1")
    subprocess.run([FFMPEG, "-y", "-loglevel", "error"] + args +
                   ["-filter_complex", fc, dest], capture_output=True)
    return dest if os.path.exists(dest) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angle", default="front")
    ap.add_argument("--mode", default=None)
    a = ap.parse_args()
    made = []
    for mode in sorted(os.listdir(OUT)):
        if not re.match(r"^\d\d_", mode) or (a.mode and mode != a.mode):
            continue
        md = os.path.join(OUT, mode)
        if not os.path.isdir(md):
            continue
        for task in sorted(os.listdir(md)):
            td = os.path.join(md, task)
            if not os.path.isdir(td):
                continue
            for scen in sorted(os.listdir(td)):
                mp4 = os.path.join(td, scen, "rviz_%s.mp4" % a.angle)
                if not os.path.exists(mp4):
                    continue
                dest = os.path.join(
                    SHEETS, "sheet_%s_%s_%s.png" % (mode[:2], task, a.angle))
                if sheet(mp4, dest):
                    made.append(dest)
                    print("  %s" % dest)
    print("\n%d sheets" % len(made))
    return 0 if made else 1


if __name__ == "__main__":
    sys.exit(main())
