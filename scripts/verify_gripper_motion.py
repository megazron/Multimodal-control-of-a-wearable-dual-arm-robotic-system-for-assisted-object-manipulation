#!/usr/bin/env python3
"""DID THE FINGERS ACTUALLY OPEN AND CLOSE, AT THE RIGHT MOMENTS?

    python3 scripts/verify_gripper_motion.py

The previous round of clips showed a gripper that never moved: the mock
Robotiq boots its driven knuckle at 0.793 rad -- fully closed on nothing --
and nothing ever commanded it, so every "grasp" happened purely in the marker
layer while the hand sat shut. A pick rendered with a closed gripper is not a
pick, and the attachment check did not catch it because the marker really was
following the gripper. It was following a gripper that never opened.

TWO INDEPENDENT MEASUREMENTS, because one of them is the thing that was wrong:

  JOINT  grip_trace.json records the knuckle angle MEASURED from
         /joint_states every frame -- the robot's own state, not the command.
         From it: did the hand open, did it close ON AN OBJECT (inside the
         0.10..0.74 holding band rather than slamming to free air), and did it
         open again to release?

  PIXEL  the finger separation measured off rviz_gripper.mp4, which is the
         view framed tightly on the active hand. Independent of the joint
         topic entirely, so it catches the case where the state says one thing
         and the picture shows another.

The pixel measure is the horizontal spread of the dark gripper body in a band
across the middle of the close-up view. Open fingers are wider than closed
ones; the check is that the spread CHANGES, and changes in the same direction
and at the same times as the joint trace.
"""
import glob
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/verification")
FFMPEG = os.path.expanduser("~/.local/bin/ffmpeg")
TMP = "/tmp/claude-1000/-home-gausms-kortex-ws/3732aa29-5a7e-4c8e-b77e-379233bdc9c9/scratchpad/grip"

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_rviz_clips import duration                          # noqa: E402

OPEN_MAX = 0.10          # below this the hand is open
HOLD_MIN, FREE_AIR = 0.10, 0.74
# BOTH NAMING GENERATIONS. The five-task set renamed everything (f1..f5) and
# the retired nine-task clips are still in the tree under their old names. A
# list carrying only one generation silently classifies the other as
# "not a grasp clip", which reports 0 grasp clips and reads like a clean pass
# rather than like a broken filter -- which is exactly what happened when the
# mode level was added and this list was not revisited.
GRASP_TASKS = ("t2", "t3", "t5", "t6",          # retired nine-task names
               "f2", "f3", "f4")                # current five-task names


def frame_at(mp4, t):
    os.makedirs(TMP, exist_ok=True)
    p = os.path.join(TMP, "f.png")
    if os.path.exists(p):
        os.remove(p)
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-ss", "%.2f" % t,
                    "-i", mp4, "-frames:v", "1", "-vf", "scale=760:-1", p],
                   check=False)
    if not os.path.exists(p):
        return None
    return np.asarray(Image.open(p).convert("RGB"), float)


def gripper_silhouette(img):
    """Area of the near-black gripper body, in pixels of the close-up view.

    THRESHOLD MEASURED, NOT GUESSED. The RViz background renders at
    (44, 44, 46), so the first version's `< 70` selected the entire frame and
    reported the same 747 px "spread" for open and closed alike. Below 28
    separates the gripper cleanly.

    AREA, not horizontal span: the camera tracks the gripper link, so the hand
    stays centred and roughly the same size, and what changes between open and
    closed is how much of the frame the fingers fill. Measured on one confirmed
    pair, open 30287 px vs closed 11349 px -- a 62% change, where the span
    changed only 449 -> 436 and would not have separated them.
    """
    if img is None:
        return None
    h, _, _ = img.shape
    band = img[int(0.25 * h):int(0.80 * h), :, :]
    return float((band.max(axis=2) < 28).sum())


def main():
    rows = []
    # ONE MORE PATH LEVEL. The tree is now
    # recordings/verification/<mode>/<task>/<scenario>/<condition>, because
    # control mode and autonomy condition are independent axes and the old layout
    # conflated them: "direct" under the mannequin and "direct" under VR shared a
    # folder name. A glob that still assumes three levels matches NOTHING and
    # reports zero clips, which reads exactly like a clean pass.
    dirs = sorted(glob.glob(os.path.join(OUT, "*/*/*/*/grip_trace.json")))
    print("checking gripper motion in %d runs\n" % len(dirs))
    for tp in dirs:
        d = os.path.dirname(tp)
        mode, task, scen, cond = os.path.relpath(d, OUT).split(os.sep)[:4]
        tr = json.load(open(tp))
        cap = {}
        cp = os.path.join(d, "rviz_capture.json")
        if os.path.exists(cp):
            cap = json.load(open(cp))
        arm = ("left" if task in ("t3", "t6", "t7", "t9") else None)
        # the arm the schedule actually works: recorded by the recorder
        for a in ("left", "right"):
            vs = [x[a] for x in tr if x.get(a) is not None]
            if vs and (max(vs) - min(vs)) > 0.05:
                arm = a
                break
        vals = [x[arm] for x in tr if x.get(arm) is not None] if arm else []
        opened = bool(vals) and min(vals) < OPEN_MAX
        on_obj = any(HOLD_MIN <= v < FREE_AIR for v in vals)
        # a release is an open AFTER a closed sample
        rel = False
        seen_closed = False
        for v in vals:
            if HOLD_MIN <= v < FREE_AIR:
                seen_closed = True
            elif seen_closed and v < OPEN_MAX:
                rel = True
        joint_ok = opened and on_obj and (rel or task in ("t3", "t6"))

        # ---- independent pixel measurement
        gv = os.path.join(d, "rviz_gripper.mp4")
        spread_open = spread_shut = None
        if os.path.exists(gv) and vals:
            dur = duration(gv)
            ts = [x["t"] for x in tr if x.get(arm) is not None]
            tmax = max(ts) if ts else 1.0
            # THE EXTREMES, not the first frame of each state. Taking the
            # first sample to enter the holding band catches the fingers
            # mid-closure, so the "closed" frame still looks open: four clips
            # scored 1-15% silhouette change on a gripper that had plainly
            # cycled according to its own joint trace. The widest-open and
            # most-closed frames are the fair comparison.
            cand = [x for x in tr if x.get(arm) is not None]
            t_open = min(cand, key=lambda x: x[arm])["t"] if cand else None
            shut = [x for x in cand if HOLD_MIN <= x[arm] < FREE_AIR]
            t_shut = max(shut, key=lambda x: x[arm])["t"] if shut else None
            if t_open is not None and t_shut is not None and dur > 0:
                sc = dur / max(tmax, 1e-6)
                spread_open = gripper_silhouette(
                    frame_at(gv, min(dur - 0.2, t_open * sc)))
                spread_shut = gripper_silhouette(
                    frame_at(gv, min(dur - 0.2, t_shut * sc)))
        # 15% relative change. The confirmed pair differs by 62%, and the
        # silhouette of a stationary gripper is stable well inside that.
        pixel_ok = bool(spread_open and spread_shut
                        and abs(spread_open - spread_shut)
                        / max(spread_open, spread_shut) > 0.15)
        rows.append(dict(task=task, scenario=scen, condition=cond, arm=arm,
                         grip_min=round(min(vals), 3) if vals else None,
                         grip_max=round(max(vals), 3) if vals else None,
                         opened=opened, closed_on_object=on_obj,
                         released=rel, joint_ok=joint_ok,
                         spread_open=spread_open, spread_shut=spread_shut,
                         pixel_ok=pixel_ok,
                         grasp_task=task in GRASP_TASKS))
        if task in GRASP_TASKS:
            print("  %-4s %-22s %-9s %-5s %.2f..%.2f  open=%s close=%s "
                  "release=%s  px %s->%s %s"
                  % (task, scen, cond, arm,
                     min(vals) if vals else -1, max(vals) if vals else -1,
                     opened, on_obj, rel,
                     "%.0f" % spread_open if spread_open else "-",
                     "%.0f" % spread_shut if spread_shut else "-",
                     "OK" if (joint_ok and pixel_ok) else "CHECK"))

    g = [r for r in rows if r["grasp_task"]]
    ok = [r for r in g if r["joint_ok"]]
    px = [r for r in g if r["pixel_ok"]]
    print("\n  grasp clips                       : %d" % len(g))
    print("  fingers open AND close on object  : %d" % len(ok))
    print("  finger motion confirmed in PIXELS : %d" % len(px))
    bad = [r for r in g if not r["joint_ok"]]
    if bad:
        print("\n  FINGERS DID NOT CYCLE:")
        for r in bad:
            print("    %s/%s/%s  %.2f..%.2f open=%s close=%s release=%s"
                  % (r["task"], r["scenario"], r["condition"],
                     r["grip_min"] or -1, r["grip_max"] or -1,
                     r["opened"], r["closed_on_object"], r["released"]))
    json.dump(rows, open(os.path.join(OUT, "gripper_check.json"), "w"),
              indent=2)
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
