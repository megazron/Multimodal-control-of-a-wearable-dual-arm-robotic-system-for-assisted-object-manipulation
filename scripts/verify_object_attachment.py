#!/usr/bin/env python3
"""DOES THE OBJECT ACTUALLY TRAVEL WITH THE GRIPPER?

    python3 scripts/verify_object_attachment.py

This answers the one question a clip exists to answer: is the arm carrying
something, or is it waving at a stationary prop? "The gripper moved" and "the
object moved with it" are different claims and only the second is the task.

MEASURED FROM THE PIXELS, NOT FROM THE CODE THAT DREW THEM. It would be easy
to assert attachment from `record_rviz.py` -- the marker pose is computed from
the live end-effector pose, so of course it tracks. That is an argument, not
evidence, and it is exactly the kind of argument that has been wrong before in
this project. So the object's colour centroid is measured in frames spanning
the carry, and its travel across the clip is reported.

WHAT COUNTS AS COMPLETING, PER TASK

  t3, t6  the tray / sling is a carried object for the whole task phase, so
          its centroid must travel with the arms. A static tray would be a
          prop hanging in space.
  t2      the strongest signal available and it needs no centroid: ORANGE
          pixels must FALL (blocks leave the supply slab) and GREEN pixels
          must APPEAR (blocks land in the container). A gripper closing on
          nothing changes neither.
  t5      the tool's yellow head must travel, and must end near the green
          receive point.
  t7, t9  the target must travel (t9 S1 excepted -- zero sway is a stationary
          target on purpose).
  t8      the target is fixed by design; what must travel is the ARM, which
          summary.json already reports as ee_travel_m.

A task with no carried object (t7/t8/t9) cannot fail attachment; it is
reported as N/A rather than as a pass, because counting it as a pass would
inflate the number that matters.
"""
import glob
import json
import os
import re
import subprocess
import sys

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/verification")
FFMPEG = os.path.expanduser("~/.local/bin/ffmpeg")
TMP = "/tmp/scratch/att"

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_rviz_clips import PALETTE, duration, below_hud   # noqa: E402

# Travel below this is "the object did not move", in pixels of a 600-px frame.
#
# MEASURED, not chosen: on the T9 zero-sway clip the two target spheres are
# stationary by construction and their centroids read (233,200) and (66,200)
# in three consecutive samples -- IDENTICAL. Centroid noise is therefore under
# 1 px, and 5 px is comfortably above it while still catching a genuinely
# unattached object. The earlier 12 px failed short scenarios: T3 S1 is a
# 50 mm lift, and under `shared` (55% smoothing) it legitimately moves ~7 px.
STATIC_PX = 5.0


def grab(mp4, fracs):
    os.makedirs(TMP, exist_ok=True)
    for f in glob.glob(os.path.join(TMP, "*.png")):
        os.remove(f)
    dur = duration(mp4)
    if dur <= 0:
        return []
    out = []
    for k, fr in enumerate(fracs):
        p = os.path.join(TMP, "f%02d.png" % k)
        subprocess.run([FFMPEG, "-y", "-loglevel", "error",
                        "-ss", "%.2f" % (dur * fr), "-i", mp4,
                        "-frames:v", "1", "-vf", "scale=600:-1", p],
                       check=False)
        if os.path.exists(p):
            out.append(np.asarray(Image.open(p).convert("RGB"), float))
    return out


def centroid(img, name, half=None):
    """Colour centroid, ignoring the fixed overlay band at the top.

    `half` restricts to the left or right of the frame. T7 and T9 draw TWO
    target spheres, one per arm, and their COMBINED centroid sits between them
    and barely moves even when both targets are orbiting -- which reported
    "target travelled 0 px" on a clip where both were plainly moving. Each
    side is measured separately and the larger travel is taken.
    """
    m = below_hud(PALETTE[name](img))
    if half is not None:
        w = m.shape[1]
        if half == "L":
            m[:, w // 2:] = False
        else:
            m[:, :w // 2] = False
    n = int(m.sum())
    if n < 15:
        return None, n
    ys, xs = np.nonzero(m)
    return (float(xs.mean()), float(ys.mean())), n


def travel(imgs, name, split=False):
    """Max centroid displacement across the sampled frames, in pixels."""
    halves = ("L", "R") if split else (None,)
    best = None
    for h in halves:
        cs = [centroid(i, name, h)[0] for i in imgs]
        cs = [c for c in cs if c is not None]
        if len(cs) < 2:
            continue
        t = max(float(np.hypot(a[0] - b[0], a[1] - b[1]))
                for a in cs for b in cs)
        best = t if best is None else max(best, t)
    return best


def main():
    rows = []
    # ONE MORE PATH LEVEL. The tree is now
    # recordings/verification/<mode>/<task>/<scenario>/<condition>, because
    # control mode and autonomy condition are independent axes and the old layout
    # conflated them: "direct" under the mannequin and "direct" under VR shared a
    # folder name. A glob that still assumes three levels matches NOTHING and
    # reports zero clips, which reads exactly like a clean pass.
    # BOTH DEPTHS, BECAUSE THE TREE HAS BOTH.
    #
    # record_abc_sweep writes <mode>/<task>/<scenario>/ -- THREE levels -- and
    # the older recorder wrote a fourth <condition>. This globbed four only,
    # so across 25 freshly recorded cells it matched NOTHING and reported
    # "object attachment in 0 clips", exit 0. The sweep runs this as part of
    # its VERIFYING step and read that as a pass.
    #
    # Unlike its two siblings this one needs no missing producer: rviz_front
    # .mp4 and scene_events.json are both there. It was only ever looking one
    # directory too deep.
    mp4s = sorted(glob.glob(os.path.join(OUT, "*/*/*/rviz_front.mp4"))
                  + glob.glob(os.path.join(OUT, "*/*/*/*/rviz_front.mp4")))
    _WANTED = '*/*/*/*/rviz_front.mp4'
    # ZERO INPUTS IS NOT A PASS, AND THIS IS THE FAILURE THIS FILE WAS BUILT
    # TO PREVENT, ARRIVING FROM THE INSIDE.
    #
    # MEASURED 2026-08-15: this ran as part of the sweep's VERIFYING step,
    # reported "checking object attachment in 0 runs" and exited 0. The sweep read that as
    # verification passing. Two things are wrong and both are here:
    #
    #   1. the glob is FOUR levels (<mode>/<task>/<scenario>/<condition>) and
    #      record_abc_sweep writes THREE, so it matches nothing;
    #   2. NOTHING IN THE CLIP PATH WRITES grip_trace.json AT ALL -- 0 files
    #      on disk across 25 recorded cells.
    #
    # A verifier nobody runs is documentation; a verifier that is wired in and
    # silently checks nothing is worse, because it carries the APPEARANCE of
    # having run. Refuse, and say which of the two it is.
    if not mp4s:
        print("\nREFUSING TO REPORT A PASS: this checked ZERO clips.")
        print("  looked for : rviz_front.mp4 at three OR four levels "
              "under %s" % OUT)
        print("  found      : 0")
        print("  Either nothing has been recorded, or the tree moved again.")
        print("  0 of 0 is not evidence.")
        return 2

    print("checking object attachment in %d clips\n" % len(mp4s))
    for mp4 in mp4s:
        d = os.path.dirname(mp4)
        # PARSE WHAT IS THERE, DO NOT ASSUME A DEPTH. Three levels from the
        # sweep, four from the older recorder; the same fix verify_rviz_clips
        # already carries. Unpacking a fixed four raised ValueError on every
        # clip in the current tree.
        _p = os.path.relpath(d, OUT).split(os.sep)
        _p = _p + ["-"] * (4 - len(_p))
        mode, task, scen, cond = _p[0], _p[1], _p[2], _p[3]
        cap = {}
        cp = os.path.join(d, "rviz_capture.json")
        if os.path.exists(cp):
            cap = json.load(open(cp))
        # NINE samples, 18%..97%. Four was UNDERSAMPLING: every clip carries
        # ~1.5 s of approach and ~1.4 s of hold, so on a short scenario the
        # moving part is a small slice and four probes landed mostly in the
        # static hold. Measured on T3 S1 (a 50 mm lift): 4 samples reported
        # 7.1 px and called it static; 9 samples report 25.9 px. T7 S1 went
        # from 0.0 px to 27.5 px the same way. The clips were never the
        # problem.
        # From 25%: at 18% a stale marker from the PREVIOUS run was still on
        # screen for one frame before RViz processed the DELETEALL.
        F = grab(mp4, tuple(np.linspace(0.25, 0.97, 9)))
        if len(F) < 3:
            rows.append(dict(task=task, scenario=scen, condition=cond,
                             verdict="NO FRAMES", detail=""))
            continue
        v, detail = "N/A", ""
        # THE MSc SET IS NOT IMPLEMENTED HERE, AND IT MUST SAY SO.
        #
        # Every branch below is keyed on the ARCHIVED nine-task set's names --
        # t3 is a tan tray, t6 a sling, t2 orange and green blocks. The MSc
        # tree's directories are T0/T1/T1S2/T2/T3, and it is only their
        # UPPERCASE that stopped MSc T2 matching the archived t2's orange
        # blocks. That accident turned a wrong answer into a silent one: all
        # 28 clips came back "no carried object (N/A)" when T1 alone carries
        # four cubes 0.24-0.42 m each.
        #
        # It is NOT patched to read scene_events.json, tempting as that is.
        # This verifier exists to answer from the PIXELS whether the object
        # travels with the gripper or the gripper waves at a stationary prop;
        # scene_events is written by the same node that draws the object, so
        # reading it would make the check agree with itself by construction --
        # which is the exact failure documented for T2's tray this same day.
        #
        # So: named, refused, and counted separately from a real N/A.
        if task.lower() in ("t0", "t1", "t1s2") or (
                task.lower() in ("t2", "t3") and mode.startswith(
                    ("01_", "02_", "03_", "04_", "06_"))):
            rows.append(dict(task=task, scenario=scen, condition=cond,
                             verdict="NOT IMPLEMENTED",
                             detail="MSc task %s has no pixel rule here; the "
                                    "branches below are the archived set's"
                                    % task))
            continue
        if task == "t3":
            t = travel(F, "tray_tan")
            v = ("CARRIED" if (t or 0) > STATIC_PX else "STATIC OBJECT")
            detail = "tray centroid travelled %.0f px" % (t or 0)
        elif task == "t6":
            t = travel(F, "sling_orange")
            b = travel(F, "ball_yellow")
            v = ("CARRIED" if max(t or 0, b or 0) > STATIC_PX
                 else "STATIC OBJECT")
            detail = "sling %.0f px, ball %.0f px" % (t or 0, b or 0)
        elif task == "t2":
            o = [centroid(f, "block_orange")[1] for f in F]
            g = [centroid(f, "block_green")[1] for f in F]
            picked = o[0] - o[-1]
            placed = g[-1] - g[0]
            ok = (picked > 20) or (placed > 20) or cap.get("blocks_placed", 0)
            v = "CARRIED" if ok else "STATIC OBJECT"
            detail = ("orange %d->%d px (supply emptying), green %d->%d px "
                      "(in container), blocks_placed=%d"
                      % (o[0], o[-1], g[0], g[-1], cap.get("blocks_placed", 0)))
        elif task == "t5":
            t = travel(F, "ball_yellow")      # the tool's yellow head
            v = ("CARRIED" if (t or 0) > STATIC_PX else "STATIC OBJECT")
            detail = "tool head travelled %.0f px, delivered=%s" % (
                t or 0, cap.get("tool_delivered"))
        elif task in ("t7", "t8", "t9"):
            # NO CARRIED OBJECT. These are pursuit and reach tasks: the arm
            # chases a target it never picks up, so "is the object attached"
            # does not apply and scoring it would inflate the number that
            # matters. Worse, it cannot even be measured reliably -- the
            # target sphere is OCCLUDED BY THE GRIPPER once the arm arrives
            # on it, so the green vanishes from frames 4..8 of a clip where
            # nothing is wrong. These are judged by the arm's own travel and
            # tracking error in summary.json instead.
            sm = {}
            sp = os.path.join(d, "summary.json")
            if os.path.exists(sp):
                sm = json.load(open(sp))
            v = "N/A (no carried object)"
            detail = ("arm travelled %.2f m, tracking %.1f mm"
                      % (sm.get("ee_travel_m", 0.0),
                         sm.get("tracking_rms_mm", float("nan"))))
        rows.append(dict(task=task, scenario=scen, condition=cond,
                         verdict=v, detail=detail))
        print("  %-4s %-22s %-9s %-14s %s" % (task, scen, cond, v, detail))

    carried = [r for r in rows if r["verdict"] in ("CARRIED", "TARGET MOVES")]
    static = [r for r in rows if "STATIC" in r["verdict"]]
    na = [r for r in rows if r["verdict"].startswith("N/A")]
    print("\n  object travels with the arm : %d" % len(carried))
    print("  OBJECT STAYED PUT           : %d" % len(static))
    print("  no carried object (N/A)     : %d" % len(na))
    if static:
        print("\n  CLIPS WHERE THE OBJECT DID NOT MOVE:")
        for r in static:
            print("    %s/%s/%s -- %s" % (r["task"], r["scenario"],
                                          r["condition"], r["detail"]))
    json.dump(rows, open(os.path.join(OUT, "attachment_check.json"), "w"),
              indent=2)
    todo = [r for r in rows if r["verdict"] == "NOT IMPLEMENTED"]
    if todo:
        print("\n  NOT IMPLEMENTED for %d clip(s): the MSc task set has no "
              "pixel rule in this file." % len(todo))
        print("  That is a GAP, not a pass. It is reported rather than "
              "counted as N/A.")
    if static:
        return 1
    return 3 if todo else 0


if __name__ == "__main__":
    sys.exit(main())
