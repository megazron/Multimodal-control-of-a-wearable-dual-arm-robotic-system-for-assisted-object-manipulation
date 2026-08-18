#!/usr/bin/env python3
"""STAGE THE LOOK: detect the cubes BEFORE the mode starts, and write them out.

    python3 scripts/stage_observe_and_detect.py --arm left --out /tmp/cubes.json

WHY THIS IS A SEPARATE STEP AND NOT PART OF THE RUN.

`vision_grasp.observe_and_detect()` commands the observe pose by publishing a
JointTrajectory to /<arm>_arm_controller/joint_trajectory. That is the topic
`ik_follower_node` publishes to (ik_follower_node.py:260). Under a control
mode the follower OWNS that topic, so a look performed inside the run puts two
publishers on the arm controller -- the project's one-source-at-a-time rule,
at the process level.

It is not theoretical. Measured, recording T1 through the sweep with the look
embedded in `run_abc`:

    through the sweep      8.90 m of travel, placed 101 mm from target,
                           NO GRASP RECORDED AT ALL
    standalone, with       0.0000 m of travel -- the follower was tracking
    master_pose_node live  master_pose_node, not the runner

So the look moves to where every other pre-run arm motion already lives: the
STAGING phase, alongside `stage_presentation_pose.py`, which the sweep runs as
a subprocess before it presses the GUI button precisely because staging must
not happen inside the mode. This script looks, detects, returns the arm to
home, and writes the cubes to a file. `run_abc --vision` then READS that file
and builds the path from it -- no camera work, no arm motion, nothing
competing with the follower.

CONTROLS, and there is no file written without them:

    the arm returns HOME        the mode starts from home or the run is not
                               the run it is named after
    the expected cube count     seeing three cubes and picking three is a
                               silent half-task; it refuses instead
    the file records WHICH      a stale file from another layout is worse than
    layout it saw              none, so the detections carry their own stamp
"""
import argparse
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "config"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default=None)
    ap.add_argument("--task", default="t1", choices=("t1", "t1s2"),
                    help="which stage. Stage 2 draws its cubes from the seed, "
                         "so the expected count, the layout stamp and the "
                         "scene the camera is looking at all follow from it.")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--expect", type=int, default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--settle-s", type=float, default=6.0)
    a = ap.parse_args()

    import rclpy
    import msc_clip_tasks as M
    import home_positions as hp
    from vision_grasp import observe_and_detect, DetectionUnavailable

    # WHICH ARM LOOKS. T1 is a two-arm task since the 2026-08-17 rebuild, so
    # there is no `T1_ARM` any more and the looking arm is a separate question
    # from the working arm. ONE look serves the whole task: the observe pose
    # is solved against all four cubes and both pads
    # (`scripts/solve_observe_pose.py`), so whichever arm can see all six does
    # the looking and both arms then work from what it saw.
    import t1_task as T1M
    arm = a.arm or T1M.LOOK_ARM
    layout = (T1M.stage2_layout(a.seed) if a.task == "t1s2"
              else [(cx, cy, T1M.T1_PAIR[i])
                    for i, (cx, cy) in enumerate(T1M.T1_CUBES)])
    expect = a.expect if a.expect is not None else len(layout)

    rclpy.init()
    try:
        cubes, info = observe_and_detect(arm, expect=expect,
                                         settle_s=a.settle_s)
    except DetectionUnavailable as e:
        print("STAGE-DETECT REFUSED: %s" % e)
        return 5
    finally:
        try:
            rclpy.shutdown()
        except Exception:                                        # noqa: BLE001
            pass

    # CONTROL: the arm is back at home, because the mode starts from there.
    import numpy as np
    import rclpy as _r
    _r.init()
    from verify_task_scenes import Solver
    n = Solver()
    n.spin(4.0)
    off, j = n.home_ok(arm)
    n.destroy_node()
    _r.shutdown()
    if off > 0.05:
        print("STAGE-DETECT REFUSED: the %s arm is %.4f rad from home at "
              "joint_%d after the look. The mode would start from the wrong "
              "place." % (arm, off, j))
        return 6

    # CONTROL: EVERY DETECTION IS WHERE A CUBE IS, not merely four of them.
    #
    # THE COUNT ALONE IS SATISFIABLE BY THE WRONG FOUR BLOBS, and it was.
    # Measured 2026-08-18 on the first observe pose solved for this layout:
    # four detections, count correct, refusal silent -- and one of them was a
    # fragment of the BLUE PAD at x = 0.312 while the real cube at x = -0.480
    # had been rejected as "not cube-sized at its own range". The pose was
    # looking across the row from 0.335 m on the near side to 1.17 m on the
    # far one, and at that range ratio a piece of pad near the camera is the
    # same number of pixels as a cube far from it.
    #
    # IT IS A POSITION CHECK AND DELIBERATELY NOT A COLOUR ONE. The mislabel
    # control changes what a cube LOOKS like, never where it is, so comparing
    # against the declared positions cannot weaken the claim that vision
    # drives the grasp -- and `verify_vision_drives_grasp` still flips the
    # declaration and requires the plan not to move. Checking the colour here
    # would be exactly the circularity that control exists to break.
    #
    # The tolerance is the task's own capture gate: a detection further than
    # this from the cube it is supposed to be is a detection the hand would
    # close short of anyway.
    TOL_M = 0.030
    want = [(float(c[0]), float(c[1])) for c in layout]
    unmatched, worst = list(want), 0.0
    for cx, cy, _pad in cubes:
        if not unmatched:
            break
        j = min(range(len(unmatched)),
                key=lambda k: math.dist((cx, cy), unmatched[k]))
        d = math.dist((cx, cy), unmatched[j])
        worst = max(worst, d)
        if d <= TOL_M:
            unmatched.pop(j)
    if unmatched:
        print("STAGE-DETECT REFUSED: %d detection(s) are not at a cube. "
              "Worst match %.1f mm against a %.0f mm gate; unmatched cubes %s. "
              "Picking from this would pick a piece of scenery."
              % (len(unmatched), worst * 1000.0, TOL_M * 1000.0,
                 [[round(v, 3) for v in u] for u in unmatched]))
        for c in cubes:
            print("      saw (%+.4f, %+.4f) -> %s"
                  % (c[0], c[1], M.PLANE_COLOURS[int(c[2])]))
        return 7

    rec = dict(arm=arm, cubes=cubes, timing=info,
               worst_match_mm=round(worst * 1000.0, 2),
               returned_home_worst_rad=round(float(off), 5),
               task=a.task, seed=a.seed,
               layout=dict(T1_CUBES=[list(c[:2]) for c in layout],
                           T1_PLANES=[list(p) for p in T1M.T1_PLANES],
                           T1_Z=T1M.T1_Z),
               stamp=time.time())
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(rec, open(a.out, "w"), indent=2, default=float)
    print("[stage-detect] %d cubes SEEN -> %s" % (len(cubes), cubes))
    print("[stage-detect] added %.2f s, home to %.4f rad -> %s"
          % (info["added_total_s"], off, a.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
