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
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "config"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default=None)
    ap.add_argument("--expect", type=int, default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--settle-s", type=float, default=6.0)
    a = ap.parse_args()

    import rclpy
    import msc_clip_tasks as M
    import home_positions as hp
    from vision_grasp import observe_and_detect, DetectionUnavailable

    arm = a.arm or M.T1_ARM
    expect = a.expect if a.expect is not None else len(M.T1_CUBES)

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

    rec = dict(arm=arm, cubes=cubes, timing=info,
               returned_home_worst_rad=round(float(off), 5),
               layout=dict(T1_CUBES=[list(c) for c in M.T1_CUBES],
                           T1_PLANES=[list(p) for p in M.T1_PLANES],
                           T1_Z=M.T1_Z),
               stamp=time.time())
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(rec, open(a.out, "w"), indent=2, default=float)
    print("[stage-detect] %d cubes SEEN -> %s" % (len(cubes), cubes))
    print("[stage-detect] added %.2f s, home to %.4f rad -> %s"
          % (info["added_total_s"], off, a.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
