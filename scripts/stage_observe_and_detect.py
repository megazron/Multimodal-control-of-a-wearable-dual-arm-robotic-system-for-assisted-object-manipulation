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
    layout = (T1M.stage2_layout(a.seed) if a.task == "t1s2"
              else [(cx, cy, T1M.T1_PAIR[i])
                    for i, (cx, cy) in enumerate(T1M.T1_CUBES)])

    # ONE LOOK PER ARM, EACH AT ITS OWN SIDE. NOT ONE LOOK FOR THE WHOLE ROW.
    #
    # "One look serves the whole task" was true of a layout whose objects sat
    # in a 300 mm patch beside the person. This one spans 1.08 m of table with
    # a pad pair in the middle, and MEASURED on 2026-08-18 there is NO pose
    # from which one arm sees all twelve columns within 1.05 m: the solver
    # returns nothing at all.
    #
    # WHAT THE OVER-LONG POSE COST. The only poses that saw everything spanned
    # 0.62 to 1.34 m, and a 40 mm cube at 1.34 m is 18 px across while the
    # same-coloured pad 60 mm behind it is 200. Three runs detected all four
    # cubes; the fourth merged the far green cube with its pad and returned it
    # as a 55 px blob against an expected 20.2, which the size gate rejected --
    # correctly -- and the clip was skipped for want of a fourth cube.
    #
    # Per arm, each side is 0.46 to 0.75 m away: a 1.6x span rather than 2.2x,
    # and every cube 26 px or more. The looking arm is the arm that will do
    # the work, which is also the honest arrangement -- an arm that cannot
    # reach a cube has no business being the authority on where it is.
    arms = [a.arm] if a.arm else ["left", "right"]
    per_arm = {"left": [c for c in layout if c[0] >= 0.0],
               "right": [c for c in layout if c[0] < 0.0]}

    rclpy.init()
    cubes, info = [], {"per_arm": {}}
    try:
        for _arm in arms:
            want = (a.expect if a.expect is not None
                    else len(per_arm[_arm]))
            if want == 0:
                continue
            # EACH ARM ANSWERS FOR ITS OWN SIDE, AND ONLY ITS OWN SIDE.
            #
            # The camera sees whatever is in frame, which from either observe
            # pose is most of the table -- so counting everything it returns
            # against "how many cubes are on this arm's side" refuses a
            # perfectly good look. Measured: the left arm's frame returned the
            # far green cube at 1.31 m alongside its own two, and the count
            # came back 5 against an expected 2.
            #
            # The SIDE is not a colour and not a declared position: it is
            # which half of the centreline the detection deprojected to, and
            # the arm that can reach it. `expect` is checked after the split,
            # so a missing cube on this side is still a refusal.
            got, t = observe_and_detect(_arm, expect=None,
                                        settle_s=a.settle_s)
            mine = [c for c in got if (c[0] >= 0.0) == (_arm == "left")]
            if len(mine) != want:
                raise DetectionUnavailable(
                    "the %s arm saw %d cubes on its own side and the task "
                    "puts %d there. Refusing to pick blind.\n    it "
                    "returned: %s"
                    % (_arm, len(mine), want,
                       ", ".join("(%+.3f, %+.3f)" % (c[0], c[1])
                                 for c in got)))
            got = mine
            cubes.extend(got)
            info["per_arm"][_arm] = t
            for k, v in t.items():
                if k.endswith("_s") and isinstance(v, (int, float)):
                    info[k] = round(info.get(k, 0.0) + v, 2)
            info.setdefault("seen", []).extend(t.get("seen") or [])
    except DetectionUnavailable as e:
        print("STAGE-DETECT REFUSED: %s" % e)
        return 5
    finally:
        try:
            rclpy.shutdown()
        except Exception:                                        # noqa: BLE001
            pass
    cubes.sort(key=lambda c: c[0])
    info["added_total_s"] = round(sum(v for k, v in info.items()
                                      if k.endswith("_s")
                                      and isinstance(v, (int, float))), 2)
    info["added_per_pick_s"] = (round(info["added_total_s"] / len(cubes), 2)
                                if cubes else None)
    arm = arms[-1]

    # CONTROL: EVERY arm that looked is back at home, because the mode starts
    # from there. It used to check one, which is the arm that happened to look
    # last.
    import rclpy as _r
    _r.init()
    from verify_task_scenes import Solver
    n = Solver()
    n.spin(4.0)
    worst_home = []
    for _arm in arms:
        off, j = n.home_ok(_arm)
        worst_home.append((off, j, _arm))
    n.destroy_node()
    _r.shutdown()
    off, j, arm = max(worst_home)
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

    rec = dict(arm=arm, arms=list(arms), cubes=cubes, timing=info,
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
