#!/usr/bin/env python3
"""Pick the green cube with the LEFT arm, start to finish, with no hand-solving.

    bash scripts/bringup_arm.sh left          # once, brings up bridge + camera
    python3 scripts/srl_pick_cube.py          # then this, as often as you like
    python3 scripts/srl_pick_cube.py --dry-run

It finds its own viewpoint, measures the cube, servos onto it, closes, lifts,
and says whether the cube actually came up.  Nothing here depends on a pose
somebody saved earlier.

WHAT THIS PIPELINE KNOWS THAT THE EARLIER ATTEMPTS DID NOT
---------------------------------------------------------
1. THE CAMERA EXTRINSIC IS WRONG BY ~8.5 deg, AND IT ROTATES WITH THE WRIST.
   The table cannot move, yet the table normal measured from four different
   arm poses spanned 8.45 deg -- 34 mm of position error at 0.23 m, 59 mm at
   0.40 m.  So the grasp is NOT planned open-loop through that transform.  The
   pad-to-cube error is measured in the CAMERA's own frame, where the data is
   good (0.54 mm plane RMS, a 40 mm cube measured at 40.9 mm), and nulled by
   iteration.  Closed loop converges through a wrong extrinsic; open loop
   cannot, and did not.

2. THE PAD MIDPOINT MOVES WHEN THE HAND CLOSES.  The Robotiq's fingers swing
   on a four-bar: wrist-to-pad is 0.09833 m wide open and 0.10976 m closed on
   a 40 mm cube.  Solving the grasp with the hand OPEN and then closing drives
   the pads 11.43 mm further along the tool axis -- into the table.  Every
   pose here is solved at CUBE_GRIP, the opening the hand will actually hold
   the cube at.

3. THE WRIST CAMERA CANNOT SEE ITS OWN GRASP.  Closing the last ~80 mm pushes
   the cube out of the bottom of the frame.  That is geometry, not failure:
   the last good fix is carried in WORLD, where the cube does not move, and
   the short remainder is completed from it.

4. JOINTS 3, 5 AND 7 ARE CONTINUOUS.  -179.05 deg and +181.02 deg are the SAME
   pose; subtracting them gives 360.08 deg.  That "error" aborted a run that
   had actually ARRIVED.  All joint arithmetic goes through ang_wrap.

5. A PUBLISH LOOP STARVES ITS OWN SUBSCRIPTION.  Spinning with timeout 0 while
   publishing at 20 Hz let the cached joint vector go seconds stale; a motion
   check then read 0.032 deg of movement on a move that was really running and
   aborted it.  Motion is judged only through fresh_q().

6. ARRIVAL IS WAITED FOR.  The bridge closes error proportionally (kp = 0.5,
   ~2 s time constant), so a fixed 2.5 s settle leaves a third of the error
   standing and reports a miss.

7. PATHS ARE CHECKED AGAINST THE TABLE BEFORE THEY ARE COMMANDED, over the
   whole hand, because a straight line in joint space says nothing about where
   the hand goes -- that is how the gripper was driven into the table.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from auto_observe import find_cube, plausible  # noqa: E402
from guarded_descent import Blind, descend  # noqa: E402
from execute_pick_left import ang_wrap  # noqa: E402
from plan_pick_left import ik  # noqa: E402
from safe_goto_left import TableCheck  # noqa: E402
from servo_pick_left import (CUBE_GRIP, Eye, GRIP_OPEN, GRIP_SQUEEZE,  # noqa: E402
                             MAX_STEP_M, MIN_CLEAR_M, STEP_FRAC, TOL_M,
                             measure, pads_in_cam)
from srl_fk import FK  # noqa: E402

WS = "/home/gausms/kortex_ws"
MAX_SERVO_ITERS = 9
LIFT_M = 0.12
FINGER_TIP_ORIGIN_GAP_M = 0.0507   # tip-origin separation when fully closed


def real_gap_mm(fk, q, grip):
    """Actual opening between the pads, in mm.

    The finger-tip LINK ORIGINS are 50.7 mm apart when the hand is fully shut,
    so the origin separation is not the opening.  Subtracting that constant
    gives 84.8 mm at 0 rad, which is the Robotiq 85's rated 85 mm -- the check
    that this mapping is right.
    """
    Tl, Tr = fk.poses("left", q, ["robotiq_85_left_finger_tip_link",
                                  "robotiq_85_right_finger_tip_link"],
                      gripper=grip)
    return (np.linalg.norm(Tl[:3, 3] - Tr[:3, 3]) - FINGER_TIP_ORIGIN_GAP_M) * 1000


def servo_onto_cube(node, fk, tab, verbose=True, seed=None):
    """Null the pad-to-cube error in the camera frame. Returns the last fix.

    SEEDED WITH THE FIX THAT WAS ALREADY MADE, AND IT WAS NOT.
    ----------------------------------------------------------
    `find_cube` searches viewpoints and RETURNS A MEASUREMENT -- it has the
    cube. This loop then threw that away and re-measured from scratch, so a
    single empty frame on iteration 1 left `last = None` and the run ended
    "never got a fix on the cube -- NOT closing the hand" one second after
    printing "FOUND (123 pts, 32.0 mm tall)". Measured 2026-08-26: the search
    found it on try 3 and the servo gave up immediately afterwards.

    A DETECTION THAT BLINKS IS NOT A DETECTION THAT IS GONE. The colour gate
    sits near its threshold at this range, so single frames drop out. One
    miss now costs a retry, not the run.
    """
    last = seed
    print("\n%-4s %9s %9s %9s %8s  %s"
          % ("it", "err_mm", "pads>tbl", "cube_mm", "rms_mm", "action"))
    for it in range(1, MAX_SERVO_ITERS + 1):
        m = None
        for attempt in range(4):
            m = measure(node)
            # SAY WHY, do not just fail. "cube out of view" was printed for
            # four different causes -- no frames, no green pixels, too few
            # points, a height outside the gate -- and they need different
            # responses. A run that ends "never got a fix" after printing
            # FOUND one second earlier is unfalsifiable without this.
            if m is None:
                print("    [%d] measure() -> None (no green cluster, or the "
                      "plane had too few inliers)" % attempt)
            elif not plausible(m):
                print("    [%d] rejected: %d pts, %.1f mm tall, rms %.2f"
                      % (attempt, m["n_pts"], m["height_mm"],
                         m["plane_rms_mm"]))
            else:
                break
            node.spin(0.25)
        if not plausible(m):
            if last is None:
                print("  no cube in 4 frames and no earlier fix -- stopping")
            else:
                print("  cube out of view (expected this close) -- "
                      "finishing from the last fix")
            break
        q = m["q"]
        Tw_d, = fk.poses("left", q, ["camera_depth_frame"])
        last = {"cube_w": (Tw_d @ np.r_[m["centre"], 1])[:3],
                "n_w": Tw_d[:3, :3] @ m["n"]}
        pad = pads_in_cam(fk, q, CUBE_GRIP)
        err_v = m["centre"] - pad
        err = float(np.linalg.norm(err_v))
        pad_h = float(pad @ m["n"] + m["d"])
        print("%-4d %9.1f %9.1f %9.1f %8.2f  " % (it, err * 1000, pad_h * 1000,
                                                  m["height_mm"],
                                                  m["plane_rms_mm"]), end="")
        if err < TOL_M:
            print("WITHIN TOLERANCE")
            break
        step = err_v * STEP_FRAC
        if np.linalg.norm(step) > MAX_STEP_M:
            step = step / np.linalg.norm(step) * MAX_STEP_M
        new_h = pad_h + float(step @ m["n"])
        if new_h < MIN_CLEAR_M:
            step = step + m["n"] * (MIN_CLEAR_M - new_h)
            print("(floor-limited) ", end="")
        step_w = Tw_d[:3, :3] @ step
        Tl, Tr, Tee = fk.poses("left", q,
                               ["robotiq_85_left_finger_tip_link",
                                "robotiq_85_right_finger_tip_link",
                                "end_effector_link"], gripper=CUBE_GRIP)
        mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
        qn, ep, _, _ = ik(mid_w + step_w, Tee[:3, :3], q, grip=CUBE_GRIP)
        if ep > 0.004:
            print("IK residual %.1f mm -- stopping" % (ep * 1000))
            break
        print("step %.1f mm" % (np.linalg.norm(step) * 1000))
        if node.goto(qn, "servo-%d" % it) is None:
            print("  motion guard fired; stopping")
            break
    return last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-scan", action="store_true",
                    help="fail instead of searching for a viewpoint")
    args = ap.parse_args()

    rclpy.init()
    node = Eye()
    node.fk = fk = FK()
    print("=== 1/5  connect ===")
    if not node.preflight():
        raise SystemExit(2)
    node.set_deadband(0.10)
    node._hold = node.fresh_q().copy()
    print("  gripper opening at CUBE_GRIP=%.3f rad: %.1f mm  (open: %.1f mm)"
          % (CUBE_GRIP, real_gap_mm(fk, node.fresh_q(), CUBE_GRIP),
             real_gap_mm(fk, node.fresh_q(), 0.0)))
    try:
        tab = TableCheck(WS + "/recordings/baselines/cube_located_v2.json")
    except Exception:
        tab = None

    print("\n=== 2/5  find a viewpoint that sees the cube ===")
    if not args.dry_run:
        node.hold_gripper(GRIP_OPEN, 2.0, "OPEN gripper")
    if args.no_scan:
        m = measure(node)
        if not plausible(m):
            raise SystemExit("cube not visible and --no-scan was given")
    else:
        m, q = find_cube(node, fk, tab, verbose=True)
        if m is None:
            raise SystemExit("could not find the cube from any viewpoint")
    print("  cube: %.1f mm tall, %d depth points, plane RMS %.2f mm"
          % (m["height_mm"], m["n_pts"], m["plane_rms_mm"]))
    # keep the table fix current, so path checks use THIS surface
    q = node.fresh_q()
    Tw_d, = fk.poses("left", q, ["camera_depth_frame"])
    json.dump({"q": list(map(float, q)),
               "grasp_world": list(map(float, (Tw_d @ np.r_[m["centre"], 1])[:3])),
               "table_normal_world": list(map(float, Tw_d[:3, :3] @ m["n"])),
               "height_mm": m["height_mm"], "n_cube": m["n_pts"],
               "plane_rms_mm": m["plane_rms_mm"]},
              open(WS + "/recordings/baselines/cube_located_v2.json", "w"), indent=1)

    if args.dry_run:
        print("\ndry run: viewpoint found and cube measured; nothing grasped")
        return

    print("\n=== 3/5  servo the pads onto the cube ===")
    # SEED THE SERVO WITH THE FIX THE SEARCH ALREADY MADE. `m` is the
    # measurement `find_cube` returned, lifted into world here exactly as the
    # servo does it. Without this a single dropped frame on iteration 1
    # discards a cube that was found a second earlier.
    seed = {"cube_w": (Tw_d @ np.r_[m["centre"], 1])[:3],
            "n_w": Tw_d[:3, :3] @ m["n"]}
    last = servo_onto_cube(node, fk, tab, seed=seed)
    if last is None:
        raise SystemExit("never got a fix on the cube -- NOT closing the hand")

    print("\n=== 4/5  guarded descent onto the cube ===")
    # The stopping height is measured EVERY STEP inside the camera frame, so
    # the 8.45 deg mount error cannot push the pads into the table.  Target is
    # half the cube's own measured height: that is where the pads should close.
    target = (m["height_mm"] / 2.0) / 1000.0
    print("  target gap: %.1f mm (half of the %.1f mm cube)"
          % (target * 1000, m["height_mm"]))

    def measure_fn():
        mm = measure(node)
        if not plausible(mm):
            return None
        pad = pads_in_cam(fk, node.fresh_q(), CUBE_GRIP, node.arm_name)
        return pad, mm["n"], mm["d"], mm.get("inlier_frac", 0.6)

    def move_fn(vec_cam):
        q = node.fresh_q()
        Tw_d, = fk.poses(node.arm_name, q, ["camera_depth_frame"])
        Tl, Tr, Tee = fk.poses(node.arm_name, q,
                               ["robotiq_85_left_finger_tip_link",
                                "robotiq_85_right_finger_tip_link",
                                "end_effector_link"], gripper=CUBE_GRIP)
        mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
        qn, ep, _, _ = ik(mid_w + Tw_d[:3, :3] @ vec_cam, Tee[:3, :3], q,
                          grip=CUBE_GRIP)
        # A SKIPPED MOVE MUST SAY SO. This was `if ep < 0.004: goto(...)` with
        # no else, so an IK residual over 4 mm silently commanded NOTHING --
        # the descent then looped twenty times computing steps that were never
        # sent, the gap sat at 43.5 mm against a 15 mm target, the gain ramped
        # to its ceiling, and the bridge reported "zero speed: watchdog (82.78
        # s since last target)". From the log it looked like the arm was
        # refusing to descend; in fact nothing had asked it to.
        if ep < 0.004:
            node.goto(qn, "descend")
            return True
        print("      IK residual %.1f mm at this step -- NOT commanded "
              "(the pose asked for is not reachable from here)" % (ep * 1000))
        return False

    try:
        final = descend(measure_fn, move_fn, target)
        print("  final measured gap: %.1f mm" % (final * 1000))
    except Blind as e:
        # Losing sight this close is expected -- the wrist camera cannot see
        # its own grasp -- so fall back to the last world fix rather than
        # descending blind.
        print("  %s" % e)
        q = node.fresh_q()
        Tl, Tr, Tee = fk.poses(node.arm_name, q,
                               ["robotiq_85_left_finger_tip_link",
                                "robotiq_85_right_finger_tip_link",
                                "end_effector_link"], gripper=CUBE_GRIP)
        mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
        rem = float(np.linalg.norm(last["cube_w"] - mid_w))
        print("  completing %.1f mm from the last world fix" % (rem * 1000))
        if rem > 0.002:
            qn, ep, _, _ = ik(last["cube_w"], Tee[:3, :3], q, grip=CUBE_GRIP)
            if ep < 0.004:
                node.goto(qn, "FINAL-APPROACH")

    node.hold_gripper(GRIP_SQUEEZE, 5.0, "CLOSE on the cube")

    up = last["n_w"] / np.linalg.norm(last["n_w"])
    q = node.fresh_q()
    Tl, Tr, Tee = fk.poses("left", q,
                           ["robotiq_85_left_finger_tip_link",
                            "robotiq_85_right_finger_tip_link",
                            "end_effector_link"], gripper=GRIP_SQUEEZE)
    mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    qn, ep, _, _ = ik(mid_w + up * LIFT_M, Tee[:3, :3], q, grip=GRIP_SQUEEZE)
    if ep < 0.004:
        node.goto(qn, "LIFT")
    else:
        print("  lift IK residual %.1f mm -- not lifting" % (ep * 1000))

    print("\n=== 5/5  did the cube actually come up? ===")
    # A grasped cube RIDES WITH THE HAND: it stays at the pads and it is no
    # longer on the table.  A missed grasp leaves it on the table, which from
    # the lifted pose reads as a cube far below the pads.  This is the check
    # that the earlier runs never made, which is why they reported success.
    m2 = measure(node)
    if not plausible(m2):
        print("  no cube in view after the lift.")
        print("  That is consistent with EITHER a successful grasp (the cube")
        print("  is held under the fingers, out of frame) OR a miss. Look at")
        print("  the scene camera to settle it.")
        return
    pad2 = pads_in_cam(fk, node.fresh_q(), GRIP_SQUEEZE)
    d = float(np.linalg.norm(m2["centre"] - pad2))
    print("  cube seen %.1f mm from the pads after lifting" % (d * 1000))
    if d < 0.05:
        print("  HELD: the cube moved with the hand.")
    else:
        print("  MISSED: the cube is still on the table, %.0f mm below the pads."
              % (d * 1000))


if __name__ == "__main__":
    main()
