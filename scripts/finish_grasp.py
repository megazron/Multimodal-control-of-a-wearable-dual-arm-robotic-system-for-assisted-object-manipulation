#!/usr/bin/env python3
"""Close the last few centimetres onto an object already between the fingers.

    python3 scripts/finish_grasp.py --target-mm 20

WHY A SEPARATE STEP
-------------------
The servo gets the pads over the object and then the object leaves the wrist
camera's frame -- it is directly underneath, which is exactly where this
camera cannot look.  The descent then refused to run because `measure()`
requires the CUBE to be visible.

But the descent does not need the cube.  It needs the TABLE, which fills the
frame, and the pad position, which is FK.  So this fits the plane alone and
descends on

    gap = (pad midpoint in the CAMERA frame) . n_cam + d_cam

re-measured every step and carrying no hand-eye error.  It stops on the
measured gap, closes, and lifts.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from guarded_descent import Blind, descend  # noqa: E402
from plan_pick_left import ik  # noqa: E402
from srl_sequence import ik_relaxed  # noqa: E402
from servo_pick_left import CUBE_GRIP, Eye, GRIP_OPEN, GRIP_SQUEEZE, pads_in_cam  # noqa: E402
from srl_fk import FK  # noqa: E402
from srl_scene import fit_plane  # noqa: E402


def table_only(node, fk):
    """Fit the table plane from the live depth image. No object required."""
    if not node.frames():
        return None
    d, dk = node.img["d"], node.img["dk"]
    dep = np.frombuffer(d.data, np.uint16).reshape(
        d.height, d.width).astype(np.float32) * 0.001
    fx, fy, cx, cy = dk.k[0], dk.k[4], dk.k[2], dk.k[5]
    v, u = np.nonzero((dep > 0.05) & (dep < 1.5))
    if len(v) < 400:
        return None
    Z = dep[v, u]
    P = np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], 1)
    n, dd, msk = fit_plane(P)
    return n, dd, float(msk.mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target-mm", type=float, default=20.0)
    ap.add_argument("--lift-mm", type=float, default=120.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Eye("left")
    node.fk = fk = FK()
    if not node.preflight():
        raise SystemExit(2)
    node.set_deadband(0.10)
    node._hold = node.fresh_q().copy()
    if not node.frames():
        raise SystemExit("no camera frames")

    t = table_only(node, fk)
    if t is None:
        raise SystemExit("cannot fit the table from here")
    n, d, inl = t
    pad = pads_in_cam(fk, node.fresh_q(), CUBE_GRIP, "left")
    gap = float(pad @ n + d)
    print("table: %.0f%% inliers | pads are %.1f mm above it | target %.1f mm"
          % (inl * 100, gap * 1000, args.target_mm))
    if args.dry_run:
        return

    print("\nopening the hand wide so the pads straddle the cube")
    node.hold_gripper(GRIP_OPEN, 2.5, "OPEN")

    def measure_fn():
        tt = table_only(node, fk)
        if tt is None:
            return None
        nn, ddd, ii = tt
        return pads_in_cam(fk, node.fresh_q(), CUBE_GRIP, "left"), nn, ddd, ii

    def move_fn(vec_cam):
        q = node.fresh_q()
        Tw, = fk.poses("left", q, ["camera_depth_frame"])
        Tl, Tr, Tee = fk.poses("left", q,
                               ["robotiq_85_left_finger_tip_link",
                                "robotiq_85_right_finger_tip_link",
                                "end_effector_link"], gripper=CUBE_GRIP)
        mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
        # Exact top-down is often unreachable this far out: the descent
        # stalled at 46.9 mm with a 13.5 mm residual on EVERY step, commanding
        # nothing 25 times. Allow the approach to tilt.
        qn, ep, tl = ik_relaxed(mid + Tw[:3, :3] @ vec_cam, Tee[:3, :3], q,
                                CUBE_GRIP, Tw[:3, :3] @ n, fk)
        if ep < 0.008:
            node.goto(qn, "descend")
        else:
            print("     (IK residual %.1f mm even with a cone)" % (ep * 1000))

    print("\nguarded descent")
    try:
        final = descend(measure_fn, move_fn, args.target_mm / 1000.0)
        print("final measured gap: %.1f mm" % (final * 1000))
    except Blind as e:
        raise SystemExit("stopped: %s" % e)

    print("\nCLOSING")
    node.hold_gripper(GRIP_SQUEEZE, 5.0, "CLOSE")

    print("LIFTING %.0f mm straight up the table normal" % args.lift_mm)
    q = node.fresh_q()
    Tw, = fk.poses("left", q, ["camera_depth_frame"])
    up_w = Tw[:3, :3] @ n
    Tl, Tr, Tee = fk.poses("left", q,
                           ["robotiq_85_left_finger_tip_link",
                            "robotiq_85_right_finger_tip_link",
                            "end_effector_link"], gripper=GRIP_SQUEEZE)
    mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    for step in (0.04, args.lift_mm / 1000.0):
        qn, ep, _ = ik_relaxed(mid + up_w * step, Tee[:3, :3], q, GRIP_SQUEEZE,
                               up_w, fk)
        if ep < 0.008:
            if node.goto(qn, "LIFT-%dmm" % int(step * 1000)) is None:
                break
            q = node.fresh_q()
        else:
            print("  lift IK residual %.1f mm at %d mm" % (ep * 1000, int(step * 1000)))
            break

    t2 = table_only(node, fk)
    if t2:
        n2, d2, _ = t2
        pad2 = pads_in_cam(fk, node.fresh_q(), GRIP_SQUEEZE, "left")
        print("\npads are now %.1f mm above the table"
              % ((pad2 @ n2 + d2) * 1000))
    print("DONE -- look at the gripper camera to confirm the cube is held")


if __name__ == "__main__":
    main()
