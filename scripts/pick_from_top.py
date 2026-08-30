#!/usr/bin/env python3
"""Pick the cube from directly above, accounting for the camera->pad offset.

    python3 scripts/pick_from_top.py

THE GEOMETRY THAT DRIVES ALL OF THIS
------------------------------------
Measured from FK on this arm, in the DEPTH CAMERA's own axes:

    camera origin -> pad midpoint = (+27.5, +66.0, +113.2) mm

The pads are 113 mm in front of the camera and 66 mm BELOW its optical axis.
Project that: when the pads sit exactly on the cube, the cube appears at
v = 66*360/113.2 + 137.9 = 348 px in an image only 270 px tall.  THE CUBE IS
BELOW THE BOTTOM OF THE FRAME AT THE MOMENT OF GRASP.  No servo can see its
own grasp on this hardware.

So the pick is staged so that nothing important happens while blind:

  1. FIND      locate the cube by COLOUR AND DEPTH ONLY -- no table plane
               needed, because when the camera is low and looking sideways the
               plane fit latches onto the table's EDGE and reports the hand as
               "-2 mm above the table", which rejected every scan candidate.
  2. OVER      move the pads to a point straight above the cube, high enough
               that the camera looks DOWN at the table.  Iterated, so the
               8.45 deg mount error is corrected by re-measurement rather than
               trusted.
  3. ALIGN     null the LATERAL error only, holding height.  The cube stays in
               frame throughout because the pads are still high.
  4. DESCEND   straight down the measured table normal, re-measuring the gap
               from the TABLE alone every step.  Needs no sight of the cube and
               carries no hand-eye error.

Only step 4 is blind to the cube, and step 4 moves only along the one axis
that step 3 did not need to fix.
"""
import argparse
import math
import sys

import cv2
import numpy as np
import rclpy

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from guarded_descent import Blind, descend  # noqa: E402
from servo_pick_left import CUBE_GRIP, Eye, GRIP_OPEN, GRIP_SQUEEZE, pads_in_cam  # noqa: E402
from srl_sequence import ik_relaxed  # noqa: E402
from srl_fk import FK  # noqa: E402
from srl_scene import fit_plane  # noqa: E402

STANDOFF_M = 0.25          # how high above the cube to stage
LATERAL_TOL_M = 0.005
MAX_ITERS = 8


def cloud(node):
    d, dk = node.img["d"], node.img["dk"]
    dep = np.frombuffer(d.data, np.uint16).reshape(
        d.height, d.width).astype(np.float32) * 0.001
    fx, fy, cx, cy = dk.k[0], dk.k[4], dk.k[2], dk.k[5]
    v, u = np.nonzero((dep > 0.05) & (dep < 2.0))
    Z = dep[v, u]
    return np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], 1)


def cube_points(node, fk):
    """Green points in the DEPTH camera frame. No table plane required."""
    if not node.frames():
        return None
    P = cloud(node)
    if len(P) < 300:
        return None
    c, ck = node.img["c"], node.img["ck"]
    q = node.fresh_q()
    Tw_d, Tw_c = fk.poses("left", q, ["camera_depth_frame", "camera_color_frame"])
    col = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, -1)
    if c.encoding == "rgb8":
        col = cv2.cvtColor(col, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(col, cv2.COLOR_BGR2HSV)
    mask = cv2.morphologyEx(cv2.inRange(hsv, (40, 80, 40), (85, 255, 255)),
                            cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    if mask.sum() < 255 * 200:
        return None
    T_cd = np.linalg.inv(Tw_c) @ Tw_d
    Pc = (T_cd @ np.c_[P, np.ones(len(P))].T).T[:, :3]
    uu = Pc[:, 0] * ck.k[0] / Pc[:, 2] + ck.k[2]
    vv = Pc[:, 1] * ck.k[4] / Pc[:, 2] + ck.k[5]
    ok = (uu >= 0) & (uu < c.width) & (vv >= 0) & (vv < c.height)
    g = np.zeros(len(P), bool)
    g[ok] = mask[vv[ok].astype(int), uu[ok].astype(int)] > 0
    if g.sum() < 60:
        return None
    Pk = P[g]
    # keep the nearest compact blob: a green reflection further away is not it
    dist = np.linalg.norm(Pk, axis=1)
    keep = Pk[dist < np.median(dist) + 0.06]
    return keep if len(keep) >= 50 else Pk


def look_down_R(Tee, up_world):
    """Rotation that points the TOOL AXIS straight down at the table.

    move_pads_to() only translates -- it keeps whatever orientation the wrist
    already has.  Staging "above the cube" with a sideways wrist leaves the
    camera looking ACROSS the table instead of down it, which is why the
    descent then had no clean view of the surface and the arm parked in mid
    air.  The staging move has to command an ORIENTATION as well as a point.
    """
    cur = Tee[:3, :3] @ np.array([0.0, 0.0, 1.0])
    tgt = -np.asarray(up_world, float)
    v = np.cross(cur, tgt)
    s_ = np.linalg.norm(v)
    c_ = float(cur @ tgt)
    if s_ < 1e-9:
        return Tee[:3, :3] if c_ > 0 else -Tee[:3, :3]
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return (np.eye(3) + vx + vx @ vx * ((1 - c_) / (s_ * s_))) @ Tee[:3, :3]


def move_pads_pose(node, fk, target_cam, label, grip=CUBE_GRIP, look_down=True):
    """Move the pads to a camera-frame point AND point the tool downward."""
    q = node.fresh_q()
    Tw, = fk.poses("left", q, ["camera_depth_frame"])
    Tl, Tr, Tee = fk.poses("left", q,
                           ["robotiq_85_left_finger_tip_link",
                            "robotiq_85_right_finger_tip_link",
                            "end_effector_link"], gripper=grip)
    mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    pad_cam = Tw[:3, :3].T @ (mid - Tw[:3, 3])
    tgt_world = mid + Tw[:3, :3] @ (np.asarray(target_cam) - pad_cam)
    up = np.array([0.0, 0.0, 1.0])
    Rd = look_down_R(Tee, up) if look_down else Tee[:3, :3]
    qn, ep, tl = ik_relaxed(tgt_world, Rd, q, grip, up, fk)
    if ep > 0.012:
        print("     IK residual %.1f mm -- not moving" % (ep * 1000))
        return False
    return node.goto(qn, label) is not None


def move_pads_to(node, fk, target_cam, label, grip=CUBE_GRIP):
    q = node.fresh_q()
    Tw, = fk.poses("left", q, ["camera_depth_frame"])
    Tl, Tr, Tee = fk.poses("left", q,
                           ["robotiq_85_left_finger_tip_link",
                            "robotiq_85_right_finger_tip_link",
                            "end_effector_link"], gripper=grip)
    mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    pad_cam = Tw[:3, :3].T @ (mid - Tw[:3, 3])
    delta_world = Tw[:3, :3] @ (np.asarray(target_cam) - pad_cam)
    qn, ep, tl = ik_relaxed(mid + delta_world, Tee[:3, :3], q, grip,
                            Tw[:3, :3] @ np.array([0.0, 0.0, 1.0]), fk)
    if ep > 0.010:
        print("     IK residual %.1f mm -- not moving" % (ep * 1000))
        return False
    return node.goto(qn, label) is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    rclpy.init()
    node = Eye("left")
    node.fk = fk = FK()
    if not node.preflight():
        raise SystemExit(2)
    node.set_deadband(0.10)
    node._hold = node.fresh_q().copy()

    q = node.fresh_q()
    Tw, = fk.poses("left", q, ["camera_depth_frame"])
    pad_cam = pads_in_cam(fk, q, CUBE_GRIP, "left")
    print("camera -> pads, in camera axes: (%+.1f, %+.1f, %+.1f) mm"
          % tuple(pad_cam * 1000))

    print("\n1. FIND the cube (colour + depth, no table plane needed)")
    Pk = cube_points(node, fk)
    if Pk is None:
        raise SystemExit("no green object in view -- point the camera at the cube")
    cube = np.median(Pk, 0)
    print("   %d green points, cube at (%+.3f, %+.3f, %+.3f) m, range %.3f m"
          % (len(Pk), *cube, np.linalg.norm(cube)))
    if args.dry_run:
        print("   pad-to-cube offset: %.1f mm"
              % (np.linalg.norm(cube - pad_cam) * 1000))
        return

    node.hold_gripper(GRIP_OPEN, 2.0, "OPEN gripper")

    print("\n2. OVER -- stage the pads %.0f mm straight above the cube"
          % (STANDOFF_M * 1000))
    for it in range(1, 5):
        Pk = cube_points(node, fk)
        if Pk is None:
            print("   lost the cube; stopping")
            break
        cube = np.median(Pk, 0)
        q = node.fresh_q()
        Tw, = fk.poses("left", q, ["camera_depth_frame"])
        up_cam = Tw[:3, :3].T @ np.array([0.0, 0.0, 1.0])   # world up, in camera
        target = cube + up_cam * STANDOFF_M
        pad_cam = pads_in_cam(fk, q, CUBE_GRIP, "left")
        err = float(np.linalg.norm(target - pad_cam))
        print("   it %d: pads are %.0f mm from the staging point" % (it, err * 1000))
        if err < 0.02:
            break
        if not move_pads_pose(node, fk, target, "over-%d" % it,
                              look_down=True):
            break

    print("\n3. ALIGN laterally (cube still visible, pads still high)")
    for it in range(1, MAX_ITERS + 1):
        Pk = cube_points(node, fk)
        if Pk is None:
            print("   cube not visible; stopping alignment")
            break
        P = cloud(node)
        n, d, msk = fit_plane(P)
        if msk.mean() < 0.15:
            n = fk.poses("left", node.fresh_q(),
                         ["camera_depth_frame"])[0][:3, :3].T @ np.array([0, 0, 1.0])
            d = -float(n @ np.median(Pk, 0)) + 0.02
        cube = np.median(Pk, 0)
        pad_cam = pads_in_cam(fk, node.fresh_q(), CUBE_GRIP, "left")
        err = cube - pad_cam
        lat = err - n * float(err @ n)
        L = float(np.linalg.norm(lat))
        print("   it %d: lateral %.1f mm" % (it, L * 1000))
        if L < LATERAL_TOL_M:
            print("   aligned")
            break
        step = lat * 0.7
        if np.linalg.norm(step) > 0.05:
            step = step / np.linalg.norm(step) * 0.05
        if not move_pads_to(node, fk, pad_cam + step, "align-%d" % it):
            break

    print("\n4. DESCEND straight down on the measured table gap")
    Pk = cube_points(node, fk)
    cube_h = 0.040
    if Pk is not None:
        P = cloud(node)
        n0, d0, m0 = fit_plane(P)
        hh = Pk @ n0 + d0
        if m0.mean() > 0.15 and 0.02 < hh.max() < 0.08:
            cube_h = float(hh.max())
    print("   cube height %.1f mm -> target gap %.1f mm"
          % (cube_h * 1000, cube_h * 500))

    def measure_fn():
        if not node.frames():
            return None
        P = cloud(node)
        if len(P) < 400:
            return None
        n, d, msk = fit_plane(P)
        return (pads_in_cam(fk, node.fresh_q(), CUBE_GRIP, "left"),
                n, d, float(msk.mean()))

    def move_fn(vec):
        pad = pads_in_cam(fk, node.fresh_q(), CUBE_GRIP, "left")
        move_pads_to(node, fk, pad + vec, "descend")

    try:
        final = descend(measure_fn, move_fn, cube_h / 2.0)
        print("   final measured gap: %.1f mm" % (final * 1000))
    except Blind as e:
        raise SystemExit("stopped: %s" % e)

    print("\n5. CLOSE and LIFT")
    node.hold_gripper(GRIP_SQUEEZE, 5.0, "CLOSE")
    m = measure_fn()
    if m:
        pad, n, d, _ = m
        for up in (0.05, 0.12):
            if not move_pads_to(node, fk, pad + n * up, "lift-%d" % int(up * 1000),
                                grip=GRIP_SQUEEZE):
                break
            pad = pads_in_cam(fk, node.fresh_q(), GRIP_SQUEEZE, "left")
    print("DONE -- check the gripper camera")


if __name__ == "__main__":
    main()
