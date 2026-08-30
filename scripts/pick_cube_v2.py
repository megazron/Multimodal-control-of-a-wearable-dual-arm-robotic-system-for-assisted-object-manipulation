#!/usr/bin/env python3
"""Pick the cube: align laterally WHILE IT IS VISIBLE, then descend blind-safe.

    python3 scripts/pick_cube_v2.py

WHY THE OLD SERVO MISSED
------------------------
Measured from FK on this arm: the pad midpoint sits at (+27.5, +66.0, +113.2) mm
from the depth camera's origin, in camera axes.  So when the pads reach the
cube, the cube lies atan(66.0/113.2) = 30.2 deg below the optical axis -- while
the camera's vertical half-FOV is only atan(135/360) = 20.6 deg.

THE CUBE IS GUARANTEED TO LEAVE THE FRAME BEFORE THE GRASP.  That is the mount
geometry, not a defect, and no amount of servoing changes it.

The old loop drove along the FULL 3D error vector, so it was still laterally
mis-aligned when the cube disappeared, and finished the last 40-90 mm open
loop -- through an 8.45 deg extrinsic error.  That is the miss.

THE FIX: SPLIT THE ERROR
------------------------
  Phase A  null only the IN-PLANE (lateral) error, holding height constant.
           The pads stay high, so the cube stays in view the whole time and
           the alignment is closed-loop right to the end.
  Phase B  descend STRAIGHT DOWN along the measured table normal, with the
           gap re-measured every step from the TABLE alone -- which needs no
           sight of the cube and carries no hand-eye error.

Lateral alignment therefore never runs blind, and the descent never needs the
cube.  Each phase uses only the measurement it can actually trust.

The cube centre is taken from the TOP FACE, not from all visible points: the
camera sees the top plus one or two sides, so an all-points centroid is pulled
toward the faces that happen to be visible.  The top face is centred on the
cube, so its centroid is the honest one.
"""
import argparse
import math
import sys
import time

import cv2
import numpy as np
import rclpy

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from auto_observe import find_cube  # noqa: E402
from guarded_descent import Blind, descend  # noqa: E402
from srl_sequence import ik_relaxed  # noqa: E402
from servo_pick_left import CUBE_GRIP, Eye, GRIP_OPEN, GRIP_SQUEEZE, pads_in_cam  # noqa: E402
from srl_fk import FK  # noqa: E402
from srl_scene import fit_plane  # noqa: E402

LATERAL_TOL_M = 0.004
LATERAL_STEP_FRAC = 0.7
LATERAL_MAX_STEP_M = 0.05
MAX_LATERAL_ITERS = 12
DESCEND_TARGET_FRAC = 0.5      # of the cube's own measured height


def scene(node, fk, want_cube=True):
    """Table plane, and optionally the cube, in the DEPTH camera frame."""
    if not node.frames():
        return None
    d, dk, c, ck = node.img["d"], node.img["dk"], node.img["c"], node.img["ck"]
    dep = np.frombuffer(d.data, np.uint16).reshape(
        d.height, d.width).astype(np.float32) * 0.001
    fx, fy, cx, cy = dk.k[0], dk.k[4], dk.k[2], dk.k[5]
    v, u = np.nonzero((dep > 0.05) & (dep < 1.5))
    if len(v) < 400:
        return None
    Z = dep[v, u]
    P = np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], 1)
    n, dd, msk = fit_plane(P)
    out = {"n": n, "d": dd, "inl": float(msk.mean()),
           "rms_mm": float(np.sqrt(((P[msk] @ n + dd) ** 2).mean()) * 1000)}
    if not want_cube:
        return out
    q = node.fresh_q()
    Tw_d, Tw_c = fk.poses("left", q, ["camera_depth_frame", "camera_color_frame"])
    col = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, -1)
    if c.encoding == "rgb8":
        col = cv2.cvtColor(col, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(col, cv2.COLOR_BGR2HSV)
    mask = cv2.morphologyEx(cv2.inRange(hsv, (40, 80, 40), (85, 255, 255)),
                            cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    T_cd = np.linalg.inv(Tw_c) @ Tw_d
    Pc = (T_cd @ np.c_[P, np.ones(len(P))].T).T[:, :3]
    uu = Pc[:, 0] * ck.k[0] / Pc[:, 2] + ck.k[2]
    vv = Pc[:, 1] * ck.k[4] / Pc[:, 2] + ck.k[5]
    ok = (uu >= 0) & (uu < c.width) & (vv >= 0) & (vv < c.height)
    g = np.zeros(len(P), bool)
    g[ok] = mask[vv[ok].astype(int), uu[ok].astype(int)] > 0
    h = P @ n + dd
    sel = g & (h > 0.005)
    if sel.sum() < 80:
        return out
    Pk = P[sel]
    hk = Pk @ n + dd
    top = Pk[hk > hk.max() - 0.008]
    if len(top) < 25:
        top = Pk
    foot = top.mean(0) - n * float(top.mean(0) @ n + dd)
    out.update({"cube_foot": foot, "cube_h": float(hk.max()),
                "cube_centre": foot + n * (hk.max() / 2.0),
                "n_pts": int(sel.sum()), "n_top": int(len(top))})
    return out


def move_cam(node, fk, vec_cam, label, grip=CUBE_GRIP):
    q = node.fresh_q()
    Tw, = fk.poses("left", q, ["camera_depth_frame"])
    Tl, Tr, Tee = fk.poses("left", q,
                           ["robotiq_85_left_finger_tip_link",
                            "robotiq_85_right_finger_tip_link",
                            "end_effector_link"], gripper=grip)
    mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    up = Tw[:3, :3] @ np.array([0.0, 0.0, 1.0])
    qn, ep, tl = ik_relaxed(mid + Tw[:3, :3] @ vec_cam, Tee[:3, :3], q, grip,
                            up, fk)
    if ep > 0.008:
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

    s = scene(node, fk)
    if s is None or "cube_centre" not in s:
        print("cube not in view -- searching for a viewpoint")
        node.hold_gripper(GRIP_OPEN, 1.5, "OPEN gripper")
        m, _ = find_cube(node, fk, None, verbose=True)
        if m is None:
            raise SystemExit("no viewpoint found that sees the cube")
        s = scene(node, fk)
        if s is None or "cube_centre" not in s:
            raise SystemExit("found it while scanning but lost it again")
    print("table %.0f%% inliers, %.2f mm RMS | cube %.1f mm tall "
          "(%d pts, %d on the top face)"
          % (s["inl"] * 100, s["rms_mm"], s["cube_h"] * 1000,
             s["n_pts"], s["n_top"]))
    if args.dry_run:
        pad = pads_in_cam(fk, node.fresh_q(), CUBE_GRIP, "left")
        lat = (s["cube_centre"] - pad) - s["n"] * float(
            (s["cube_centre"] - pad) @ s["n"])
        print("lateral error %.1f mm | pads %.1f mm above the table"
              % (np.linalg.norm(lat) * 1000, (pad @ s["n"] + s["d"]) * 1000))
        return

    node.hold_gripper(GRIP_OPEN, 2.5, "OPEN gripper")

    # ---------------- PHASE A: lateral only, cube stays in view -------------
    print("\nPHASE A -- lateral alignment at constant height "
          "(the cube stays visible)")
    print("  %-4s %10s %10s %s" % ("it", "lateral_mm", "pads>tbl", "action"))
    for it in range(1, MAX_LATERAL_ITERS + 1):
        s = scene(node, fk)
        if s is None or "cube_centre" not in s:
            print("  lost sight of the cube -- stopping lateral phase here")
            break
        pad = pads_in_cam(fk, node.fresh_q(), CUBE_GRIP, "left")
        err = s["cube_centre"] - pad
        lat = err - s["n"] * float(err @ s["n"])      # strip the vertical part
        L = float(np.linalg.norm(lat))
        print("  %-4d %10.1f %10.1f  " % (it, L * 1000,
                                          (pad @ s["n"] + s["d"]) * 1000), end="")
        if L < LATERAL_TOL_M:
            print("aligned")
            break
        step = lat * LATERAL_STEP_FRAC
        if np.linalg.norm(step) > LATERAL_MAX_STEP_M:
            step = step / np.linalg.norm(step) * LATERAL_MAX_STEP_M
        print("step %.1f mm" % (np.linalg.norm(step) * 1000))
        if not move_cam(node, fk, step, "lateral-%d" % it):
            break

    s = scene(node, fk)
    cube_h = s.get("cube_h", 0.040) if s else 0.040
    target = cube_h * DESCEND_TARGET_FRAC
    print("\nPHASE B -- straight down to a measured gap of %.1f mm "
          "(cube is %.1f mm tall)" % (target * 1000, cube_h * 1000))

    def measure_fn():
        t = scene(node, fk, want_cube=False)
        if t is None:
            return None
        return (pads_in_cam(fk, node.fresh_q(), CUBE_GRIP, "left"),
                t["n"], t["d"], t["inl"])

    def move_fn(vec):
        move_cam(node, fk, vec, "descend")

    try:
        final = descend(measure_fn, move_fn, target)
        print("  final measured gap: %.1f mm" % (final * 1000))
    except Blind as e:
        raise SystemExit("stopped: %s" % e)

    print("\nCLOSING")
    node.hold_gripper(GRIP_SQUEEZE, 5.0, "CLOSE")

    print("LIFTING")
    t = scene(node, fk, want_cube=False)
    if t:
        for up in (0.05, 0.12):
            if not move_cam(node, fk, t["n"] * up, "lift-%d" % int(up * 1000),
                            grip=GRIP_SQUEEZE):
                break
    t2 = scene(node, fk, want_cube=False)
    if t2:
        pad2 = pads_in_cam(fk, node.fresh_q(), GRIP_SQUEEZE, "left")
        print("  pads are now %.1f mm above the table"
              % ((pad2 @ t2["n"] + t2["d"]) * 1000))
    print("DONE -- check the gripper camera")


if __name__ == "__main__":
    main()
