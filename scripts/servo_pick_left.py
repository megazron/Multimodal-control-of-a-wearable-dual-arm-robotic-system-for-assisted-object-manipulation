#!/usr/bin/env python3
"""Pick the cube by VISUAL SERVOING, not by trusting the camera extrinsic.

WHY OPEN LOOP CANNOT WORK HERE
------------------------------
The table cannot move, so the table normal measured from any arm pose must be
the same vector.  Measured from four poses today it swung 8.45 deg -- 34 mm of
position error at 0.23 m, 59 mm at 0.40 m.  The camera mount rotation is wrong
and it turns WITH the wrist, so the error changes every time the arm moves.
Planning a grasp through that extrinsic and executing it blind cannot land
inside a 30 mm capture gate, and it did not.

WHAT IS ACTUALLY RELIABLE
-------------------------
The depth camera itself: the table plane fits at 0.54 mm RMS and the cube
measures 40.9 mm tall against a real 40 mm cube.  The measurement is good; only
the transform out of the camera is bad.

So the error is measured where it is trustworthy -- between the PADS and the
CUBE, both expressed in the camera frame -- and nulled iteratively.  Each step
converts that error to joint motion through the imperfect extrinsic, moves only
a FRACTION of the way, and measures again.  Closed loop converges as long as
the extrinsic is better than 90 deg wrong, which it is by a wide margin.

THE PADS ARE PLACED AT THE OPENING THEY WILL HOLD THE CUBE AT
------------------------------------------------------------
Wrist-to-pad is 0.09833 m wide open and 0.10976 m closed on a 40 mm cube.
Servoing the OPEN pad midpoint onto the cube and then closing drives the pads
11.43 mm further down -- into the table.  Everything below uses CUBE_GRIP.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image, JointState
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
import cv2  # noqa: E402
from execute_pick_left import Executor, ang_wrap  # noqa: E402
from plan_pick_left import ik  # noqa: E402
from srl_fk import FK  # noqa: E402

CUBE_GRIP = 0.447      # knuckle rad at which the pads hold a 40 mm cube
GRIP_OPEN = 0.0
GRIP_SQUEEZE = 0.60    # past CUBE_GRIP so the pads actually load the cube
STEP_FRAC = 0.60       # fraction of the measured error to take each iteration
MAX_STEP_M = 0.045     # never command more than this in one go
TOL_M = 0.005          # done when the pads are this close to the cube centre
MAX_ITERS = 9
MIN_CLEAR_M = 0.004    # pads may not go below the cube's own resting plane


class Eye(Executor):
    """Executor plus the camera, so one node does both."""

    def __init__(self, arm="left"):
        super().__init__(arm)
        self.img = {}
        f = lambda T, t, k: self.create_subscription(   # noqa: E731
            T, t, lambda m, k=k: self.img.__setitem__(k, m), qos_profile_sensor_data)
        f(Image, "/%s_camera/color/image_raw" % arm, "c")
        f(CameraInfo, "/%s_camera/color/camera_info" % arm, "ck")
        f(Image, "/%s_camera/depth/image_raw" % arm, "d")
        f(CameraInfo, "/%s_camera/depth/camera_info" % arm, "dk")

    def frames(self, wait=20.0):
        self.img.clear()
        t0 = time.time()
        while time.time() - t0 < wait and len(self.img) < 4:
            rclpy.spin_once(self, timeout_sec=0.02)
        return len(self.img) == 4


def fit_plane(P, it=400, tol=0.006, seed=0):
    rng = np.random.default_rng(seed)
    best, bc = None, -1
    for _ in range(it):
        p = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        L = np.linalg.norm(n)
        if L < 1e-9:
            continue
        n = n / L
        d = -n @ p[0]
        c = int((np.abs(P @ n + d) < tol).sum())
        if c > bc:
            bc, best = c, (n, d)
    n, d = best
    m = np.abs(P @ n + d) < tol
    Q = P[m]
    cen = Q.mean(0)
    C = np.cov((Q - cen).T)
    w, V = np.linalg.eigh(C)
    n = V[:, 0] / np.linalg.norm(V[:, 0])
    # THE INLIER MASK MUST USE THE REFINED PLANE, BOTH HALVES OF IT.
    #
    # This returned `abs(P @ n + d) < tol` -- the NEW normal from the PCA
    # refinement with the OLD offset from RANSAC. The two do not describe the
    # same plane, so the mask could select ZERO points, and `mean()` of an
    # empty slice is NaN. That is the "plane RMS nan mm" the pick reported
    # while also reporting FOUND: the cube's centre is computed against this
    # plane, so a NaN plane means the position was meaningless, and the run
    # then tried to close 540 mm blind on it.
    d = -float(n @ cen)
    return n, d, np.abs(P @ n + d) < tol


class NoFrames(Exception):
    """The camera delivered nothing. NOT the same as 'no cube'."""


def measure(node):
    """Cube centre and table plane, in the DEPTH camera's own frame."""
    if not node.frames():
        # A blind scan once reported "no" at 14 viewpoints in a row and
        # concluded the cube was not there, while the camera was publishing
        # nothing at all. Those are different failures and must not share a
        # return value.
        raise NoFrames("no camera frames on /%s_camera" % node.arm_name)
    c, ck, d, dk = (node.img["c"], node.img["ck"], node.img["d"], node.img["dk"])
    dep = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) * 0.001
    fx, fy, cx, cy = dk.k[0], dk.k[4], dk.k[2], dk.k[5]
    v, u = np.nonzero((dep > 0.08) & (dep < 1.5))
    Z = dep[v, u]
    P = np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], 1)
    n, dd, inl = fit_plane(P)
    if dd < 0:
        n, dd = -n, -dd
    h = P @ n + dd
    col = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, -1)
    if c.encoding == "rgb8":
        col = cv2.cvtColor(col, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(col, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (40, 80, 40), (85, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    # depth -> colour pixel, via the URDF baseline between the two frames
    fk = node.fk
    q = node.fresh_q()
    Tw_d, Tw_c = fk.poses(node.arm_name, q, ["camera_depth_frame", "camera_color_frame"])
    T_cd = np.linalg.inv(Tw_c) @ Tw_d
    Pc = (T_cd @ np.c_[P, np.ones(len(P))].T).T[:, :3]
    uu = Pc[:, 0] * ck.k[0] / Pc[:, 2] + ck.k[2]
    vv = Pc[:, 1] * ck.k[4] / Pc[:, 2] + ck.k[5]
    ok = (uu >= 0) & (uu < c.width) & (vv >= 0) & (vv < c.height)
    green = np.zeros(len(P), bool)
    green[ok] = mask[vv[ok].astype(int), uu[ok].astype(int)] > 0
    cube = green & (h > 0.005)
    if cube.sum() < 60:
        return None
    Pk = P[cube]
    hk = Pk @ n + dd
    foot = Pk.mean(0) - n * (Pk.mean(0) @ n + dd)
    centre = foot + n * (hk.max() / 2.0)
    # A PLANE WITH NO INLIERS IS NOT A PLANE. Returning a measurement whose
    # geometry is NaN is worse than returning nothing: everything downstream
    # treats it as a fix and the arm moves on it.
    if int(inl.sum()) < 200:
        return None
    rms = float(np.sqrt((h[inl] ** 2).mean()) * 1000)
    if not np.isfinite(rms) or not np.all(np.isfinite(centre)):
        return None
    return {"centre": centre, "n": n, "d": dd, "height_mm": float(hk.max() * 1000),
            "n_pts": int(cube.sum()), "plane_rms_mm": rms, "q": np.array(q),
            "inlier_frac": float(inl.mean())}


def pads_in_cam(fk, q, grip, arm="left"):
    """Pad midpoint expressed in the DEPTH camera frame."""
    Tw_d, Tl, Tr = fk.poses(arm, q,
                            ["camera_depth_frame",
                             "robotiq_85_left_finger_tip_link",
                             "robotiq_85_right_finger_tip_link"], gripper=grip)
    mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    R = Tw_d[:3, :3]
    return R.T @ (mid_w - Tw_d[:3, 3])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Eye()
    node.fk = FK()
    if not node.preflight():
        raise SystemExit(2)
    node.set_deadband(0.10)
    node._hold = node.fresh_q().copy()
    fk = node.fk

    print("\nopening the hand")
    if not args.dry_run:
        node.hold_gripper(GRIP_OPEN, 2.5, "OPEN gripper")

    last = None      # last good measurement, carried in WORLD coordinates
    print("\n%-4s %9s %9s %9s %9s %8s %s"
          % ("it", "err_mm", "pads>tbl", "cube_mm", "pts", "rms_mm", "action"))
    for it in range(1, MAX_ITERS + 1):
        m = measure(node)
        if m is None:
            # THE WRIST CAMERA CANNOT SEE ITS OWN GRASP.
            #
            # Closing on the cube drives it out of the bottom of the frame at
            # about 80 mm to go -- the cube was last seen sitting between the
            # fingers at the frame edge.  Losing sight of it is geometry, not
            # failure, so the last good fix is carried in WORLD (where it does
            # not move) and the short remainder is completed from that.  Over
            # ~40 mm the 8.45 deg extrinsic error is worth about 6 mm, well
            # inside the pads' 85 mm opening on a 40 mm cube.
            print("  cube out of view (expected this close) -- finishing from "
                  "the last fix")
            break
        q = m["q"]
        Tw_d_now, = fk.poses("left", q, ["camera_depth_frame"])
        last = {"cube_w": (Tw_d_now @ np.r_[m["centre"], 1])[:3],
                "n_w": Tw_d_now[:3, :3] @ m["n"], "err_mm": None}
        pad = pads_in_cam(fk, q, CUBE_GRIP)
        err_v = m["centre"] - pad                      # in CAMERA frame
        err = float(np.linalg.norm(err_v))
        pad_h = float(pad @ m["n"] + m["d"])
        print("%-4d %9.1f %9.1f %9.1f %9d %8.2f"
              % (it, err * 1000, pad_h * 1000, m["height_mm"], m["n_pts"],
                 m["plane_rms_mm"]), end="  ")
        if err < TOL_M:
            print("WITHIN TOLERANCE")
            break
        step_v = err_v * STEP_FRAC
        if np.linalg.norm(step_v) > MAX_STEP_M:
            step_v = step_v / np.linalg.norm(step_v) * MAX_STEP_M
        # do not let the pads dive under the cube's resting plane
        new_h = pad_h + float(step_v @ m["n"])
        if new_h < MIN_CLEAR_M:
            step_v = step_v + m["n"] * (MIN_CLEAR_M - new_h)
            print("(floor-limited)", end=" ")
        # camera-frame step -> world -> IK on the pad midpoint
        Tw_d, = fk.poses("left", q, ["camera_depth_frame"])
        step_w = Tw_d[:3, :3] @ step_v
        Tl, Tr, Tee = fk.poses("left", q,
                               ["robotiq_85_left_finger_tip_link",
                                "robotiq_85_right_finger_tip_link",
                                "end_effector_link"], gripper=CUBE_GRIP)
        mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
        qn, ep, er, _ = ik(mid_w + step_w, Tee[:3, :3], q, grip=CUBE_GRIP)
        if ep > 0.004:
            print("IK residual %.1f mm -- stopping" % (ep * 1000))
            break
        dq = math.degrees(np.abs(ang_wrap(qn - q)).max())
        print("step %.1f mm (%.1f deg)" % (np.linalg.norm(step_v) * 1000, dq))
        if args.dry_run:
            break
        if node.goto(qn, "servo-%d" % it) is None:
            print("  motion guard fired; stopping")
            break
    else:
        print("  reached the iteration limit")

    if args.dry_run:
        print("\ndry run: nothing further commanded")
        return

    if last is None:
        print("\nnever got a fix on the cube -- NOT closing the hand.")
        return

    # ---- complete the approach from the last world fix ----
    q = node.fresh_q()
    Tl, Tr, Tee = fk.poses("left", q,
                           ["robotiq_85_left_finger_tip_link",
                            "robotiq_85_right_finger_tip_link",
                            "end_effector_link"], gripper=CUBE_GRIP)
    mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    rem = last["cube_w"] - mid_w
    print("\nremaining pad-to-cube from the last fix: %.1f mm"
          % (np.linalg.norm(rem) * 1000))
    if np.linalg.norm(rem) > 0.002:
        qn, ep, er, _ = ik(last["cube_w"], Tee[:3, :3], q, grip=CUBE_GRIP)
        if ep < 0.004:
            if node.goto(qn, "FINAL-APPROACH") is None:
                print("  guard fired on the final approach")
        else:
            print("  final-approach IK residual %.1f mm -- not moving"
                  % (ep * 1000))

    print("\nCLOSING on the cube")
    node.hold_gripper(GRIP_SQUEEZE, 5.0, "CLOSE gripper")

    print("LIFTING straight up along the table normal")
    q = node.fresh_q()
    Tl, Tr, Tee = fk.poses("left", q,
                           ["robotiq_85_left_finger_tip_link",
                            "robotiq_85_right_finger_tip_link",
                            "end_effector_link"], gripper=GRIP_SQUEEZE)
    mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    up = last["n_w"] / np.linalg.norm(last["n_w"])
    for lift in (0.06, 0.12):
        qn, ep, er, _ = ik(mid_w + up * lift, Tee[:3, :3], q, grip=GRIP_SQUEEZE)
        if ep < 0.004:
            if node.goto(qn, "LIFT-%dmm" % int(lift * 1000)) is None:
                break
            q = node.fresh_q()
        else:
            print("  lift IK residual %.1f mm at %d mm" % (ep * 1000, int(lift * 1000)))
            break
    print("DONE")


if __name__ == "__main__":
    main()
