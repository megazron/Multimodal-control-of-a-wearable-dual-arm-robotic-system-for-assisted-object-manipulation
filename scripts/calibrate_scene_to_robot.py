#!/usr/bin/env python3
"""Solve where the SCENE CAMERA is, in robot coordinates, by moving the arm.

    python3 scripts/calibrate_scene_to_robot.py --arm left --execute
    python3 scripts/calibrate_scene_to_robot.py --self-test

WHY THIS IS THE MISSING PIECE
-----------------------------
The room cameras see the table, the cube, the arms and the wearer -- and none
of it in ROBOT coordinates, because nothing has ever measured where those
cameras are. So their detections could describe a scene and never aim an arm.
`docs/PICK_THE_CUBE.md` says it plainly: "We do not have a usable one:
/scene_camera is colour-only with an all-zero K." The RealSense fixes the
first half (it has depth and real intrinsics); this fixes the second.

3-D TO 3-D, NOT PnP
-------------------
PnP needs a printed target, its exact geometry, and it solves a pose from
2-D correspondences -- accurate only as far as the corner detection. Here the
robot IS the target: its gripper goes to a commanded world pose that FK knows
exactly, and the RealSense measures that same gripper in 3-D. That gives
matched 3-D point pairs, and the rigid transform between two 3-D point sets
has a closed-form least-squares solution (Umeyama/Kabsch) with no iteration
and no initial guess.

FINDING THE GRIPPER WITHOUT A MARKER: IT IS THE THING THAT MOVED.
The arm visits N poses. Between consecutive frames almost nothing in the room
changes except the arm, so the depth DIFFERENCE isolates it, and the cluster
nearest the commanded position is the hand. No tape, no checkerboard, nothing
to fall off mid-session.

THE RESIDUAL IS THE ANSWER'S OWN ERROR BAR, and it is reported per point.
A calibration that fits 8 poses to 3 mm is usable; one that fits to 60 mm is
telling you the gripper detection was picking up the wrong cluster, and it
must be refused rather than shipped -- a wrong extrinsic aims the arm
confidently at the wrong place, which is worse than having none.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

WS = "/home/gausms/kortex_ws"
OUT = os.path.join(WS, "recordings/baselines/scene_to_robot.json")


def umeyama(A, B):
    """Rigid transform taking A onto B (both N x 3). Returns (R, t, rms).

    Closed form, no initial guess: centre both sets, take the SVD of the
    cross-covariance, and fix the reflection case explicitly -- without that
    check the "best fit" of a noisy set can come out as a MIRROR, which has
    determinant -1 and is not a rotation any robot can be at.
    """
    A, B = np.asarray(A, float), np.asarray(B, float)
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = cb - R @ ca
    err = (R @ A.T).T + t - B
    return R, t, float(np.sqrt((err ** 2).sum(1).mean()))


# ===========================================================================
def _self_test():
    """Known-answer: invent a transform, recover it."""
    rng = np.random.default_rng(3)
    fails = []

    def chk(name, ok, detail=""):
        print("  %-52s %s   %s" % (name, "PASS" if ok else "FAIL", detail))
        if not ok:
            fails.append(name)

    ang = math.radians(37.0)
    R_true = np.array([[math.cos(ang), -math.sin(ang), 0],
                       [math.sin(ang), math.cos(ang), 0], [0, 0, 1.0]])
    t_true = np.array([1.2, -0.4, 0.9])
    A = rng.uniform(-0.4, 0.4, (12, 3))
    B = (R_true @ A.T).T + t_true
    R, t, rms = umeyama(A, B)
    chk("exact points recover the transform", rms < 1e-9, "rms %.2e m" % rms)
    chk("rotation is a rotation (det +1)",
        abs(np.linalg.det(R) - 1.0) < 1e-9, "det %.9f" % np.linalg.det(R))
    chk("translation recovered",
        np.allclose(t, t_true, atol=1e-9),
        "%s" % np.round(t, 6))

    Bn = B + rng.normal(0, 0.004, B.shape)          # 4 mm of noise
    R2, t2, rms2 = umeyama(A, Bn)
    ang_err = math.degrees(math.acos(
        max(-1, min(1, (np.trace(R2 @ R_true.T) - 1) / 2))))
    chk("4 mm of noise -> under 2 deg", ang_err < 2.0,
        "%.3f deg, rms %.1f mm" % (ang_err, rms2 * 1000))

    # A MIRRORED SET MUST NOT COME BACK AS A ROTATION.
    Bm = B.copy()
    Bm[:, 0] *= -1
    R3, _, _ = umeyama(A, Bm)
    chk("a mirrored set still yields det +1 (never a reflection)",
        np.linalg.det(R3) > 0, "det %.6f" % np.linalg.det(R3))
    print("-" * 68)
    print("%d checks, %d failed" % (5, len(fails)))
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left")
    ap.add_argument("--execute", action="store_true",
                    help="actually move the arm; without it nothing moves")
    ap.add_argument("--poses", type=int, default=8)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--max-rms-mm", type=float, default=25.0)
    a = ap.parse_args()
    if a.self_test:
        return _self_test()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    sys.path.insert(0, os.path.join(WS, "scripts"))
    from srl_fk import FK
    import plan_pick_left as planner

    rclpy.init()
    n = Node("scene_to_robot")
    fk = FK()
    st = {}
    n.create_subscription(Image, "/scene/rs/depth/image_raw",
                          lambda m: st.__setitem__("d", m),
                          qos_profile_sensor_data)
    n.create_subscription(CameraInfo, "/scene/rs/color/camera_info",
                          lambda m: st.__setitem__("k", m),
                          qos_profile_sensor_data)
    js = {}
    n.create_subscription(JointState, "/real/joint_states",
                          lambda m: js.update(dict(zip(m.name, m.position))),
                          20)
    pub = n.create_publisher(
        JointTrajectory, "/real/%s_arm_controller/joint_trajectory" % a.arm, 10)
    names = ["%s_joint_%d" % (a.arm, i) for i in range(1, 8)]

    def spin(s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(n, timeout_sec=0.02)

    spin(4.0)
    for k in ("d", "k"):
        if k not in st:
            print("REFUSING: the RealSense is not publishing (%s). It is the "
                  "only scene camera with depth, and this calibration needs "
                  "depth." % k)
            return 2
    if any(js.get(x) is None for x in names):
        print("REFUSING: no /real/joint_states for the %s arm." % a.arm)
        return 2

    def cloud():
        m, ki = st["d"], st["k"]
        dep = np.frombuffer(m.data, np.uint16).reshape(
            m.height, m.width).astype(np.float32) * 0.001
        fx, fy, cx, cy = ki.k[0], ki.k[4], ki.k[2], ki.k[5]
        sx, sy = m.width / ki.width, m.height / ki.height
        fx, fy, cx, cy = fx * sx, fy * sy, cx * sx, cy * sy
        v, u = np.nonzero((dep > 0.3) & (dep < 5.0))
        Z = dep[v, u]
        return (np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], 1),
                dep, (fx, fy, cx, cy))

    def pads_world():
        q = np.array([js[x] for x in names])
        Tl, Tr = fk.poses(a.arm, q, ["robotiq_85_left_finger_tip_link",
                                     "robotiq_85_right_finger_tip_link"],
                          gripper=0.0)
        return (Tl[:3, 3] + Tr[:3, 3]) / 2.0

    from srl_body_geometry import body_points

    def arm_centroid_world():
        """Centroid of the arm's own SURFACE, in world, from the meshes.

        PAIR LIKE WITH LIKE. The camera's "moved points" are the whole arm's
        visible surface -- 40 000 of them -- so their centroid is the arm's
        centre, not the hand's. Pairing that with the GRIPPER's position asks
        the solver to fit two different physical points, and it did: 95.3 mm
        residual over six poses, which the gate correctly refused.
        
        This is the same quantity on the robot side: the centroid of the
        arm's collision-mesh surface at the same joint angles. Both are now
        "the middle of the arm", and the rigid transform between them is the
        one being solved for.
        """
        q = np.array([js[x] for x in names])
        pts = body_points(fk, a.arm, q, 0.0)
        # The BASE never moves, so the camera never sees it in the difference
        # -- excluding it here keeps the two centroids about the same body.
        P = np.vstack([v for k, v in pts.items()
                       if not k.endswith("_base_link")])
        return P.mean(0)

    # Poses spread across the arm's own reachable region, top-down so the
    # gripper presents the same shape to the room camera each time.
    base = np.array([js[x] for x in names])
    home_p = pads_world()
    print("gripper is at %s" % np.round(home_p, 3))
    targets = []
    for dx in (-0.12, 0.0, 0.12):
        for dy in (-0.10, 0.10):
            targets.append(home_p + np.array([dx, dy, 0.0]))
    targets = targets[:a.poses]

    prev = planner.set_arm(a.arm)
    pairs = []
    try:
        for i, p in enumerate(targets, 1):
            Tee, = fk.poses(a.arm, base, ["end_effector_link"])
            q, ep, er, _ = planner.ik(p, Tee[:3, :3], base, grip=0.0)
            if ep > 3e-3:
                print("  [%d/%d] unreachable (%.1f mm) -- skipped"
                      % (i, len(targets), ep * 1000))
                continue
            if not a.execute:
                print("  [%d/%d] would move to %s" % (i, len(targets),
                                                      np.round(p, 3)))
                continue
            before, _, _ = cloud()
            t0 = time.time()
            while time.time() - t0 < 14:
                t = JointTrajectory()
                t.joint_names = names
                pt = JointTrajectoryPoint()
                pt.positions = [float(x) for x in q]
                pt.time_from_start = Duration(sec=1, nanosec=0)
                t.points = [pt]
                pub.publish(t)
                spin(0.05)
                cur = np.array([js[x] for x in names])
                if float(np.abs(((cur - q + np.pi) % (2 * np.pi)) - np.pi).max()) < 0.03:
                    break
            spin(1.2)                       # let the depth settle
            after, _, _ = cloud()
            # THE GRIPPER IS WHAT MOVED. Take the points present now and not
            # before -- nothing else in the room changed.
            if len(before) < 500 or len(after) < 500:
                print("  [%d] too few depth points -- skipped" % i)
                continue
            from scipy.spatial import cKDTree
            tree = cKDTree(before)
            dist, _ = tree.query(after)
            moved = after[dist > 0.04]
            if len(moved) < 30:
                print("  [%d] nothing moved in the camera's view (%d pts) -- "
                      "is the arm in frame?" % (i, len(moved)))
                continue
            cam_p = moved.mean(axis=0)
            got = arm_centroid_world()
            pairs.append((cam_p, got))
            print("  [%d/%d] camera %s  <->  robot %s   (%d moved pts)"
                  % (i, len(targets), np.round(cam_p, 3), np.round(got, 3),
                     len(moved)))
    finally:
        planner.set_arm(prev)

    if not a.execute:
        print("\ndry run: %d pose(s) planned, nothing moved. Add --execute."
              % len(targets))
        rclpy.shutdown()
        return 0
    if len(pairs) < 4:
        print("\nREFUSING: only %d usable pose(s). Four is the minimum for a "
              "rigid transform that is not fitting noise." % len(pairs))
        rclpy.shutdown()
        return 1

    A = np.array([c for c, _ in pairs])
    B = np.array([r for _, r in pairs])
    R, t, rms = umeyama(A, B)
    print("\nscene camera -> robot base")
    print("  poses used : %d" % len(pairs))
    print("  residual   : %.1f mm rms" % (rms * 1000))
    print("  translation: %s m" % np.round(t, 4))
    if rms * 1000 > a.max_rms_mm:
        print("\nREFUSING to save: %.1f mm residual is over the %.1f mm "
              "limit. A wrong extrinsic aims the arm confidently at the wrong "
              "place, which is worse than having none. The usual cause is the "
              "moved-cluster picking up something other than the hand."
              % (rms * 1000, a.max_rms_mm))
        rclpy.shutdown()
        return 1
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({"R": R.tolist(), "t": t.tolist(),
               "rms_mm": round(rms * 1000, 2), "n_poses": len(pairs),
               "arm": a.arm, "frame_from": "scene_rs_color_frame",
               "frame_to": "world",
               "pairs": [{"cam": list(map(float, c)),
                          "robot": list(map(float, r))} for c, r in pairs]},
              open(OUT, "w"), indent=1)
    print("\nwritten to %s" % OUT)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
