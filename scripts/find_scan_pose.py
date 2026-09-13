#!/usr/bin/env python3
"""Find a SCAN POSE per arm: one the wrist camera can actually see from.

    python3 scripts/find_scan_pose.py            # search and report
    python3 scripts/find_scan_pose.py --save     # write the baseline JSON

THE PROBLEM, MEASURED. At the home pose neither wrist camera frames the work
surface. For the right camera the bench projects to pixel (954, 539) and the
table to (1115, 765) -- both IN FRONT of the camera and both outside a
640 x 480 image. The left camera is parked looking up and back (elevation
+38.3 deg; docs/ENGINEERING_LOG.md: "sees nothing at table height from home").

This is the documented parking, not a fault. It matters because on camera day
an empty image from a correctly-working camera is indistinguishable from a
dead one, and this project has lost days to exactly that shape of ambiguity
(the frozen /real/joint_states, the dead j7 pot, the 0.000 noise floor). The
fix is to have a pose that DOES frame the surface, verified before anyone
stands in a lab wondering whether the camera is broken.

WHY THE PINNED WRIST CANNOT DO THIS, which is the whole reason a separate
pose is needed. The camera's optical axis IS the gripper approach axis (the
camera frames are rpy = (pi, pi, 0) from end_effector_link, a 180 deg spin
about z). Teleop pins that axis at (-0.153, +0.846, +0.511) -- 30.7 deg ABOVE
horizontal. A camera on that axis looks UP AND FORWARD from wherever the hand
is, so no position whatever will make it look down at a table. The scan pose
therefore commands its own orientation, which is legitimate because scanning
is not teleoperation: it is a one-off posture, taken before the trial, with no
operator in the loop and the same collision-aware IK as everything else.

VERIFIED THE SAME WAY AS EVERY OTHER POSE HERE: N=10 against a live
/compute_ik with avoid_collisions=True, plus a control that must fail. The
FOV test is separate and geometric -- the eight corners of the work region
must project inside the image.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments", "experiments",
                                "abc"))
sys.path.insert(0, os.path.join(WS, "src", "srl_perception", "srl_perception"))

from geometry_msgs.msg import Quaternion                    # noqa: E402
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402


def _quat_msg(q):
    """Solver.solve() takes a geometry_msgs Quaternion, not a 4-list.

    Passing a list does not raise a Python TypeError -- it trips a rosidl C
    assertion deep in the conversion layer and aborts the interpreter, which
    reads as a crash rather than as a wrong argument.
    """
    m = Quaternion()
    m.x, m.y, m.z, m.w = (float(v) for v in q)
    return m

N = 10
OUT = os.path.join(WS, "recordings", "baselines", "scan_pose.json")

# THE MEASURED WORK REGION (survey_work_surface.py, full path, N=3): two
# 0.40 x 0.15 m strips at |x| 0.30..0.70, y 0.05..0.20, on the work plane.
REGION = {"left": dict(x=(0.30, 0.70), y=(0.05, 0.20)),
          "right": dict(x=(-0.70, -0.30), y=(0.05, 0.20))}
Z_SURF = 1.12


def _look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Rotation whose +z (the OPTICAL AXIS) points from eye to target.

    Optical convention: z forward, x right, y DOWN. y is down, not up, which
    is why the up vector is negated into the y column -- getting this wrong
    renders a vertically mirrored image, and a mirrored AprilTag decoded at
    0/120 in this project once already.
    """
    z = np.asarray(target, float) - np.asarray(eye, float)
    n = np.linalg.norm(z)
    if n < 1e-9:
        return None
    z = z / n
    u = np.asarray(up, float)
    x = np.cross(u, z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(np.array([0.0, 1.0, 0.0]), z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _R_to_q(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


def _q_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def ee_to_cam(node, arm):
    """The FIXED end_effector -> camera transform, read from TF.

    Read, not assumed. The camera frames are declared in the URDF and the
    rpy = (pi, pi, 0) relationship is documented, but a transform that is
    looked up cannot silently disagree with the model that is actually
    running -- which is how a stale install has bitten this project before.
    """
    for _ in range(200):
        try:
            t = node.buf.lookup_transform("%s_end_effector_link" % arm,
                                          "%s_camera_color_frame" % arm,
                                          rclpy.time.Time())
            tr, ro = t.transform.translation, t.transform.rotation
            return (np.array([tr.x, tr.y, tr.z]),
                    _q_to_R(ro.x, ro.y, ro.z, ro.w))
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.05)
    return None, None


def fov_cover(p_cam, R_cam, arm, fx=615.0, w=640, h=480):
    """Fraction of the work region's corners inside the image."""
    r = REGION[arm]
    cx, cy = w / 2.0 - 0.5, h / 2.0 - 0.5
    pts = [(x, y, Z_SURF) for x in r["x"] for y in r["y"]]
    pts.append(((r["x"][0] + r["x"][1]) / 2, (r["y"][0] + r["y"][1]) / 2,
                Z_SURF))
    inside = 0
    for q in pts:
        pc = R_cam.T @ (np.asarray(q, float) - p_cam)
        if pc[2] <= 0.05:
            continue
        u = fx * pc[0] / pc[2] + cx
        v = fx * pc[1] / pc[2] + cy
        if 0 <= u < w and 0 <= v < h:
            inside += 1
    return inside / float(len(pts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", action="store_true")
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik")
        return 2
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s is %.4f rad from home -- the EE->camera "
                  "transform is fixed, but a run from the wrong pose "
                  "measures the wrong arm" % (arm, w))
            return 3

    out, ok_all = {}, True
    for arm in ("left", "right"):
        p_ec, R_ec = ee_to_cam(n, arm)
        if p_ec is None:
            print("%s: no EE->camera TF" % arm)
            return 4
        r = REGION[arm]
        tgt = np.array([(r["x"][0] + r["x"][1]) / 2.0,
                        (r["y"][0] + r["y"][1]) / 2.0, Z_SURF])
        best = None
        # STAND OFF, ABOVE AND BEHIND. Behind (smaller y) because the arm
        # mounts on the wearer's back and reaching past the target to look
        # back at it is the configuration the arm cannot fold into.
        for dz in (0.25, 0.35, 0.45, 0.20):
            for dy in (-0.25, -0.15, -0.35, -0.05):
                for dx in (0.0, 0.10, -0.10, 0.20):
                    sx = 1.0 if arm == "left" else -1.0
                    eye = tgt + np.array([sx * dx, dy, dz])
                    R_cam = _look_at(eye, tgt)
                    if R_cam is None:
                        continue
                    cov = fov_cover(eye, R_cam, arm)
                    if cov < 1.0:
                        continue
                    # The CAMERA pose is what we want; solve for the EE pose
                    # that puts the camera there.
                    R_ee = R_cam @ R_ec.T
                    p_ee = eye - R_ee @ p_ec
                    q = _R_to_q(R_ee)
                    hits = sum(1 for _ in range(N)
                               if n.solve(arm, list(p_ee), _quat_msg(q), tries=6))
                    if hits == N:
                        best = dict(arm=arm, ee_xyz=[float(v) for v in p_ee],
                                    ee_quat=[float(v) for v in q],
                                    cam_xyz=[float(v) for v in eye],
                                    target=[float(v) for v in tgt],
                                    fov_cover=cov, ik=f"{hits}/{N}",
                                    standoff=dict(dx=sx * dx, dy=dy, dz=dz))
                        break
                    if hits:
                        print("   %s standoff (%.2f,%.2f,%.2f) MARGINAL "
                              "%d/%d -- not usable" % (arm, sx * dx, dy, dz,
                                                       hits, N))
                if best:
                    break
            if best:
                break
        if best:
            print("%-5s SCAN POSE  cam (%.3f, %.3f, %.3f)  IK %s  "
                  "region corners in frame %.0f%%"
                  % (arm, best["cam_xyz"][0], best["cam_xyz"][1],
                     best["cam_xyz"][2], best["ik"],
                     100 * best["fov_cover"]))
            out[arm] = best
        else:
            print("%-5s NO SCAN POSE FOUND -- see the bring-up doc" % arm)
            ok_all = False

    # CONTROL: a pose the arm cannot reach must not be accepted.
    ctl = n.solve("left", [1.6, 0.35, 1.15],
                  _quat_msg(out.get("left", {}).get("ee_quat",
                                                    [0, 0, 0, 1])), tries=6)
    print("CONTROL 1.6 m out -> %s" % ("REACHABLE(BAD)" if ctl
                                       else "unreachable"))
    ok_all = ok_all and not ctl

    if a.save and ok_all:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        json.dump(dict(poses=out, repeats=N, region=REGION, z_surface=Z_SURF,
                       note="camera axis IS the gripper approach axis; the "
                            "pinned teleop wrist points 30.7 deg ABOVE "
                            "horizontal and can never frame a table"),
                  open(OUT, "w"), indent=2)
        print("-> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
