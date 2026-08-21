#!/usr/bin/env python3
"""Calibrate the scene camera by driving the gripper to known points.

    # put a patch of saturated MAGENTA tape on the gripper first
    python3 scripts/calibrate_scene_camera.py --arm left --dry-run
    python3 scripts/calibrate_scene_camera.py --arm left

WHY A MARKER. Locating the gripper by rolling joint_7 and taking the centroid
of the changed pixels was tried on 2026-08-21 and gave 24 px mean reprojection
error -- about +-75 mm at 2 m, against a 30 mm grasp gate. The cause is
structural: the changed region is the gripper's whole body and its centroid
moves with the roll angle. More samples cannot fix a systematic offset. A
patch of colour the room does not already contain is found to a pixel or two
and sits at a fixed offset from the hand.

MAGENTA, SPECIFICALLY. The lab already contains teal robots (hue ~76-90), a
green cube (74), a skin-toned mannequin and black staging. Magenta (~165) is
the largest gap in that histogram. Any strongly saturated hue outside 60-100
works; check with --probe before committing to a colour.

--dry-run plans and prints the stations without moving anything.
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(WS, "src/srl_perception"))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(WS, "config"))

MAGENTA = ((160, 110, 80), (172, 255, 255))
SEAM, FLOOR = 0.30, 0.150
CONT = (0, 2, 4, 6)


def plan_stations(sc, arm, home, spread):
    """Reachable, safe stations spread through the volume the camera sees."""
    import numpy as np
    from scipy.optimize import minimize
    rng = np.random.default_rng(17)
    lo, hi, _ = sc.lim[arm]
    p0 = np.array(sc.arm_terms(arm, home, with_clearance=False)["hand"], float)
    offs = [(0, 0, 0), (spread, 0, 0), (-spread, 0, 0), (0, spread, 0),
            (0, -spread, 0), (0, 0, spread), (0, 0, -spread),
            (spread, 0, spread), (-spread, -spread, 0), (0, spread, -spread)]
    out = []
    for d in offs:
        goal = p0 + np.array(d, float)

        def cost(q):
            h = np.array(sc.arm_terms(arm, q, with_clearance=False)["hand"], float)
            c = 400.0 * float(np.sum((h - goal) ** 2))
            for i in CONT:
                sh = (SEAM + 0.05) - (math.pi - abs(float(q[i])))
                if sh > 0:
                    c += 50.0 * sh * sh
            return c

        best = None
        for s0 in [home] + [rng.uniform(lo, hi) for _ in range(12)]:
            r = minimize(cost, s0, method="L-BFGS-B", bounds=list(zip(lo, hi)),
                         options=dict(maxiter=400, ftol=1e-14))
            if best is None or r.fun < best.fun:
                best = r
        q = best.x
        h = np.array(sc.arm_terms(arm, q, with_clearance=False)["hand"], float)
        if float(np.linalg.norm(h - goal)) > 0.012:
            continue
        t = sc.arm_terms(arm, q)
        mg = min(math.pi - abs(float(q[i])) for i in CONT)
        pmin = min(sc.arm_terms(arm, home + (k / 25.0) * (q - home))["clearance_m"]
                   for k in range(26))
        if t["clearance_m"] < FLOOR or mg < SEAM or pmin < FLOOR:
            continue
        out.append(dict(q=[float(v) for v in q], hand=[float(v) for v in h],
                        clearance=float(t["clearance_m"]), seam=float(mg)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=("left", "right"), default="left")
    ap.add_argument("--spread", type=float, default=0.18,
                    help="how far the stations sit from home, metres")
    ap.add_argument("--hsv-lo", type=int, nargs=3, default=list(MAGENTA[0]))
    ap.add_argument("--hsv-hi", type=int, nargs=3, default=list(MAGENTA[1]))
    ap.add_argument("--out", default="recordings/baselines/scene_camera.json")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--probe", action="store_true",
                    help="just report the hue histogram of the scene, so a "
                         "marker colour can be chosen that the room lacks")
    a = ap.parse_args()

    import numpy as np
    import cv2
    from srl_perception import scene_calibration as SCAL
    from srl_perception import srl_cameras as CAMS
    import solve_home_pose as SHP
    import home_positions as hp

    if a.probe:
        cam = CAMS.SceneCamera(width=1280, height=720).open()
        try:
            img = cam.read()
        finally:
            cam.close()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        sat = hsv[:, :, 1] > 110
        h = hsv[:, :, 0][sat]
        hist, _ = np.histogram(h, bins=18, range=(0, 180))
        print("saturated-hue histogram of the scene (bin = 10 deg):")
        for i, n in enumerate(hist):
            print("   hue %3d-%3d  %6d px %s"
                  % (i * 10, i * 10 + 9, n, "#" * min(int(n / 400), 50)))
        print("\nPick a marker colour in the EMPTIEST bin.")
        return 0

    sc = SHP.Scorer()
    home = np.array(hp.load_home_radians(a.arm), float)
    stations = plan_stations(sc, a.arm, home, a.spread)
    print("planned %d safe stations for the %s arm" % (len(stations), a.arm))
    for i, st in enumerate(stations):
        print("   %2d  hand %s  clearance %.3f  seam %.2f"
              % (i, np.round(st["hand"], 3), st["clearance"], st["seam"]))
    if len(stations) < 6:
        print("\nREFUSING: %d stations is too few. 7 unknowns want 8 or more, "
              "spread out." % len(stations))
        return 3
    if a.dry_run:
        print("\ndry run: nothing moved.")
        return 0

    # ---- drive, look, collect -------------------------------------------
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    names = ["%s_joint_%d" % (a.arm, i) for i in range(1, 8)]

    class Drv(Node):
        def __init__(self):
            super().__init__("srl_scene_cal")
            self.q = {}
            self.create_subscription(JointState, "/real/joint_states",
                                     self._cb, 50)
            self.pub = self.create_publisher(
                JointTrajectory,
                "/real/%s_arm_controller/joint_trajectory" % a.arm, 10)

        def _cb(self, m):
            for n, p in zip(m.name, m.position):
                self.q[n] = p

        def pose(self):
            return (np.array([self.q[n] for n in names])
                    if all(n in self.q for n in names) else None)

        def spin(self, s):
            t0 = time.time()
            while time.time() - t0 < s:
                rclpy.spin_once(self, timeout_sec=0.02)

        def goto(self, q, tmo=45):
            t0 = time.time()
            while time.time() - t0 < tmo:
                m = JointTrajectory()
                m.joint_names = names
                pt = JointTrajectoryPoint()
                pt.positions = [float(v) for v in q]
                pt.time_from_start = Duration(sec=3)
                m.points = [pt]
                self.pub.publish(m)
                self.spin(0.08)
                p = self.pose()
                if p is None:
                    continue
                e = max(abs((lambda v: (v + math.pi) % (2 * math.pi) - math.pi)
                            (q[i] - p[i])) if i in CONT else abs(q[i] - p[i])
                        for i in range(7))
                if e < math.radians(0.7):
                    break
            self.spin(1.5)
            return self.pose()

    rclpy.init()
    n = Drv()
    n.spin(2.0)
    if n.pose() is None:
        print("REFUSING: no /real/joint_states -- is the arm connected?")
        return 2
    cam = CAMS.SceneCamera(width=1280, height=720).open()
    P, UV = [], []
    try:
        for i, st in enumerate(stations):
            qa = n.goto(np.array(st["q"], float))
            img = cam.read()
            hit = SCAL.find_marker(img, tuple(a.hsv_lo), tuple(a.hsv_hi))
            hand = np.array(sc.arm_terms(a.arm, qa,
                                         with_clearance=False)["hand"], float)
            if hit is None:
                print("   %2d  marker NOT VISIBLE -- skipped" % i)
                continue
            print("   %2d  hand %s -> pixel (%.1f, %.1f)  %d px"
                  % (i, np.round(hand, 3), hit["uv"][0], hit["uv"][1],
                     hit["area"]))
            P.append(hand)
            UV.append(hit["uv"])
        n.goto(home)
    finally:
        cam.close()
        n.destroy_node()
        rclpy.shutdown()

    if len(P) < 6:
        print("\nREFUSING: the marker was visible at only %d stations. Is it "
              "on the gripper, in frame, and the right colour? Run --probe."
              % len(P))
        return 3
    sol = SCAL.solve_camera(P, UV, (1280, 720))
    print("\nfocal %.1f px   camera at %s"
          % (sol["f"], np.round(sol["camera_position"], 3)))
    print("reprojection: mean %.2f px  max %.2f px  over %d points"
          % (sol["reproj_mean_px"], sol["reproj_max_px"], sol["n_points"]))
    try:
        SCAL.check(sol)
        print("ACCEPTED.")
    except SCAL.CalibrationRefusal as exc:
        print("NOT ACCEPTED: %s" % exc)
        return 4
    out = {k: (v.tolist() if hasattr(v, "tolist") else v)
           for k, v in sol.items()}
    os.makedirs(os.path.dirname(os.path.join(WS, a.out)), exist_ok=True)
    json.dump(out, open(os.path.join(WS, a.out), "w"), indent=1, default=float)
    print("-> %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
