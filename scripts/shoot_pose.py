#!/usr/bin/env python3
"""Command a joint pose to both arms, WAIT FOR ARRIVAL, and photograph it.

    python3 scripts/shoot_pose.py --from-json recordings/baselines/symmetric_home.json
    python3 scripts/shoot_pose.py --left <7 rad> --right <7 rad> --tag candidate

WHY THIS EXISTS SEPARATELY FROM CHANGING HOME. A pose can be looked at without
being adopted. `config/home_positions_*.txt` is the source every runtime
consumer reads and HARD CONSTRAINT 1 says the task set is re-measured BEFORE it
moves, so a candidate gets commanded and photographed first and only becomes
home once it has been paid for. This is that first step.

IT WAITS FOR ARRIVAL, and that is not a detail. Publishing a trajectory and
grabbing a frame photographs whatever the arm happened to be doing, which for
a large joint move is a picture of the arm somewhere in the middle. Arrival is
checked against /joint_states, and a pose that does not arrive is reported as
not arrived rather than photographed anyway.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from verify_task_scenes import Solver                              # noqa: E402
import measure_home_render as MHR                                  # noqa: E402

TOL_RAD = 0.02


class Mover(Solver):
    def __init__(self):
        super().__init__()
        self.pub = {a: self.create_publisher(
            JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 5)
            for a in ("left", "right")}

    def send(self, arm, q, seconds=6.0):
        m = JointTrajectory()
        m.joint_names = self.names(arm)
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.time_from_start.sec = int(seconds)
        p.time_from_start.nanosec = int((seconds % 1.0) * 1e9)
        m.points = [p]
        self.pub[arm].publish(m)

    def arrived(self, arm, q, tol=TOL_RAD):
        live = [self.js.get(k) for k in self.names(arm)]
        if any(v is None for v in live):
            return False, None
        worst = max(abs(a - b) for a, b in zip(live, q))
        return worst <= tol, worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from-json", default=None,
                    help="symmetric_home.json; uses the best PAIR in it")
    ap.add_argument("--left", type=float, nargs=7, default=None)
    ap.add_argument("--right", type=float, nargs=7, default=None)
    ap.add_argument("--tag", default="candidate")
    ap.add_argument("--views", nargs="*", default=["front", "left", "iso"])
    ap.add_argument("--settle", type=float, default=10.0)
    a = ap.parse_args()

    want = {}
    if a.from_json:
        d = json.load(open(a.from_json))
        pairs = [p for p in d.get("pairs", []) if p.get("left")]
        if not pairs:
            print("no pairs in %s" % a.from_json)
            return 2
        best = min(pairs, key=lambda p: p["mirror_residual_m"])
        want["left"] = best["left"]["q"]
        want["right"] = best["right"]["q"]
        print("using the pair at x=%.3f, mirror residual %.4f m"
              % (best["x"], best["mirror_residual_m"]))
    else:
        if a.left is None or a.right is None:
            print("give --from-json or both --left and --right")
            return 2
        want["left"], want["right"] = list(a.left), list(a.right)

    rclpy.init()
    n = Mover()
    n.spin(10.0)
    for arm in ("left", "right"):
        n.send(arm, want[arm])
    print("commanded; waiting for arrival")
    end = time.time() + 40.0
    ok = {}
    while time.time() < end:
        n.spin(0.5)
        ok = {arm: n.arrived(arm, want[arm]) for arm in ("left", "right")}
        if all(v[0] for v in ok.values()):
            break
    for arm, (got, worst) in ok.items():
        print("   %-5s arrived=%s worst joint error %s rad"
              % (arm, got, "----" if worst is None else "%.4f" % worst))
    if not all(v[0] for v in ok.values()):
        print("NOT PHOTOGRAPHING: the arms did not arrive. A frame taken now "
              "would show a pose nobody chose.")
        n.destroy_node()
        rclpy.shutdown()
        return 3

    n.spin(a.settle)
    shots = {}
    for v in a.views:
        png = MHR.shoot(v)
        if png and a.tag:
            tagged = png.replace("home_%s.png" % v, "%s_%s.png" % (a.tag, v))
            os.replace(png, tagged)
            png = tagged
        shots[v] = png
        print("   %-6s %s" % (v, png or "NOT CAPTURED"))
    n.destroy_node()
    rclpy.shutdown()
    return 0 if all(shots.values()) else 4


if __name__ == "__main__":
    sys.exit(main())
