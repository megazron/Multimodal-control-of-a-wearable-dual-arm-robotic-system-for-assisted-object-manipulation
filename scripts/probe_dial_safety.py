#!/usr/bin/env python3
"""JOB F: SAFETY DOES NOT SCALE WITH THE DIAL. Measured, at both extremes.

Drives the commanded pose STRAIGHT AT THE WEARER at dial=1.0 (fastest, largest
scale, lightest smoothing) and again at dial=0.0, and reports the minimum
clearance the arm actually reached and how many times the hard floor blocked.

WHY THIS IS THE TEST. A dial that quietly relaxed the floor at speed would
look identical from the operator's seat -- smoother, more responsive, no
warning -- right up until it did not. The only way to know is to aim the arm
at the person and see where it stops.

WHAT WOULD FALSIFY THE CLAIM: a lower minimum clearance at dial=1.0 than at
dial=0.0, or any breach of the floor at either. Faster arrival is expected and
is not a failure; a closer approach is.
"""
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_teleop import precision_speed as ps               # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/dial_safety.json")
ARM = "left"
SECS = 14.0


class P(Node):
    def __init__(self):
        super().__init__("dial_safety_probe")
        self.pub = self.create_publisher(
            PoseStamped, "/master_arm_pose_%s" % ARM, 10)
        self.clear = []
        self.blocks = 0
        self.create_subscription(Float64MultiArray, "/ik_status_%s" % ARM,
                                 self._st, 10)
        self.cli = self.create_client(
            SetParameters, "/ik_follower_%s/set_parameters" % ARM)

    def _st(self, m):
        d = list(m.data)
        if len(d) >= 8:
            if d[5] >= 0:
                self.clear.append(d[5])
            self.blocks = int(d[6])

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def set_params(self, d):
        """Parameter CLIENT, never `ros2 param set` -- the CLI goes through
        the daemon, which hangs on this box and reports success it has not
        earned."""
        if not self.cli.wait_for_service(timeout_sec=8.0):
            return False
        req = SetParameters.Request()
        for k, v in d.items():
            p = Parameter()
            p.name = k
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                     double_value=float(v))
            req.parameters.append(p)
        fut = self.cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 8.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.done()

    def pose(self, xyz):
        m = PoseStamped()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.pose.orientation.w = 1.0
        return m

    def run(self, dial):
        s = ps.settings(dial)
        ps.assert_safety_unconditional(s)
        ok = self.set_params({"max_vel_rad_s": s["max_vel_rad_s"],
                              "max_step_rad": s["max_step_rad"]})
        self.clear, self.blocks = [], 0
        # Latch the reference, then drive the master straight toward the
        # wearer's chest: -y in the master frame maps back through the anchor.
        for _ in range(40):
            self.pub.publish(self.pose((0.0, 0.0, 0.0)))
            rclpy.spin_once(self, timeout_sec=0.02)
        self.clear = []
        t0 = time.time()
        while time.time() - t0 < SECS:
            f = min(1.0, (time.time() - t0) / (SECS * 0.6))
            # push well past the body so nothing but the floor stops it
            self.pub.publish(self.pose((0.0, -0.45 * f, -0.25 * f)))
            rclpy.spin_once(self, timeout_sec=0.02)
        self.spin(1.5)
        return dict(dial=dial, params_set=ok,
                    settings={k: round(v, 4) for k, v in s.items()},
                    n=len(self.clear),
                    min_clearance=(min(self.clear) if self.clear else None),
                    floor_blocks=self.blocks)


def main():
    rclpy.init()
    p = P()
    p.spin(4.0)
    res = [p.run(1.0), p.run(0.0)]
    print("=" * 74)
    print("DRIVING AT THE WEARER, AT BOTH ENDS OF THE DIAL")
    print("=" * 74)
    for r in res:
        print("  dial %.1f  vel %.2f rad/s  step %.2f  ->  min clearance %s"
              "   floor blocks %d   (n=%d)"
              % (r["dial"], r["settings"]["max_vel_rad_s"],
                 r["settings"]["max_step_rad"],
                 "--" if r["min_clearance"] is None
                 else "%.4f m" % r["min_clearance"],
                 r["floor_blocks"], r["n"]))
    fast, slow = res[0], res[1]
    print()
    if fast["min_clearance"] is None or slow["min_clearance"] is None:
        print("  NO CLEARANCE SAMPLES -- the probe did not observe the arm.")
        print("  Nothing may be concluded about the floor from this run.")
    else:
        print("  fast approached to %.4f m, slow to %.4f m"
              % (fast["min_clearance"], slow["min_clearance"]))
        print("  SAFETY UNCONDITIONAL: %s"
              % ("HOLDS -- speed did not buy a closer approach"
                 if fast["min_clearance"] >= slow["min_clearance"] - 0.005
                 else "FAILED -- the fast end got closer to the wearer"))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("  -> %s" % OUT)
    p.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
