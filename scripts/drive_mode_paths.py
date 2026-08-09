#!/usr/bin/env python3
"""JOB D: inject at the head of each mode's chain, count at the arm.

STATIC PUB/SUB COUNTS ARE NOT ENOUGH. A topic can have a publisher and a
subscriber and still carry nothing, and a chain can be wired correctly and
still be gated shut. So this DRIVES each mode from its real input and counts
trajectories arriving at the arm controller -- the only place a command
becomes motion.

Chains corrected after the first pass: the vendor VR bridge publishes BOTH
/vr_pose_<s> (legacy, nothing consumes it) and /vr/controller_pose_<s> (what
vr_pose_mapper actually reads). Injecting at the first would have reported
mode 2 dead for a reason that is not the command path.
"""
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

ARM = "left"
TAIL = "/%s_arm_controller/joint_trajectory" % ARM


class D(Node):
    def __init__(self):
        super().__init__("mode_driver")
        self.n = 0
        self.create_subscription(JointTrajectory, TAIL, self._t, 10)
        self.master = self.create_publisher(
            PoseStamped, "/master_arm_pose_%s" % ARM, 10)
        self.vr = self.create_publisher(
            PoseStamped, "/vr/controller_pose_left", 10)

    def _t(self, _):
        self.n += 1

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self, x, y, z):
        m = PoseStamped()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        return m

    def drive(self, pub, secs=6.0, hz=50.0):
        """Wiggle inside the MASTER ball so the pose genuinely changes.

        /master_arm_pose_<arm> carries a MASTER-FRAME displacement, which the
        follower maps through the anchor -- it is NOT a world-frame target.
        The first version injected a world position (0.32, 0.35, 1.15), which
        the follower read as a metre-scale displacement, and IK correctly
        refused every frame. That produced 0 trajectories and would have been
        reported as 'mode 1 does not work', against a mode this project has
        measured tracking to 0.4 mm on.

        The reference latches on the first valid frame, so hold still briefly
        before moving.
        """
        self.n = 0
        for _ in range(40):                       # latch the boot reference
            pub.publish(self.pose(0.0, 0.0, 0.0))
            rclpy.spin_once(self, timeout_sec=1.0 / hz)
        self.n = 0
        t0 = time.time()
        i = 0
        while time.time() - t0 < secs:
            k = 0.04 * math.sin(2 * math.pi * 0.25 * (time.time() - t0))
            pub.publish(self.pose(k, 0.0, 0.0))
            i += 1
            rclpy.spin_once(self, timeout_sec=1.0 / hz)
        self.spin(1.0)
        return self.n


def main():
    rclpy.init()
    d = D()
    d.spin(3.0)
    print("=" * 74)
    print("END-TO-END: inject at the head, count trajectories at the arm")
    print("=" * 74)

    res = {}
    n = d.drive(d.master)
    res["1/3 master -> arm"] = n
    print("  mode 1/3  /master_arm_pose_%s  -> %-4d trajectories at %s"
          % (ARM, n, TAIL))

    procs = []
    for pk, ex in (("srl_vr_teleop", "vr_pose_mapper"),):
        procs.append(subprocess.Popen(["ros2", "run", pk, ex],
                                      stdout=subprocess.DEVNULL,
                                      stderr=subprocess.DEVNULL,
                                      start_new_session=True))
    d.spin(8.0)
    n = d.drive(d.vr, secs=8.0)
    res["2 vr -> arm"] = n
    print("  mode 2    /vr/controller_pose_left -> %-4d trajectories" % n)
    for q in procs:
        try:
            os.killpg(os.getpgid(q.pid), 15)
        except Exception:
            pass

    print()
    print("  modes 4/5/6 are not driven here because there is nothing to")
    print("  drive INTO: /autonomy/assist_pose_<arm> has no subscriber, and")
    print("  autonomy_executive publishes no pose, no trajectory and holds")
    print("  no action client. Verified by grep AND by the live graph.")
    d.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
