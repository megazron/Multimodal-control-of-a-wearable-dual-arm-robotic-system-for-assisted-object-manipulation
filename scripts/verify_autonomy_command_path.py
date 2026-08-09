#!/usr/bin/env python3
"""BLOCKER 1: does an autonomy pose actually move the arm? Live graph + drive.

THREE CHECKS, ALL REQUIRED.

  (a) THE GRAPH. /autonomy/assist_pose_<arm> must have a SUBSCRIBER. It had
      none, which is how three of the four study modes shipped: the pose was
      computed correctly and dropped. Source inspection cannot see this --
      the publish call is present and the topic name matches the docstring.

  (b) THE DRIVE. A graph edge is not motion. Publish a world-frame target and
      count trajectories arriving at the arm controller.

  (c) THE SAFETY CHECK. Aim an autonomy pose INSIDE the wearer and confirm it
      is refused by the same clearance/collision machinery teleop uses. A path
      that moves the arm but skips the floor would be worse than no path.

INSTRUMENT NOTE -- READ BEFORE TRUSTING (c).
The previous version of this script decided "which blockers fired" with a
SUBSTRING match over the raw /blocking payload. That payload is JSON and
carries `blockers`, the list of every blocker the unit has REGISTERED, active
or not. So `"clearance_floor" in msg.data` was true in every message the
follower ever published -- including at a perfectly reachable target, and
including with the arm doing nothing at all. It measured registration, not
assertion, and it reported the safety stack as firing when nothing had fired.

Fixed by parsing the JSON and reading `active` only. The reachable drive is now
also used as an explicit CONTROL: if the same blocker names appear there, the
reading is meaningless. That control is printed, not assumed.
"""
import json
import sys
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

ARM = "left"
TOPIC = "/autonomy/assist_pose_%s" % ARM
TAIL = "/%s_arm_controller/joint_trajectory" % ARM
REACHABLE = (0.32, 0.35, 1.15)      # verified solvable in Job B
INSIDE_WEARER = (0.0, -0.10, 1.25)  # the torso/backpack, must be refused


class V(Node):
    def __init__(self):
        super().__init__("autonomy_path_probe")
        self.traj = 0
        self.active = set()
        self.registered = set()
        self.saw_blocking = 0
        self.create_subscription(JointTrajectory, TAIL, self._t, 10)
        self.create_subscription(String, "/blocking", self._b, 10)
        self.pub = self.create_publisher(PoseStamped, TOPIC, 10)
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)

    def anchor_quat(self):
        """The arm's CURRENT end-effector orientation.

        An identity quaternion is not a neutral choice -- it is a specific
        orientation, and at these positions it is unreachable. Driving with it
        produced 0 trajectories and named ik_failed, which reads as 'the path
        does not work' when the path is fine and the ASK is impossible. Same
        mistake as the Job D injection; the anchor is the honest reference.
        """
        t = self.buf.lookup_transform("world", "%s_end_effector_link" % ARM,
                                      rclpy.time.Time())
        r = t.transform.rotation
        return (r.x, r.y, r.z, r.w)

    def _t(self, _):
        self.traj += 1

    def _b(self, m):
        """Parse /blocking properly.

        `active` is what is being asserted right now. `blockers` is every
        blocker the unit registered at construction and says nothing about
        state -- reading it as evidence is the bug this script used to have.
        """
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if d.get("unit", "").endswith(ARM) or ARM in d.get("unit", ""):
            self.saw_blocking += 1
            self.active.update(d.get("active") or [])
            # expired counts as blocking: the asserting loop stopped, so the
            # condition is UNKNOWN, not clear. See CLAUDE.md.
            self.active.update(d.get("expired") or [])
            self.registered.update(b.get("name") for b in d.get("blockers") or []
                                   if isinstance(b, dict))

    def spin(self, s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, xyz, secs=8.0, quat=None):
        self.traj = 0
        self.active = set()
        self.saw_blocking = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            m = PoseStamped()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            (m.pose.position.x, m.pose.position.y,
             m.pose.position.z) = xyz
            q = quat or (0.0, 0.0, 0.0, 1.0)
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = q
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.spin(1.0)
        return self.traj, set(self.active)


def main():
    rclpy.init()
    v = V()
    v.spin(5.0)
    print("=" * 74)
    print("(a) LIVE GRAPH")
    print("=" * 74)
    pub = len(v.get_publishers_info_by_topic(TOPIC))
    sub = len(v.get_subscriptions_info_by_topic(TOPIC))
    print("  %-36s pub %d  sub %d  -> %s"
          % (TOPIC, pub, sub, "CONNECTED" if sub else "DEAD END"))

    print("\n" + "=" * 74)
    print("(b) DRIVE: autonomy pose -> trajectories at the arm")
    print("=" * 74)
    q = v.anchor_quat()
    print("  using the arm's OWN anchor orientation "
          "(%.3f %.3f %.3f %.3f), not identity" % q)
    n_ok, act_ok = v.drive(REACHABLE, quat=q)
    print("  reachable target %-22s -> %4d trajectories"
          % (str(REACHABLE), n_ok))
    print("  /blocking msgs seen for this arm: %d" % v.saw_blocking)
    print("  ACTIVE blockers at the reachable target : %s"
          % (", ".join(sorted(act_ok)) or "none"))

    print("\n" + "=" * 74)
    print("(c) SAFETY: the SAME floor applies to autonomy")
    print("=" * 74)
    n_bad, act_bad = v.drive(INSIDE_WEARER, quat=q)
    print("  target inside the wearer %-14s -> %4d trajectories"
          % (str(INSIDE_WEARER), n_bad))
    print("  ACTIVE blockers inside the wearer       : %s"
          % (", ".join(sorted(act_bad)) or "none"))

    # THE CONTROL. A blocker that is active in BOTH cases distinguishes
    # nothing. Only the difference is evidence about the wearer target.
    only_bad = act_bad - act_ok
    print("  registered-but-not-active names (would have been reported as")
    print("    'fired' by the old substring match): %s"
          % (", ".join(sorted(v.registered - act_bad)) or "none"))
    print("  DISCRIMINATING blockers (inside wearer but NOT at reachable): %s"
          % (", ".join(sorted(only_bad)) or "NONE -- (c) proves nothing"))

    print("\nVERDICT")
    print("  path connected      : %s" % bool(sub))
    print("  autonomy moves arm  : %s  (%d trajectories)" % (n_ok > 0, n_ok))
    print("  refused at the body : %s"
          % ("yes" if (n_bad == 0 and n_ok > 0) else
             "INCONCLUSIVE -- need motion at the reachable target first"))
    v.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
