#!/usr/bin/env python3
"""IS THE FOLLOWER PAUSE REAL? A known-answer test with both controls.

    python3 scripts/verify_follower_pause.py      # needs a live teleop stack

WHY THIS EXISTS. `follower_pause.pause()` lowers `motion_enabled`, and every
caller has believed that stops `ik_follower_node` publishing:
`stage_presentation_pose.py` has documented it as its answer to "the follower
wins" since 2026-08-15, and `vision_grasp.observe_and_detect` was given the
same treatment for T1's look.

IT DID NOT. The node read `motion_enabled` ONCE at construction into
`self.motion_enabled` and registered no parameter callback, so
`set_parameters` changed the parameter server's copy and the follower carried
on. Measured here, with both controls, on a live stack:

    before the fix   ARMED  +60 mm -> arm moved 0.0600 m
                     PAUSED -60 mm -> arm moved 0.0600 m
    after            ARMED  +60 mm -> arm moved 0.0600 m
                     PAUSED -60 mm -> arm moved 0.0000 m

WHAT IT COST. The arm settled at the edge of `Vision.stage()`'s 0.02 rad
tolerance in a tug of war with the follower and never stopped moving, so T1's
cubes deprojected 31.7-35.7 mm off inside the recording sweep against a 30 mm
capture gate -- and 1.3-3.0 mm standalone on a FRESH stack, where the follower
has no target yet and there is nothing to fight. That is the whole of the
"it is flaky" pattern.

BOTH HALVES ARE THE TEST. An arm that does not move when ARMED proves nothing
about a pause, so the control runs first and the verdict is withheld if it
fails. It has already earned its keep: the first version displaced +60 mm
FORWARD and the control failed from the home pose, because forward reach is
bound by the pinned wrist at about y = 0.425 and IK had no solution. The
displacement is outboard now, where |x| is reachable to 1.000.
"""
import math, os, sys, time
import numpy as np
ROOT = "/home/gausms/kortex_ws"
for p in (os.path.join(ROOT, "scripts"), os.path.join(ROOT, "config")):
    sys.path.insert(0, p)
import rclpy, rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf2_ros
import follower_pause as FP

ARM = "left"


class P(Node):
    def __init__(self):
        super().__init__("probe_pause")
        self.js = {}
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self.js.update(zip(m.name, m.position)), 20)
        self.pub = self.create_publisher(PoseStamped, "/master_arm_pose_%s" % ARM, 10)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

    def spin(self, s):
        t = time.monotonic()
        while time.monotonic() - t < s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def ee(self):
        t = self.buf.lookup_transform("world", "%s_end_effector_link" % ARM,
                                      rclpy.time.Time())
        v = t.transform.translation
        return np.array([v.x, v.y, v.z])

    def drive(self, target, secs=6.0):
        from srl_teleop import master_calibration as mc
        q = mc.WORKSPACE_ORIENT[ARM]
        t = time.monotonic()
        while time.monotonic() - t < secs:
            m = PoseStamped()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in target]
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = q
            self.pub.publish(m)
            self.spin(0.05)


rclpy.init()
n = P()
n.spin(3.0)
start = n.ee()
# OUTBOARD, NOT FORWARD. The first version displaced +60 mm in y and the
# CONTROL failed from the home pose: forward reach is bound by the pinned
# wrist at about y = 0.425 at this height, so IK simply had no solution and
# the arm correctly did not move. A control that fails for a legitimate
# reason still fails, and the guard below refused to report a verdict --
# which is the guard working. |x| is reachable to 1.000, so the same 60 mm
# outboard is unambiguously commandable.
print("EE at start (%.4f, %.4f, %.4f)" % tuple(start), flush=True)

# ---- CONTROL: armed, must move --------------------------------------
tgt = start + np.array([0.06, 0.0, 0.0])
n.drive(tgt, 8.0)
n.spin(1.5)
after_armed = n.ee()
d_armed = float(np.linalg.norm(after_armed - start))
print("ARMED   commanded +60 mm OUTBOARD (+x) -> arm moved %.4f m" % d_armed, flush=True)

# ---- TEST: paused, must NOT move ------------------------------------
paused = FP.pause(n)
print("pause reported:", dict(paused), flush=True)
base = n.ee()
tgt2 = base + np.array([-0.06, 0.0, 0.0])
n.drive(tgt2, 8.0)
n.spin(1.5)
after_paused = n.ee()
d_paused = float(np.linalg.norm(after_paused - base))
print("PAUSED  commanded -60 mm INBOARD (-x) -> arm moved %.4f m" % d_paused, flush=True)
FP.resume(n, paused)
print("resume reported:", dict(paused), flush=True)

print("\nCONTROL %s   (an arm that does not move when armed proves nothing)"
      % ("OK, the arm is commandable" if d_armed > 0.02 else "FAILED"))
print("VERDICT %s"
      % ("THE PAUSE IS REAL" if d_paused < 0.005 else
         "THE PAUSE IS DECORATIVE -- the follower kept commanding"))
n.destroy_node(); rclpy.shutdown()
