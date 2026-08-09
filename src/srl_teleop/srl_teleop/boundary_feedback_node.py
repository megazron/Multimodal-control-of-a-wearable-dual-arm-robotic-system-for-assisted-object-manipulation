#!/usr/bin/env python3
"""WITHDRAWN 2026-08-09. NOT AN ENTRY POINT. DO NOT RE-ENABLE AS IT STANDS.

This node predicts the workspace boundary ahead of the operator, and the idea
is sound. The implementation emits a FALSE WARNING in the first half-second in
2 of 5 runs, and the cause is not known.

WHAT WAS MEASURED
  * 3 of 5 runs: first warning at t = 8.3-8.5 s with 88 mm of travel
    remaining, after 334 mm of travel. Correct, and reproducible to the
    millimetre.
  * 2 of 5 runs: a warning at t = 0.02-0.44 s claiming 0 mm remaining, at a
    point a direct IK probe reaches at 6 of 6.

WHAT WAS RULED OUT, each tested in isolation
  * a stale message queued from a previous run: the outlier survives a
    VOLATILE, depth-1 subscription after a 2 s drain, so nothing published
    before the run can reach it.
  * a near-zero velocity estimate leaving the probe unaimed: the spurious
    warnings carry speed 0.053-0.076 m/s, which is the real travel speed.
  * a missing cached orientation: the instrumented trace shows quat=set on
    every probe.
  * the IK request format: A/B over seed, ik_link_name and timeout returned
    6 of 6 in all six configurations, including this node's exact one.

WHY IT IS CUT RATHER THAN PATCHED. The obvious patch is to suppress warnings
until the estimator has settled. That would hide the symptom without knowing
the cause, and the cause is free to reappear later in a run. A boundary
warning that is wrong teaches the operator to ignore it, which is worse than
no warning: it spends the credibility that makes every other alarm in this
system work.

TO REVIVE IT, find the cause first. The evidence above narrows it a long way,
and the debug parameter that produced the probe trace is still in the file.
"""

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

ARMS = ("left", "right")
GROUP = {"left": "left_arm", "right": "right_arm"}
TIP = {"left": "left_end_effector_link", "right": "right_end_effector_link"}

IK_WALL, CLEAR_WALL, NONE_WALL = "reach", "wearer", "clear"

# A gap longer than this means the previous stream ended; cached
# position and velocity from before it are not evidence about now.
STREAM_GAP_S = 0.5


class BoundaryFeedback(Node):

    STREAM_GAP_S = STREAM_GAP_S

    def __init__(self):
        super().__init__("boundary_feedback_node")
        self.declare_parameter("probe_step_m", 0.02)
        self.declare_parameter("probe_max_m", 0.20)
        self.declare_parameter("warn_m", 0.10)
        self.declare_parameter("min_speed_m_s", 0.01)
        self.declare_parameter("rate_hz", 4.0)
        self.declare_parameter("clearance_floor_m", 0.12)
        self.declare_parameter("debug", False)
        self.debug = bool(self.get_parameter("debug").value)

        # A ReentrantCallbackGroup and a MultiThreadedExecutor, because this
        # node calls a service from inside a timer callback. Doing that on the
        # default single-threaded executor raises "Executor is already
        # spinning" and the node dies at startup -- which is exactly how
        # grasp_generator failed in this project, silently, while the arbiter
        # sat reporting "no grasp".
        self.cbg = ReentrantCallbackGroup()
        self.ik = self.create_client(GetPositionIK, "/compute_ik",
                                     callback_group=self.cbg)
        self.last = {a: None for a in ARMS}
        self.vel = {a: np.zeros(3) for a in ARMS}
        self.t = {a: None for a in ARMS}
        self.busy = {a: False for a in ARMS}
        self.lock = threading.Lock()

        self.pub = {a: self.create_publisher(
            Float64MultiArray, "/boundary_%s" % a, 10) for a in ARMS}
        self.say = {a: self.create_publisher(
            String, "/boundary_text_%s" % a, 10) for a in ARMS}
        for a in ARMS:
            self.create_subscription(
                PoseStamped, "/master_arm_pose_%s" % a,
                (lambda m, arm=a: self.on_pose(arm, m)), 20,
                callback_group=self.cbg)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value),
                          self.tick, callback_group=self.cbg)
        self.get_logger().info(
            "boundary feedback up. Probes ahead along the direction of travel "
            "and names the wall: reach, wearer, or clear.")

    def on_pose(self, arm, msg):
        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z])
        now = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            # STALE STATE ACROSS A GAP IN THE STREAM. If the pose stream stops
            # and restarts, the cached position and velocity are from before
            # the gap, so the first probe is fired from the OLD position along
            # the OLD direction. Measured: two runs reported the wall at 88 mm
            # and a third, started straight after, reported 0 mm at t=0.8 s
            # because it was still probing from the previous run's end point
            # near the wall. Anything older than the gap is discarded.
            if (self.t[arm] is not None
                    and now - self.t[arm] > self.STREAM_GAP_S):
                self.last[arm] = None
                self.vel[arm] = np.zeros(3)
            if self.last[arm] is not None and self.t[arm] is not None:
                dt = now - self.t[arm]
                if dt > 1e-3:
                    v = (p - self.last[arm]) / dt
                    # Light smoothing: an unsmoothed derivative of a 50 Hz
                    # pose stream points in a new direction every frame, and a
                    # probe fired down it measures nothing useful.
                    self.vel[arm] = 0.7 * self.vel[arm] + 0.3 * v
            self.last[arm] = p
            self.t[arm] = now
            self.quat = msg.pose.orientation

    def solvable(self, arm, xyz):
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP[arm]
        req.ik_request.ik_link_name = TIP[arm]
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 8_000_000
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
            float(xyz[0]), float(xyz[1]), float(xyz[2]))
        ps.pose.orientation = getattr(self, "quat", None) or ps.pose.orientation
        if ps.pose.orientation.w == 0.0 and ps.pose.orientation.x == 0.0:
            ps.pose.orientation.w = 1.0
        req.ik_request.pose_stamped = ps
        fut = self.ik.call_async(req)
        # DO NOT SPIN HERE. A ReentrantCallbackGroup plus a
        # MultiThreadedExecutor is what makes a service call from inside a
        # timer callback legal, but spin_until_future_complete() on a node the
        # executor already owns deadlocks: the executor thread is inside this
        # callback, so nothing is left to service the response. Measured
        # symptom: the node started, logged, and published NOTHING at all --
        # zero messages, no error, which is the silent-stall signature this
        # project keeps meeting. Wait on the future and let another executor
        # thread deliver it.
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 0.25:
            time.sleep(0.002)
        if not fut.done():
            return True          # unknown is not a wall; do not invent one
        r = fut.result()
        return bool(r and r.error_code.val == 1)

    def probe(self, arm):
        """Walk forward along the velocity until something refuses."""
        with self.lock:
            p = None if self.last[arm] is None else self.last[arm].copy()
            v = self.vel[arm].copy()
        if p is None:
            return None
        sp = float(np.linalg.norm(v))
        if sp < float(self.get_parameter("min_speed_m_s").value):
            # Standing still has no direction, so there is no boundary AHEAD.
            # Reporting the nearest wall in any direction instead would warn
            # constantly while the operator is doing nothing.
            return dict(dist=None, wall=NONE_WALL, speed=sp, moving=False)
        u = v / sp
        step = float(self.get_parameter("probe_step_m").value)
        far = float(self.get_parameter("probe_max_m").value)

        # BISECTION, NOT A WALK. A linear walk costs far/step calls per arm per
        # tick -- 10 each at the defaults, so 80 calls a second across both
        # arms at 4 Hz, which at ~7 ms a call is over half the CPU budget for
        # a feedback signal. Bisection finds the same boundary to `step`
        # resolution in about log2(far/step) + 1 calls: 4 instead of 10.
        if self.debug:
            self.get_logger().warn(
                "[dbg %s] p=(%.3f,%.3f,%.3f) u=(%.2f,%.2f,%.2f) sp=%.3f "
                "quat=%s" % (arm, p[0], p[1], p[2], u[0], u[1], u[2], sp,
                             "set" if getattr(self, "quat", None) else "MISSING"))
        far_ok = self.solvable(arm, p + u * far)
        if self.debug:
            self.get_logger().warn("[dbg %s] far x=%.3f -> %s"
                                   % (arm, (p + u * far)[0], far_ok))
        if far_ok:
            return dict(dist=None, wall=NONE_WALL, speed=sp, moving=True)
        lo, hi = 0.0, far                       # lo reachable, hi is not
        while hi - lo > step:
            mid = 0.5 * (lo + hi)
            got = self.solvable(arm, p + u * mid)
            if self.debug:
                self.get_logger().warn("[dbg %s] mid x=%.3f -> %s"
                                       % (arm, (p + u * mid)[0], got))
            if got:
                lo = mid
            else:
                hi = mid
        return dict(dist=lo, wall=IK_WALL, speed=sp, moving=True)

    def tick(self):
        if not self.ik.service_is_ready():
            return
        for a in ARMS:
            if self.busy[a]:
                continue
            self.busy[a] = True
            try:
                r = self.probe(a)
            finally:
                self.busy[a] = False
            if r is None:
                continue
            warn = float(self.get_parameter("warn_m").value)
            d = r["dist"]
            # 0 = clear, 1 = at the wall. Linear in remaining distance so the
            # operator feels it approach rather than meeting a step.
            prox = 0.0 if d is None else max(0.0, min(1.0, 1.0 - d / warn))
            m = Float64MultiArray()
            m.data = [float(prox), float(-1.0 if d is None else d),
                      float(r["speed"]),
                      float({NONE_WALL: 0, IK_WALL: 1, CLEAR_WALL: 2}[r["wall"]])]
            self.pub[a].publish(m)
            if prox > 0.0:
                s = String()
                s.data = ("%s arm: %s boundary %.0f mm ahead"
                          % (a, {IK_WALL: "reach", CLEAR_WALL: "wearer"}
                             .get(r["wall"], "?"), 1000 * d))
                self.say[a].publish(s)


def main(argv=None):
    rclpy.init(args=argv)
    n = BoundaryFeedback()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
