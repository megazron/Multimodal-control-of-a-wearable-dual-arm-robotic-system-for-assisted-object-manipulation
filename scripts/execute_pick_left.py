#!/usr/bin/env python3
"""Execute the planned top-down grasp of the green cube with the LEFT arm.

Reads the plan written by plan_pick_left.py and drives the real arm through
PRE-GRASP -> GRASP -> (close gripper) -> LIFT.

WHY THIS DOES NOT READ TF
-------------------------
The sim TF tree disagreed with the real arm by 36 deg on left joint 6, and
that single error put the measured table 27 deg off level.  Every pose here
comes from `/real/joint_states` (the arm's own encoders, republished by
kortex_highlevel_bridge) and from srl_fk driven by those angles.

WHY THE PREFLIGHT IS LOUD
-------------------------
The first run moved nothing and said nothing useful.  Silence has two very
different causes -- the script talking to the wrong ROS_DOMAIN_ID, and the
bridge ignoring targets -- so both are now checked BEFORE anything is
commanded, and each failure names itself.

WHY THE DEADBAND IS LOWERED
---------------------------
The bridge runs a proportional law with a deadband inside which the term is
off by design.  At the shipped 1.0 deg, a target ramped in small steps can sit
inside the deadband and produce no velocity at all.  CLAUDE.md already records
0.10 deg as the correct value; this sets it for the run.

SAFETY PROPERTIES
-----------------
* Interpolated at DEG_PER_S deg/s, under the bridge's vmax (22.9 deg/s).
* Targets are republished continuously.  The bridge watchdog zeroes speed
  0.5 s after targets stop, so killing this script stops the arm.
* MOTION IS VERIFIED: if the arm has not moved after MOVE_CHECK_S while a
  target is standing, the run ABORTS rather than pushing blindly.
* Each stage must arrive within SETTLE_TOL_DEG or the sequence stops.

--dry-run prints the sequence and the preflight, and commands nothing.
"""
import argparse
import json
import math
import os
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

NAMES = ["left_joint_%d" % i for i in range(1, 8)]          # default arm
ARM_TOPIC = "/real/left_arm_controller/joint_trajectory"
GRIP_TOPIC = "/left_gripper_controller/joint_trajectory"


def arm_topics(arm):
    """Joint names and command topics for either arm."""
    return (["%s_joint_%d" % (arm, i) for i in range(1, 8)],
            "/real/%s_arm_controller/joint_trajectory" % arm,
            "/%s_gripper_controller/joint_trajectory" % arm,
            "/real/telemetry_%s" % arm,
            "/kortex_highlevel_bridge_%s/set_parameters" % arm)
RATE = 20.0
DEG_PER_S = 8.0
# ARRIVAL IS WAITED FOR, NOT ASSUMED.
#
# The first run held each target for a fixed 2.5 s and then judged arrival.
# The bridge closes the error with a proportional law at kp = 0.5, i.e. a
# time constant of ~2 s, so 2.5 s of settling leaves roughly a third of the
# error standing.  Measured: the arm stopped 7.2 deg short of PRE-GRASP, the
# stage was declared a miss, and the descent never ran.  Hold until it
# actually arrives, and only give up when it STOPS IMPROVING.
CONVERGE_MAX_S = 25.0     # absolute ceiling per stage
ARRIVE_TOL_DEG = 1.0      # close enough to call it arrived
STALL_S = 5.0             # no improvement for this long => stalled
STALL_MIN_DEG = 0.05      # improvement smaller than this does not count
SETTLE_TOL_DEG = 2.5      # a stalled stage this far out still aborts
MOVE_CHECK_S = 3.0        # by now the arm must have moved measurably
MOVE_MIN_DEG = 0.15       # ... by at least this much
FORCE_JUMP_N = 12.0       # a step this big in 1.5 s is a collision
GRIP_OPEN = 0.0
GRIP_CLOSE = 0.55



def ang_wrap(d):
    """Angular difference wrapped to [-pi, pi], elementwise.

    Joints 3, 5 and 7 of a Gen3 are CONTINUOUS: -179.05 deg and +181.02 deg
    are the SAME pose.  Comparing them by plain subtraction gives 360.08 deg,
    which is what made a run that had actually ARRIVED at PRE-GRASP report a
    stall and abort.  Every joint difference in this file goes through here.
    """
    return (np.asarray(d) + np.pi) % (2 * np.pi) - np.pi


class Executor(Node):
    def __init__(self, arm="left"):
        super().__init__("execute_pick_%s" % arm)
        self.arm_name = arm
        self.names, at, gt, tt, self._param_srv = arm_topics(arm)
        self.arm = self.create_publisher(JointTrajectory, at, 10)
        self.grip = self.create_publisher(JointTrajectory, gt, 10)
        self._arm_topic = at
        self.q = None
        self.q_t = 0.0
        self._hold = None
        self.create_subscription(JointState, "/real/joint_states", self._on_js, 10)
        # FORCE GUARD.
        #
        # The geometric wearer check cannot be trusted: asked for the pose the
        # arm was physically jammed against the mannequin in, it returned
        # 367.0 mm -- the SAME number it returns for a visibly clear pose. It
        # samples link ORIGINS, so it misses the arm's body between joints.
        # This guard is physical instead: the arm's own tool wrench. Gravity
        # changes smoothly as the arm reconfigures; a collision is a STEP, so
        # the trigger is a sudden RISE rather than an absolute level.
        self.force = None
        self.force_hist = []
        self.create_subscription(String, tt, self._on_telem, 10)

    def _on_telem(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        f = d.get("force_n")
        if f is None:
            return
        self.force = float(f)
        self.force_hist.append((time.time(), self.force))
        self.force_hist = [x for x in self.force_hist if time.time() - x[0] < 4.0]

    def force_jump(self, window=1.5):
        """Rise in tool force over the last `window` seconds, in newtons."""
        if len(self.force_hist) < 4:
            return 0.0
        now = time.time()
        recent = [f for t, f in self.force_hist if now - t < window * 0.4]
        older = [f for t, f in self.force_hist if window * 0.4 <= now - t < window]
        if not recent or not older:
            return 0.0
        return float(np.median(recent) - np.median(older))

    def _on_js(self, m):
        z = dict(zip(m.name, m.position))
        if all(n in z for n in self.names):
            self.q = np.array([z[n] for n in self.names])
            self.q_t = time.time()

    def fresh_q(self, max_age=0.35, wait=1.0):
        """Current joints, guaranteed recently received.

        Spinning with timeout_sec=0.0 inside a tight publish loop starves the
        subscription: self.q can lag seconds behind the arm.  A motion check
        run against that stale copy reported 0.032 deg of movement on a move
        that was actually running, and aborted it.  Never judge motion without
        going through here.
        """
        t0 = time.time()
        while time.time() - t0 < wait:
            if self.q is not None and time.time() - self.q_t < max_age:
                return self.q
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.q

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ------------------------------------------------------------- preflight
    def preflight(self, wait=25.0):
        """Wait for the graph to match, then report what is and is not there.

        This used to spin for a flat 6 s.  FastDDS matching on this host is
        slow and erratic -- the same subscription that returns nothing at 6 s
        delivers ~12 Hz at 8 s -- so a short window reported "no
        /real/joint_states" against a bridge that was publishing perfectly.
        A run that aborts here moves nothing and looks exactly like a dead
        arm, which is the wrong thing to go and debug.
        """
        dom = os.environ.get("ROS_DOMAIN_ID", "(unset -> 0)")
        print("preflight: ROS_DOMAIN_ID=%s (waiting up to %.0f s for the "
              "graph)" % (dom, wait))
        t0 = time.time()
        while time.time() - t0 < wait:
            self.spin(0.5)
            if self.q is not None and self.arm.get_subscription_count() >= 1:
                break
            if int(time.time() - t0) % 5 == 0:
                print("   ... %4.1f s: arm_subs=%d joint_states=%s"
                      % (time.time() - t0, self.arm.get_subscription_count(),
                         "yes" if self.q is not None else "no"),
                      flush=True)
                self.spin(1.0)
        n_arm = self.arm.get_subscription_count()
        n_grip = self.grip.get_subscription_count()
        print("  %-46s %d" % ("subscribers on the arm command topic:", n_arm))
        print("  %-46s %d" % ("subscribers on the gripper command topic:", n_grip))
        print("  %-46s %s" % ("/real/joint_states seen:",
                              "yes" if self.q is not None else "NO"))
        problems = []
        if self.q is None:
            problems.append(
                "no /real/joint_states -- the bridge is not on this "
                "ROS_DOMAIN_ID (it is on 7), or it is not running")
        if n_arm < 1:
            problems.append(
                "nothing is subscribed to %s -- the bridge is not listening; "
                "wrong domain, or the bridge died" % self._arm_topic)
        if problems:
            print("\nPREFLIGHT FAILED:")
            for p in problems:
                print("  * " + p)
            return False
        print("  preflight OK")
        return True

    def set_deadband(self, deg):
        """Lower the bridge's proportional deadband for this run."""
        cli = self.create_client(SetParameters,
                                 self._param_srv)
        if not cli.wait_for_service(timeout_sec=5.0):
            print("  (could not reach the bridge parameter service; "
                  "leaving the deadband alone)")
            return False
        p = Parameter()
        p.name = "deadband_deg"
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(deg))
        req = SetParameters.Request()
        req.parameters = [p]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        ok = bool(res and res.results and res.results[0].successful)
        print("  deadband_deg -> %.2f : %s" % (deg, "OK" if ok else "REFUSED"))
        return ok

    # ------------------------------------------------------------- commands
    def send_arm(self, q):
        t = JointTrajectory()
        t.joint_names = self.names
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in q]
        p.time_from_start = Duration(sec=0, nanosec=100_000_000)
        t.points = [p]
        self.arm.publish(t)

    def send_grip(self, rad):
        t = JointTrajectory()
        t.joint_names = ["%s_robotiq_85_left_knuckle_joint" % self.arm_name]
        p = JointTrajectoryPoint()
        p.positions = [float(rad)]
        p.time_from_start = Duration(sec=0, nanosec=100_000_000)
        t.points = [p]
        self.grip.publish(t)

    def hold_gripper(self, rad, secs, label):
        print("  %s (knuckle %.2f rad) for %.1f s" % (label, rad, secs))
        t0 = time.time()
        while time.time() - t0 < secs:
            self.send_grip(rad)
            if self._hold is not None:
                self.send_arm(self._hold)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(1.0 / RATE)

    def goto(self, q_target, label):
        q_start = self.fresh_q().copy()
        d = ang_wrap(np.array(q_target) - q_start)
        span = math.degrees(np.abs(d).max())
        secs = max(1.0, span / DEG_PER_S)
        steps = max(2, int(secs * RATE))
        print("  %s: worst joint moves %.2f deg over %.1f s (%d steps)"
              % (label, span, secs, steps))
        t_begin = time.time()
        checked = False
        for k in range(1, steps + 1):
            a = k / steps
            a = a * a * (3 - 2 * a)
            self._hold = q_start + d * a
            self.send_arm(self._hold)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(1.0 / RATE)
            j = self.force_jump()
            if j > FORCE_JUMP_N:
                print("     COLLISION GUARD: tool force rose %.1f N in 1.5 s "
                      "(limit %.1f). STOPPING at once." % (j, FORCE_JUMP_N))
                self._hold = self.fresh_q().copy()
                return None
            if not checked and time.time() - t_begin > MOVE_CHECK_S:
                checked = True
                moved = math.degrees(np.abs(ang_wrap(self.fresh_q() - q_start)).max())
                print("     moved %.3f deg after %.1f s" % (moved, MOVE_CHECK_S))
                if moved < MOVE_MIN_DEG and span > 1.0:
                    print("     ARM IS NOT RESPONDING to targets. Aborting "
                          "before the whole ramp is played out.\n"
                          "     The bridge is receiving them (preflight "
                          "passed), so suspect: e-stop latched, arm not in "
                          "SERVOING_READY, or a fault bank set on the arm.")
                    return None
        # Hold the target and WAIT FOR ARRIVAL.
        self._hold = np.array(q_target)
        t0 = time.time()
        best = float("inf")
        t_improve = t0
        err = float("inf")
        while time.time() - t0 < CONVERGE_MAX_S:
            self.send_arm(self._hold)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(1.0 / RATE)
            err = math.degrees(np.abs(ang_wrap(self.fresh_q() - self._hold)).max())
            if err < ARRIVE_TOL_DEG:
                print("     arrived in %.1f s; worst joint error %.3f deg"
                      % (time.time() - t0, err))
                return err
            if err < best - STALL_MIN_DEG:
                best = err
                t_improve = time.time()
            elif time.time() - t_improve > STALL_S:
                print("     STALLED at %.3f deg after %.1f s (no improvement "
                      "for %.1f s)" % (err, time.time() - t0, STALL_S))
                return err
        print("     did not converge in %.1f s; worst joint error %.3f deg"
              % (CONVERGE_MAX_S, err))
        return err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    plan = json.load(open(args.plan))
    if not plan.get("ok"):
        raise SystemExit("plan is not marked safe; refusing to run")
    print("stages:")
    for s in plan["plan"]:
        print("  %-10s pos_err %.2f mm  rot_err %.2f deg  pad above table %.1f mm"
              % (s["name"], s["pos_err_mm"], s["rot_err_deg"],
                 s["pad_above_table_mm"]))

    rclpy.init()
    node = Executor()
    if not node.preflight():
        raise SystemExit(2)
    if args.dry_run:
        print("\ndry run: nothing commanded")
        return

    node.set_deadband(0.10)
    print("start q (deg):", [round(math.degrees(x), 2) for x in node.q])
    node._hold = node.q.copy()

    node.hold_gripper(GRIP_OPEN, 2.5, "OPEN gripper")
    for stage in plan["plan"]:
        print("\n%s" % stage["name"])
        err = node.goto(np.array(stage["q"]), stage["name"])
        if err is None:
            break
        if err > SETTLE_TOL_DEG:
            print("ABORT: %s did not arrive (%.2f deg > %.2f). Holding."
                  % (stage["name"], err, SETTLE_TOL_DEG))
            break
        if stage["name"] == "GRASP":
            node.hold_gripper(GRIP_CLOSE, 4.0, "CLOSE gripper on the cube")

    print("\nfinal q (deg):", [round(math.degrees(x), 2) for x in node.q])
    t0 = time.time()
    while time.time() - t0 < 2.0:
        node.send_arm(node._hold)
        node.send_grip(GRIP_CLOSE)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.05)
    print("DONE")


if __name__ == "__main__":
    main()
