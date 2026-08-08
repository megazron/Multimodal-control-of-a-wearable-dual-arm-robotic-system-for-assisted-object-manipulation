#!/usr/bin/env python3
"""
selftest_node.py — prove the arm ACTUALLY MOVES, at startup, automatically.

Three times now a safety mechanism has silently blocked all motion and the
system looked like dead hardware:
  1. the IK guard deadlock  (reject_count == success_count, arm never left home)
  2. the pot validator lockup (every frame rejected, pose frozen)
  3. the e-stop dead-man latching on a CPU hiccup (only the e-stop's own
     hold-position was reaching the controller, at 10 Hz)

Every one of those published messages. Counting messages is not evidence.
The only evidence is MEASURED END-EFFECTOR DISPLACEMENT, so that is what this
checks: nudge the anchor a few centimetres, watch tf2, put it back.

It fails LOUD. A silent block is the failure mode being guarded against, so a
quiet log line would defeat the purpose.

Run standalone:   ros2 run srl_teleop selftest_node
In the launch:    self_test:=true   (runs once, ~20 s after startup)
"""
import subprocess
import sys
import threading
import time

# INTERVALS USE time.monotonic(). Under WSL the wall clock steps
# backwards on host resync - it produced a measured send latency of
# -2321 ms once. Wall-clock time.time() is kept ONLY where the value
# is a human-readable timestamp, never for a duration.

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

NUDGE_M = 0.08          # anchor offset applied during the test
SETTLE_S = 7.0          # time allowed for the arm to follow
PASS_M = 0.010          # EE must move at least this far to count as alive


def _wait(fut, timeout):
    """Wait on a future that a BACKGROUND executor is servicing."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if fut.done():
            return fut.result() is not None
        time.sleep(0.02)
    return False


class SelfTest(Node):
    def __init__(self):
        super().__init__("selftest_node")
        self.declare_parameter("arms", ["left", "right"])
        self.declare_parameter("nudge_m", NUDGE_M)
        self.declare_parameter("settle_s", SETTLE_S)
        self.declare_parameter("pass_m", PASS_M)
        self.declare_parameter("startup_delay_s", 8.0)
        self.arms = list(self.get_parameter("arms").value)
        self.nudge = float(self.get_parameter("nudge_m").value)
        self.settle = float(self.get_parameter("settle_s").value)
        self.pass_m = float(self.get_parameter("pass_m").value)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.estopped = None
        self.create_subscription(Bool, "/estop_state",
                                 lambda m: setattr(self, "estopped", m.data), 10)
        self.cli = self.create_client(
            SetParameters, "/master_pose_node/set_parameters")

    def ee(self, arm):
        try:
            t = self.buf.lookup_transform(
                "world", f"{arm}_end_effector_link",
                rclpy.time.Time()).transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None

    def set_offset(self, arm, axis, value):
        # Deliberately the CLI, not an in-process service client. An
        # in-process client competes with the background executor that is
        # feeding the TF buffer; the buffer went stale and the test reported
        # 0.0000 m for an arm that was measurably moving 0.1000 m.
        r = subprocess.run(
            ["ros2", "param", "set", "/master_pose_node",
             f"{arm}_offset_{axis}", str(float(value))],
            capture_output=True, text=True, timeout=25)
        return "successful" in r.stdout.lower()

    def _unused_set_offset(self, arm, axis, value):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            return False
        req = SetParameters.Request()
        p = Parameter()
        p.name = f"{arm}_offset_{axis}"
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(value))
        req.parameters = [p]
        # Do NOT spin_until_future_complete here: a background executor is
        # already spinning this node, and two spinners on one node drop
        # callbacks -- which stalls the TF buffer and makes the arm look
        # frozen when it is actually moving. Just wait for the future.
        f = self.cli.call_async(req)
        return _wait(f, 5.0)

    def get_offset(self, arm, axis):
        r = subprocess.run(
            ["ros2", "param", "get", "/master_pose_node",
             f"{arm}_offset_{axis}"],
            capture_output=True, text=True, timeout=25)
        for tok in r.stdout.split():
            try:
                return float(tok)
            except ValueError:
                continue
        return None

    def _unused_get_offset(self, arm, axis):
        from rcl_interfaces.srv import GetParameters
        c = self.create_client(GetParameters, "/master_pose_node/get_parameters")
        if not c.wait_for_service(timeout_sec=5.0):
            return None
        r = GetParameters.Request(); r.names = [f"{arm}_offset_{axis}"]
        f = c.call_async(r)
        if not _wait(f, 5.0):
            return None
        res = f.result()
        return res.values[0].double_value if res and res.values else None

    def run(self):
        ok_all = True
        print("\n" + "=" * 70)
        print("STARTUP SELF-TEST -- does the arm actually MOVE?")
        print("=" * 70)

        if self.estopped:
            print("  !! E-STOP IS LATCHED. Nothing can move until /estop_reset.")
            print("     This is the single most likely reason the sim looks dead.")
            return False

        for arm in self.arms:
            base = self.get_offset(arm, "x")
            start = self.ee(arm)
            if base is None or start is None:
                print("  %-6s SKIP -- no anchor param or no tf2 for "
                      "%s_end_effector_link" % (arm, arm))
                ok_all = False
                continue

            self.set_offset(arm, "x", base + self.nudge)
            time.sleep(self.settle)
            moved = self.ee(arm)
            self.set_offset(arm, "x", base)
            time.sleep(self.settle * 0.5)
            back = self.ee(arm)

            d = float(np.linalg.norm(moved - start)) if moved is not None else 0.0
            r = float(np.linalg.norm(back - start)) if back is not None else -1.0
            ok = d >= self.pass_m
            ok_all &= ok
            print("  %-6s anchor nudged %+.3f m -> EE moved %.4f m "
                  "(returned to within %.4f m)  %s"
                  % (arm, self.nudge, d, r, "PASS" if ok else "*** FAIL ***"))
            if not ok:
                print("         EE moved less than %.3f m. Messages may still be"
                      % self.pass_m)
                print("         publishing -- that is NOT evidence of motion.")
                print("         Check, in order: /estop_state latched;")
                print("         [SAFETY] clearance holds; [GUARD] all-rejected;")
                print("         clutch disengaged; max_vel_rad_s tiny.")

        print("=" * 70)
        print("SELF-TEST %s" % ("PASSED - the arm moves." if ok_all
                                else "*** FAILED - THE ARM IS NOT MOVING ***"))
        print("=" * 70 + "\n")
        return ok_all


def main(args=None):
    rclpy.init(args=args)
    n = SelfTest()
    ex = SingleThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(float(n.get_parameter("startup_delay_s").value))
    try:
        ok = n.run()
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
