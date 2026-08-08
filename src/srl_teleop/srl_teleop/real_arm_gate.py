#!/usr/bin/env python3
"""
real_arm_gate.py — the typed yes/no between sim teleoperation and real motion.

Sim teleoperation is live the moment teleop.launch.py comes up. Nothing is
connected to the real arms and nothing can command them, because the real
driver stack is not running at all. This node prints:

    Sim teleoperation active. Connect and home the REAL arms? [y/N]

and blocks on stdin. ONLY the literal 'y' or 'yes' proceeds. Anything else,
EOF, or no answer at all leaves the system exactly as it is -- sim-only,
forever, with no timeout that could start hardware while nobody is watching.
"Default no" here means "no" is what happens when the operator does nothing,
which is the only default a gate like this may have.

On 'y' it starts real_arms.launch.py as a child process, which brings up the
Kortex driver, homes under the velocity law, and enables the cascade.

WHY A CHILD LAUNCH RATHER THAN A CONDITIONED INCLUDE. Launch resolves
conditions once, at description time -- there is no way to include a stack
"later, if a human types something". Spawning a child `ros2 launch` is the
mechanism that actually matches the requirement, and it has a second benefit:
killing the child tears the real stack down without disturbing the sim.

STDIN. Under `ros2 launch` all nodes share one stdin and launch does not give
it to any of them, so this node is run with a pty by the launch file
(emulate_tty) and reads /dev/tty directly when stdin is not usable. If
neither is available it says so and stays sim-only rather than guessing.
"""
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

BANNER = """
======================================================================
  SIM TELEOPERATION IS ACTIVE.
  The real arms are NOT connected and cannot be commanded -- the real
  driver stack is not running.

  Answering 'y' will:
    1. connect the Kortex driver to the LEFT arm
    2. home it under a velocity law (kp 0.5, vmax 0.05 rad/s)
    3. enable the sim->real cascade, 1.0 s behind the sim

  Keep a hand on the e-stop before you answer.
======================================================================
"""


class Gate(Node):
    def __init__(self):
        super().__init__("real_arm_gate")
        self.declare_parameter("arm", "left")
        self.declare_parameter("prompt_delay_s", 8.0)
        self.declare_parameter("preview_delay_s", 1.0)
        self.declare_parameter("max_vel_rad_s", 0.05)
        self.declare_parameter("max_step_rad", 0.05)
        self.declare_parameter("lag_trip_rad", 0.15)
        self.arm = self.get_parameter("arm").value
        self.child = None
        self.pub = self.create_publisher(Bool, "/real_arms_requested", 10)
        self.create_subscription(Bool, "/estop_state", self.on_estop, 10)
        self.estopped = False
        threading.Thread(target=self.run, daemon=True).start()

    def on_estop(self, m):
        if m.data and not self.estopped and self.child is not None:
            self.get_logger().error(
                "E-STOP with the real stack running. The Kortex router cannot "
                "be reactivated in-process, so the real stack is being torn "
                "down; relaunch through the gate to recover.")
            self.stop_child()
        self.estopped = bool(m.data)

    # ---------------- prompt ----------------

    def _readline(self):
        """stdin if it is a terminal, else /dev/tty, else None."""
        try:
            if sys.stdin is not None and sys.stdin.isatty():
                return sys.stdin.readline()
        except Exception:
            pass
        try:
            with open("/dev/tty", "r") as t:
                return t.readline()
        except Exception:
            return None

    def run(self):
        time.sleep(float(self.get_parameter("prompt_delay_s").value))
        print(BANNER, flush=True)
        print("Sim teleoperation active. Connect and home the REAL arms? "
              "[y/N] ", end="", flush=True)
        ans = self._readline()
        if ans is None:
            print("\n[gate] No terminal available for input. Staying SIM-ONLY.\n"
                  "[gate] To go live, run in a terminal:\n"
                  "[gate]   ros2 launch srl_teleop real_arms.launch.py\n",
                  flush=True)
            return
        ans = ans.strip().lower()
        if ans not in ("y", "yes"):
            print("\n[gate] Answer was %r -- staying SIM-ONLY. Nothing is "
                  "connected to the real arms.\n" % ans, flush=True)
            self.pub.publish(Bool(data=False))
            return
        if self.estopped:
            print("\n[gate] E-STOP is latched. Reset it before connecting.\n",
                  flush=True)
            return
        self.pub.publish(Bool(data=True))
        self.start_child()

    # ---------------- child stack ----------------

    def start_child(self):
        cmd = ["ros2", "launch", "srl_teleop", "real_arms.launch.py",
               "arm:=%s" % self.arm,
               "preview_delay_s:=%s" % self.get_parameter("preview_delay_s").value,
               "max_vel_rad_s:=%s" % self.get_parameter("max_vel_rad_s").value,
               "max_step_rad:=%s" % self.get_parameter("max_step_rad").value,
               "lag_trip_rad:=%s" % self.get_parameter("lag_trip_rad").value]
        print("\n[gate] starting: %s\n" % " ".join(cmd), flush=True)
        # start_new_session so the whole child launch tree can be signalled as
        # one process group -- ros2 launch spawns several children of its own.
        self.child = subprocess.Popen(cmd, start_new_session=True)
        self.get_logger().warn(
            "REAL ARM STACK STARTING (pid %d). Homing will begin shortly."
            % self.child.pid)

    def stop_child(self):
        if self.child is None:
            return
        try:
            os.killpg(os.getpgid(self.child.pid), signal.SIGINT)
        except Exception:
            pass
        self.child = None


def main(args=None):
    rclpy.init(args=args)
    n = Gate()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.stop_child()
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
