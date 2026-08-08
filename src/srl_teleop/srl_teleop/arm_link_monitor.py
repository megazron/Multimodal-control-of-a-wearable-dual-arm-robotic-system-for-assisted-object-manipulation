#!/usr/bin/env python3
"""arm_link_monitor.py — is the network to each arm actually up?

Publishes /arm_link_status, which recovery_manager consumes to freeze BOTH
arms on a link loss. Separated from the recovery logic on purpose: this is the
only part that touches the network, so recovery can be tested by publishing
this topic without anyone faking a network outage.

Real arms: LEFT 192.168.1.10, RIGHT 192.168.1.9.

WHY PING AND NOT "the driver looks fine". The Kortex driver's feedback goes
FLAT rather than absent when the link degrades -- measured on this rig,
/real/joint_states kept publishing at 100 Hz with one distinct value per joint
and std 0.000e+00. A frozen cache is indistinguishable from a still arm on
every topic the driver owns, so link health has to be measured off the driver
entirely.
"""
import json
import shutil
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_HOSTS = {"left": "192.168.1.10", "right": "192.168.1.9"}


class ArmLinkMonitor(Node):
    def __init__(self):
        super().__init__("arm_link_monitor")
        self.declare_parameter("left_ip", DEFAULT_HOSTS["left"])
        self.declare_parameter("right_ip", DEFAULT_HOSTS["right"])
        self.declare_parameter("period_s", 1.0)
        self.declare_parameter("timeout_s", 1.0)
        # Consecutive failures before calling it down. A single dropped ping
        # is normal; freezing a wearer's arms on one is not.
        self.declare_parameter("fail_threshold", 2)
        self.declare_parameter("enabled", True)

        self.fails = {"left": 0, "right": 0}
        self.pub = self.create_publisher(String, "/arm_link_status", 10)
        self.create_timer(float(self.get_parameter("period_s").value), self._tick)
        if shutil.which("ping") is None:
            self.get_logger().error(
                "no `ping` binary -- link monitoring is DISABLED and this node "
                "will report nothing rather than reporting a false 'up'.")
        self.get_logger().info("arm_link_monitor up (left %s, right %s)"
                               % (self.get_parameter("left_ip").value,
                                  self.get_parameter("right_ip").value))

    def _ping(self, ip):
        to = float(self.get_parameter("timeout_s").value)
        try:
            r = subprocess.run(
                ["ping", "-c", "1", "-W", str(int(max(1, to))), ip],
                capture_output=True, timeout=to + 2.0)
            return r.returncode == 0
        except Exception:                                        # noqa: BLE001
            return False

    def _tick(self):
        if not bool(self.get_parameter("enabled").value):
            return
        if shutil.which("ping") is None:
            return
        thr = int(self.get_parameter("fail_threshold").value)
        arms = {}
        for a in ("left", "right"):
            ip = self.get_parameter("%s_ip" % a).value
            ok = self._ping(ip)
            self.fails[a] = 0 if ok else self.fails[a] + 1
            up = self.fails[a] < thr
            arms[a] = dict(ip=ip, up=bool(up), consecutive_failures=self.fails[a])
            if not up and self.fails[a] == thr:
                self.get_logger().error(
                    "link to %s arm (%s) is DOWN after %d consecutive failed "
                    "pings. recovery_manager will freeze BOTH arms."
                    % (a, ip, thr))
        m = String()
        m.data = json.dumps(dict(t=time.time(), arms=arms))
        self.pub.publish(m)


def main():
    rclpy.init()
    n = ArmLinkMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
