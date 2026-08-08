#!/usr/bin/env python3
"""
blocking_aggregator.py — ONE PLACE that answers "why isn't the arm moving?".

Every BlockMonitor in the system publishes to /blocking. This node collects
them, and does the two things a per-node monitor cannot:

  * NOTICES A UNIT THAT HAS GONE SILENT. A node that crashed publishes no
    blocking state at all, which from a dashboard looks identical to a node
    that is fine. Silence is treated as a fault after `unit_timeout_s`.
  * NOTICES THAT NOTHING IS MOVING WHILE NOTHING IS BLOCKING. That is the
    worst case and the one that has actually happened: every guard says it is
    fine and the arm still does not move. If commands are flowing and the
    joints are not changing and no blocker is active, that is reported as
    UNEXPLAINED, which is a much more useful thing to see than silence.

Publishes /blocking_summary and prints a compact ERROR banner when anything is
blocking for more than the escalation window.
"""
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String


class BlockingAggregator(Node):
    def __init__(self):
        super().__init__('blocking_aggregator')
        self.declare_parameter('arms', ['left', 'right'])
        self.declare_parameter('unit_timeout_s', 5.0)
        self.declare_parameter('escalate_s', 3.0)
        self.declare_parameter('motion_epsilon_rad', 1e-4)
        self.declare_parameter('unexplained_after_s', 3.0)

        self.arms = list(self.get_parameter('arms').value)
        self.units = {}
        self.last_cmd_t = {a: 0.0 for a in self.arms}
        self.joints = {}
        self.last_joints = {}
        self.last_motion_t = time.monotonic()
        self.unexplained_since = None

        self.create_subscription(String, '/blocking', self._on_block, 20)
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        for a in self.arms:
            self.create_subscription(
                PoseStamped, f'/master_arm_pose_{a}',
                lambda m, a=a: self.last_cmd_t.__setitem__(a, time.monotonic()), 10)
        self.pub = self.create_publisher(String, '/blocking_summary', 10)
        self.unexplained_pub = self.create_publisher(Bool, '/motion_unexplained', 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            'blocking_aggregator up. One place to answer "why is the arm not '
            'moving?" - including the case where NOTHING is blocking and it '
            'still is not moving.')

    def _on_block(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        d['_rx'] = time.monotonic()
        self.units[d.get('unit', '?')] = d

    def _on_js(self, m):
        cur = dict(zip(m.name, m.position))
        if self.last_joints:
            moved = max((abs(cur[k] - self.last_joints.get(k, cur[k]))
                         for k in cur if k.endswith(tuple('1234567'))), default=0.0)
            if moved > float(self.get_parameter('motion_epsilon_rad').value):
                self.last_motion_t = time.monotonic()
        self.last_joints = cur

    def _tick(self):
        now = time.monotonic()
        timeout = float(self.get_parameter('unit_timeout_s').value)
        esc = float(self.get_parameter('escalate_s').value)

        silent = [u for u, d in self.units.items() if now - d['_rx'] > timeout]
        blocking = []
        unknown = []
        for u, d in self.units.items():
            if now - d['_rx'] > timeout:
                continue
            for b in d.get('blockers', []):
                if b.get('active'):
                    blocking.append((u, b['name'], b.get('held_s', 0.0),
                                     b.get('reason', ''), b.get('recovery', '')))
                elif b.get('expired'):
                    # EXPIRED IS NOT CLEAR. The unit that was asserting this
                    # blocker stopped running, so the condition is unknown.
                    # It is counted as blocking so that "nothing is blocking"
                    # can never be produced by a crashed asserting loop.
                    unknown.append((u, b['name'], b.get('unknown_for_s', 0.0)))
                    blocking.append((u, b['name'] + ':EXPIRED',
                                     b.get('unknown_for_s', 0.0),
                                     'asserting unit stopped; state UNKNOWN',
                                     b.get('recovery', '')))

        cmds_flowing = any(now - t < 1.0 for t in self.last_cmd_t.values())
        still = now - self.last_motion_t
        unexplained = (cmds_flowing and not blocking and
                       still > float(self.get_parameter('unexplained_after_s').value))
        if unexplained and self.unexplained_since is None:
            self.unexplained_since = now
        elif not unexplained:
            self.unexplained_since = None

        for u, n, age in unknown:
            self.get_logger().error(
                'STATE UNKNOWN: %s/%s expired %.1f s ago. The blocker was '
                'neither held nor cleared - whatever asserts it STOPPED '
                'RUNNING. This is counted as BLOCKING, never as an all-clear.'
                % (u, n, age), throttle_duration_sec=5.0)
        if silent:
            self.get_logger().error(
                'UNIT SILENT: %s has published no blocking state for > %.0f s. '
                'A crashed node looks exactly like a healthy one on a '
                'dashboard; it is reported as a FAULT here.'
                % (', '.join(silent), timeout), throttle_duration_sec=10.0)
        for u, n, held, reason, rec in blocking:
            if held > esc:
                self.get_logger().error(
                    'BLOCKING %.1f s — %s/%s: %s. Recovery: %s'
                    % (held, u, n, reason or '-', rec), throttle_duration_sec=5.0)
        if unexplained:
            self.get_logger().error(
                'MOTION UNEXPLAINED: commands are flowing, the joints have not '
                'moved for %.1f s, and NO blocker is active. This is the '
                'silent-stall signature. Check the controllers '
                '(ros2 control list_controllers) and /joint_states rate.'
                % still, throttle_duration_sec=5.0)

        b = Bool()
        b.data = bool(unexplained)
        self.unexplained_pub.publish(b)
        m = String()
        m.data = json.dumps(dict(
            units=sorted(self.units), silent_units=silent,
            blocking=[dict(unit=u, name=n, held_s=round(h, 2), reason=r)
                      for u, n, h, r, _ in blocking],
            state_unknown=[dict(unit=u, name=n, unknown_for_s=round(a, 2))
                           for u, n, a in unknown],
            commands_flowing=cmds_flowing,
            seconds_since_motion=round(still, 2),
            motion_unexplained=bool(unexplained)))
        self.pub.publish(m)


def main():
    rclpy.init()
    n = BlockingAggregator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
