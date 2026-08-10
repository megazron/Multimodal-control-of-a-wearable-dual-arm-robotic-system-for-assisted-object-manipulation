#!/usr/bin/env python3
"""
vr_gripper_node.py — trigger -> Robotiq, with the SAME latch behaviour as the
mannequin's FSR path.

Deliberately identical semantics, because the two input modalities are two
CONDITIONS in one study: if the gripper behaved differently, a difference in
grasp success would be partly the gripper and nobody could separate the two.

From the FSR node, restated in trigger units (0..1 instead of 0..4095):
  * proportional control on the way in;
  * LATCH CLOSED above `latch_close`, and hold the firmest value seen, so a
    tiring hand does not drop the object;
  * release only below `latch_open` held for `release_hold_s`, so a momentary
    dip does not open the hand;
  * output hysteresis, because a new setpoint every frame at 72 Hz makes the
    fingers buzz;
  * a deadband, so trigger noise at rest does not creep the gripper shut.

THE GRIPPER IS NEVER CLOSED BY THE AUTONOMY. Same rule as the mannequin path.
"""
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from srl_teleop import gripper_state as _gs

# FROM THE OWNER, not restated. These were literals here (0.0, 0.80) while
# srl_teleop.gripper_state calls 0.10 "OPEN_RAD" -- this file's 0.0 is the
# COMMAND to open, which is a different quantity from the threshold that
# classifies a hand as open. Both now come from the one module that defines
# them, under names that say which is which.
OPEN_RAD, CLOSED_RAD = _gs.CMD_OPEN_RAD, _gs.CLOSED_RAD


class VrGripper(Node):
    def __init__(self):
        super().__init__('vr_gripper_node')
        self.declare_parameter('hands', ['left', 'right'])
        self.declare_parameter('left_controller_arm', 'left')
        self.declare_parameter('right_controller_arm', 'right')
        self.declare_parameter('deadband', 0.06)
        self.declare_parameter('latch_close', 0.30)     # ~ FSR 1200/4095
        self.declare_parameter('latch_open', 0.10)      # ~ FSR  400/4095
        self.declare_parameter('release_hold_s', 0.5)
        self.declare_parameter('hysteresis_rad', 0.02)
        self.declare_parameter('rate_hz', 50.0)

        self.hands = list(self.get_parameter('hands').value)
        self.arm_of = {'left': self.get_parameter('left_controller_arm').value,
                       'right': self.get_parameter('right_controller_arm').value}
        self.trig = {h: 0.0 for h in self.hands}
        self.latched = {h: False for h in self.hands}
        self.hold = {h: 0.0 for h in self.hands}
        self.low_since = {h: None for h in self.hands}
        self.out = {h: OPEN_RAD for h in self.hands}

        self.pub, self.st = {}, {}
        for h in self.hands:
            arm = self.arm_of[h]
            self.pub[h] = self.create_publisher(
                JointTrajectory, f'/{arm}_gripper_controller/joint_trajectory', 10)
            self.st[h] = self.create_publisher(String, f'/vr/gripper_{h}', 10)
            self.create_subscription(Joy, f'/vr/controller_joy_{h}',
                                     lambda m, h=h: self._on_joy(h, m), 20)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)

    def _on_joy(self, hand, m):
        if m.axes:
            self.trig[hand] = float(m.axes[0])

    def _tick(self):
        db = float(self.get_parameter('deadband').value)
        lc = float(self.get_parameter('latch_close').value)
        lo = float(self.get_parameter('latch_open').value)
        hold_s = float(self.get_parameter('release_hold_s').value)
        hyst = float(self.get_parameter('hysteresis_rad').value)
        now = self.get_clock().now().nanoseconds * 1e-9

        for h in self.hands:
            t = self.trig[h]
            cmd = 0.0 if t < db else (t - db) / max(1e-6, 1.0 - db) * CLOSED_RAD
            if not self.latched[h]:
                if t > lc:
                    self.latched[h] = True
                    self.hold[h] = cmd
                    self.low_since[h] = None
                    self.get_logger().info('[%s] gripper LATCHED at %.3f rad' % (h, cmd))
            else:
                self.hold[h] = max(self.hold[h], cmd)
                cmd = self.hold[h]
                if t < lo:
                    self.low_since[h] = self.low_since[h] or now
                    if now - self.low_since[h] > hold_s:
                        self.latched[h] = False
                        self.hold[h] = 0.0
                        self.get_logger().info('[%s] gripper RELEASED' % h)
                else:
                    self.low_since[h] = None

            if abs(cmd - self.out[h]) > hyst or cmd in (0.0,):
                self.out[h] = cmd
                jt = JointTrajectory()
                arm = self.arm_of[h]
                jt.joint_names = [f'{arm}_robotiq_85_left_knuckle_joint']
                p = JointTrajectoryPoint()
                p.positions = [float(self.out[h])]
                p.time_from_start.sec = 0
                p.time_from_start.nanosec = 200_000_000
                jt.points.append(p)
                self.pub[h].publish(jt)

            s = String()
            s.data = json.dumps(dict(hand=h, arm=self.arm_of[h],
                                     trigger=round(t, 3),
                                     command_rad=round(self.out[h], 4),
                                     latched=self.latched[h],
                                     closed_by='operator'))
            self.st[h].publish(s)


def main():
    rclpy.init()
    n = VrGripper()
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
