#!/usr/bin/env python3
"""
real_robot_state_display.py — READ-ONLY visualization of the real robot's
exact current joint values in RViz, highlighted in RED.
=============================================================================
SAFETY: this node NEVER publishes to any controller or command topic.
It only SUBSCRIBES to /joint_states (published by kortex_driver when
running with use_fake_hardware:=false, i.e. real hardware) and republishes
a moveit_msgs/DisplayRobotState overlay -- a pure visualization message
that RViz can show as a colored "ghost" robot, with no effect whatsoever
on the actual arms. There is no code path in this file that can move
the real robot.

Covers ALL joints found on the leader/real /joint_states message --
both arms, any grippers -- not just one arm.

Run:
  ros2 run srl_teleop real_robot_state_display

Then in RViz:
  Add -> RobotState
  Set "Robot State Topic" to /real_robot_state
  (Robot Description stays 'robot_description', same URDF)
This shows the real robot's exact current pose as a red overlay, distinct
from the normal (possibly sim/fake) robot model already displayed.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayRobotState, ObjectColor
from std_msgs.msg import ColorRGBA


RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)


class RealRobotStateDisplay(Node):
    def __init__(self):
        super().__init__("real_robot_state_display")

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("display_topic", "/real_robot_state")

        js_topic = self.get_parameter("joint_states_topic").value
        display_topic = self.get_parameter("display_topic").value

        # READ-ONLY: only a subscriber to joint states.
        self.create_subscription(JointState, js_topic, self.on_joint_state, 10)

        # Only publisher in this whole node: a pure visualization message.
        # DisplayRobotState is NOT a command -- it cannot move the robot.
        self.display_pub = self.create_publisher(
            DisplayRobotState, display_topic, 10)

        self.get_logger().info(
            "real_robot_state_display started -- READ ONLY, no commands "
            "are ever sent to the robot.")
        self.get_logger().info(f"  Reading real values from: {js_topic}")
        self.get_logger().info(f"  Publishing red overlay to: {display_topic}")
        self.get_logger().info(
            "  In RViz: Add -> RobotState, set topic to "
            f"'{display_topic}'")

    def on_joint_state(self, msg: JointState):
        display = DisplayRobotState()
        display.state.joint_state = msg   # exact real values, unmodified

        # Color every link red so it's visually unmistakable that this
        # overlay is the REAL robot's exact current pose.
        for link_name in self._link_names_from_joint_names(msg.name):
            oc = ObjectColor()
            oc.id = link_name
            oc.color = RED
            display.highlight_links.append(oc)

        self.display_pub.publish(display)

    @staticmethod
    def _link_names_from_joint_names(joint_names):
        # Best-effort: Kinova/URDF convention here is "<prefix>joint_N" ->
        # highlighting is per-link, but since we don't have the full link
        # list without parsing the URDF, we approximate using known link
        # name patterns. If a link doesn't exist, RViz just ignores that
        # entry -- harmless either way, purely cosmetic.
        links = set()
        for j in joint_names:
            prefix = j.split("joint_")[0] if "joint_" in j else ""
            links.update([
                f"{prefix}base_link", f"{prefix}shoulder_link",
                f"{prefix}half_arm_1_link", f"{prefix}half_arm_2_link",
                f"{prefix}forearm_link", f"{prefix}spherical_wrist_1_link",
                f"{prefix}spherical_wrist_2_link", f"{prefix}bracelet_link",
                f"{prefix}end_effector_link",
                f"{prefix}robotiq_85_base_link",
                f"{prefix}robotiq_85_left_finger_tip_link",
                f"{prefix}robotiq_85_right_finger_tip_link",
            ])
        return links


def main(args=None):
    rclpy.init(args=args)
    node = RealRobotStateDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
