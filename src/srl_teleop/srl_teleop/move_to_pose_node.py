#!/usr/bin/env python3
"""
move_to_pose_node.py — drive the arm to a target pose via MoveIt (moveit_py)
=============================================================================
Subscribes to /located_object_pose and plans+executes a motion to it using
MoveIt's Python API (moveit_py). Runs ALONGSIDE the already-launched
srl_moveit_config demo.launch.py -- the resulting motion appears live in
that same RViz window, driven through the same real controllers.

TEST RIGHT NOW WITHOUT VISION/CAMERA (manual pose, proves the pose->motion
half of the pipeline):
  Terminal 1: ros2 launch srl_moveit_config demo.launch.py
  Terminal 2: ros2 run srl_teleop move_to_pose_node
  Terminal 3: ros2 topic pub --once /located_object_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: "world"}, pose: {position: {x: 0.3, y: 0.0, z: 1.2}, orientation: {w: 1.0}}}'

Once vision is live, vlm_locate_node publishes to this same topic and the
arm will move to the real detected object instead of a manual test pose.

CAVEAT: this runs a SECOND, in-process MoveIt planning pipeline (moveit_py
instantiates its own, it is not just a thin client to demo.launch.py's
move_group node). If you see node-name or planning-scene conflicts with
the move_group already running from demo.launch.py, that's the likely
cause -- tell me and we'll adjust (e.g. run demo.launch.py without its own
move_group, letting this node be the sole planner).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy


class MoveToPoseNode(Node):
    def __init__(self):
        super().__init__("move_to_pose_node")

        self.declare_parameter("planning_group", "left_arm")
        self.declare_parameter("ee_link", "left_end_effector_link")
        self.group_name = self.get_parameter("planning_group").value
        self.ee_link = self.get_parameter("ee_link").value

        # Config now comes from ROS parameters injected by the companion
        # launch file (move_to_pose.launch.py) -- matching the same proven
        # mechanism move_group.launch.py uses. Passing config_dict directly
        # from a plain `ros2 run` script did not load planning pipelines
        # correctly; going through launch-based parameters does.
        self.get_logger().info("Starting MoveItPy (this can take a few seconds)...")
        self.moveit = MoveItPy(node_name="move_to_pose_moveit_py")
        self.arm = self.moveit.get_planning_component(self.group_name)
        self.get_logger().info(
            f"MoveItPy ready. Group='{self.group_name}' ee_link='{self.ee_link}'")

        self.create_subscription(PoseStamped, "/located_object_pose",
                                 self.on_target_pose, 10)
        self.get_logger().info(
            "Waiting on /located_object_pose ... "
            "(publish a PoseStamped there -- via vlm_locate_node, or "
            "manually with `ros2 topic pub` for a test)")

    def on_target_pose(self, msg: PoseStamped):
        self.get_logger().info(
            f"Target received: ({msg.pose.position.x:.3f}, "
            f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) "
            f"frame='{msg.header.frame_id}'")

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=msg, pose_link=self.ee_link)

        self.get_logger().info("Planning...")
        plan_result = self.arm.plan()

        if not plan_result:
            self.get_logger().error(
                "Planning FAILED -- pose may be unreachable or in collision. "
                "Try a pose closer to the arm's current workspace.")
            return

        self.get_logger().info("Plan found -- executing...")
        self.moveit.execute(plan_result.trajectory, controllers=[])
        self.get_logger().info("Execution complete.")


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
