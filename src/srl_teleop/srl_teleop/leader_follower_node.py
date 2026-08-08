#!/usr/bin/env python3
"""
leader_follower_node.py — mirrors a "leader" arm's joint state to a
"follower" arm's controller (1:1, no scaling yet -- that's the next
feature to layer on top).
=============================================================================
ARCHITECTURE:
  Right now the pot drives BOTH the sim and real arm directly, with no
  collision checking on the real arm's path at all. This node changes
  that: the pot should drive ONLY the leader (typically the sim arm,
  running through MoveIt's collision-aware planning). This node watches
  the leader's resulting /joint_states and mirrors them to the follower
  (typically the real arm's controller) -- so every real motion has
  already been collision-checked in sim before reaching hardware.

TESTING TODAY (no real hardware connected yet):
  You can prove the mirroring mechanism itself right now by pointing
  "follower_topic" at a SECOND controller in the same sim (e.g. run two
  separate robot descriptions under different namespaces), or simply by
  publishing synthetic /joint_states manually and confirming this node
  produces the correct JointTrajectory output -- see the manual test
  command in the module docstring below.

REAL DEPLOYMENT (once hardware is connected):
  Launch the sim stack (use_fake_hardware:=true) as the leader, and a
  SEPARATE real-hardware stack (use_fake_hardware:=false), ideally under
  a ROS2 namespace so their topics don't collide (e.g. /sim/... vs
  /real/...). Point leader_joint_states_topic at the sim's /joint_states
  and follower_command_topic at the real stack's controller topic.

Manual mirroring test (no hardware, proves the mechanism):
  ros2 run srl_teleop leader_follower_node --ros-args \\
    -p leader_joint_states_topic:=/joint_states \\
    -p follower_command_topic:=/test_follower/joint_trajectory \\
    -p joint_names:="['left_joint_1','left_joint_2','left_joint_3','left_joint_4','left_joint_5','left_joint_6','left_joint_7']"

  Then in another terminal, watch it produce trajectories as the sim
  arm moves (e.g. via the pot, or by dragging the MoveIt marker):
    ros2 topic echo /test_follower/joint_trajectory
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class LeaderFollowerNode(Node):
    def __init__(self):
        super().__init__("leader_follower_node")

        self.declare_parameter("leader_joint_states_topic", "/joint_states")
        self.declare_parameter("follower_command_topic",
                               "/left_arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", [
            "left_joint_1", "left_joint_2", "left_joint_3",
            "left_joint_4", "left_joint_5", "left_joint_6", "left_joint_7",
        ])
        # Mirror rate cap -- avoids flooding the follower controller faster
        # than it can usefully consume trajectory points.
        self.declare_parameter("publish_rate_hz", 50.0)

        self.joint_names = list(self.get_parameter("joint_names").value)
        leader_topic = self.get_parameter("leader_joint_states_topic").value
        follower_topic = self.get_parameter("follower_command_topic").value
        rate_hz = self.get_parameter("publish_rate_hz").value

        self.latest_positions = None   # dict: joint_name -> position (rad)

        self.create_subscription(
            JointState, leader_topic, self.on_leader_state, 10)
        self.pub = self.create_publisher(
            JointTrajectory, follower_command_topic := follower_topic, 10)

        self.create_timer(1.0 / rate_hz, self.mirror_tick)

        self.get_logger().info("Leader-follower bridge started.")
        self.get_logger().info(f"  Leader (input):    {leader_topic}")
        self.get_logger().info(f"  Follower (output): {follower_topic}")
        self.get_logger().info(f"  Joints: {self.joint_names}")

    def on_leader_state(self, msg: JointState):
        # Build a name->position lookup from whatever the leader publishes
        # (the leader's /joint_states may include OTHER joints too, e.g.
        # the right arm or gripper -- we only care about our joint_names).
        self.latest_positions = dict(zip(msg.name, msg.position))

    def mirror_tick(self):
        if self.latest_positions is None:
            return   # haven't heard from the leader yet

        try:
            positions = [self.latest_positions[n] for n in self.joint_names]
        except KeyError as e:
            self.get_logger().warn(
                f"Leader state missing joint {e} -- not mirroring this tick.",
                throttle_duration_sec=2.0)
            return

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        # Short time_from_start -- we're mirroring continuously, not doing
        # a single big planned move, so each point should land quickly.
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        traj.points = [point]
        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
