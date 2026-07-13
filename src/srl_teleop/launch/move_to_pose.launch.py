"""
move_to_pose.launch.py — launches move_to_pose_node with proper MoveIt config
================================================================================
Adds .moveit_cpp(file_path=...) pointing at moveit_cpp.yaml -- MoveItPy reads
planning pipeline config from THIS file, in a different format than what
move_group reads from its own auto-detected *_planning.yaml. Without it,
MoveItPy fails with "Failed to load planning pipelines from parameter server"
even though move_group works fine with the same MoveItConfigsBuilder call.

Run:
  ros2 launch srl_teleop move_to_pose.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("srl_dual", package_name="srl_moveit_config")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("srl_moveit_config"),
                "config",
                "moveit_cpp.yaml",
            )
        )
        .to_moveit_configs()
    )

    move_to_pose_node = Node(
        package="srl_teleop",
        executable="move_to_pose_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([move_to_pose_node])
