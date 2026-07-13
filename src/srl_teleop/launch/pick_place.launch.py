"""
pick_place.launch.py — launches pick_place_node with proper MoveIt config
=============================================================================
Same pattern as move_to_pose.launch.py: injects MoveIt config (including
moveit_cpp.yaml for the planning pipeline) via ROS parameters, which is
what actually made MoveItPy load correctly.

Run:
  ros2 launch srl_teleop pick_place.launch.py
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

    pick_place_node = Node(
        package="srl_teleop",
        executable="pick_place_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([pick_place_node])
