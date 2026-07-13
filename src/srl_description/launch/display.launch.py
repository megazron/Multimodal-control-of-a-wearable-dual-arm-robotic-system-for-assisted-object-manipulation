"""
display.launch.py — visualize the SRL scene in RViz (no controllers).
Shows the human + backpack + both Gen3 arms. Use this to tune the
mount positions before bringing controllers online.

Run: ros2 launch srl_description display.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_fake = LaunchConfiguration("use_fake_hardware")

    urdf = PathJoinSubstitution([
        FindPackageShare("srl_description"), "urdf", "srl_dual.urdf.xacro"
    ])
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("srl_description"), "rviz", "srl.rviz"
    ])

    robot_description = {
        "robot_description": Command([
            "xacro ", urdf,
            " use_fake_hardware:=", use_fake,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        # joint_state_publisher_gui lets you wiggle joints with sliders
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen",
        ),
    ])
