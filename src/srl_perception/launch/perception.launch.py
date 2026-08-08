#!/usr/bin/env python3
"""
perception.launch.py — both wrist cameras through the AprilTag detector,
fused into one world-frame object list.

    ros2 launch srl_perception perception.launch.py
    ros2 launch srl_perception perception.launch.py fallback:=true

`fallback:=true` additionally starts the colour/shape detector. Do that
deliberately: colour segmentation fails by being confidently wrong, which is
the one failure mode AprilTag does not have. See the package README.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("tag_size_m", default_value="0.040"),
        DeclareLaunchArgument("family", default_value="36h11"),
        DeclareLaunchArgument("corner_refine", default_value="subpix",
                              choices=["none", "subpix", "apriltag"],
                              description="apriltag is 43x slower for 0.2 mm"),
        DeclareLaunchArgument("fallback", default_value="false",
                              description="also run the colour/shape fallback"),
    ]
    nodes = []
    for arm in ("left", "right"):
        nodes.append(Node(
            package="srl_perception", executable="apriltag_detector",
            name=f"apriltag_detector_{arm}", output="screen",
            parameters=[{"arm": arm,
                         "family": LaunchConfiguration("family"),
                         "tag_size_m": LaunchConfiguration("tag_size_m"),
                         "corner_refine": LaunchConfiguration("corner_refine")}]))
        nodes.append(Node(
            package="srl_perception", executable="colour_shape_detector",
            name=f"colour_shape_detector_{arm}", output="screen",
            condition=IfCondition(LaunchConfiguration("fallback")),
            parameters=[{"arm": arm, "enabled": True}]))
    nodes.append(Node(package="srl_perception", executable="object_pose_tracker",
                      name="object_pose_tracker", output="screen"))
    return LaunchDescription(args + nodes)
