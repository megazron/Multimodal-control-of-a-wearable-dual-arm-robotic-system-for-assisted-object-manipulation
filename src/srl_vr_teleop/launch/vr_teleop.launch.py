#!/usr/bin/env python3
"""
vr_teleop.launch.py — the VR input path.

    ros2 launch srl_vr_teleop vr_teleop.launch.py                 # needs a headset
    ros2 launch srl_vr_teleop vr_teleop.launch.py mock:=true      # no headset

SEPARATE FROM THE MANNEQUIN. This launch does not start anything from
srl_teleop's input side. It publishes /master_arm_pose_<arm>, the same topic
the mannequin publishes, so the ROBOT side (ik_follower, e-stop, collision
checking, autonomy) is shared unchanged. Exactly ONE input may run at a time;
running both would have two publishers fighting over one topic, which is why
they are separate launches and never merged.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("mock", default_value="false",
                              description="drive from vr_mock_publisher: no "
                                          "headset, everything downstream of "
                                          "the bridge still exercised"),
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("scale", default_value="0.5"),
        DeclareLaunchArgument("command_orientation", default_value="true",
                              description="VR can do this; the mannequin cannot"),
        DeclareLaunchArgument("autonomy", default_value="false"),
    ]
    mock = LaunchConfiguration("mock")
    nodes = [
        Node(package="srl_vr_teleop", executable="quest_bridge_node",
             name="quest_bridge_node", output="screen",
             condition=UnlessCondition(mock),
             parameters=[{"port": LaunchConfiguration("port")}]),
        Node(package="srl_vr_teleop", executable="vr_mock_publisher",
             name="vr_mock_publisher", output="screen",
             condition=IfCondition(mock)),
        Node(package="srl_vr_teleop", executable="vr_pose_mapper",
             name="vr_pose_mapper", output="screen",
             parameters=[{"scale": LaunchConfiguration("scale"),
                          "command_orientation":
                              LaunchConfiguration("command_orientation")}]),
        Node(package="srl_vr_teleop", executable="vr_gripper_node",
             name="vr_gripper_node", output="screen"),
        Node(package="srl_vr_teleop", executable="vr_safety_node",
             name="vr_safety_node", output="screen"),
        Node(package="srl_vr_teleop", executable="vr_feedback_node",
             name="vr_feedback_node", output="screen"),
        Node(package="srl_vr_autonomy", executable="vr_intent_source",
             name="vr_intent_source", output="screen",
             condition=IfCondition(LaunchConfiguration("autonomy"))),
        Node(package="srl_vr_autonomy", executable="vr_handover_arbiter",
             name="vr_handover_arbiter", output="screen",
             condition=IfCondition(LaunchConfiguration("autonomy"))),
    ]
    return LaunchDescription(args + nodes)
