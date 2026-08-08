#!/usr/bin/env python3
"""
shared_autonomy.launch.py — teleop + perception + shared autonomy.

    bash scripts/run_autonomy.sh

LAYERING. The teleop stack is started FIRST and the autonomy layer is added on
top, so a crash in the autonomy layer leaves a working teleoperated robot
rather than a dead one. Nothing in srl_teleop subscribes to anything published
here.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("teleop", default_value="true",
                              description="also start the teleop stack"),
        DeclareLaunchArgument("perception", default_value="true"),
        DeclareLaunchArgument("autonomy_delay", default_value="15.0",
                              description="let move_group and /compute_ik settle "
                                          "before grasp_generator starts calling it"),
        DeclareLaunchArgument("p_threshold", default_value="0.60"),
        DeclareLaunchArgument("distance_threshold_m", default_value="0.25"),
        DeclareLaunchArgument("servo_time_s", default_value="0.5"),
    ]
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("srl_teleop"), "launch", "teleop.launch.py"])),
        condition=IfCondition(LaunchConfiguration("teleop")),
        launch_arguments={"gate": "false"}.items())
    percep = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("srl_perception"), "launch", "perception.launch.py"])),
        condition=IfCondition(LaunchConfiguration("perception")))
    autonomy = TimerAction(
        period=LaunchConfiguration("autonomy_delay"),
        actions=[
            Node(package="srl_teleop", executable="pointing_direction_node",
                 name="pointing_direction_node", output="screen"),
            Node(package="srl_autonomy", executable="intent_inference",
                 name="intent_inference", output="screen"),
            Node(package="srl_autonomy", executable="grasp_generator",
                 name="grasp_generator", output="screen"),
            Node(package="srl_autonomy", executable="handover_arbiter",
                 name="handover_arbiter", output="screen",
                 parameters=[{
                     "p_threshold": LaunchConfiguration("p_threshold"),
                     "distance_threshold_m":
                         LaunchConfiguration("distance_threshold_m"),
                     "servo_time_s": LaunchConfiguration("servo_time_s")}]),
        ])
    return LaunchDescription(args + [teleop, percep, autonomy])
