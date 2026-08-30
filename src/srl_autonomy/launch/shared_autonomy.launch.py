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
        # Forwarded to the followers, so this stack can be launched with the
        # pre-2026-08-23 motion generator to reproduce an older recording.
        DeclareLaunchArgument("motion_generator", default_value="ruckig"),
        # THE MASTER ARM IS NOT REQUIRED BY EVERY MODE THAT INCLUDES TELEOP.
        #
        # This stack always pulled in teleop.launch.py without forwarding
        # `master`, so `master_pose_node` started even for FULL AUTONOMY,
        # which never reads it. That node is launched respawn=True with a
        # 5 s delay because it resolves its serial port once in the
        # constructor -- so with no Teensy on the bus it dies and is
        # restarted for ever.
        #
        # Measured 2026-08-26: FIVE live /master_pose_node instances, load
        # average 8-11, the controller manager overrunning its 100 Hz loop,
        # and every `spawner` timing out after 3 attempts on
        # /controller_manager/list_controllers. joint_state_broadcaster
        # never came up, so /joint_states never published, so start_real.sh
        # refused with "Is the sim controller up?" -- a message that points
        # at the simulation when the cause is a missing USB cable.
        #
        # It is also HARD CONSTRAINT 3: two master_pose_node split the serial
        # stream and invalidated a day of measurements. Five is worse.
        DeclareLaunchArgument(
            "master", default_value="true",
            description="false leaves master_pose_node out. Use false for "
                        "full autonomy and VR-shared, which never read it -- "
                        "with no Teensy it respawns and starves the "
                        "controller manager."),
        # Forwarded so the GUI can suppress the launch's own RViz -- it
        # embeds one already, and two RViz windows was the operator's
        # "there is no rviz" (the extra one covered the window).
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # Forwarded so a virtual Teensy (a pty) can drive the master path
        # with no hardware; `auto` never finds /dev/pts/*.
        DeclareLaunchArgument("serial_port", default_value="auto"),
    ]
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("srl_teleop"), "launch", "teleop.launch.py"])),
        condition=IfCondition(LaunchConfiguration("teleop")),
        launch_arguments={
            "gate": "false",
            "master": LaunchConfiguration("master"),
            "motion_generator": LaunchConfiguration("motion_generator"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "serial_port": LaunchConfiguration("serial_port"),
        }.items())
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
