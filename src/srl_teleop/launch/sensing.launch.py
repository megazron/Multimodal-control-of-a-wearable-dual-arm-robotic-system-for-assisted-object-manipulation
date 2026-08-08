#!/usr/bin/env python3
"""
sensing.launch.py — the IMU-primary sensing chain, on its own.

    ros2 launch srl_teleop sensing.launch.py
    ros2 launch srl_teleop sensing.launch.py two_imu:=true tau_s:=0.5

Runs alongside master_pose_node (which still owns the serial port and the pot
path). This launch adds only the IMU-derived products: fused orientation, the
pointing-direction topic that srl_autonomy consumes, and the channel manager.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("tau_s", default_value="0.5",
                              description="complementary filter crossover; "
                                          "0.2 measured best on the left arm"),
        DeclareLaunchArgument("two_imu", default_value="false",
                              description="second (upper-arm) IMU present"),
        DeclareLaunchArgument("azimuth_mode", default_value="j1",
                              choices=["j1", "gyro"]),
        DeclareLaunchArgument("auto_disable_after_frames", default_value="0",
                              description="0 = never auto-disable a channel"),
    ]
    return LaunchDescription(args + [
        Node(package="srl_teleop", executable="master_imu_node",
             name="master_imu_node", output="screen",
             parameters=[{"tau_s": LaunchConfiguration("tau_s"),
                          "two_imu": LaunchConfiguration("two_imu")}]),
        Node(package="srl_teleop", executable="pointing_direction_node",
             name="pointing_direction_node", output="screen",
             parameters=[{"azimuth_mode": LaunchConfiguration("azimuth_mode")}]),
        Node(package="srl_teleop", executable="channel_manager",
             name="channel_manager", output="screen",
             parameters=[{"auto_disable_after_frames":
                          LaunchConfiguration("auto_disable_after_frames")}]),
    ])
