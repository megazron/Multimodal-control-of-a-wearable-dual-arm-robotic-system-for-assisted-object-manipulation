#!/usr/bin/env python3
"""
real_arms_highlevel.launch.py — REAL arm via the HIGH-LEVEL Kortex API.

The replacement for real_arms.launch.py on this link. Identical sequence and
identical safety envelope; the only difference is the final hop to the metal.

WHAT CHANGED, AND ONLY THIS
  real_arms.launch.py        ros2_control_node + joint_state_broadcaster +
                             left_arm_controller  -> CYCLIC write
  this file                  kortex_highlevel_bridge -> SendJointSpeedsCommand

The cyclic write costs a ~10 ms network round trip over WSL2 against a 10 ms
budget at 100 Hz, so it never completed a cycle and no command reached the
arm. See the header of kortex_highlevel_bridge.py for the measurements.

UNCHANGED: real_homing_node and sim_to_real_bridge, both of which publish the
same JointTrajectory topic the arm controller used to serve, and both of which
consume /real/joint_states -- now published by the high-level bridge. The lag
monitor and collision check therefore stay exactly where they were.

INTERPRETER. The bridge is launched with the kortex venv's python via
ExecuteProcess, not launch_ros Node, because kortex_api pins protobuf 3.5.1
and must never enter user site -- it would shadow the protobuf ROS needs and
is broken on Python 3.12 besides. The venv is built with
--system-site-packages so the same interpreter has rclpy AND kortex_api.

SEQUENCE
  1. high-level bridge connects, opens the ONE session, publishes feedback
  2. real_homing_node runs the velocity law to home (auto_home)
  3. sim_to_real_bridge polls until the arm is at home, enables, prints
     "REAL ARMS LIVE"
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

VENV_PY = os.path.expanduser("~/kortex_ws/.kortex_venv/bin/python")


def generate_launch_description():
    args = [
        DeclareLaunchArgument("arm", default_value="left",
                              description="left | right | both"),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.1.10"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.1.9"),
        DeclareLaunchArgument("preview_delay_s", default_value="1.0"),
        DeclareLaunchArgument("max_vel_rad_s", default_value="0.15"),
        DeclareLaunchArgument("max_step_rad", default_value="0.05"),
        DeclareLaunchArgument("lag_trip_rad", default_value="0.5"),
        DeclareLaunchArgument("homing_kp", default_value="0.5"),
        DeclareLaunchArgument("homing_vmax", default_value="0.15"),
        DeclareLaunchArgument("home", default_value="true"),
        DeclareLaunchArgument("bridge", default_value="true"),
        # The high-level API is a velocity command: it holds a motion between
        # sends, so it does not need the 1 kHz the cyclic path assumes. 30 Hz
        # leaves ~23 ms of slack per cycle against a ~10 ms round trip.
        DeclareLaunchArgument("rate_hz", default_value="30.0"),
        DeclareLaunchArgument("kp", default_value="0.5"),
        DeclareLaunchArgument("deadband_deg", default_value="1.0"),
        DeclareLaunchArgument("watchdog_s", default_value="0.5"),
    ]

    # Kinematics only -- nothing here loads a ros2_control hardware plugin any
    # more, so fake_hardware is the honest setting: it keeps the URDF's
    # ros2_control tags inert instead of describing a driver that is not running.
    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("srl_description"),
                              "urdf", "srl_dual.urdf.xacro"]), " ",
        "use_fake_hardware:=true", " ",
        "left_use_fake_hardware:=true", " ",
        "right_use_fake_hardware:=true", " ",
        "use_fake_gripper_hardware:=true", " ",
        "left_robot_ip:=", LaunchConfiguration("left_robot_ip"), " ",
        "right_robot_ip:=", LaunchConfiguration("right_robot_ip"),
    ]), value_type=str)

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace="real", output="log",
        parameters=[{"robot_description": robot_description,
                     "frame_prefix": "real_"}],
        remappings=[("joint_states", "/real/joint_states")],
    )

    link = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="real_world_link", output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "real_world"],
    )

    def per_arm(context):
        """Expand arm:=both into one node set per arm.

        Node NAMES must differ or the second set silently overwrites the
        first in the graph -- the same duplicate-name failure that made three
        stale followers corrupt the blocking view. Each arm also gets ITS OWN
        robot_ip: bridge_hw used to pass left_robot_ip unconditionally, so
        `arm:=right` opened a session against the LEFT arm and homed it.
        """
        which = LaunchConfiguration("arm").perform(context)
        arms = ["left", "right"] if which == "both" else [which]
        out = []
        for a in arms:
            ip = LaunchConfiguration("%s_robot_ip" % a).perform(context)
            out.append(ExecuteProcess(
                cmd=[VENV_PY, "-m", "srl_teleop.kortex_highlevel_bridge",
                     "--ros-args",
                     "-r", "__node:=kortex_highlevel_bridge_%s" % a,
                     "-p", "arm:=%s" % a,
                     "-p", "robot_ip:=%s" % ip,
                     "-p", ["rate_hz:=", LaunchConfiguration("rate_hz")],
                     "-p", ["kp:=", LaunchConfiguration("kp")],
                     "-p", ["vmax_rad_s:=", LaunchConfiguration("max_vel_rad_s")],
                     "-p", ["deadband_deg:=", LaunchConfiguration("deadband_deg")],
                     "-p", ["watchdog_s:=", LaunchConfiguration("watchdog_s")]],
                output="screen",
                # SIGINT, not SIGTERM: the node's handler zeroes speeds, calls
                # Stop() and closes the session. Killing it harder leaks the
                # session and the arm permits only one.
                sigterm_timeout="10", sigkill_timeout="15"))

            # 6 s: the bridge must have opened its session and published
            # feedback before homing can compute an error.
            out.append(TimerAction(period=6.0, actions=[Node(
                package="srl_teleop", executable="real_homing_node",
                name="real_homing_node_%s" % a, output="screen",
                emulate_tty=True,
                parameters=[{"arm": a, "auto_home": True,
                             "kp": LaunchConfiguration("homing_kp"),
                             "vmax_rad_s": LaunchConfiguration("homing_vmax")}],
                condition=IfCondition(LaunchConfiguration("home")))]))

            out.append(TimerAction(period=8.0, actions=[Node(
                package="srl_teleop", executable="sim_to_real_bridge",
                name="sim_to_real_bridge_%s" % a, output="screen",
                emulate_tty=True,
                parameters=[{"arm": a, "enabled": True,
                             "preview_delay_s": LaunchConfiguration("preview_delay_s"),
                             "max_vel_rad_s": LaunchConfiguration("max_vel_rad_s"),
                             "max_step_rad": LaunchConfiguration("max_step_rad"),
                             "lag_trip_rad": LaunchConfiguration("lag_trip_rad")}],
                condition=IfCondition(LaunchConfiguration("bridge")))]))
        return out

    return LaunchDescription(
        args + [rsp, link, OpaqueFunction(function=per_arm)])
