#!/usr/bin/env python3
"""
mock_real.launch.py — the real-arm stack with the hardware replaced by a mock.

Everything downstream of the driver is the REAL code path: the same
real_homing_node under the same velocity law, the same sim_to_real_bridge with
the same limits, the same prefixed TF the collision check reads. Only the
Kortex driver and its controller are swapped for mock_real_stack.

The point is to be able to answer "does the gated flow work" without a robot
attached, and -- more importantly -- to be able to produce failures on demand
(`ros2 param set /mock_real_stack stall true`) that hardware will not produce
politely.

  ros2 launch srl_teleop mock_real.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("arm", default_value="left",
                              description="left | right | both"),
        DeclareLaunchArgument("preview_delay_s", default_value="1.0"),
        DeclareLaunchArgument("max_vel_rad_s", default_value="0.15"),
        # HOMING speed is a separate knob from TRACKING speed -- homing runs
        # unattended to a fixed target, tracking follows an operator. They
        # were separate already in real_arms_highlevel.launch.py; this launch
        # simply forwarded NEITHER, so real_homing_node fell back to its own
        # 0.05 default while start_real.sh's banner printed the bridge's
        # 0.15. "vmax 0.15" in the header and "vmax now 0.050" on every
        # homing line were both correct and describing different parameters.
        DeclareLaunchArgument("homing_vmax", default_value="0.15"),
        DeclareLaunchArgument("homing_kp", default_value="0.5"),
        DeclareLaunchArgument("max_step_rad", default_value="0.05"),
        DeclareLaunchArgument("lag_trip_rad", default_value="0.5"),
        DeclareLaunchArgument("start_at_home", default_value="false"),
        DeclareLaunchArgument("auto_bridge", default_value="true"),
    ]

    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("srl_description"),
                              "urdf", "srl_dual.urdf.xacro"]), " ",
        "use_fake_hardware:=true use_fake_gripper_hardware:=true",
    ]), value_type=str)

    # Prefixed TF for the mock arm, so the collision check has real_<arm>_<link>
    # frames to look up exactly as it would with hardware.
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="real_rsp", namespace="realmock", output="log",
        parameters=[{"robot_description": robot_description,
                     "frame_prefix": "real_"}],
        remappings=[("joint_states", "/real/joint_states")],
    )

    link = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="real_world_link", output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "real_world"],
    )

    def per_arm(context):
        """Expand arm:=both into one mock stack, homing node and bridge per arm.

        Node names must differ per arm or the second set overwrites the first
        in the graph and the rehearsal silently tests one arm twice.
        """
        which = LaunchConfiguration("arm").perform(context)
        arms = ["left", "right"] if which == "both" else [which]
        out = []
        for a in arms:
            out.append(Node(
                package="srl_teleop", executable="mock_real_stack",
                name="mock_real_stack_%s" % a, output="screen",
                emulate_tty=True,
                parameters=[{"arm": a,
                             "start_at_home": LaunchConfiguration("start_at_home")}]))
            out.append(TimerAction(period=5.0, actions=[Node(
                package="srl_teleop", executable="real_homing_node",
                name="real_homing_node_%s" % a, output="screen",
                emulate_tty=True,
                parameters=[{"arm": a, "auto_home": True,
                             "kp": LaunchConfiguration("homing_kp"),
                             "vmax_rad_s": LaunchConfiguration(
                                 "homing_vmax")}])]))
            out.append(TimerAction(period=7.0, actions=[Node(
                package="srl_teleop", executable="sim_to_real_bridge",
                name="sim_to_real_bridge_%s" % a, output="screen",
                emulate_tty=True,
                parameters=[{"arm": a,
                             "enabled": LaunchConfiguration("auto_bridge"),
                             "preview_delay_s": LaunchConfiguration("preview_delay_s"),
                             "max_vel_rad_s": LaunchConfiguration("max_vel_rad_s"),
                             "max_step_rad": LaunchConfiguration("max_step_rad"),
                             "lag_trip_rad": LaunchConfiguration("lag_trip_rad")}])]))
        return out

    return LaunchDescription(
        args + [rsp, link, OpaqueFunction(function=per_arm)])
