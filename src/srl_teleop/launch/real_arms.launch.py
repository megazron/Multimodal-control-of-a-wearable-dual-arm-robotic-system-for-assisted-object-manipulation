#!/usr/bin/env python3
"""
real_arms.launch.py — REAL arm drivers that CAN command, plus homing + bridge.

This is the other half of real_drivers_readonly.launch.py. That file spawns no
arm controller on purpose, so it is structurally incapable of moving anything.
This one DOES spawn a controller, because homing and the cascade need to
command. It is therefore never included by teleop.launch.py directly -- it is
started only by real_arm_gate, after an explicit typed 'y'.

SEQUENCE
  1. Kortex driver comes up (LEFT real, RIGHT mock -- see below)
  2. joint_state_broadcaster + left_arm_controller spawn
  3. real_homing_node runs the VELOCITY law to the home pose (auto_home)
  4. sim_to_real_bridge polls until the arm is within tolerance of home,
     then enables itself and prints "REAL ARMS LIVE"

LEFT ONLY, and why. Two real Kortex components both export the UNPREFIXED
tcp/twist.linear.x command interface, which collides and aborts the whole
hardware load. Until that vendor issue is fixed the right arm stays mock.
Everything here is parameterised by `arm`, so flipping the right arm to real
is a launch-argument change and a right_arm_controller spawner, not a rewrite.

NOTE ON RESTARTS. Deactivating a Kortex component tears down its API router
and on_activate cannot rebuild it ("Router is not active. Unable to execute
send."). So the driver-level e-stop is ONE-WAY: after it fires, recovery is
relaunching this file, not reactivating the component.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("arm", default_value="left"),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.1.10"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.1.9"),
        DeclareLaunchArgument("preview_delay_s", default_value="1.0"),
        DeclareLaunchArgument("max_vel_rad_s", default_value="0.15"),
        DeclareLaunchArgument("max_step_rad", default_value="0.05"),
        DeclareLaunchArgument("lag_trip_rad", default_value="1.4"),   # TEMP 2026-08-27: was 0.5; RESTORE
        DeclareLaunchArgument("homing_kp", default_value="0.5"),
        DeclareLaunchArgument("homing_vmax", default_value="0.05"),
        # Bring the driver and controllers up WITHOUT moving anything. Used to
        # verify controller_manager timing on its own -- a rate problem and a
        # homing problem look alike in the log otherwise.
        DeclareLaunchArgument("home", default_value="true"),
        DeclareLaunchArgument("bridge", default_value="true"),
        # The CM services the spawner from its update loop. Over WSL2 NAT that
        # loop is slow, so the default 10 s expires and the spawner gives up
        # with "Failed to acquire lock after multiple attempts" -- which reads
        # like a lock bug but is really a timeout.
        DeclareLaunchArgument("spawner_timeout", default_value="120"),
        # Overrides update_rate in real_controllers.yaml. Exposed because the
        # right value is an EMPIRICAL property of this link and this driver,
        # not something to be guessed: node parameters are applied after the
        # params file, so this wins.
        DeclareLaunchArgument("cm_update_rate", default_value="30"),
    ]

    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("srl_description"),
                              "urdf", "srl_dual.urdf.xacro"]), " ",
        "use_fake_hardware:=false", " ",
        "left_use_fake_hardware:=false", " ",
        "right_use_fake_hardware:=true", " ",
        "use_fake_gripper_hardware:=true", " ",
        "left_robot_ip:=", LaunchConfiguration("left_robot_ip"), " ",
        "right_robot_ip:=", LaunchConfiguration("right_robot_ip"),
    ]), value_type=str)

    # frame_prefix keeps the real arm's TF distinct from the sim's; both use
    # identical link names and would otherwise fight over /tf.
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace="real", output="log",
        parameters=[{"robot_description": robot_description,
                     "frame_prefix": "real_"}],
        remappings=[("joint_states", "/real/joint_states")],
    )

    # NOTE the params file: srl_teleop/config/real_controllers.yaml, NOT the
    # sim's srl_moveit_config/config/ros2_controllers.yaml. The real link
    # cannot sustain 100 Hz (read() is a round trip to the arm over WSL2 NAT,
    # measured at 11.5-25 ms against a 10 ms budget), and a CM that never
    # completes a cycle never activates a controller. The real file is keyed
    # to /real/controller_manager and runs at 30 Hz. The sim is untouched.
    cm = Node(
        package="controller_manager", executable="ros2_control_node",
        namespace="real", output="screen", emulate_tty=True,
        remappings=[("~/robot_description", "/real/robot_description")],
        parameters=[{"robot_description": robot_description},
                    PathJoinSubstitution([
                        FindPackageShare("srl_teleop"),
                        "config", "real_controllers.yaml"]),
                    # AFTER the yaml, so it overrides it.
                    {"update_rate": ParameterValue(
                        LaunchConfiguration("cm_update_rate"),
                        value_type=int)}],
    )

    jsb = Node(
        package="controller_manager", executable="spawner",
        namespace="real", output="screen", emulate_tty=True,
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/real/controller_manager",
                   "--controller-manager-timeout",
                   LaunchConfiguration("spawner_timeout")],
    )

    # The commanding controller. Spawned 6 s in so the hardware interface has
    # finished its handshake with the arm first.
    arm_ctrl = TimerAction(period=6.0, actions=[Node(
        package="controller_manager", executable="spawner",
        namespace="real", output="screen", emulate_tty=True,
        arguments=["left_arm_controller",
                   "--controller-manager", "/real/controller_manager",
                   "--controller-manager-timeout",
                   LaunchConfiguration("spawner_timeout")],
    )])

    link = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="real_world_link", output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "real_world"],
    )

    homing = TimerAction(period=12.0, actions=[Node(
        package="srl_teleop", executable="real_homing_node",
        name="real_homing_node", output="screen", emulate_tty=True,
        parameters=[{"arm": LaunchConfiguration("arm"),
                     "auto_home": True,
                     "kp": LaunchConfiguration("homing_kp"),
                     "vmax_rad_s": LaunchConfiguration("homing_vmax")}],
        condition=IfCondition(LaunchConfiguration("home")),
    )])

    # Starts alongside homing but refuses to enable until the arm is actually
    # at home -- see _auto_enable in sim_to_real_bridge.
    bridge = TimerAction(period=14.0, actions=[Node(
        package="srl_teleop", executable="sim_to_real_bridge",
        name="sim_to_real_bridge", output="screen", emulate_tty=True,
        parameters=[{"arm": LaunchConfiguration("arm"),
                     "enabled": True,
                     "preview_delay_s": LaunchConfiguration("preview_delay_s"),
                     "max_vel_rad_s": LaunchConfiguration("max_vel_rad_s"),
                     "max_step_rad": LaunchConfiguration("max_step_rad"),
                     "lag_trip_rad": LaunchConfiguration("lag_trip_rad")}],
        condition=IfCondition(LaunchConfiguration("bridge")),
    )])

    return LaunchDescription(
        args + [rsp, cm, jsb, arm_ctrl, link, homing, bridge])
