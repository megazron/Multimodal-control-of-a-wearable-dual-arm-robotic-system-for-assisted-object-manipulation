#!/usr/bin/env python3
"""
real_drivers_readonly.launch.py — REAL Kortex drivers, READ ONLY.

Brings up the Kortex hardware interface for both real arms in the /real
namespace and spawns ONLY joint_state_broadcaster.

THE ARM TRAJECTORY CONTROLLERS ARE DELIBERATELY NOT SPAWNED. With no
controller claiming a command interface, nothing in this launch is capable of
commanding the arms -- not by accident, not by a stray topic publication, not
by a node that starts later. That is a stronger guarantee than "we promised
not to send commands", and it is the point of this file.

It also does NOT start robot_state_publisher: the real arms use the same link
names as the running mock sim, so a second publisher would fight the sim for
/tf. Reading joint angles needs no TF.

    ros2 launch srl_teleop real_drivers_readonly.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.1.10"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.1.9"),
    ]

    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("srl_description"),
                              "urdf", "srl_dual.urdf.xacro"]), " ",
        # LEFT ONLY. Two real Kortex components collide on the unprefixed
        # tcp/twist.* command interfaces, which aborts the entire hardware
        # load. Right stays mock; grippers stay mock (robotiq_driver is not
        # built in this workspace).
        "use_fake_hardware:=false", " ",
        "left_use_fake_hardware:=false", " ",
        "right_use_fake_hardware:=true", " ",
        "use_fake_gripper_hardware:=true", " ",
        "left_robot_ip:=", LaunchConfiguration("left_robot_ip"), " ",
        "right_robot_ip:=", LaunchConfiguration("right_robot_ip"),
    ]), value_type=str)

    # The controller_manager takes robot_description from a TOPIC. Publish it
    # with a robot_state_publisher in /real, but REMAP ITS TF OUTPUT AWAY:
    # the real arms use the same link names as the running mock sim, so an
    # unremapped RSP would fight the sim for /tf and corrupt both.
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace="real", output="screen",
        # frame_prefix so the REAL arm's frames are distinct from the sim's.
        # Both use identical link names; without a prefix the two TF trees
        # collide and RViz cannot show them side by side.
        parameters=[{"robot_description": robot_description,
                     "frame_prefix": "real_"}],
        remappings=[("joint_states", "/real/joint_states")],
    )

    cm = Node(
        package="controller_manager", executable="ros2_control_node",
        namespace="real", output="screen", emulate_tty=True,
        remappings=[("~/robot_description", "/real/robot_description")],
        parameters=[{"robot_description": robot_description,
                     # The shared ros2_controllers.yaml is keyed to
                     # "controller_manager" and is not found under
                     # "/real/controller_manager", so declare the one
                     # controller this read-only launch needs directly.
                     "joint_state_broadcaster.type":
                         "joint_state_broadcaster/JointStateBroadcaster",
                     "update_rate": 100},
                    PathJoinSubstitution([
                        FindPackageShare("srl_moveit_config"),
                        "config", "ros2_controllers.yaml"])],
    )

    # ONLY the broadcaster. No arm controllers => no command interfaces
    # claimed => the arms cannot be moved from this launch.
    jsb = Node(
        package="controller_manager", executable="spawner",
        namespace="real", output="screen", emulate_tty=True,
        # Type given explicitly: the shared ros2_controllers.yaml is keyed to
        # "controller_manager", which does not match "/real/controller_manager"
        # once namespaced, so the type param is not found.
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/real/controller_manager"],
    )

    # Tie the prefixed real tree to the sim world frame, identity transform,
    # so RViz renders them in the same place for direct comparison.
    link = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="real_world_link", output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "real_world"],
    )

    return LaunchDescription(args + [rsp, cm, jsb, link])
