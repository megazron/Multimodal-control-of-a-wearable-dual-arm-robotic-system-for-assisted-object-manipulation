"""
srl_bringup.launch.py — full dual-arm SRL with controllers
===========================================================
Loads human+backpack+2 Gen3 arms, starts ros2_control, spawns both
arms' trajectory controllers + joint state broadcaster, opens RViz.

Sim:  ros2 launch srl_description srl_bringup.launch.py
Real: ros2 launch srl_description srl_bringup.launch.py use_fake_hardware:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_fake = LaunchConfiguration("use_fake_hardware")

    urdf = PathJoinSubstitution([
        FindPackageShare("srl_description"), "urdf", "srl_dual.urdf.xacro"
    ])
    controllers = PathJoinSubstitution([
        FindPackageShare("srl_description"), "config", "srl_controllers.yaml"
    ])
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("srl_description"), "rviz", "srl.rviz"
    ])

    # CRITICAL: wrap in ParameterValue(..., value_type=str) for Jazzy
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf, " use_fake_hardware:=", use_fake]),
            value_type=str,
        )
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )
    left_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_joint_trajectory_controller",
                   "--controller-manager", "/controller_manager"],
    )
    right_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_joint_trajectory_controller",
                   "--controller-manager", "/controller_manager"],
    )

    delay_left = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[left_spawner]))
    delay_right = RegisterEventHandler(
        OnProcessExit(target_action=left_spawner, on_exit=[right_spawner]))

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        control_node,
        rsp,
        rviz,
        jsb_spawner,
        delay_left,
        delay_right,
    ])
