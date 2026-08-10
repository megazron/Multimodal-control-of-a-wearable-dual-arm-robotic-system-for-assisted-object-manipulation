#!/usr/bin/env python3
"""
teleop.launch.py — bring up the entire teleop stack with one command.

    ros2 launch srl_teleop teleop.launch.py

Replaces the five-terminal dance: MoveIt/RViz/controllers, the master pose
node, an IK follower per arm, and the live monitor, in one process tree that
dies together on Ctrl-C.

SEQUENCING
  The IK followers need MoveIt's /compute_ik. Two independent guarantees,
  because a startup race here is expensive to diagnose:
    1. `startup_delay` holds the teleop nodes back while move_group loads.
       This is a contention reducer, not a correctness guarantee -- the load
       time varies with machine and planning-scene size.
    2. ik_follower_node itself BLOCKS in wait_for_service("/compute_ik")
       before it subscribes to anything, so even if the delay is far too
       short it waits rather than spinning against a missing service.
  master_pose_node needs no MoveIt service at all -- it is delayed only so
  it is not competing for CPU while move_group starts.

Arguments (all optional):
  arm             both | left | right      which arm(s) to drive
  serial_port     auto | /dev/ttyACMx      auto sniffs for the Teensy
  position_mode   spherical | fk
  left_scale      motion scaling, left     (live-tunable via ros2 param set)
  right_scale     motion scaling, right
  use_rviz        true | false
  dashboard       true | false             also run the status dashboard
  startup_delay   seconds before teleop nodes start

There is deliberately NO `record` argument -- see the note near the bottom.
Run the recording in its own terminal via run_teleop_capture.sh.
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _arm_enabled(which):
    """True when `arm` is 'both' or this specific arm."""
    return IfCondition(PythonExpression(
        ["'", LaunchConfiguration("arm"), "' in ('both', '", which, "')"]))


def _normalize_ros_env():
    """Force this launch onto the same discovery settings as scripts/env.sh.

    Terminal 1 is started by hand -- `ros2 launch srl_teleop teleop.launch.py`
    -- so unlike start_real.sh it never sources scripts/env.sh, and nothing
    otherwise stops it from running on a different domain than the shell that
    later tries to see it. Every node below inherits this process's
    environment, so setting it here covers the whole stack in one place.

    Values must stay in step with scripts/env.sh; that file is the reference
    and explains why ROS_LOCALHOST_ONLY is unset rather than set.
    """
    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")

    # Deprecated in Jazzy and a partition risk when only some processes have
    # it: the ros2 daemon caches the environment of whichever shell spawned
    # it first, so a mixed setup hides the graph from exactly one terminal.
    if os.environ.pop("ROS_LOCALHOST_ONLY", None) is not None:
        print("[teleop.launch] dropped inherited ROS_LOCALHOST_ONLY "
              "(see scripts/env.sh)")


def _mount_guard():
    """A4/A5 regression guard. Runs on EVERY launch and FAILS LOUDLY if the
    home geometry is in collision or inside the clearance floor. A mount that
    buried both arm bases in the torso reached the user as a SCREENSHOT; this
    exists so that can never happen again. It deliberately ignores the SRDF."""
    return Node(package="srl_teleop", executable="mount_guard_node",
                name="mount_guard_node", output="screen",
                parameters=[{"min_clearance_m": 0.15,
                             "fail_on_violation": False}])


def generate_launch_description():
    _normalize_ros_env()

    args = [
        DeclareLaunchArgument("arm", default_value="both",
                              choices=["both", "left", "right"]),
        DeclareLaunchArgument("serial_port", default_value="auto",
                              description="'auto' sniffs /dev/ttyACM* and "
                                          "/dev/ttyUSB* for a master frame"),
        DeclareLaunchArgument("position_mode", default_value="spherical",
                              choices=["spherical", "fk"]),
        DeclareLaunchArgument("left_scale", default_value="1.0"),
        DeclareLaunchArgument("right_scale", default_value="1.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # master:=false leaves master_pose_node OUT of the stack, for a run
        # driven by a SCRIPTED operator instead of the mannequin. Without it
        # the scripted publisher and master_pose_node are two sources on
        # /master_arm_pose_<arm> at once -- the one-source-at-a-time rule this
        # project has already paid for twice -- and the recording sweep's
        # isolation check correctly refuses to record at all.
        DeclareLaunchArgument("master", default_value="true",
                              description="start master_pose_node"),
        DeclareLaunchArgument("dashboard", default_value="false"),
        DeclareLaunchArgument("startup_delay", default_value="12.0"),
        # Real-hardware safety mode: slower, stricter clearance, motion must
        # be explicitly armed, and it refuses to start on a wound-up arm.
        DeclareLaunchArgument("real_robot", default_value="false"),
        # real_arms drives the DESCRIPTION (kortex_driver + robotiq_driver
        # instead of mock_components); real_robot drives the FOLLOWER limits.
        # They are separate on purpose: you want the slow/strict follower
        # while still on mock hardware during bring-up.
        DeclareLaunchArgument("real_arms", default_value="false"),
        # Bring-up ORDERING ONLY. The sim stack comes up immediately; the real
        # driver stack starts this many seconds later, so move_group and the
        # mock controllers are settled before the Kortex sessions open.
        # It does NOT arm, enable or command anything -- the real stack it
        # starts loads NO arm controllers.
        DeclareLaunchArgument("real_start_delay_s", default_value="10.0"),
        # Runs once after startup and proves the arm actually MOVES. Default
        # ON: a silently-blocked pipeline is the failure mode this exists to
        # catch, and it has happened three times.
        DeclareLaunchArgument("self_test", default_value="true"),
        DeclareLaunchArgument("grippers", default_value="true",
                              description="start fsr_gripper_node; set false "
                                          "if you are driving the grippers "
                                          "some other way"),
        # --- the real-arm gate (see real_arm_gate.py) ---
        # ON by default: it only ever ASKS, and the answer defaults to no, so
        # a default-on prompt cannot move anything by itself.
        DeclareLaunchArgument("gate", default_value="true"),
        # Default FALSE: live_monitor belongs in terminal 2 of the
        # three-terminal split, not in this launch's shared log.
        DeclareLaunchArgument("monitor", default_value="false"),
        DeclareLaunchArgument("gate_arm", default_value="left",
                              description="LEFT only until the two-arm "
                                          "tcp/twist.linear.x collision is fixed"),
        DeclareLaunchArgument("preview_delay_s", default_value="1.0"),
        DeclareLaunchArgument("real_max_vel_rad_s", default_value="0.15"),
        DeclareLaunchArgument("real_max_step_rad", default_value="0.05"),
        DeclareLaunchArgument("lag_trip_rad", default_value="0.5"),
    ]

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("srl_moveit_config"), "launch", "demo.launch.py"])),
        launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
    )

    # respawn: the serial port is resolved ONCE, in the constructor, so a
    # Teensy attached after launch would otherwise need the whole stack
    # restarted. The node still fails loudly with the usbipd hint (Task 2e
    # behaviour is unchanged); launch simply retries it every few seconds
    # until the board appears, and it then connects on its own.
    master = Node(
        package="srl_teleop", executable="master_pose_node",
        name="master_pose_node", output="screen", emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("master")),
        respawn=True, respawn_delay=5.0,
        parameters=[{
            "arm": LaunchConfiguration("arm"),
            "serial_port": LaunchConfiguration("serial_port"),
            "position_mode": LaunchConfiguration("position_mode"),
            "left_scale": LaunchConfiguration("left_scale"),
            "right_scale": LaunchConfiguration("right_scale"),
        }],
    )

    followers = [
        Node(package="srl_teleop", executable="ik_follower_node",
             name=f"ik_follower_{side}", output="screen", emulate_tty=True,
             parameters=[{"arm": side,
                          "real_robot": LaunchConfiguration("real_robot")}],
             condition=_arm_enabled(side))
        for side in ("left", "right")
    ]

    # The dead-man is armed only when real hardware is in play. In mock-only
    # sim a stale master is a nuisance, not a hazard, and latching on it
    # freezes everything (see estop_node's note).
    deadman = PythonExpression(
        ["'", LaunchConfiguration("real_arms"), "'.lower() == 'true' or '",
         LaunchConfiguration("real_robot"), "'.lower() == 'true'"])
    estop = Node(package="srl_teleop", executable="estop_node",
                 name="estop_node", output="screen", emulate_tty=True,
                 parameters=[{"arms": ["left", "right"],
                              "deadman_enabled": deadman}])

    # FSR grippers. Previously started by hand (`ros2 run srl_teleop
    # fsr_gripper_node`) and therefore absent from every launched stack, so a
    # run that needed grippers silently had none. It reads
    # /master_fsr_buttons and commands both gripper controllers; harmless
    # when no Teensy is attached, because it simply receives nothing.
    grippers = Node(package="srl_teleop", executable="fsr_gripper_node",
                    name="fsr_gripper_node", output="screen",
                    emulate_tty=True,
                    parameters=[{"arms": ["left", "right"]}],
                    condition=IfCondition(LaunchConfiguration("grippers")))

    selftest = Node(package="srl_teleop", executable="selftest_node",
                    name="selftest_node", output="screen", emulate_tty=True,
                    condition=IfCondition(LaunchConfiguration("self_test")))

    # live_monitor is NOT started here. It is a full-screen redraw, and inside
    # a shared launch log its cursor-home escapes fight every other node's
    # output. It belongs in its own terminal (terminal 2 of the three-terminal
    # split), where it gets a clean tty:
    #     ros2 run srl_teleop live_monitor
    # Starting it here as well would also duplicate the node name.
    # Set monitor:=true to put it back in this launch anyway.
    monitor = Node(package="srl_teleop", executable="live_monitor",
                   name="live_monitor", output="screen", emulate_tty=True,
                   condition=IfCondition(LaunchConfiguration("monitor")))

    # emulate_tty is deliberately FALSE here. With a pty the dashboard sees
    # isatty() and switches to cursor-home redraw, which corrupts a shared
    # launch log where other nodes interleave their output. Without one it
    # falls back to plain periodic blocks. For the full-screen view, run it
    # in its own terminal: `ros2 run srl_teleop dashboard`.
    dash = Node(package="srl_teleop", executable="dashboard",
                name="teleop_dashboard", output="screen", emulate_tty=False,
                parameters=[{"arm": LaunchConfiguration("arm"),
                             "position_mode": LaunchConfiguration("position_mode")}],
                condition=IfCondition(LaunchConfiguration("dashboard")))

    # NO `record` argument, deliberately. Running teleop_recorder inside this
    # launch does not work in practice: launch merges every node's stdout into
    # one stream and line-prefixes it with [node-N], so RViz, MoveIt, both IK
    # followers and master_pose_node bury the recorder's countdown, and the
    # prefixes break in-place redraw. Suppressing the recorder's own output
    # cannot help -- the noise comes from the OTHER processes. Run the
    # recording in its own terminal instead, where it gets a clean tty:
    #
    #     terminal 1:  ros2 launch srl_teleop teleop.launch.py
    #     terminal 2:  bash ~/kortex_ws/run_teleop_capture.sh
    #
    # That script detects this launch's master_pose_node and attaches to its
    # topics rather than starting a second one on the same serial port.
    # Real driver stack, delayed. Read-only: no arm controllers are spawned,
    # so it is structurally incapable of commanding the arms. This is the
    # `real_arms:=true` path, kept for read-only inspection of the hardware.
    real_stack = TimerAction(
        period=LaunchConfiguration("real_start_delay_s"),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("srl_teleop"), "launch",
                "real_drivers_readonly.launch.py"]))) ],
        condition=IfCondition(LaunchConfiguration("real_arms")))

    # THE GATE. Sim teleop is live immediately; this asks, in the terminal,
    # whether to connect and home the real arms, and only a typed 'y' starts
    # anything capable of commanding hardware. Default -- including no answer
    # at all -- is no. emulate_tty gives it a pty so it can read the answer.
    #
    # Mutually exclusive with real_arms:=true, which already owns /real with a
    # read-only stack; the gate's child launch would collide with it over the
    # single Kortex session the arm permits.
    gate = Node(
        package="srl_teleop", executable="real_arm_gate",
        name="real_arm_gate", output="screen", emulate_tty=True,
        parameters=[{"arm": LaunchConfiguration("gate_arm"),
                     "preview_delay_s": LaunchConfiguration("preview_delay_s"),
                     "max_vel_rad_s": LaunchConfiguration("real_max_vel_rad_s"),
                     "max_step_rad": LaunchConfiguration("real_max_step_rad"),
                     "lag_trip_rad": LaunchConfiguration("lag_trip_rad")}],
        condition=IfCondition(LaunchConfiguration("gate")))

    # RECOVERY MANAGER. Owns fault paths (b)-(e) -- Teensy reconnect, Kortex
    # session loss, camera/zero-detection abort, network dropout -- each
    # verified by fault injection and, until now, NEVER LAUNCHED BY ANYTHING.
    # It was installed, documented and inert: a recovery layer that is not
    # running is indistinguishable from one that is, right up until a fault.
    #
    # `expect_real_stack` and `freeze_on_master_loss` keep their False
    # defaults, so on a sim stack the session and link paths fire only on an
    # explicit connected:false / up:false and never on the silence of a
    # subsystem that simply is not present. The trial is marked INVALID either
    # way -- that is data integrity and is not optional.
    recovery = Node(
        package="srl_teleop", executable="recovery_manager",
        name="recovery_manager", output="screen", emulate_tty=True,
        parameters=[{"arms": ["left", "right"],
                     "serial_port": LaunchConfiguration("serial_port"),
                     "expect_real_stack": LaunchConfiguration("real_arms")}])

    delayed = TimerAction(period=LaunchConfiguration("startup_delay"),
                          actions=[master, *followers, monitor, dash, estop,
                                   grippers, recovery, selftest, gate])

    return LaunchDescription(args + [
        _mount_guard(),moveit, delayed, real_stack])
