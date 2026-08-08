# srl_teleop — teleoperation only

Master-arm sensing, the IK follower, clutch, motion scaling, the sim→real
bridge and the e-stop. **Nothing in this package knows that perception or
autonomy exist**, and that is a hard rule: `srl_teleop` is the *baseline
condition* of every experiment, so if it ever needed `srl_autonomy` the
comparison would be circular.

    grep -rn "srl_autonomy\|srl_perception" src/srl_teleop/    # must be empty

## Depends on

`rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`,
`trajectory_msgs`, `moveit_msgs`, `tf2_ros`. Plus `srl_description` and
`srl_moveit_config` at runtime.

## Run

    bash scripts/run_teleop.sh                       # everything, sim
    bash scripts/run_teleop.sh dashboard:=true       # + status dashboard
    bash scripts/run_teleop.sh real_robot:=true      # slow/strict follower

Three-terminal split for real work:

    terminal 1:  bash scripts/run_teleop.sh gate:=false
    terminal 2:  ros2 run srl_teleop live_monitor
    terminal 3:  bash scripts/start_real.sh          # --mock to rehearse

## What is in here

| area | nodes |
| --- | --- |
| master sensing | `master_pose_node`, `pot_bridge`, `master_imu_node`, `pointing_direction_node`, `channel_manager`, `fake_pot` |
| following | `ik_follower_node`, `fsr_gripper_node`, `leader_follower_node` |
| safety | `estop_node`, `selftest_node`, `real_arm_gate`, `participant_safety_node` |
| real hardware | `real_homing_node`, `sim_to_real_bridge`, `payload_manager`, `mock_real_stack` |
| monitoring | `dashboard`, `live_monitor` |
| calibration / capture | `capture_zero`, `capture_gyro_bias`, `imu_mount_calibration`, `teleop_recorder`, `analyse_teleop` |

`pointing_direction_node` publishes `/master_pointing_<arm>`, which
`srl_autonomy` consumes. The dependency runs one way only: this package
publishes, it never subscribes to anything autonomy produces.
