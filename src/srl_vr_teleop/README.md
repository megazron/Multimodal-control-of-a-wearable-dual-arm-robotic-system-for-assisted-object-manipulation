# srl_vr_teleop — Meta Quest teleoperation

A **separate input modality** from the mannequin master. Full 6-DOF including
yaw, at 72 Hz, with no dead channels.

**Does not import `srl_teleop`, and `srl_teleop` does not import this.** They
are two conditions in one study and must be runnable and citable separately.
They share only the robot side: `ik_follower_node`, collision checking, the
e-stop, and `srl_autonomy`.

## Run

    ros2 launch srl_vr_teleop vr_teleop.launch.py mock:=true    # no headset
    ros2 launch srl_vr_teleop vr_teleop.launch.py               # with a Quest

See `docs/system/vr_bringup.md` and `quest_app/README.md`.

## Nodes

| node | role |
| --- | --- |
| `quest_bridge_node` | WebXR/WebSocket transport, frame conversion, latency measurement, stale-pose watchdog |
| `vr_pose_mapper` | controller pose → EE target, **including orientation**; clutch, indexing, live scale with anchor re-basing |
| `vr_gripper_node` | trigger → Robotiq, same latch semantics as the FSR path |
| `vr_safety_node` | tracking loss, dropout, workspace bounds, and the **mandatory observer e-stop** |
| `vr_feedback_node` | what is rendered in-headset, plus haptics on state change |
| `vr_mock_publisher` | desktop stand-in for the Quest, so the whole ROS side is testable |

## Measured (mock)

    controller pose rate   72.0 Hz     (requirement >= 60 Hz)
    robot command rate    100.0 Hz
    ROS-side lag           below the 5 ms measurement resolution
    clutch engage          anchors exactly on the robot's current EE
