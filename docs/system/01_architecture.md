# Architecture

## The dependency rule, and why it is not negotiable

```
  srl_description ──> srl_moveit_config
          │                  │
          └──────┬───────────┘
                 v
           srl_teleop            srl_perception
                 │                     │
                 └──────────┬──────────┘
                            v
                      srl_autonomy
                            │
                            v
                     srl_experiments
```

`srl_teleop` is the **baseline condition** of every experiment. If it imported
anything from `srl_autonomy`, the comparison "autonomy vs no autonomy" would
be comparing the autonomy against itself. So the rule is mechanical:

    grep -rnE "^\s*(from|import)\s+srl_(autonomy|perception)" src/srl_teleop/

must return nothing, and `srl_teleop` must still run with the other packages
physically removed from `install/`. Both are checked in Part 9 of `docs/ENGINEERING_LOG.md`.

The one signal that crosses the boundary goes the *right* way:
`srl_teleop` publishes `/master_pointing_<arm>` and `srl_autonomy` subscribes.
Teleop never subscribes to anything autonomy produces.

## Data flow, master to arm

```
Teensy 4.1 ──serial──> master_pose_node ──> /master_arm_pose_<arm>
     │                        │
     │                        ├──> /master_arm_raw_<arm>  (pots + IMU)
     │                        └──> /master_status_<arm>   (clutch, scale, valid)
     │
     ├──> master_imu_node ──────> /master_imu_<arm>       (fused orientation)
     ├──> pointing_direction_node ─> /master_pointing_<arm>
     └──> channel_manager ───────> /master_capability_<arm>

/master_arm_pose_<arm> ──> ik_follower_node ──> /<arm>_arm_controller/joint_trajectory
                                  │
                                  └──> /ik_status_<arm>  (success, guard, clearance)
```

## Data flow, autonomy

```
wrist cameras ──> apriltag_detector ──> /perception/detections/<arm>
                                              │
                                    object_pose_tracker
                                              │
                                              v
                                     /perception/objects
                                              │
        /master_pointing_<arm> ──────> intent_inference ──> /autonomy/intent_debug_<arm>
                                              │
                                              v
                                      grasp_generator  ──(validated by /compute_ik)──>
                                              │                /autonomy/grasp_<arm>
                                              v
                                      handover_arbiter ──> /autonomy/state_<arm>
                                                           /autonomy/assist_pose_<arm>
```

## Which sensor supplies what — the physical constraints

| quantity | source | why not something else |
| --- | --- | --- |
| roll, pitch | accelerometer | absolute and drift-free from gravity |
| **yaw** | **not observable from gravity** | gravity is identical under rotation about vertical. Physics, not a bug. |
| yaw *rate* | gyroscope | good over seconds, drifts over minutes (−5.0 / +44.0 °/min measured) |
| position | potentiometers ONLY | double-integrating acceleration drifts 0.05 m in 1 s against a 0.272 m workspace |
| elbow angle | j4 pot, or two IMUs | two segments' relative orientation cancels the shared yaw ambiguity |
| wrist orientation at grasp | **the autonomy** | the master cannot measure it, and that is the point of the project |

## Safety layers, outermost first

1. `estop_node` — three independent triggers, all latching.
2. `participant_safety_node` — participant-only limits, re-asserted every 2 s,
   refuses to arm until e-stop reachability is confirmed.
3. `ik_follower_node` — clearance floor from **TF**, redundancy retries, the
   slew guard, the in-flight watchdog.
4. MoveIt — collision-aware IK with the wearer in the model.
5. `handover_arbiter` — the operator keeps position authority in every state.
