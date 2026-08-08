# Bring-up, in order

Everything here assumes `source install/setup.bash`.

## 0. Before anything

```bash
bash scripts/diagnostics.sh
```

Checks the things that have actually gone wrong: controllers active,
`/joint_states` at ~100 Hz, the master serial port present, stale Fast DDS
shared-memory segments, arm network, e-stop.

## 1. Simulation only

```bash
bash scripts/run_teleop.sh                    # everything
bash scripts/run_teleop.sh dashboard:=true    # + status dashboard
```

Brings up MoveIt/RViz/controllers, `master_pose_node`, an `ik_follower_node`
per arm, and the e-stop. Sequencing has two independent guarantees:
`startup_delay` (12 s) holds the teleop nodes back while `move_group` loads,
and `ik_follower_node` blocks in `wait_for_service("/compute_ik")` regardless.

## 2. Add IMU-primary sensing

```bash
ros2 launch srl_teleop sensing.launch.py tau_s:=0.5
ros2 topic echo /master_pointing_left
ros2 topic echo /master_capability_left      # what is live, and what each loss costs
```

## 3. Add perception

```bash
ros2 launch srl_perception perception.launch.py
ros2 topic echo /perception/objects_info
```

## 4. Add autonomy

```bash
bash scripts/run_autonomy.sh
ros2 topic echo /autonomy/state_left         # DIRECT | ASSIST | GRASPED
ros2 topic echo /autonomy/arbiter_left       # and WHY it is in that state
```

## 5. Real hardware

See `03_real_robot_bringup.md`. In short, three terminals:

```
terminal 1:  bash scripts/run_teleop.sh gate:=false
terminal 2:  ros2 run srl_teleop live_monitor
terminal 3:  bash scripts/start_real.sh --mock     # drop --mock when ready
```

`start_real.sh` refuses, with a named reason and exit 1, when the sim stack is
not up, the e-stop is latched, homing finishes outside tolerance, or the
bridge will not enable.

## 6. A participant session

```bash
ros2 run srl_teleop participant_safety_node
# ... physically check the e-stop is in the participant's reach ...
ros2 topic pub --once /estop_reachable_confirmed std_msgs/Bool "{data: true}"
ros2 service call /participant/arm_session std_srvs/srv/Trigger
bash scripts/run_experiment.sh e2 --participant P01 --participant-index 0
```

The full checklist is in `docs/research/03_ethics_and_safety.md` §6.
