# Bring-up, in order

Everything here assumes `source install/setup.bash`.

## 0a. FIRST ACTION OF EVERY LAB SESSION — re-capture the channel baseline

```bash
bash scripts/check_channels.sh          # ~3 min, block A only, 14 sweeps
```

**Do this before anything else that involves the master arm, every session,
whether or not anyone touched a soldering iron.**

`master_pose_node` decides which channels to freeze from a **stored file**,
not from live hardware. If that file is older than the wiring, the software
freezes channels that now work and **nothing downstream disagrees** — the arm
simply behaves as though the repair never happened, and the only symptom is
motion that was already expected to be missing.

The runtime reads the **newest** `recordings/baselines/channels_*.json`, which
is the same file `check_channels.sh` diffs against. So making a repair take
effect is exactly one command:

```bash
python3 src/srl_experiments/trajectory_capture/channel_report.py \
    <csv> --save-baseline recordings/baselines/channels_$(date +%Y%m%d).json
```

(the script prints this line for you at the end). **Target: 12 or more of 14
coherent**, which is where degraded mode stops engaging.

If degraded mode engages, the startup banner carries a boxed warning naming
the file and its age, and `master_pose_node` re-issues it every 30 s. Seeing
that warning with a freshly captured baseline is fine; seeing it with a
baseline older than your last repair means the repair is being discarded.

## 0b. Before anything

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
