# Troubleshooting — keyed by SYMPTOM

**Start here:** `ros2 topic echo /blocking_summary`. Every mechanism that can
stop motion reports there by name, with the recovery for each. If it says
`motion_unexplained: true`, nothing is blocking and the arm still is not
moving — that is the silent-stall signature and the causes are listed under
"the arm is not moving" below.

    bash scripts/recover.sh          # back to a known-good state from anything
    ros2 run srl_teleop preflight    # refuses to start a session, and names why

## "The arm is not moving"

Every cause seen in this project, with its check:

| cause | check | recovery |
| --- | --- | --- |
| a named blocker is active | `ros2 topic echo /blocking_summary` | each blocker publishes its own recovery string |
| e-stop latched | `ros2 topic echo /estop_state` | `ros2 service call /estop_reset std_srvs/srv/Trigger` — it is a SERVICE, not a topic |
| IK failing every pose | `/blocking` shows `ik_failed` | the pose is unreachable or in collision; check the mount and the task volume |
| clearance floor | `/blocking` shows `clearance_floor` | the arm is inside the wearer margin; move the command away |
| in-flight IK guard stuck | `/blocking` shows `ik_inflight` | force-released after `pending_timeout_s`; if persistent, `move_group` restarted |
| motion not armed (`real_robot`) | `/blocking` shows `motion_disarmed` | `ros2 param set /ik_follower_<arm> motion_enabled true` |
| controllers inactive | `ros2 control list_controllers` | `set_controller_state <name> inactive` then `active` |
| cyclic path over WSL | write time 3–6 s in the driver log | use the high-level bridge; mirrored networking is mandatory |
| DUPLICATE node instances | `ps -eo args \| grep lib/srl_` | two followers publish the same unit key and overwrite each other's state — `bash scripts/recover.sh` |
| nothing blocking, still still | `/motion_unexplained` is true | controllers, `/joint_states` rate, or a dead trajectory topic |


Symptoms that have actually happened on this rig, and what they turned out to
mean. Ordered by how much time they cost.

## "The real arm does not move, but feedback is perfect"

**The cyclic path is unusable over WSL.** Every cyclic write is a network round
trip costing ~10 ms; at 100 Hz that is the entire cycle budget, the controller
manager overruns permanently and no command ever reaches the arm.

Use the **high-level API** (`kortex_highlevel_bridge`, ~18 Hz, velocity
commands that hold between sends). Also: **mirrored networking is mandatory** —
the UDP realtime channel on port 10001 is what WSL2 NAT breaks.

Do not be reassured by `ros2 control list_controllers` showing "active": the
controllers activate and the hardware deactivates underneath them.

## "No ROS 2 nodes at all"

Usually a **stale `ros2` daemon**, not a discovery partition.

```bash
ros2 node list --no-daemon      # ground truth
ros2 daemon stop && ros2 daemon start
```

Do NOT pin discovery to loopback to "fix" it. `ROS_LOCALHOST_ONLY` is
deprecated in Jazzy and, applied to only some processes, causes the partition
it is meant to prevent.

## "A service exists but `wait_for_service` times out"

Stale Fast DDS shared-memory segments in `/dev/shm` from killed stacks:

```
RTPS_TRANSPORT_SHM Error ... Failed init_port fastrtps_port7002
```

Discovery goes intermittent — one client sees a service, another does not.
**Stop the stack**, then `rm -f /dev/shm/fastrtps_*`, then relaunch.

## "controller_manager: Waiting for data on 'robot_description'" forever

A **half-killed launch**. Killing the `ros2 launch` PID alone leaves orphans,
and `robot_state_publisher` is gone while `ros2_control_node` survives. Kill
the explicit PID list — `move_group`, `ros2_control_node`,
`robot_state_publisher`, `spawner` — and confirm the count is zero before
relaunching.

Never use a broad `pkill -f "ros2 launch"` while a stack you want to keep is
running. It has cost a whole recording session before.

## "/compute_ik returns −31 (NO_IK_SOLUTION) for everything"

Ask `/check_state_validity` before believing the kinematics are wrong. On this
rig it was a **collision**, not reachability: the arms' proximal links against
the wearer's own upper arms. A single unexcluded static collision makes every
state invalid.

## "A recording has empty EE columns"

`/tf` was dead because `joint_state_broadcaster` and an arm controller had gone
`unconfigured`. Recover:

```bash
ros2 control set_controller_state <name> inactive   # configures
ros2 control set_controller_state <name> active
```

Check **before** any capture that both arm controllers and the broadcaster are
active and `/joint_states` is at ~100 Hz.

## "The value is suspiciously perfect / does not change at all"

**Zero variance is a fault signature on this rig, not a good result.** It has
appeared four separate times:

- the dead j7 pot (spread exactly 0.0),
- frozen `/real/joint_states` while the hardware component was inactive,
- a "fast" 50–160 µs driver read that was returning a cache,
- accelerometer rows repeating bit-identically because the 50 Hz recorder
  oversamples the sensor.

If a measurement is exactly zero, suspect the measurement.

## "The arm never leaves home, but IK reports 100% success"

The old step-guard deadlock: a discarded solution means the arm does not move,
so the next solution is exactly as far away. Fixed by **slewing rather than
discarding** (`clamp_towards`). `[GUARD]` reports `direct / slewed / rejected`
separately for exactly this reason.

## "A node dies with 'Executor is already spinning'"

Calling `spin_until_future_complete()` from inside a callback re-enters the
executor. Use a `ReentrantCallbackGroup` plus a `MultiThreadedExecutor` and a
blocking `client.call()`. This killed `grasp_generator` in the first pilot, and
from the outside nothing looked broken — the arbiter simply sat in DIRECT
reporting "no grasp" forever.

## "Every experiment trial takes exactly the timeout"

The trial had no completion condition. Trials end on a dwell at the target or
on a GRASPED transition; without one, "completion time" degenerates to the
timeout for every condition and the experiment measures nothing.

## "Every trial after the first logs 0.00 s"

State inherited across trials. The arbiter latches `GRASPED` until the gripper
opens, and the mock gripper boots at 0.79 rad — already "closed". Only a
**transition** into GRASPED inside the trial counts, and the arbiter now
requires a closing transition rather than a level.
