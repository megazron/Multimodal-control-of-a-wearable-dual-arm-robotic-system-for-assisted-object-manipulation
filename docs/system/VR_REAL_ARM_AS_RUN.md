# VR -> REAL ARM, AS ACTUALLY RUN — 2026-08-20

**This is the first session in which a real Kinova arm was driven from the
Quest over wifi.** It worked. It took most of a day, and almost none of that
was the VR path: it was our own tooling partitioning DDS underneath us.

`VR_REAL_ARM_RUN.md` is the designed procedure. **This file is what happened**,
in the order it has to happen, with the traps named. Follow this one.

---

## THE SEQUENCE THAT WORKED

Every step from a shell that has sourced `scripts/env.sh`. Start every
long-running node with `setsid nohup … < /dev/null & disown` so a timeout in
the calling shell cannot kill it half-started.

### 0. Teardown — always, even from an apparently clean box

```
python3 - <<'PY'
import sys; sys.path.insert(0,'src/srl_teleop')
from srl_teleop import procscan
for p in (r'lib/srl_vr_teleop/', r'ros2 launch srl_teleop',
          r'lib/moveit_ros_move_group/move_group',
          r'lib/controller_manager/ros2_control_node', r'lib/srl_teleop/',
          r'robot_state_publisher', r'rviz2', r'lib/srl_experiments/',
          r'kortex_highlevel_bridge'):
    procscan.kill_all(p)
PY
ros2 daemon stop  ; sleep 8
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*  ; sleep 8
ros2 daemon start ; sleep 4
```

**Never `pkill -f <pattern>`.** It matches the shell running it. It killed my
own shell twice today (exit 144). `procscan.kill_all` is the only sanctioned
search — it excludes self, ancestors and the process group.

### 1. Simulation — `master:=false` is NOT optional with no Teensy attached

```
bash scripts/run_teleop.sh gate:=false master:=false
sleep 100
```

### 2. Prove discovery BEFORE going near hardware

```
ros2 node list | wc -l          # expect ~29
```

A fresh rclpy participant should see **~83 topics, ~31 nodes,
`/joint_states` with 1 publisher**. If `ros2 node list` returns 0 against a
stack that is demonstrably running, **do not start the arms** — see the trap
table below, the cause is almost always something we did seconds earlier.

### 3. ONE ARM. Never both — see the rate measurement below

```
SRL_WEARER_PRESENT=0 bash scripts/start_real.sh arm:=right
```

`SRL_WEARER_PRESENT=0` is correct **only** for a bench-mounted rig with the
harness empty and nobody standing in the arms' volume. It makes the clearance
check return `inf`, which prints on every homing line as `clear inf m`. If
anybody is in or near the rig, leave it unset and let the arm refuse.

Homing is complete when the log says
`homing complete on 1 arm(s), every joint within tolerance`.

### 4. VR chain, in this order

```
CRT=$HOME/.srl_vr_cert/vr.crt ; KEY=$HOME/.srl_vr_cert/vr.key
WEB=$HOME/kortex_ws/src/srl_vr_teleop/web
ros2 run srl_vr_teleop quest_bridge_node --ros-args -p port:=8765 \
  -p certfile:="$CRT" -p keyfile:="$KEY" -p web_dir:="$WEB"
ros2 run srl_vr_teleop vr_pose_mapper
ros2 run srl_vr_teleop vr_gripper_node
ros2 run srl_vr_teleop vr_safety_node --ros-args \
  -p watch_tracking_reference:=false -p tracking_timeout_s:=0.6
ros2 run srl_vr_teleop vr_feedback_node
```

`watch_tracking_reference:=false` because the headset is **worn**, not standing
on a shelf — the reference-stability freeze is for desk operation, where the
headset is the fixed tracking reference and a nudge must freeze the arm.

`tracking_timeout_s:=0.6`, not the 0.2 default. **Measured on the real link:**
81 Hz, median gap 9 ms, p95 27 ms — and one stall of **1.14 s**. A 200 ms
watchdog freezes on ordinary wifi jitter.

### 5. Observer — LAST, after every shared-memory clear

```
python3 scripts/observer_estop.py --confirm-every 900
```

Press ENTER. It must print `present, holding`. Verify from another shell that
messages are actually **arriving** — a deaf observer looks identical to a
posted one from the terminal it runs in.

### 6. Headset

`https://172.26.244.255:8765/` → accept the certificate → **ENTER WITH
PASSTHROUGH** → hold both controllers up in front of the chest.

Grip = clutch. Trigger = gripper. The real arm follows the sim **1.0 s**
behind, vmax 0.15, lag over 0.5 rad trips the e-stop.

---

## THE TRAPS. Every one of these cost time today.

| symptom | what it really was |
| --- | --- |
| `DISCOVERY PROBLEM, not a missing stack` from `start_real.sh`, against a stack that is plainly running | **`run_teleop.sh` swept every Fast DDS segment at launch, including the running `ros2 daemon`'s.** The daemon does not die, it goes DEAF: `ros2 node list` returns nothing and the preflight blames discovery. FIXED — `srl_clear_stale_shm` now stops the daemon, sweeps, and restarts it |
| discovery dies a few seconds after any `ros2` CLI call | **`timeout N ros2 …` sends SIGTERM**, hard-killing a DDS participant holding shared memory. Ten call sites in `env.sh` and `start_real.sh` FIXED to `timeout -s INT`. **Never run `ros2 topic hz`** — it never exits, so it is always killed |
| the observer terminal says it is posted, nothing receives it | its shared-memory segments were deleted by a sweep **thirteen minutes after it started**. A clear poisons every participant older than it. **Restart the observer after any clear** — and the sweeper's live-process guard now counts the VR stack, the real-arm bridge and `observer_estop.py`, which it never used to |
| grip does nothing, gripper still works | the gripper node does not consult the freeze; the mapper does. Either the observer is unposted, or the controller pose is invalid (`emulatedPosition`) while the gamepad still reports. **`clutch out` with a working gripper is always one of those two** |
| `require_observer_estop:=false` and it is still frozen | the parameter gated ONLY `_srv_enable`, never the freeze path. FIXED — `_observer_required()` / `_observer_satisfied()`. Real-arm ENABLE still requires the observer unconditionally |
| homing stalls, `HOMING BELOW VELOCITY FLOOR` | **two arms cannot share the Kortex high-level path over WSL.** Measured, same box, same session: one arm **21.0 Hz**, latency 12–33 ms; two arms **8.1–12.2 Hz** with spikes to **519 ms** — past the 0.5 s bridge watchdog, which zeroes the speed. The joints stop WHILE STILL BEING COMMANDED and homing reports a symptom two layers down. `KORTEX_RATE` now defaults to 12 Hz per arm for `arm:=both` |
| `ROBOT_IN_FAULT` on shutdown | the arm faulted during the session. It then refuses both speed commands and `Stop()`. Clear the fault on the arm before the next run |

---

## WHAT IS STILL NOT TRUE

* **Two arms have never run together.** The rate measurement above says why.
  `KORTEX_RATE=12` is a hypothesis, not a result — it has not been run.
* **The wearer clearance check was measuring 3 of 12 body parts and only the
  DISTAL half of the arm** until this afternoon. Every "clearance" number
  taken during a real-arm run before 2026-08-20 was measured with the
  shoulder and both half-arm tubes invisible. FIXED, NOT RE-MEASURED.
* **No real-arm run has ever happened with a wearer in the model.** Every
  session so far used `SRL_WEARER_PRESENT=0`.
