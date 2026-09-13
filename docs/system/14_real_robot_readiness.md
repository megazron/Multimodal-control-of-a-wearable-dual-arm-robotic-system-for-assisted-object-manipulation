# 14 — REAL-ROBOT READINESS: what will work, what will fail, what cannot be known

**A PAPER EXERCISE, 2026-08-11.** Nothing here was tested against hardware and
no connection was attempted. Every verdict is read off the code and the
machine's current state, and where the code and `docs/ENGINEERING_LOG.md` disagree the code
wins and the disagreement is named.

Three verdicts, used strictly:

| | meaning |
| --- | --- |
| **WILL WORK** | the mechanism exists, is wired into the path that runs, and its failure mode is loud. Still unproven on metal. |
| **WILL FAIL** | something in the path is missing, wrong, or points at a component that will not be there. |
| **UNVERIFIABLE** | correctness depends on a hardware behaviour nobody here can observe. |

---

## THE ONE-LINE ANSWER

The **arm** path is in better shape than the **safety** path. The high-level
bridge, the sessions, homing and the grippers are coherent and should move a
real arm. But **the e-stop's real-driver halt points at a component that the
supported real path does not use**, and **the clearance floor measures the SIM
arm, not the real one** — both of which are silent, and both of which matter
most at exactly the moment they are needed.

---

## 1. KORTEX HIGH-LEVEL BRIDGE — WILL WORK

`srl_teleop/kortex_highlevel_bridge.py`, launched through
`~/kortex_ws/.kortex_venv/bin/python` by `real_arms_highlevel.launch.py` via
`ExecuteProcess`. Confirmed on this machine: the venv exists and
`import kortex_api` succeeds inside it.

It uses `SendJointSpeedsCommand` over the TCP session — a velocity command
that holds between sends, so it does not need the 1 kHz cyclic loop the
`ros2_control` path needs and cannot get over this link.

**Why this is the only viable path, and must not be "fixed" back:** the cyclic
write is a network round trip costing ~10 ms, measured `Write time: 10365 us`
against a 10 ms budget at 100 Hz, so the controller manager overran
permanently and no command ever reached the arm while feedback stayed
perfectly live. That failure looks exactly like a dead driver and is not one.

## 2. BOTH ARMS' SESSIONS — UNVERIFIABLE

One session per arm, opened by one bridge process each, with per-arm
`robot_ip` (left 192.168.1.10, right 192.168.1.9) — the bug where
`arm:=right` opened a session against the LEFT arm is fixed and the launch
expands `arm:=both` into one node set per arm.

**Why it cannot be graded from here: the arm permits exactly ONE session and a
leaked one refuses the next connect.** The close path (`zero speeds → Stop()
→ CloseSession`) is on every exit and `start_real.sh` kills the bridge by
name because SIGKILL leaks the session. That is the right shape. Whether two
simultaneous sessions against two arms behave is a hardware fact: **two real
Kortex sessions have never been opened.**

## 3. HOMING — WILL WORK

`real_homing_node` uses a VELOCITY law, `speed = clip(kp*angle_diff, ±vmax)`,
so error magnitude cannot produce a fast move, and it decelerates as the error
shrinks. Completion is on `home_tolerance_rad` (0.05) and not on the 1 deg
deadband — the deadband suppresses the CORRECTION, it does not decide when the
arm is home, and conflating them is what produced the phantom "HOMING
STALLED". Hysteresis (enter 1.0 deg, leave 2.0) stops the hunting, and
OSCILLATING and BELOW-VELOCITY-FLOOR are reported as distinct diagnoses from
STALLED.

**Caveat that is genuinely open:** `min_commandable_rad_s` defaults to 0
(infer-only) because **the arm's true minimum commandable speed has never been
measured.** If the real floor is above `kp*err` near the target, the error can
never close and the node will say BELOW VELOCITY FLOOR rather than time out —
which is the correct behaviour, but it means homing may need that parameter
set on the day.

## 4. GRIPPERS ON THE INTERNAL BUS — WILL WORK

The bridge subscribes to `/{arm}_gripper_controller/joint_trajectory` and
converts knuckle radians to a normalised Kortex gripper position, sent **over
the same session** on the arm's internal bus. This bypasses the Robotiq serial
driver, the `/dev/ttyUSB*` contention and the unprefixed-resource collisions
entirely.

A gripper fault is caught and logged and **does not take down the arm loop** —
correct, because the arm is the safety-relevant path.

**Correction to `docs/ENGINEERING_LOG.md`:** it states that `robotiq_driver` and
`robotiq_hardware_tests` are both `COLCON_IGNORE`d. Only
`robotiq_hardware_tests` is; `robotiq_driver` builds and is installed. The
`0003` prefix patch therefore is compiled now, contrary to the note that it
"cannot be compiled here". Neither matters for the high-level path, which does
not use that driver — but the log is stale and a reader would be misled.

## 5. PORT AUTODETECTION — WILL WORK

`serial_port.find_port()` globs `/dev/ttyACM*` and `/dev/ttyUSB*`, opens each
and sniffs for a `k1j1` frame, prefers `/dev/teensy` if a udev rule provides
one, and raises `PortNotFound` naming the `usbipd attach` command rather than
spinning silently. `claim_exclusive()` takes `LOCK_EX|LOCK_NB` so a second
reader is impossible rather than merely detectable, and `sniff()` refuses to
probe a port another process holds — the port SEARCH used to corrupt the frame
stream it was looking for.

**Right now there is no `/dev/ttyACM*` on this machine**, so the Teensy is not
attached and this is unexercised in its current form against the board.

## 6. MIRRORED NETWORKING — WILL FAIL AS THE MACHINE STANDS

**This is the first thing to fix before a lab session.**

* `/etc/wsl.conf` has **no `networkingMode`** (it is set Windows-side in
  `%UserProfile%\.wslconfig`, so its absence here is not conclusive on its
  own).
* But the interface picture has changed since the working configuration:
  `eth0` and `eth1` are **DOWN**, and the only live interface is `eth2` at
  **192.168.3.146/22**. The configuration that worked had `eth0` at
  192.168.1.20/24 with both arms answering at ~1.4 ms.
* `/mnt/c` is present, so the 9p mount is healthy and the Windows side is
  reachable for the operator to check `.wslconfig`.

The arms at 192.168.1.10/.9 do fall inside 192.168.0.0/22, so the route
resolves on-link via eth2 — but **the reason mirrored mode is mandatory is not
routing, it is the UDP realtime channel on port 10001**, which WSL2 NAT
breaks. TCP 10000 opens fine and ping is fast even when 10001 is dead, so a
successful ping proves nothing about whether the arm can be commanded.

**Action before the lab: confirm `networkingMode=mirrored` in the Windows
`.wslconfig`, `wsl --shutdown`, and re-run `scripts/check_arm_network.sh`,
which probes UDP 10001 specifically rather than just pinging.**

## 7. CASCADE RATE LIMIT — WILL WORK

`sim_to_real_bridge` carries `max_vel_rad_s` 0.15, `lag_trip_rad` 0.5, a
separate (tighter) enable gate, `rate_hz` 20, and a configured delay; the SIM
is rate-limited to the same cap through `/cascade_active`, so the sim cannot
run away from the real arm it is driving. It refuses to enable until the sim
is near home — measured as ~0.19 rad off with the master on the bench, which
is correct behaviour and not a fault.

Both intervals use `time.monotonic()`. That is load-bearing: the WSL wall
clock steps backwards on host resync and once produced a send latency of
**−2321 ms**.

## 8. CLEARANCE FLOOR — WILL FAIL (silently) ON THE REAL ARM

`measure_clearance()` looks up `lookup_transform(part, f"{self.arm}_{link}")`
— **unprefixed link names, which are the SIM arm's frames.** The real arm's
frames are `real_*`, published by a `robot_state_publisher` with
`frame_prefix="real_"`.

So under the cascade the floor is measured on the COMMANDED sim pose, one
bridge delay (1.0 s) ahead of where the real arm actually is. As a
*pre-publication* check on a command that is about to be issued that is
defensible and useful. As "the arm is at least 0.12 m from the wearer" it is
**not true of the real arm**, and nothing says so.

It also returns `inf` when TF is unavailable and deliberately does NOT block —
correct, because an interlock that fires on missing data makes the arm
undriveable, but it means a real-arm TF outage silently disables the floor.

## 9. E-STOP UNDER REAL MOTION — PARTLY WILL FAIL

Three things are right:

* the bridge **subscribes to `/estop_state`** and stops commanding on it;
* `sim_to_real_bridge` subscribes too and disables;
* the halt publishes each arm's measured joint positions as a short
  trajectory, republished while latched, so a follower that keeps commanding
  is continuously overridden; and `trip()` parks FIRST and only then touches
  the real driver, because two `wait_for_service(2.0)` calls used to delay
  every e-stop by **4.0 s** on any stack without a `/real` namespace.

The failure: **`halt_real_driver()` targets `/real/controller_manager` and the
`real_components` / `real_controllers` parameters — the `ros2_control` cascade
— and the supported real path is the HIGH-LEVEL BRIDGE, which has no
controller manager.** On a real high-level run that service does not exist, so
the driver-level halt does nothing. It is guarded by `service_is_ready()` so
it fails silently rather than hanging.

**In practice the arm still stops**, because the bridge honours `/estop_state`
directly. But the defence-in-depth layer — cut it at the driver as well as at
the commander — is absent on the path that will actually run, and the logs
will not say so.

## 10. DEAD-MAN — WILL WORK

Keyed on `msg.header.stamp`, not arrival, which is the whole point:
`master_pose_node` substitutes `last_good` and keeps publishing at 50 Hz when
the master degrades, so an arrival-based dead-man cannot fire on the failure
that actually happens. An empty stamp falls back to arrival **and warns**.
Measured: trip at 0.94–0.99 s on a frozen-but-publishing master, 0.97 s on a
silent one, no false positive over 243 fresh frames.

**`deadman_enabled` defaults to False.** That is deliberate for sim, where the
master goes stale constantly — but it means **it must be turned on for a real
session**, and `/estop_deadman` is the topic that proves it is armed.

**UNVERIFIABLE part:** the real board's behaviour across a `usbipd`
re-enumeration (ACM0 ↔ ACM1) has never been observed through this path. The
frozen-data case was reproduced synthetically.

## 11. READINESS GATE — WILL WORK, with a caveat about what it proves

`sim_session.wait_ready()` waits on the CONDITION — a real node RECEIVING arm
joint states plus `/compute_ik` answering plus both followers present — not on
a word printed by another process. It re-spawns the probe in chunks and it is
preceded by `wait_launched()`, which waits for the controller manager's own
"Configured and activated joint_state_broadcaster" and then a settle period
before any participant is created.

**Caveat:** it proves the SIM stack is up. It says nothing about the real arm.
`start_real.sh` is the gate that covers the real side, and it refuses with a
named reason when the sim stack is down, the e-stop is latched, homing
finishes outside tolerance, or the bridge will not enable.

## 12. CAMERA PATH FOR PERCEPTION — WILL FAIL as configured

`kinova_vision` is built and installed with its launch file, so the driver
side exists. The detectors, though, default to **`/camera/color/image_raw`,
`/camera/depth/image_raw`, `/camera/color/camera_info`**, and the Kinova
driver publishes under its own namespace with different names.

The topic names are parameters, so this is a configuration fix, not a code
change — but **nothing currently asserts that the subscribed topic has a
publisher**, so a wrong name gives a detector that runs, reports nothing, and
looks exactly like a scene with no objects in it. That is the project's own
absence-read-as-a-value failure and it is unguarded here.

Separately: **detection rate at working distance is UNMEASURED.** The 0–4%
figure came from a renderer that is out of distribution, and the same model
scores 0.89–0.91 on a real photograph. The ≥95% gate is neither passed nor
failed, so **mode 06's perception is not participant-ready**.

---

## SIM-ONLY, AND WOULD SILENTLY DO NOTHING ON REAL ARMS

Each of these runs, returns success, and means nothing without hardware:

1. **`mock_components/GenericSystem` echoes commands with no dynamics.** Every
   tracking, payload and drift number taken against it is a property of the
   mock. Payload made *exactly zero* difference — 0.0164 m RMS both ways.
2. **The payload manager's hardware call is a STUB.** It publishes what it
   would have set so the value lands in the trial log; it does not set it.
3. **`/real/joint_states` is a CACHE when the hardware component is inactive**
   — 313 samples, one distinct value per joint, peak-to-peak 0.000e+00,
   identical to a reading 40 minutes earlier. Any hold-vs-limp test through
   that topic cannot detect motion.
4. **A fast `read()` from the Kortex driver is the DEAD-CHANNEL signature,**
   not an improvement: once a write error deactivates the component, reads
   return a cache. 50–160 µs where 11479 µs is normal means the channel is
   dead.
5. **T2's ball is drawn, not simulated.** A tilt past 6.8 deg drops nothing.
6. **Objects are FIXTURED** — see `13_fixturing_and_the_approach_cone.md`. On
   a real bench they would be resting and would fall; `drops` becomes a real
   outcome the moment the scene is physical, and the current clips cannot
   speak to it.
7. **The gripper's "grasp" is a knuckle angle and a distance test.** There is
   no force, no slip, no compliance.
8. **`expect_real_stack` and `freeze_on_master_loss` both default OFF**, so
   the session/link recovery paths fire only on an explicit `connected:false`
   / `up:false` and never on silence. On a real stack they must be ON or a
   dropped link is invisible.
9. **`arm_link_monitor` pings 192.168.1.10/.9** — only the injected topic has
   ever been exercised.
10. **The Kortex close-then-reconnect** is exercised through
    `/real/session_state` and `/real/session_recover`, never against an arm.
    The one-session constraint makes this the riskiest thing in the recovery
    layer and the first a real run will hit.

---

## THE SHORT LIST, IN ORDER, FOR THE FIRST LAB SESSION

1. Confirm `networkingMode=mirrored` Windows-side, `wsl --shutdown`, then
   `scripts/check_arm_network.sh` — it probes **UDP 10001**, which is the port
   that actually matters and the one a ping does not test.
2. `bash scripts/check_channels.sh` and save the new baseline; the runtime
   picks the newest `channels_*.json` by mtime.
3. Turn **`deadman_enabled` ON** and confirm from `/estop_deadman` that it is
   armed before anything moves.
4. Set `expect_real_stack` and `freeze_on_master_loss` ON.
5. Fix the detector topic names against `ros2 topic list` with the camera
   running — and check each has a publisher, because a wrong name is silent.
6. Accept that the clearance floor is a check on the COMMANDED pose, and that
   the driver-level e-stop halt does nothing on the high-level path. The
   bridge's own `/estop_state` subscription is what stops the arm.
