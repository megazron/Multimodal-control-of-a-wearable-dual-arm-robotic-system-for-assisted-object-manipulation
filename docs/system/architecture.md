# Architecture: packages, nodes, GUIs, modes

Package layout, the one-way dependency arrow, the IK follower, the operating modes, the VR stack and the three GUIs. Split out of docs/ENGINEERING_LOG.md on 2026-08-12. Nothing deleted.

## IK follower

Joint wrapping **is implemented** (this used to say otherwise):
`CONTINUOUS_IDX = (0,2,4,6)` for joints 1/3/5/7, `wrap_continuous()` on both
the IK seed and the published solution, `time_from_start` from the true
unwrapped delta over `max_vel_rad_s` 0.6 floored at `min_time_s` 0.05, plus a
startup unwind. `right_joint_5` is −175° in both `home_positions_right.txt`
and the URDF (−3.0543 rad), clear of the ±π seam.

### The guard deadlock, and its fix

Symptom: `reject_count` exactly equalled `success_count`, with 100% IK
success and the arm never leaving home. The step guard discarded every
solution further than `max_step_rad` from the current state — but a discarded
solution means the arm does not move, so the next solution is exactly as far.
It could never converge.

Fix: **slew, don't discard.** `clamp_towards()` commits a bounded step toward
the solution, converging in `ceil(distance / max_step)` cycles. A solution is
only *rejected* when the target pose barely moved (pose deadband) yet the
solution jumped — a redundancy flip — and even then `flip_reject_limit`
force-accepts after 10 consecutive rejections so rejection can never deadlock
either. `[GUARD]` reports `direct / slewed / rejected` separately; conflating
them made a working solver look like a failing one.

Regression tests: `src/srl_teleop/test/test_ik_guard.py` (6 tests, includes
convergence from 2.0 rad within `ceil(2.0/0.35)` cycles).

### In-flight watchdog

`pending` guards against overlapping IK calls and is cleared in the response
callback. If a call never returns — `/compute_ik` restarted, `move_group`
disturbed — it stayed `True` forever and every later pose was skipped in
silence. Observed exactly that: node alive, poses at 50 Hz, zero IK calls.
`pending_timeout_s` (1.0) now force-releases it.

## Analysis

`ros2 run srl_teleop analyse_teleop <csv>` — gain matrix, effective scale,
lag, tracking error, clutch jump, dropouts, and four plots.

## Status dashboard

`ros2 run srl_teleop dashboard` (own terminal, full-screen) or
`teleop.launch.py dashboard:=true` (plain blocks in the log). Plain ANSI, no
toolkit, works over SSH. Pot colouring is **mode-aware**: exact-0.0 is red
only on channels the current `position_mode` consumes, so a dead-but-unused
j5 shows yellow rather than training you to ignore red.

## Three-terminal split (2026-07-31)

    terminal 1:  ros2 launch srl_teleop teleop.launch.py gate:=false
    terminal 2:  ros2 run srl_teleop live_monitor
    terminal 3:  bash scripts/start_real.sh          (--mock to rehearse)

`gate:=false` skips the y/N prompt; sim teleop is live immediately and the
real driver stack is not running at all. `monitor:=false` is now the default
in teleop.launch.py, because live_monitor is a full-screen redraw whose
cursor-home escapes corrupt a shared launch log, and running it in both
places would duplicate the node name.

`live_monitor` was rewritten: it previously coloured dropouts against j1-j5
(the **FK-mode** channels) while the default mode is spherical, which uses
only j1/j2/j4 -- so it reddened j3/j5 for being idle. Now mode-aware, matching
dashboard.py, and it shows gyro as well as accel.

`scripts/start_real.sh` refuses with a named reason and exit 1 when: the sim
stack is not up, the e-stop is latched, homing finishes outside tolerance, or
the bridge will not enable. `--mock` runs the identical sequence against
mock_real.launch.py, which is how it is tested without hardware.

**Operational fact worth knowing:** after the real arm homes, the bridge will
not enable until the SIM also sits near home, i.e. until the operator is
holding the master at neutral. With the master resting on the bench the sim
sits ~0.19 rad off and the bridge refuses (correctly). This is not a fault.

**Two script bugs found by running it, not by reading it:** `grep -c` prints
`0` AND exits 1 when there is no match, so a `|| echo 0` fallback produced
`"0\n0"` and broke an integer test; and `ros2 topic hz` needs several samples
before printing, so a short timeout reported a perfectly healthy master as
"silent". Both fixed.

# Restructure and research build (2026-08-05)

## Part 2 — six packages, one-way dependencies

Moved out of `srl_teleop`: `vlm_locate_node` → `srl_perception/vlm_object_locator`,
`pick_place_node` → `srl_autonomy/scripted_pick_place`, `move_to_pose_node` →
`srl_autonomy/move_to_pose`, `spawn_cubes` → `srl_experiments/scene_spawner`.
Deleted `master_pose_node.py.bak`. Root clutter (`read_teensy.py`,
`srl_gui_wsl.py`) moved to `scripts/legacy/`.

`colcon build` with no flags now builds 16 packages clean. Two vendor gripper
packages are `COLCON_IGNORE`d for a missing `serial` dependency — pre-existing
and already noted in this file; `robotiq_description`, which is the part the
URDF needs, still builds.

**One real defect found by auditing entry points against modules:**
`capture_zero` had no `main()` — its whole body was under
`if __name__ == "__main__"`, so `ros2 run srl_teleop capture_zero` had never
worked. Wrapped in `main(argv=None)`.

# VR TELEOPERATION (2026-08-05)

Two new packages, `srl_vr_teleop` and `srl_vr_autonomy`, plus `quest_app/`.
Verified: **no import of `srl_teleop` in either direction**, and `srl_teleop`
still lists 30 executables and runs with all five other srl packages removed
from `install/`.

**Transport: WebXR over a WebSocket.** The dominant latency term is the
headset's own frame period (13.9 ms at 72 Hz), identical for Unity; the
WebSocket adds single digits on a LAN. Unity would buy a few ms in exchange
for a licence, adb sideloading and a rebuild per iteration.

Measured against the desktop mock: **controller 72.0 Hz** (requirement ≥60),
**robot command 100 Hz**, ROS-side lag below the 5 ms measurement resolution.
Clutch engage anchors exactly on the robot's current EE, so the re-engage jump
is **zero by construction**.

**Assistance differs from the mannequin, and that is the interesting part.**
The mannequin's autonomy supplies wrist orientation the input cannot express —
capability recovered, a categorical result. VR already has 6-DOF, so its
autonomy supplies PRECISION: a funnel that blends toward the validated grasp
over the last 3 cm, and **yields entirely if the operator starts rotating the
controller**.

Two bugs the mock caught that reading would not: the clutch consumed its
rising edge on a refused engage and could never engage again; and the mock
moved from frame zero, so the quasi-static engage check correctly refused it
forever.

---

# PART 4 — VR ON THE VENDOR PROTOCOL, BUILT AND MEASURED (2026-08-07)

Full checklist: `docs/system/vr_bringup.md`.

`srl_vr_teleop/quest_vendor_bridge.py` is a WebSocket server on 8766 speaking
the vendor's exact JSON, so the user's existing Unity client works
unmodified. `quest_vendor_mock.py` is a desktop client emitting the same
schema, with scenarios for each safety path.

## Measured against the mock, no headset

| | |
| --- | --- |
| packet rate in | **68.5 Hz** (822 packets / 12.0 s) |
| status frames pushed back | 242 |
| **round-trip latency** | **median 8.66 ms, p95 16.5 ms** |
| tracking-loss freeze (limit 0.20 s) | fired at **0.24 s** |
| link-loss freeze (limit 0.30 s) | fired at **0.32-0.38 s** |
| observer e-stop absent | **refuses to drive at all** |
| clutch cycle | DISENGAGED then ENGAGED with the reference latched at the controller's CURRENT pose |

Round trip is measured by the CLIENT, because only the client has one clock
at both ends; the bridge echoes `timestamp` untouched. Measuring it in the
bridge would need the two clocks to agree, which they do not.

## Frames

    p_world = (x_q, z_q, y_q)          Unity x right, y UP, z fwd
    q_world = (qx, qz, qy, -qw)        -> world x right, y fwd, z up

Swapping two axes flips handedness, which is the conversion required; the
rotation carries it in the negated scalar. **A yaw about Unity's up axis
comes out NEGATED about world z, and that is correct** — +theta in a
left-handed frame is the same physical turn as -theta in a right-handed one.
My first test asserted the sign was preserved and failed; the assertion was
wrong, not the code. The property that actually matters — position and
orientation ending up in the SAME frame — is pinned separately, because if
they disagree IK is asked for a pose that does not exist. 7 tests.

## Two bugs found by running it

1. **`self.clients` collides with `rclpy.Node.clients`** (its service
   clients), a property with no setter. The node died at construction.
2. **The freeze log flooded at 839 ERROR lines in one 11 s run**, because the
   "has it changed" test compared a message containing the elapsed time, so
   every tick looked like a new event. Now compares the freeze CATEGORY, with
   a 5 s heartbeat: 839 -> 21 lines. A flood buries the transition it reports.

## First-person view

`wearer_view` publishes TF `head -> wearer_eyes` (+90 mm forward, +60 mm up,
along the wearer's +y) and a CameraInfo. Verified with `tf2_echo`. Not
cosmetic: whether the arms feel attached or external is what the experiments
measure, and a third-person camera answers that before the study starts.

## NOT built, deliberately

**The Unity-side overlay rendering.** The data channel is built, running and
republished on `/vr_state`; the vendor server cannot do this at all (it is
receive-only, no `send()` anywhere). Writing a Unity scene that cannot be
compiled or tested here would be worse than saying so. The panel spec is in
the bring-up doc.

## Blocker

**`adb` is installed neither in WSL nor on Windows.** Android Platform-Tools
is the one hard prerequisite before any headset test.

---

# FULL AUTONOMY — THE SIX MODES (2026-08-07)

## The mode registry

`srl_teleop/operating_modes.py` — in the package that depends on nothing
in-repo, so every other package can import it without inverting the
dependency arrow the experimental design rests on.

| # | mode | input | robot decides | max_vel | status |
| --- | --- | --- | --- | --- | --- |
| 1 | DIRECT_MANNEQUIN | master | nothing | 0.60 | implemented |
| 2 | DIRECT_VR | Quest | nothing | 0.60 | implemented |
| 3 | ORIENTATION_ASSIST | master + vision | wrist | 0.60 | **STUB** — this `/compute_ik` plugin ignores `OrientationConstraint` (measured: identical IK success with and without it) |
| 4 | SHARED_AUTONOMY | master + vision | wrist, target, approach | 0.60 | implemented |
| 5 | SUPERVISED_AUTO | point/voice | + grasp, transport, place | **0.25** | implemented |
| 6 | FULL_AUTONOMY | voice | + target | **0.15** | implemented; **perception models not installed** |

**Autonomy runs SLOWER than teleop**, by design: nobody is watching, so the
only bound on a wrong motion is how long it takes to happen.

**One safety stack, no per-mode opt-out.** `assert_safety_invariant()` raises
on every transition. 36 ordered mode pairs tested; dropping ANY one of the
six shared mechanisms is refused in ALL six modes (36 x 6 assertions).

## Model selection — decided by a 4 GB VRAM ceiling

This machine is an **RTX A500 Laptop, 4096 MiB**, 15 GB RAM, no torch
installed. That, not benchmark rank, decides most of it.

| need | chosen | why |
| --- | --- | --- |
| STT | **faster-whisper `small`, int8, CTranslate2** | ~1 GB, 3.4% WER. int8 cuts VRAM ~40% and gives ~4x throughput over the reference implementation. `large-v3-turbo` is better English but needs ~6 GB and would leave nothing for the detector, which must be resident at the same time |
| detection | **YOLO-World** (Grounding DINO Swin-T as an optional higher-accuracy backend) | YOLO-World is ~5x smaller and ~3x faster (35 FPS vs 11 FPS on a 4090); on an A500 the accuracy leader would be far too slow, and autonomy is deliberately slow-moving anyway |
| pointing | **Molmo — RULED OUT** | smallest Molmo 2 is **4B** (~8 GB fp16). It does not fit. And its value would be resolving deictic "that thing over there", which mode 6 CANNOT USE: voice-only autonomy has no pointing input to ground against. Weight for a capability we cannot exercise |
| segmentation | **depth inside the detected box**; SAM 2.1-tiny optional | we need a grasp axis, not a pixel-perfect mask, and the RGB-D stream already gives the 3D structure. SAM 2 with visual prompts has been reported at ~10 GB on high-end GPUs; tiny is far less but still competes for a budget that is already spent |
| intent | **deterministic grammar, NO LLM** | see below |

### Why a grammar and not a local LLM

The command set is closed and tiny: grab, handover, place, stop. A 7B model
quantised to fit would take 3-4 GB of the 4 GB that EXISTS, add 200-800 ms,
and put non-determinism in the component that decides whether a robot beside
a person's head starts moving.

The failure modes differ in KIND. A grammar that does not recognise an
utterance returns `unparsed` and the robot asks again. An LLM asked for JSON
can emit a **different, plausible** verb, and nothing downstream can tell
that from a correct parse. For five verbs that is a bad trade.

### WHY NOT AN END-TO-END VLA — do not reverse this later

1. **Embodiment.** OpenVLA, pi0 and similar are trained on Open X-Embodiment,
   which contains no backpack-mounted dual-Kinova SRL. Zero-shot transfer to
   a novel embodiment is poor, and fine-tuning needs demonstration data this
   project does not have.
2. **It bypasses the safety stack.** A VLA emits actions directly. Every gate
   this repo has — collision-aware IK, the clearance floor, graduated
   avoidance, the step guard — sits between a *pose* and the arm. A model
   that emits joint targets goes around all of it. On a robot mounted beside
   a person's head that is the wrong trade.
3. **Unexplainable failure.** The modular pipeline fails at a NAMED stage
   with its inputs logged (`/autonomy_decision`). A VLA failure is a number
   that was wrong.

## MEASURED — mode 6, 10/10 checks

`python3 scripts/verify_autonomy.py`

| check | result |
| --- | --- |
| no matching object | REFUSED, "I can't see anything matching 'green cube'" |
| ambiguous (2 matches) | **ASKS** — "on the left and on the right. Which one?" and picks NOTHING |
| below confidence floor | REFUSED |
| target BEHIND the wearer | REFUSED — "I will not plan a path there" |
| deictic with no pointing input | REFUSED |
| unknown verb | "Sorry, I didn't understand that" |
| no wake word | **silent** |
| accept | announces, waits, then moves. **voice -> motion 2008 ms** (2 s of that is the configured confirmation wait) |
| voice "stop" mid-sequence | **2 ms**, `/estop` published |
| decision log | intent, resolve, pick_target, voice_stop — all with inputs |

### Two real bugs found by running it

1. **The wearer keep-out was evaluated too late.** It sat in `start_pick`,
   i.e. AFTER disambiguation, so a target behind the wearer became one of
   three candidates and the robot asked *"which one -- left, right, near or
   far?"*, offering a position it must never reach. Asking the operator to
   choose an option the robot would then refuse is worse than not offering
   it: it invites them to say "the far one" and trust the answer. The
   keep-out is now a FILTER applied before resolution.
2. **A no-wake utterance was answered** with "Sorry, I didn't understand",
   which defeats the wake word — the robot was replying to conversations it
   was never addressed in. Now silent unless the wake word was present.

Also: `self.handle` collided with `rclpy.Node.handle` and killed the node at
construction — the same class as `self.clients` in the VR bridge. rclpy
reserves more names on Node than is obvious.

## T4 INTER-ARM HANDOVER — THE PREMISE IS FALSE, AND THAT IS THE FINDING

The brief expected: teleop cannot hand over because `orientation_mode: fixed`
gives both wrists the same approach direction, but modes 5/6 command full
6-DOF and can give each arm its own wrist angle — "a task impossible under
teleoperation and possible under autonomy".

**Measured in sim, arms at home, 16 candidate transfer points including the
protocol's own transfer station (-0.05, 0.28, 1.060):**

    points reachable by BOTH arms: 0 of 16
    protocol transfer station: left=False right=False

**A handover point does not exist.** The blocker was never wrist
orientation — the two arms' reachable sets are DISJOINT (see the Part 3
finding: 0 of 63 frontal cells reachable by both, entire centreline reachable
by neither). Giving each arm its own wrist angle cannot help, because there
is nowhere to hand anything over.

So T4 is blocked for **both** teleop and autonomy, by geometry, until the
right arm is re-parked in hardware. The "impossible under teleop, possible
under autonomy" result is NOT available on this rig as configured — and
claiming it without this check would have been wrong in print.

---

# GUI LAUNCHER — `scripts/srl_launcher.py` (2026-08-07)

**Primary interface.** `teleop_gui` remains the terminal fallback for SSH.

    python3 scripts/srl_launcher.py
    ros2 run srl_teleop launcher          # same thing, via a shim

## WSLg works — verified before anything was built

    DISPLAY=:0   WAYLAND_DISPLAY=wayland-0   /mnt/wslg present
    Tk 8.6, window created, screen 1920x1200, mainloop entered and exited

## What it does

Four tabs — Teleoperation, Autonomy, Experiments, Calibration & diagnostics —
plus an always-visible live status pane, live parameter sliders, and a large
red E-STOP. Every button launches a managed subprocess in its own process
GROUP, and its output is pumped to a scrollable console. The only second
window needed is RViz, which opens its own.

## The four things that make it more than a button board

**1. It refuses to start a second stack.** Preflight lists every running
stack process and names why: two `master_pose_node` instances split the
serial stream and invalidated a full day of measurements. Measured:
`refused: True`, `job created: False`, with the reason shown.

**2. Preflight shows what failed instead of starting and dying.** With no
Teensy and off the lab network, `Mannequin -> real` refuses with BOTH causes
listed, and **the confirmation dialog is never reached** — preflight runs
first.

**3. Live controls use parameter CLIENTS, never `ros2 param set`.** The CLI
goes through the ros2 daemon, which hangs on this box and has reported
success it had not earned. Every slider calls SetParameters directly and
reads the value back:

    [PARAM OK] /ik_follower_left max_vel_rad_s: 0.6000 -> 0.3000
    [PARAM FAILED] /no_such_node x -> 1.0000 : no set_parameters service

A failure raises a dialog saying **the value on the robot is UNCHANGED**.

**4. Clean shutdown.** SIGINT to the process group FIRST (the Kortex bridge
closes its session on SIGINT and LEAKS it on SIGKILL, and the arm permits
exactly one session), then SIGKILL after a grace period, then an
explicit-PID sweep of orphans. Verified: **0 real stack processes after
close.**

Disabled buttons say why on hover: Mode 3 (`OrientationConstraint` ignored by
this plugin), T2 (needs the redesigned container and would displace T5).

## Two threading bugs found by clicking, not by reading

1. **`log()` from a worker thread** raised `main thread is not in main loop`
   and killed the worker mid-flight, so a failed parameter write produced
   NEITHER an echo NOR a dialog — a silent failure in the component built to
   make failures loud.
2. **`self.after()` from a worker thread raises the same error.** The fix is
   a `queue.Queue` drained by the main-thread tick; workers never touch Tk.

## And one in the verification, worth recording

The first "leftover processes after close" check grepped for bare names
(`move_group`, `master_pose_node`) and **matched its own heredoc**, which
quoted those words — reporting 2-3 leftovers when there were none. The check
now matches installed executable PATHS
(`lib/moveit_ros_move_group/move_group`), which a shell command line cannot
accidentally contain. Fourth instance of the measuring instrument being the
fault, and the reason the standing rule exists.

## Verified by clicking through every path

| | result |
| --- | --- |
| window opens under WSLg | yes, ROS link connected |
| Sim only | 6 stack processes, **546 lines captured** in the console |
| second stack | REFUSED by name, no job created |
| Mannequin with no Teensy | refused, cause shown |
| Mannequin -> real, no Teensy + off-network | refused with BOTH causes, confirm never reached |
| Autonomy mode 4 | started, banner captured |
| param apply / param failure | echoed / dialog raised |
| pilot button | ran, produced `t3 72 trials` |
| E-STOP | published, status flips to `E-STOP LATCHED` |
| status pane | STACK, SAFETY, ACTIVE BLOCKERS, CHANNELS, FOLLOWERS, REAL ARM |
| window close | **0 real stack processes** |

## teleop_gui hung on quit — found by the launcher work, not by the GUI's own tests

While verifying the launcher, the terminal console's pty tests began failing
with exit status 250. It was not a crash: **`teleop_gui` never exited at
all** on `q`. `ex.shutdown()` on a MultiThreadedExecutor blocks indefinitely
if a callback is still running, and the test's wait then expired.

From the outside a hung exit is indistinguishable from a crash — the same
class as every other silent-stop in this project. The teardown is now
time-boxed (`ex.shutdown(timeout_sec=2.0)`, each step guarded) and ends with
`os._exit(0)`: curses has already restored the terminal, so there is nothing
left to clean up in that process, and waiting on an rclpy thread that may
never return is the worse failure. 5/5 pty tests pass.

---

# SRL CONSOLE — VISUAL REDESIGN (2026-08-08)

Three principles taken from operator-display practice (ISA-101, NASA display
standard, annunciator panels):

1. **Colour is a scarce alarm channel.** Low-saturation grey base; saturated
   colour reserved for the abnormal. The corollary I had wrong: a NORMAL
   state is largely UNCOLOURED, not green. The old safety bar painted five
   items red at once, which is the same as painting none.
2. **Hierarchy from size and weight, not boxes and lines.** Three type levels
   only, monospace for numerics so digits are fixed-width, grouping by
   whitespace.
3. **A sentinel must never look measured, and the payload must never be
   truncated.** `-1.000 m` reads as a measurement; `--` reads as absent. The
   REASON is the actionable content -- shorten the label instead.

## What changed

| defect | fix |
| --- | --- |
| pipeline overflowed, last stage cut mid-word | **VERTICAL list**, full-width rows |
| status floated outside the box it described | status colour tints the ROW; label, status and reason are one unit |
| reasons truncated mid-word | `wrap=`, never truncated |
| one font size | three levels (30 px mono bold / 17 px mono / 13-15 px sans) |
| two thirds empty | pipeline fills it; recent events below |
| undifferentiated red safety bar | ONE focal readout; observer amber; caps and frame time plain grey |
| clearance `-1.000 m` | `fmt()` renders every sentinel as `--` |
| 8 stages BLOCKED vs "no blockers reported" | reconciled -- see below |

## The contradiction, reconciled

The two panels answer different questions and one said so badly. `/blocking`
is BlockMonitor **inside ik_follower_node**: it reports guards the FOLLOWER
registered, not whether data is reaching it. When the follower is not running
there is no publisher at all, which is NOT "nothing is blocking". The panel
now states which, and carries the pipeline's own count.

## FIRST-OUT ANNUNCIATION — the change that mattered most

The first redesign still rendered **nine of eleven rows red**. Those were not
nine problems; they were one problem and eight consequences. Annunciator
panels solve this with first-out logic: only the ORIGINATING alarm shows at
full severity. Now exactly one row is red (the root cause), downstream stages
render grey as "consequence of <stage> -- fix that first", and the blocker
text says "root cause Teensy, 9 downstream" so the number matches the
display.

Measured frame time after the redesign: **median 0.01 ms, p95 0.94 ms,
max 2.70 ms** against a 33 ms budget.

# SRL CONSOLE — Dear PyGui operations GUI (2026-08-08)

    python3 scripts/srl_console.py        or  ros2 run srl_teleop console

Three interfaces now, deliberately:

| | use |
| --- | --- |
| **`srl_console`** | **primary.** Dear PyGui, real-time, full instrumentation |
| `srl_launcher` | tkinter. Simple launch + status, no plotting |
| `teleop_gui` | terminal fallback for SSH with no display |

## Framework chosen by measurement

    Dear PyGui under WSLg, 110 frames with a live plot:
        median 1.85 ms   p95 2.29 ms   max 2.89 ms

WSLg's D3D12 layer caps hardware OpenGL below 4.0; Dear PyGui targets 3.3 and
renders fine. **tkinter+matplotlib rejected** — a canvas redraw costs tens of
ms and Tk's threading model already bit twice in the launcher. **A FastAPI web
UI is the better long-term answer** (phone, Quest browser, SSH port-forward)
and is deliberately deferred: it is a second process, a second failure mode
and a websocket to debug, when the point of this tool is to debug everything
else.

## Architecture

ROS owns one thread and every subscription, service call and parameter write.
The draw loop reads ONE immutable snapshot dict, rebound atomically — it never
calls a service, never waits, never locks. Parameter writes are QUEUED to the
ROS thread and answered by callback. Tiered refresh: charts 30 Hz, numerics
10 Hz, verdicts 1 Hz, process/disk 0.2 Hz.

**MEASURED FRAME TIME UNDER FULL LOAD** (stack running, autonomy node,
subprocess streaming, a node killed underneath it):

    median 0.01 ms    p95 3.13 ms    max 4.85 ms     budget 33 ms

## The two panels that justify the whole thing

**PIPELINE FLOW** — eleven stages from Teensy to real arm, each FLOWING /
STALE / BLOCKED with the reason, clickable for detail.

**[WHY IS NOTHING MOVING?]** walks the chain and names the FIRST stage not
passing data. Verified: with no Teensy attached it answers
`FIRST STAGE NOT PASSING DATA: Teensy [BLOCKED] no /dev/ttyACM* and no frames`.

**The master-arm panel leads with STALENESS, not value.** Per channel:
update count, **time since last DISTINCT value**, and a >60 deg jump counter.
A channel unchanged for more than 2 s renders RED even while data arrives —
the dead-man bug was stale data republished with fresh timestamps, and the
0.000 noise floor was rows duplicating faster than the sensor updated. A value
that looks live but has not changed is the failure this panel exists to show.

## Verified by clicking every path

| | result |
| --- | --- |
| Dear PyGui renders under WSLg | yes |
| Sim only | 3 stack processes, pipeline goes FLOWING |
| Mode 4 alongside a running stack | starts (add-ons are not second stacks) |
| second STACK | refused by name |
| param apply / failure | `0.6000 -> 0.6000` / `[error] /nope ... FAILED` |
| **kill move_group underneath the GUI** | **GUI keeps rendering**, count drops 3 -> 2 |
| E-STOP | `E-STOP LATCHED` in the always-visible bar |
| pilot / diagnostics buttons | ran, output streamed |
| window close | **0 stack processes** |

## Two bugs found by clicking

1. **IK reported `BLOCKED — success 0%` with ZERO attempts.** No attempts is
   not failure; it pointed the operator at the solver instead of at the
   missing input. Now `STALE — follower up, NO poses received yet`. Same
   no-data/bad-data conflation this GUI exists to prevent.
2. **The second-stack refusal was too broad** — it blocked autonomy nodes and
   diagnostics, which are ADD-ONS to a running stack. Refusing those whenever
   a stack exists makes the refusal useless in the only situation where you
   would use them. Now scoped by `starts_stack`.

---

# PART 4 — THE HOLOGRAPHIC ARM PANEL (2026-08-10)

Both master arms and both robot arms as kinematic diagrams rather than tables.
Previous HUD kept at `archive/scripts/srl_hud_20260810_pre_holographic.py`.

**Master arm.** Seven nodes in the real J1 roll / J2 bend chain, drawn at the
arm's TRUE bend angles with a uniform scale-to-fit, so the picture is the
posture and not a fixed row of boxes. Roll joints are rings carrying a
rotating index mark; bend joints are hinges. Live angle and **time since the
last DISTINCT value** sit under each node — the second number is the one that
matters, because this rig's characteristic failure is data that arrives on
time and never changes. IMU is a gravity dial with the 1 ± 0.15 g gate drawn
as a dotted annulus; FSR is a bar with the 250 deadband and the 1200/400
latch marked.

**Robot arm.** Seven joints with their LIMITS drawn and the position shown
within the range, wrap counts on the four continuous joints, aperture against
the object width, and clearance as a margin against the 0.12 m floor rather
than a bare number.

**Three-state discipline throughout, by SHAPE as well as colour** so it
survives greyscale: UNKNOWN draws DASHED, healthy draws solid, abnormal gets
the only glow on the panel. Same rule as `readiness.py` — not-checked can
never render as healthy.

## It found the seam risk on its own

With the sim at home the panel put **left J5 in red at a stop**, unprompted.
That is the documented hazard: left joint_5 sits at −165.90°, **14.10°
(0.246 rad) from the ±180 seam**, inside the 0.3 rad margin used elsewhere.
The number was already in this file; nothing had ever *shown* it.

## Frame time, measured with a full sim stack running

    median 2.80 ms   p95 9.57   max 27.57   n=300   budget 100 ms

against 2.26 / 9.14 / 16.66 before Part 4 — about 0.5 ms of median for the
two schematics.

## A STRAY robot_state_publisher KILLS THE STACK, AND LOOKS LIKE A POSTURE

Every robot joint read **0.000** — a plausible number, in a plausible place,
on a panel that had just been rewritten. It was not the panel and not the
arms. A `robot_state_publisher` left over from the CAD-arm figure work, 59
minutes old and in nobody's process group, owned `/robot_description`. The
CAD master-arm URDF has no `ros2_control` tag, so the stack's
`ros2_control_node` read it, threw

    what():  no 'ros2_control' tag found in the URDF

and **died at startup**. With no controller manager, `/joint_states` came
from a fallback publisher at all zeros. Nothing downstream disagreed.

This is the second-stack failure in a new costume, and the one-stack guard did
not cover it: that guard counts `master_pose_node`, and a bare
`robot_state_publisher` is not one. `_foreign_description()` in
`scripts/srl_gui.py` now refuses to start a stack while one exists, naming the
PIDs and the kill line. It distinguishes a stray from the stack's own by
`--params-file`: a launched publisher is handed the description through a
parameter file, a hand-started one is handed a URDF path. That difference
needs no bookkeeping to stay true.

**Verified by making one**, in its own session, and watching the guard go
empty → non-empty → empty. On its first live run it immediately caught a
SECOND real stray I had orphaned: `p.kill()` on a `ros2 run` wrapper kills the
wrapper and leaves the executable behind. Kill the process GROUP.

`procscan.find()` excludes this process's whole group by design, so a stray
started as a CHILD of the test is invisible to it — the first version of this
test therefore reported "not detected" against working code. Launch it with
`start_new_session=True`, as a real stray is.
