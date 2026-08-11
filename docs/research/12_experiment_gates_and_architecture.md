# MSc experiment — the four gates, then the architecture

Gates first, as instructed. Every number below is measured in this repository
and reproducible from the scripts named. Nothing was designed until the gates
were answered, because two of them change what gets built.

---

# GATE 1 — the timeline is NOT 2.5× over, and the cap does not need raising

**The brief's premise is wrong on the parameter.** It assumes a real-arm cap
of `max_vel_rad_s = 0.05`. The shipped value is **0.15**
(`teleop.launch.py`, `real_max_vel_rad_s`). `0.05` is two *different*
parameters — the homing velocity in `real_homing_node`, and
`real_max_step_rad`. Neither caps teleoperated travel.

**Measured, not estimated.** `scripts/gate_check_experiment.py` walks Task A's
own declared clip path (41 densified waypoints, 0.537 m of end-effector
travel, 5.22 rad of joint travel) and sums the durations using
`ik_follower_node`'s own rule — `time_from_start` = largest joint delta ÷ cap,
floored at `min_time_s`:

| cap (rad/s) | one pick-and-place |
| --- | --- |
| 0.05 — the brief's assumption | 36.0 s |
| **0.15 — shipped real cap** | **11.2 s** |
| 0.30 | 5.7 s |
| 0.60 — sim default | 2.9 s |

So a pick-and-place is **11.2 s**, not the ~60 s the brief derives, and it is
already inside the 22–30 s the schedule allows. The 2.5× overrun came from
compounding a wrong cap with an over-estimate of joint travel.

## Whole-matrix arithmetic

Robot travel per trial. T1 is measured; the others are scaled by declared
end-effector path length at the measured 20.9 s/m and are marked as estimates.

| task | per-trial robot travel | basis |
| --- | --- | --- |
| T0 target reaching (3 touches) | ~7.5 s | **est.** 0.36 m |
| T1 pick-and-place (4 cubes) | **44.8 s** | **measured** 4 × 11.2 s |
| T2 coupled carry (see Gate 3) | ~15 s | **measured** carry 2.1 s + approach est. |
| T3 circuit box + multimeter | ~25 s | **est.** 0.604 m out, and back |

Per condition (2 trials each): ≈ 185 s. Across all five conditions:
**≈ 15.4 minutes of arm motion for the entire 40-trial matrix.**

**Robot travel is not the binding constraint and never was.** What fills two
hours is human time: five familiarisation blocks, five questionnaire batteries,
per-trial object re-randomisation by hand, instruction and reset — plus the
wearer's own limits, which are already enforced in `session_timeline.check()`
at 12 min continuous and 48 min total pack-on. The existing A/B/C timeline
lands at exactly 120/120 min with those overheads and 15 min of robot motion in
it.

## The safety question, answered without a new measurement

No cap increase is needed, so the clearance question is already answered — and
answered *conservatively*. The graduated-collision probe
(`scripts/probe_collision_response.py`) was run at the **sim** cap of 0.6, i.e.
**four times the real cap**, driving the commanded EE straight into the head
and torso and holding it there for 6 s:

| arm / zone | min clearance | floor | clearance-floor blocks | recovery |
| --- | --- | --- | --- | --- |
| left head | 0.0791 m | 0.05 | **0** | +189 IK successes on retreat |
| left torso | 0.0752 m | 0.05 | **0** | +188 |
| right head | 0.0730 m | 0.05 | **0** | +173 |
| right torso | 0.0722 m | 0.05 | **0** | +173 |

The hard floor was never reached: collision-aware IK and redundancy re-seeding
stopped the arm 72–79 mm out, and every zone resumed on retreat rather than
latching. At 0.15 rad/s the arm is slower than that in every respect.

## Recommendation

**Change nothing about velocity, trials or sessions.** Do not raise the cap
(nothing needs it, and raising it spends the safety margin above for no gain).
Do not cut trials (no power is bought back — the time is not there). Do not
split into two sessions (it doubles recruitment and changes the ethics
submission for a constraint that does not bind).

**Re-derive the schedule against human overheads instead**, using
`session_timeline.check()`, which already refuses a session that does not fit
and already carries the wearer caps. That is the arithmetic that decides it.

---

# GATE 2 — the pinned wrist CAN grasp; the constraint is WHERE, not HOW

15 bench poses per arm, N=10 for the fixed policy, bench in the planning
scene. Four orientation policies on identical poses:

| arm | fixed (anchor) | tilt ±0.3 rad | **yaw-free** | top-down |
| --- | --- | --- | --- | --- |
| left | 13/15 = **86.7 %** | 15/15 = 100 % | 15/15 = 100 % | 15/15 = 100 % |
| right | 0/15 = **0.0 %** | 4/15 = 26.7 % | 5/15 = 33.3 % | 8/15 = 53.3 % |

The right arm's zero looks like the grasping finding — and it is not. Those 15
poses are the *left* arm's band mirrored. Asked in the right arm's **own**
measured band (|x| 0.48–0.57, from `12_randomisation_region.md`):

| right arm, own band | fixed | yaw-free | top-down |
| --- | --- | --- | --- |
| | **5/12** | **11/12** | **12/12** |

**Three conclusions.**

1. **Teleoperated grasping is not blocked.** The pinned wrist reaches 86.7 %
   of the left arm's own graspable band. The 169.7° / 164.6° figure in
   `09_task2_grasping_finding.md` is the rotation needed for a **top-down**
   grasp — a specific approach the pinned wrist cannot make, not grasping in
   general. The anchor approach comes in from the near side, 30.7° *above*
   horizontal, and it works.
2. **Yaw-free objects recover almost everything.** Left 100 %, right 11/12.
   The brief's alternative is the right one: **spheres and vertical cylinders**
   grip at any yaw. Tilt buys nothing over yaw-free (left identical at 100 %,
   right 26.7 % vs 33.3 %) and would need a constraint-aware IK plugin this
   setup does not have — the `/compute_ik` here ignores the `constraints`
   field entirely, which is why tilt was never shipped.
3. **The matrix has no holes on orientation grounds.** It has a hole on
   *position* grounds: the arms' graspable regions do not overlap in mirrored
   x, so a task cannot present both arms with the same layout reflected.

**Recommendation:** keep all five conditions for T1 and T3, and **specify the
graspable objects as vertical cylinders** (the "cubes" become 40 mm cylinders,
the multimeter and circuit box get cylindrical grip features). Per-arm object
layouts, never mirrored ones.

---

# GATE 3 — cup transfer is IMPOSSIBLE. Measured, not inferred.

A cup-over-cup pour is **two different points**, one above the other, so the
0-of-16 transfer-point result did not settle it and it had to be measured
separately.

`scripts/gate3_cup_stack.py` builds a reachability table over
31 × 5 × 8 = **1240 poses**, per arm and per orientation policy, bench in the
planning scene, k = 3 — then reads stacks out of it by lookup:

| | |
| --- | --- |
| stacks tested | **6975** (dz = 0.08 / 0.12 / 0.16 m, three orientation policies, both arm assignments) |
| stacks admitting a pair | **0** |
| solo-reachable control | anchor L 198, R 236; top-down L 209, R 188 — all non-zero |

**Verdict: IMPOSSIBLE**, and the controls make the zero the robot's rather
than the sweep's. The cause is the one already on record: a vertical stack
shares x and y, the left arm reaches only +x and the right only −x, and the
centreline is reachable by neither. The 70–80 mm cup against an 85 mm gripper
and the pour rotation Gate 2 constrains are downstream of a pose that does not
exist.

## The replacement, and which kind of bimanual it is

**Recommend PHYSICAL COUPLING — the existing T3 coordinated carry, already in
the B slot.** A rigid 500 mm tray held at two points with a loose ball on it;
failure is tilt past 6.8°; verified band z 1.10–1.30.

Justified against the research question, not against ease:

* The question is whether an operator can coordinate two supernumerary arms.
  **Coupling is the stronger claim**: remove one arm and the task becomes
  *impossible*, not merely slower. Under simultaneity, one arm doing both
  targets sequentially is degraded performance, not failure — so the dependent
  variable is contaminated by strategy.
* It produces **joint metrics no single arm can generate** — tilt RMS,
  height-difference RMS, separation error — and the pilot shows they
  discriminate across conditions: **7.698 → 4.219 → 1.911 mm**.
* It survives the geometry that killed the cup: two points 500 mm apart *span*
  the dead band instead of needing to meet inside it.
* It is already specified, already verified N=10 over the full densified path
  (3573 IK calls, 0 failures), and already has metrics in the trial logger.

Dual-pursuit (simultaneity) stays available as Task C but is the weaker
bimanual claim and should not be the headline.

---

# GATE 4 — what exists, and the minimum change

Nothing below is a new framework. Every item is an extension point.

| exists | file | reuse as |
| --- | --- | --- |
| session runner, counterbalancing | `srl_experiments/runner.py`, `conditions.py` | condition order for the 5 modes |
| per-trial atomic state + resume | `session.py` (`Trial`, `Session`) | trial state, resume |
| live session manager | `session_manager.py` (`LiveProbe`, `SessionManager`) | the experimenter loop |
| three-state readiness | `readiness.py` (`Check`, `Readiness`, `evaluate`) | Gate G's "confirmed" state |
| trial + sample schema | `trial_logger.py` (`SAMPLE_COLUMNS`, `TRIAL_COLUMNS`) | extend, do not replace |
| mid-trial failure, redo/skip/abort | `runner.py` + `recovery_manager` | unchanged |
| validity / exclusion | `validity.py` | unchanged |
| process isolation | `srl_teleop/procscan.py` | unchanged |
| Qt GUI, every mode launchable | `scripts/srl_gui.py` | add the 4 task buttons |
| scene furniture, one definition | `scripts/clip_scene.py` | object spawning |
| verified reachability | `verify_task_scenes.Solver` | randomisation validation |

**Counterbalancing, checked rather than assumed.** `williams_square(5)`
returns **10 sequences** (odd n needs 2n). Measured with `check_balance`:

| n participants | position imbalance | sequence imbalance | perfect |
| --- | --- | --- | --- |
| 10 | 0 | 0 | **yes** |
| **15 (the brief's number)** | 0 | **2** | **no** |
| 20 | 0 | 0 | **yes** |

At n = 15 the *position* of each condition is perfectly balanced but the
immediate-sequence balance is not. **Recruit 20, or state the residual
sequence imbalance.** Do not silently use 15 and call it counterbalanced.

---

# A — current architecture

Six ROS 2 packages, one-way dependencies (`srl_teleop` imports nothing
in-repo, and that is load-bearing: it is the baseline condition of every
comparison). Master arm → `master_pose_node` → `/master_arm_pose_<arm>` →
`ik_follower_node` (TRAC-IK, collision-aware, clearance floor, step guard,
BlockMonitor) → controllers. VR enters at `/vr/controller_pose_*` through
`vr_pose_mapper` onto the *same* topic. Shared autonomy publishes
`/autonomy/assist_pose_<arm>` into the same `request_ik` gate, so the whole
safety stack is in every path. Full autonomy runs voice → `autonomy_executive`
→ the same topic.

**One source owns the graph at a time.** Measured: after a VR run, modes
01/04/06 all recorded 0.0000 m of travel until `vr_pose_mapper` was killed.
This is enforced at process level and must stay enforced.

# B — proposed experiment architecture

One new node, `experiment_conductor`, and one new module, `task_actions.py`.
Everything else is existing code.

```
srl_gui  ──launch──▶  mode stack (one of five, isolation enforced)
                          │
experiment_conductor ─────┼─▶ task_actions:  REACH_TARGET | PICK_OBJECT
  (owns the trial FSM)    │                  PLACE_OBJECT | GRASP | RELEASE
  reuses Session,         │                  TRANSFER     | RETURN
  Readiness, TrialLogger  ▼
                     /task/command  (action)  ──▶  the ACTIVE mode adapter
                     /task/feedback (topic)   ◀──
```

**The task layer never names a mode.** `REACH_TARGET(arm, target_id)` is
identical under all five; only the *adapter* differs — master, VR, assisted,
or executive. That is the architecture principle, and it is what makes the
40-trial matrix one implementation rather than five.

# C — files to add

| file | why |
| --- | --- |
| `srl_experiments/task_actions.py` | the seven-verb command API + the action definition |
| `srl_experiments/experiment_conductor.py` | trial FSM, randomisation, per-trial seed |
| `srl_experiments/mode_adapters.py` | five thin adapters, one per condition |
| `srl_experiments/experiments/msc/tasks_msc.py` | T0–T3 declarative specs |
| `scripts/verify_msc_scenarios.py` | N=10 over full paths, bench in scene |
| `srl_experiments/action/TaskCommand.action` | goal / feedback / result |

# D — files to modify (minimum change)

| file | change |
| --- | --- |
| `trial_logger.py` | **add columns only** — `finish()` already raises on unknown columns, so nothing can be silently dropped |
| `conditions.py` | nothing; call `williams_square(5)` and record the n=15 imbalance |
| `readiness.py` | add the object-initialised check as a `Check`; do not add a boolean |
| `session.py` | add `seed` to `Trial` |
| `scripts/srl_gui.py` | four task buttons, disabled-with-reason when a mode cannot run one |
| `clip_scene.py` | cylinders alongside cubes (Gate 2) |
| `session_timeline.py` | the four MSc blocks |

# E — topics, services, actions

Reuse, do not invent: `/master_arm_pose_<arm>`, `/vr/controller_pose_*`,
`/autonomy/assist_pose_<arm>`, `/ik_status_<arm>`, `/blocking`,
`/blocking_summary`, `/estop`, `/estop_state`, `/estop_reset` (**a service**,
not a topic — six fault-injection runs were once wasted on that), `/joint_states`,
`/planning_scene`, `/apply_planning_scene`, `/compute_ik`,
`/scene/state`, `/scene/resweep`, `/recovery_state`, `/participant/abort`,
`/trial_state`.

New: `/task/command` (action), `/task/feedback`, `/experiment/state`.

# F — data schema

`TRIAL_COLUMNS` and `SAMPLE_COLUMNS` already carry participant, session,
task, condition, trial index, timestamps, duration, EE pose, commanded pose,
gripper, master pose, autonomy state, intent distribution, clearance, e-stop.
**Add**, per the brief:

`seed`, `mode`, `task`, `difficulty`, `object_poses_json`,
`target_poses_json`, `vr_controller_pose_*`, `blend_alpha`,
`human_cmd_*`, `autonomy_cmd_*`, `intervention_count`, `utterance`,
`parse_json`, `target_identified`, `execution_result`,
`wrong_colour_placements`, `failed_grasps`.

**Wrong-colour placement and failed grasp are separate columns**, as the brief
requires — one is perception, the other manipulation, and a single
"failures" count would confound the two effects the study is trying to
separate.

Add `dynamics: mock | physics` to every row (physics-gate rule: mock and
physics figures must never share a column).

# G — trial state machine

```
IDLE → CONFIGURE → INITIALISE_OBJECTS → CONFIRM → ARMED
     → RUNNING → COMPLETE|FAILED|ABORTED → LOGGED → IDLE
```

**Recording does not start until CONFIRM passes**, and CONFIRM is a
`Readiness` evaluation — three states, never a boolean, because a boolean
cannot represent "nobody asked". Every object and target must report
*confirmed present at its commanded pose* (via `/scene/state`), not merely
*commanded*. An expired or unknown check blocks ARMED exactly as an active
one does.

# H — randomisation

Sample only from the **measured** safe regions —
`recordings/baselines/randomisation_region.json` and
`task0_verification.json` — never from a nominal box. Per-arm regions, since
there is no shared one. `seed` is drawn once per trial, saved in the trial
row, and replayed by `--seed` so any trial reproduces exactly. Every sampled
configuration is **validated before use**: N=10 IK over the full densified
path with the bench in scene, and a rejected sample is redrawn and *counted*,
so a region that is quietly too tight shows up as a rejection rate rather than
as silence.

# I — safety

Nothing new is invented; the existing stack is reused and the randomiser is
constrained to it. Joint and workspace limits from the URDF and
`min_clearance_m` (0.15 in the launch, 0.12 real); velocity cap unchanged at
0.15 real / 0.6 sim (Gate 1); e-stop via `/estop` and the dead-man, which
trips on a **frozen-but-publishing** master in 0.94 s as well as a silent one
in 0.97 s; collision-aware IK with redundancy re-seeding and the graduated
response measured above; `max_trial_duration_s` aborting to `FAILED` rather
than hanging; `blocking_aggregator` for abnormal state, where an **expired**
blocker counts as blocking because the condition is unknown, not clear; safe
reset to the verified home between trials; experimenter override through the
GUI and `runner`'s existing redo/skip/abort.

**The randomiser can never emit an unsafe configuration** because sampling is
restricted to verified regions and every draw is re-validated (H).

# J — implementation plan

1. `task_actions.py` + the action definition; five adapters; **prove
   mode-independence** by running `REACH_TARGET` under all five and asserting
   identical task-layer traces.
2. `tasks_msc.py` for T0–T3 with cylinders (Gate 2) and coupled carry as T2
   (Gate 3); `verify_msc_scenarios.py` at N=10 with the bench.
3. `experiment_conductor` FSM + seeded randomisation, reusing `Session`,
   `Readiness`, `TrialLogger`.
4. Schema extension; a test that every new column is written by at least one
   code path, since the logger raises on unknown columns but cannot detect a
   column nothing fills.
5. GUI buttons; `verify_gui_buttons.py` extended — it already refuses to pass
   a button that launches nothing.
6. Dry run: 40 scripted trials end-to-end, faults injected mid-trial, using
   the existing `inject_experiment_faults.py` harness.
7. Timeline re-derivation and pilot.

**Not started until the above is green:** anything that assumes grasping works
on hardware. Nothing in this repository has ever run against a real arm.
