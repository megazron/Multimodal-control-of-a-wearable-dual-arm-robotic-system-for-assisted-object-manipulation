# PLAN VERSUS DELIVERY

*Phase 2 output. The planning report of 23 April 2026 and the delivered system
diverge substantially. Scope change during an MSc project is normal and
defensible when justified; each row below states the engineering reason rather
than apologising for the change.*

---

## PART A — THREE PRELIMINARY INVESTIGATIONS

### A.1 The scale factor: NOT a bug, and my Gate-1 figure was a red herring

At Gate 1 I reported a derived ratio of $0.902/0.272 \approx 3.32$ against
`WORKSPACE_SCALE = 1.0` and flagged it as unresolved. Traced through the live
code path, there is no discrepancy and no bug.

**Radial scaling *is* applied**, but not by `WORKSPACE_SCALE`. The commanded
position is formed at `master_pose_node.py:1345`:

$$\mathbf{p}_{\text{cmd}} = \mathbf{p}_{\text{anchor}} + \Delta_{\text{anchor}} + s\,(\mathbf{p} - \mathbf{p}_{\text{ref}})$$

where $s$ is the per-arm ROS parameter `{arm}_scale`, defaulting to
`SCALE_DEFAULT = {"left": 1.0, "right": 1.0}` (`master_pose_node.py:85`) and
re-read every frame so it can be retuned live.

`WORKSPACE_SCALE` and `map_to_workspace()` in `master_calibration.py:19,115`
are **not on the live path at all** — their only callers are two self-test
functions inside the same module (`:546`, `:557`). They are effectively dead
code, and a second, unused mapping implementation is a maintenance hazard
worth noting in the Discussion.

**The 1:1 choice is documented and measured, not accidental.**
`master_pose_node.py:80–84` records: at $s = 1.0$ the commanded span is
$\approx 0.44$ m with 92 % (left) and 95 % (right) of recorded poses
IK-reachable, orientation fixed; and "1:1 is also the most predictable mapping
for the operator".

**Therefore the $3.32$ ratio is a ratio of physical sizes, not a design
parameter, and was never intended to be applied.** The master's tip moves at
most $\pm 0.272$ m about its reference and the robot's end effector follows
one-for-one in metres.

**One genuine tension remains, and it belongs in the Discussion.** The later
workspace measurement found the *guaranteed* reachable sphere — the radius
reachable in **every** direction — to be only 0.060 m (left) and 0.090 m
(right). At $s = 1.0$ the operator can therefore command well outside the
guaranteed set, and only 17 of 26 (left) and 14 of 26 (right) directions are
reachable in a single sweep. The engineering log's own recommendation is to
keep $s = 1.0$ until the potentiometer wiring is repaired, on the grounds that
raising the gain amplifies incoherent channels along with the signal. That is
a defensible tuning decision, and it is a decision, not an oversight.

### A.2 The disjoint-workspace result: how strong is it?

Strong within its tested volume, and robust to the two obvious objections —
but **not** a claim about all configurations, and **not** invariant to the
mount.

**Original test** (`scripts/sweep_mount_overlap.py:51–54`): 63 cells at
0.2 m resolution, $x \in \{-0.6 \dots +0.6\}$ (7), $y \in \{0, 0.2, 0.4\}$
(3), $z \in \{0.90, 1.10, 1.30\}$ (3); each arm's **fixed home orientation**;
`tries=3`; single pass.

**Re-test performed for this report**, with both arms commanded to home and
the home offset confirmed at 0.0000 rad immediately beforehand:

| Variant | both | left | right |
| --- | --- | --- | --- |
| fixed orientation, `tries=3` (original) | **0/63** | 16 | 18 |
| fixed orientation, `tries=6`, **N = 3 repeats** | **0/63** | 17 | 18 |
| **yaw free**, `tries=6` | **0/63** | 21 | 21 |

Freeing yaw raises each arm's individual reach from 16–18 to 21 cells and
still produces **no shared cell**, which disposes of the objection that the
result is an artefact of the fixed wrist orientation.

**Fine sweep** at $y = 0.35$, $z = 1.10$ in 0.05 m steps — the plane on which
the arms would have to meet:

```
   right arm reaches   x <= -0.10
   left arm reaches    x >= +0.15
   NOTHING in between: a 0.25 m dead band across the midline
```

**What the result therefore does and does not establish.**

*Established:* within $x \in [-0.6, +0.6]$, $y \in [0, 0.4]$,
$z \in [0.90, 1.30]$, with the as-built mount and the legacy home joint
angles, the two arms' reachable sets do not intersect, and this survives yaw
freedom, more IK retries and repetition.

*Not established:* (i) behaviour outside that box — above 1.30 m, below
0.90 m, behind the wearer, or beyond $\lvert x \rvert = 0.6$; (ii) a shared
region smaller than the 0.2 m grid pitch, although the 0.05 m fine sweep
across the midline argues strongly against one; (iii) any claim about mounts
other than the one in the URDF.

**It is explicitly NOT mount-invariant.** `mount_overlap_sweep.json` shows a
buildable mount change (inboard 0.10 m, forward 0.15 m, down 0.15 m, tilt
$-60°$) raising shared cells from 3 to 22 on the purely kinematic measure.
The correct claim is therefore *"as mounted and as parked, the arms share no
workspace"*, and the fix is mechanical.

### A.3 The master-arm design artefacts EXIST — outside the repository

At Gate 1 I reported zero CAD and zero firmware. That was true of the
repository and false of the machine. A filesystem search located a substantial
design record in **`/mnt/c/Users/Gausms/Desktop/MSc_Project/`**.

| What | Path | Significance |
| --- | --- | --- |
| **Miniature arm links J1–J7** | `MSc_Project/3dprint/J1.stl … J7.stl` | the printed kinematic chain — the core hardware contribution |
| J7 holders, left and right | `3dprint/J7_left.stl`, `J7_right.stl`, `J7hold.stl`, `J7 LID.stl` | wrist mounting |
| Arm stand, base | `3dprint/ARMSTAND.stl`, `base.stl`, `base.3mf`, `BASEPRINT.3mf` | master frame |
| Backpack | `3dprint/backpackfinal.stl`, `backpacklid.stl` | wearable frame |
| Print jobs with author name | `3dprint/Gaus_Sayyad*.gcode.3mf` | fabrication record, sliced |
| **Pot-arm CAD** | `CAD/PotArm.step` | the master arm assembly |
| **Verified forward kinematics** | `basic_control/kinematics.cpp`, `kinematics.h` | the C++ the Python port is derived from — `master_pose_node.py:5` cites it by name |
| **Bill of materials** | `BOM/Gaus_Sayyad_dococ{1,1.1,1.2,1.3,2,3,4,5}.pdf` + expense form | components and cost |
| **Recorded master trajectories** | `Trajectories/{LEFT,RIGHT}ARM{UP,DOWN,LEFT,RIGHT,FRONT,GRIPPERCLOSE}_20260727_*.csv` | **12 CSVs dated 27 July 2026 — real hardware data, per-direction** |
| MuJoCo model zoo incl. Kinova | `mujoco_menagerie/kinova_gen3/gen3.xml`, `robotiq_2f85/` | the simulator actually used |
| MuJoCo run log | `MUJOCO_LOG.TXT`, dated 7 June 2026 | proves MuJoCo executed |
| Two-arm scene and drivers | `two_arms.py`, `two_arms.xml`, `two_armsbackpack.py`, `singlekinova.py` | dual-arm simulation |
| Host serial and test scripts | `com_read.py`, `serialcommand.py`, `recordval.py`, `test_reader.py`, `testjoints.py`, `fakecom.py` | bring-up tooling |
| Earlier GUI | `srl_gui.py`, `srl_launcher.py`, `srl_teleop.py` | precursors to the repo versions |

Additional CAD in `Downloads/`: `Dr_Octopus.STEP` and `Dr_Octopus (1).STEP`
(the platform), `Kinova Gen3 Modular Robotic Arm.step` (vendor reference),
**`WH148 PH1 Single-Joint Potentiometer.f3d`** — which identifies the actual
sensing component — plus `Force Sensor.SLDPRT`, `PotArm`-related knobs and
`Gaus_Sayyad_potarm.gcode`.

**Still not located: the Teensy firmware that emits the `k1j1` protocol.** The
six sketches under `Documents/Arduino/libraries/` (`MPU6050kaProgram`,
`FlexKaProgram`, `ForceKaProgram`, `Combined2_Uplyft`, `CombinedupLYFT`,
`BuzzerKaProgram`) contain **no** `k1j1`, `K1IMU` or `fsr1` tokens and appear
to belong to a different project. `com_read.py` opens COM3 at 9600 baud,
whereas the master runs at 115200 — an earlier, unrelated script. The pin map
and ADC resolution therefore remain unavailable.

**Nothing has been copied.** These are reported for your decision.

---

## PART B — DIVERGENCE TABLE

| Planned (planning report) | Delivered | Engineering reason |
| --- | --- | --- |
| **Isaac Lab** simulation of Kinova dynamics and virtual springs | **MuJoCo** (`mujoco_menagerie/kinova_gen3`, log 7 Jun 2026) for early dynamics; final system uses **ROS 2 + MoveIt 2 with `mock_components`** and collision checking against a wearer model | Isaac Lab requires an RTX GPU and a heavyweight Omniverse stack; the development machine has an RTX A500 with 4 GB VRAM under WSL2, already committed to the perception models. MuJoCo runs on CPU and loads the Kinova model directly. The final verification need was *collision-aware kinematics against a wearer*, which MoveIt provides natively and neither simulator provides for free |
| Physical springs on the figurine providing mechanical force reflection | **Not implemented in software.** Spring study exists in the planning report; the delivered master is a **sensing-only** instrumented mannequin (potentiometers + IMU) | The delivered control loop is position-in, position-out. No force path was built, so springs at the master have nothing to reflect *from*. See E3/E7 below |
| Virtual springs via Kortex **impedance control** at the slave | **Not implemented.** No impedance controller, stiffness or damping gain exists anywhere in the repository | The Kortex **cyclic** interface, which impedance mode requires, is unusable over WSL2: measured write time 3–6 s per cycle against a 10 ms budget, because the realtime channel is UDP 10001 and NAT breaks it. The delivered real-arm path uses the **high-level velocity API** instead (18.4–18.7 Hz measured), which cannot express joint impedance. This is an infrastructure constraint, not a design preference |
| Miniature arms with **real motorised** joints (objective 3) | **Passive** instrumented arms; no actuation at the master | Follows from the above: with no force channel there is nothing for master actuators to render. Also removes a failure mode next to the operator's fingers |
| Three-stage timeline with May/June/July/August milestones | Cannot be evidenced from version control | `git log` holds 16 commits: one import (14 Jul 2026) and fifteen on 8 Aug 2026. Milestone dating is possible only from `CLAUDE.md` (31 Jul – 8 Aug) and the external `Trajectories/` CSVs (27 Jul 2026) and MuJoCo log (7 Jun 2026) |
| Focus: master mannequin design **and** control mapping | Control mapping, safety and verification delivered in depth; **master design record lives outside the repository** | See A.3. The hardware exists and was fabricated; it was simply never committed |
| Coordinated dual-arm pick-and-place on the real platform | Seven task families implemented and verified **in simulation**; **no real-arm execution logged** | Arms unavailable on the development machine (off the lab network; `eth0` down). Everything is `mock_components` |
| Integration of the team's vision/gesture modules | Vision integrated as an **autonomy mode** with open-vocabulary detection; gesture not integrated | Detection rate proved unmeasurable with the available synthetic renderer, so the mode is not participant-ready |
| Dual-arm **coordinated** manipulation | **Blocked by geometry**: the arms share no reachable workspace | Discovered by measurement, not assumed. Drives the title question in Part D |
| User study, n = 8–10 | **Not run.** Protocol, measures, counterbalancing and analysis all designed and exercised with scripted input | Time; and the direct-teleoperation baseline depends on potentiometer channels that are currently incoherent, so a study run now would measure the wiring |

### Risk register outcomes

| ID | Risk | Outcome | What actually happened |
| --- | --- | --- | --- |
| R1 | Master/slave kinematic mismatch | **MATERIALISED** | Handled by 1:1 displacement mapping about a re-latched anchor rather than by joint-to-joint mapping. The residual issue is azimuth: it comes from $q_1$ alone, whose lever arm scales with arm bend, so lateral tracking is wrong and cannot be fixed by the axis map |
| R2 | Latency or instability in the mapping | **AVOIDED / UNTESTED** | No instability observed in simulation; latency never measured (E8) |
| R3 | Hardware access delays | **MATERIALISED, dominant** | No real-arm artefact exists. This is the single largest cause of divergence |
| R4 | Integration with team modules | **PARTIALLY MATERIALISED** | Vision integrated; gesture not |
| R5 | Safety during dual-arm demo | **AVOIDED** | No demo occurred. The safety layer was instead validated by fault injection, 14/14 |
| R6 | Fabrication delays | **AVOIDED** | J1–J7 printed; gcode and sliced jobs present |
| R7 | Poor haptic transparency | **NOT APPLICABLE** | No haptic channel was built |
| R8 | Communication lag master↔slave | **MATERIALISED, differently** | Not ROS 2 lag but the Kortex UDP cyclic channel under WSL2 NAT; forced the high-level API |
| R9 | Overloading arms in coordination | **NOT APPLICABLE** | No real-arm coordination attempted |
| R10 | Finger fatigue in long demos | **NOT APPLICABLE** | No participant sessions |

---

## PART C — THE NINE EVALUATION TARGETS

| # | Target | Verdict | Note |
| --- | --- | --- | --- |
| E1 | Positional error < 10 cm | **NOT MEASURED** (measurable) | Simulated tracking is 0.7–4.7 mm RMS across 87 runs, but that is command-to-simulated-EE, not master-to-slave against ground truth. Needs the real arm, or a defined sim proxy stated as such |
| E2 | Angular error < 15° | **NOT MEASURABLE AS BUILT** | `orientation_mode` is `fixed`: commanded orientation is pinned to the anchor and never tracks the master's wrist. **To build:** the wrist channels must be repaired (left j6/j7, right j3/j5/j7 are dead or incoherent) *and* `orientation_mode` moved to `anchored`. Until then there is no angular tracking to measure |
| E3 | Force reflection error < 0.8 N | **NOT MEASURABLE AS BUILT** | No force sensing at the master, no impedance at the slave, no force path at all. **To build:** either instrument the master springs with load cells and add a feedback channel, or implement Kortex impedance mode — which needs the cyclic interface, which needs mirrored networking. This is a subsystem, not a measurement |
| E4 | Pick-and-place success > 80 % | **NOT MEASURED** (partially measurable) | T2 completes in simulation with the gripper cycling correctly across 12 runs, but a success *rate* requires trials that can fail. In `mock_components` a grasp cannot fail physically |
| E5 | Completion < 45 s per object | **NOT MEASURED** (measurable now) | The scripted T2 trials run 10 s but are paced by the script, not by an operator. Needs a human driving the master |
| E6 | Phase lag < 200 ms across arms | **NOT MEASURED** (measurable now) | `coupled_metrics.phase_lag_s` is implemented and unit-tested to recover an injected 140 ms lag. It has never been run on two arms following a common command |
| E7 | Overshoot < 8 % | **NOT MEASURABLE AS BUILT** | Overshoot is a closed-loop dynamic property. `mock_components` echoes commands with no dynamics, so simulated overshoot is identically zero and meaningless. **To build:** a dynamic simulator (MuJoCo is already present) or the real arm |
| E8 | End-to-end latency < 200 ms | **NOT MEASURED — highest value** | See Part E. Instrumentation is small and can be built in a day |
| E9 | User study, n = 8–10 | **NOT DONE** | Design complete; no participant |

**Summary: 0 of 9 met, 0 refuted. Four are measurable with what exists
(E1 partially, E4 partially, E5, E6, E8); three require building a subsystem
first (E2, E3, E7); one requires participants (E9).**

---

## PART D — TITLE AND OBJECTIVES

### D.1 The problem with the current title

*"Multimodal control of a wearable dual-arm robotic system for assisted object
manipulation"* asserts three things the delivered system does not support.
**"Multimodal"** — voice and vision exist but the detection rate is
unmeasured and voice input has no audio device, so the only validated modality
is the mannequin. **"Assisted object manipulation"** implies manipulation was
performed; it was performed in simulation only. Most seriously, **"dual-arm"**
in the coordinated sense is contradicted by the finding that the two arms
share no reachable workspace.

### D.2 Three options

**Option 1 — narrow to what was built and measured.**
> *Design and verification of a miniature-mannequin master interface for a
> backpack-mounted dual-arm supernumerary robotic system*

*For:* every word is supported. Puts the master design (the planning report's
declared focus) and the verification methodology (the strongest contribution)
at the centre. *Against:* drops "multimodal" and "manipulation" entirely, so
it reads as a narrower project than was attempted, and loses continuity with
the approved title.

**Option 2 — keep the scope, foreground the constraint.** *(recommended)*
> *Multimodal teleoperation of a wearable dual-arm supernumerary robotic
> system: master interface design, safety architecture, and a workspace
> characterisation*

*For:* retains the approved title's opening so the examiner sees continuity;
"teleoperation" is accurate where "control" over-claims; the third clause
signals that a workspace result is a deliverable rather than an excuse.
*Against:* still carries "multimodal", which needs the thesis to be explicit
that vision and voice are implemented-but-unvalidated.

**Option 3 — lead with the negative result.**
> *Workspace limits of backpack-mounted dual-arm supernumerary limbs:
> a master--slave teleoperation system and its geometric constraints*

*For:* intellectually the most honest; the disjoint-workspace finding is the
most transferable thing the project produced. *Against:* a substantial
departure from the approved title, likely needing supervisor agreement; and it
subordinates the master design, which is the declared focus and the larger
body of work.

**Recommendation: Option 2.** It requires no defence of a title change while
being accurate, and it creates a natural home for the workspace finding.

### D.3 Revised objectives, aligned to delivery

1. Design and fabricate a miniature instrumented mannequin master replicating
   the Gen3 morphology at approximately one third scale, and characterise its
   sensing.
2. Derive and implement the mapping from master pose to slave end-effector
   command, including the coordinate-convention conversion between Kortex and
   ROS.
3. Implement a safety architecture appropriate to a robot mounted beside a
   person, and validate it by systematic fault injection.
4. Characterise the reachable workspace of the mounted configuration and
   determine which manipulation tasks it supports.
5. Establish a verification methodology giving traceable evidence for each
   claim, and apply it across the task set.
6. Identify, by measurement, the constraints that must be resolved before
   hardware validation and a user study.

---

## PART E — WHERE THE WORKSPACE FINDING BELONGS

**Recommendation: a short dedicated chapter between Results and Discussion,
titled "Workspace Characterisation and its Consequences" — roughly 1,200
words.**

*Argument.* It fails as a Results subsection because it is not one measurement
but a chain: individual reach envelopes, the 63-cell overlap test, the yaw-free
and repeated re-tests, the fine midline sweep, the mount sweep, and the derived
consequence that inter-arm handover is impossible. Burying that in Results
forces the reader to hold it until Discussion to learn why it matters. It
fails as a Discussion section because it contains primary measurements, and
Discussion should interpret rather than report.

It also *drives* the rest of the thesis: it removes a task family, it explains
why the delivered task set has the shape it does, and it produces the concrete
mechanical recommendation from the mount sweep. A finding with that much
downstream consequence earns its own heading. A full chapter of the usual
length would be disproportionate, so a short chapter is the right size.

---

## PART F — THE LATENCY EXPERIMENT (E8)

**Objective:** measure end-to-end latency from a master joint moving to the
corresponding slave joint command, against the 200 ms target, and attribute it
per stage. Runnable in one day; no hardware beyond what already works.

### F.1 Instrumentation points

| # | Point | Where | Method |
| --- | --- | --- | --- |
| $t_0$ | frame leaves the master | Teensy | **firmware change required** — add a monotonic millisecond counter to the frame. If firmware cannot be modified, substitute $t_0'$ = host read time and report the result as excluding the serial link, stating so |
| $t_1$ | frame read by host | `master_pose_node`, immediately after `readline()` | `time.monotonic()` |
| $t_2$ | pose published | `master_pose_node`, at `publish()` | stamp already exists; record `monotonic` alongside |
| $t_3$ | pose received by follower | `ik_follower_node.on_pose` entry | `time.monotonic()` |
| $t_4$ | IK solution returned | `ik_follower_node`, in the `/compute_ik` response callback | `time.monotonic()` |
| $t_5$ | trajectory published | `ik_follower_node`, at `publish()` | `time.monotonic()` |
| $t_6$ | joint state reflects the command | subscriber on `/joint_states`, first sample within tolerance of the commanded value | `time.monotonic()` |

Stage latencies: serial $t_1-t_0$; processing $t_2-t_1$; transport
$t_3-t_2$; IK $t_4-t_3$; publish $t_5-t_4$; execution $t_6-t_5$.
End-to-end $= t_6-t_0$.

**Use `time.monotonic()` throughout, never `time.time()`** — the WSL wall
clock steps backwards on host resync and has already produced a $-2321$ ms
latency in this project.

### F.2 Code to add

1. A `LatencyProbe` helper in `srl_teleop` writing one CSV row per frame with
   all available stamps and a sequence number. **~60 lines.**
2. A sequence number carried end to end. `master_pose_node` already maintains
   per-arm `master_seq_*` counters that increment only on payload change; put
   that number in the pose message's `header.frame_id` suffix or a parallel
   `Int32` topic so a trajectory can be matched to the frame that caused it.
   **~20 lines.**
3. An analysis script producing per-stage medians, 95th percentiles and a
   stacked bar figure. **~80 lines**, reusing the existing plotting pattern.

### F.3 Protocol

- **Stimulus:** the virtual Teensy (`scripts/virtual_teensy.py`) in
  `--mode script`, generating a step of a single joint every 2 s, so onset is
  unambiguous and known. Repeat with the real board if available.
- **Trials:** 200 steps, alternating direction, at each of three master rates
  if the firmware allows; otherwise 200 at the native rate. 200 gives a stable
  95th percentile.
- **Conditions:** (i) sim only; (ii) sim with RViz running, since rendering
  competes for CPU; (iii) with the `/real` bridge enabled against
  `mock_real.launch.py`, to expose the cascade's contribution.
- **Report:** median and p95 per stage and end to end; the fraction of frames
  exceeding 200 ms; and the largest single contributor.

### F.4 What it will probably show, stated in advance

The dominant terms are expected to be the **sensor update period** (the pots
update at 14.7–17 Hz, so a change waits up to ~68 ms to be sampled at all) and
the **IK call** (median solve 6.5–7.0 ms, but the redundancy retry can reach
$7 \times 6.7 = 47$ ms). Writing the prediction down first makes the
measurement a test rather than a description.

**Caveat to state in the thesis:** with `mock_components`, $t_6$ measures
command echo, not physical motion. The figure is therefore a *control-path*
latency and an underestimate of the true end-to-end value. Only the real arm
closes that gap.
