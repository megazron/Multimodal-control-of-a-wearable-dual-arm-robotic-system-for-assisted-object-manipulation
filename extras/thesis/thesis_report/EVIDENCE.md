# EVIDENCE LEDGER

*Phase 1 output. One row per prospective thesis claim, with file and line
provenance. Status is one of `IMPLEMENTED` / `SIM-TESTED` /
`HARDWARE-VALIDATED` / `PARTIAL` / `NOT-DONE`.*

**Headline: nothing in this repository is `HARDWARE-VALIDATED`.** No logged
artefact from a physical Kinova arm, a physical Teensy or a human participant
exists anywhere in `recordings/`. Every recorded number was produced against
`mock_components/GenericSystem`, which echoes joint commands with no dynamics.

---

## 0. Structural inventory (§1.1)

| | |
| --- | --- |
| Tracked files | 1097 |
| Python source | 207 files |
| ROS 2 packages | 12 (8 project, 4 vendor) |
| Unit test files | 28 |
| Distinct ROS topics found in source | 57 |
| CSV logs | 219 |
| Recorded clips | 174 mp4 |
| Branches | `main`, `thesis-report`. **No tags.** |

### 0.1 THE COMMIT HISTORY CANNOT DATE THE WORK

The prompt asks for the commit history to be used as "the primary record of
what was built and when". It cannot serve that purpose here, and this must be
stated rather than worked around.

`git log` contains **16 commits**. One, `df8cc54 Initial commit`
(**2026-07-14**), contains essentially the entire system. The remaining
fifteen are all dated **2026-08-08** and are the work of the final
consolidation session. There is no intermediate history.

**Consequence:** milestone dating against the planning report's May/June/July
schedule is impossible from version control. The only usable chronology is the
dated section headings in `docs/ENGINEERING_LOG.md`, which run 2026-07-31 to 2026-08-08 and
are a laboratory notebook rather than an independent record. The thesis must
therefore organise the Method chapter by subsystem, not chronologically, and
the DIVERGENCE table's milestone column must be sourced from `docs/ENGINEERING_LOG.md` with
that caveat stated.

---

## 1. THE MATHEMATICS (§1.2)

Everything below is implemented in code and is quotable as a typeset equation.

### 1.1 Master-arm forward kinematics

| Item | Value / form | Provenance | Status |
| --- | --- | --- | --- |
| Link lengths $\ell_{1..7}$ | `[0.043, 0.037, 0.043, 0.037, 0.043, 0.036, 0.033]` m | `master_calibration.py:5` | IMPLEMENTED |
| Joint types (roll vs pitch) | `IS_ROLL = [T,F,T,F,T,F,T]` — alternating roll/bend, matching the Gen3 morphology | `master_calibration.py:6` | IMPLEMENTED |
| Full extension $L$ | $\sum \ell_i = \mathbf{0.272}$ m | `master_calibration.py:7` | IMPLEMENTED |
| Transform chain | $T = \prod_i R_z(q_i)\,T_z(\ell_i)$ for roll joints, $R_y$ for bend joints | `master_calibration.py:48–74` (`_Rz`, `_Ry`, `_Tz`, `fk_matrix`, `fk`) | IMPLEMENTED |

**Scale factor.** The master's full extension is 0.272 m; the Kinova Gen3's
reach is 0.902 m (quoted in `docs/ENGINEERING_LOG.md`). The ratio is therefore
$0.902/0.272 \approx \mathbf{3.32}$. **This is a derived quantity, not a
designed one** — no scale factor is declared anywhere in the repository, and
`WORKSPACE_SCALE = 1.0`. See gap **G4**.

### 1.2 Master-to-slave mapping

$$\mathbf{p}_{\text{cmd}} = \mathbf{c}_{\text{arm}} + s\,\big(A\mathbf{p} - A\mathbf{n}\big),\qquad \mathbf{n}=(0,0,L)$$

| Symbol | Meaning | Value | Provenance |
| --- | --- | --- | --- |
| $A$ | axis permutation/sign matrix from `AXIS_MAP` | `("+x","+y","+z")` = $I$ | `master_calibration.py:18`, `:79` |
| $s$ | workspace scale | `1.0` | `master_calibration.py:19` |
| $\mathbf{c}_{\text{left}}$ | left anchor (world) | `(0.6966, 0.2481, 1.1463)` m | `master_calibration.py:43` |
| $\mathbf{c}_{\text{right}}$ | right anchor | `(-0.7627, 0.2979, 1.1788)` m | `master_calibration.py:43` |
| orientation anchors | per-arm quaternion | `master_calibration.py:44–45` | |

Implementation `map_to_workspace`, `master_calibration.py:115–118`.
`validate_axis_map` asserts $\det A = +1$ at import (`:97`).

**Note for the Discussion:** `AXIS_MAP` was `("+x","-y","-z")` and was
corrected after regression against recorded sweeps showed
$\mathrm{cmd}_z = -1.011\,\mathrm{phys}_z$ — raising the master drove the
robot down. Recorded in `docs/ENGINEERING_LOG.md`; the regression coefficients are quoted
there but **the regression script's output was not retained** (gap G6).

### 1.3 Spherical pose decomposition

$$\mathbf{p} = r\big(\cos e\cos a,\ \cos e\sin a,\ \sin e\big)$$

| Component | Source | Provenance |
| --- | --- | --- |
| elevation $e$ | $e = \arcsin(\hat{\mathbf{a}}\cdot\hat{\mathbf{u}})$, $\hat{\mathbf{u}} = \mathrm{normalise}(\mathbf{a}_{\text{meas}})$ | `master_calibration.py:125–145` |
| azimuth $a$ | shoulder roll pot $q_1$, zero-referenced | `master_pose_node.py`, `azimuth_mode` param `:283` |
| reach $r$ | $\lVert \mathrm{fk}(q_1,q_2,q_3,q_4,0,0,0)\rVert$ | `master_calibration.py:74` |
| composition | `spherical_position()` | `master_calibration.py:165–169` |

**Sign convention, and a documented bug class.** $\hat{\mathbf{a}}$ is the arm
axis in the sensor frame, captured with the arm hanging. Hanging reads
$-90°$, horizontal $0°$, up $+90°$. The specification called for *two*
negations which compound and mirror the $z$ axis; only one is correct. The
reasoning is preserved verbatim in `ELEVATION_SIGN_NOTE`,
`master_calibration.py:147–158`, and asserted by `check_rest_elevation`
(`:160`). **This is good thesis material** — a concrete convention bug with a
self-check.

### 1.4 Coordinate-convention conversion — a genuine contribution

Kortex reports degrees over $[0,360)$; ROS uses radians over $(-\pi,\pi]$.

$$\mathrm{wrap}_{180}(d) = 180 - \big((180-d)\bmod 360\big),\qquad
\mathrm{wrap}_{\pi}(\theta) = \pi - \big((\pi-\theta)\bmod 2\pi\big)$$

| Function | Form | Provenance |
| --- | --- | --- |
| `kortex_deg_to_ros_rad` | wrap to $(-180,180]$ **then** to radians | `kortex_convention.py:35–37` |
| `ros_rad_to_kortex_deg` | to degrees **then** fold into $[0,360)$ | `:40–42` |
| `shortest_delta_rad` | $\mathrm{wrap}_\pi(\theta_t - \theta_c)$ | `:53–58` |
| `pose_delta_rad` | shortest path on continuous joints $\{1,3,5,7\}$ only; limited joints use plain subtraction | `:65–76` |

The ordering is the substance: converting before wrapping gives the wrong
answer at the seam. The module docstring states the failure concretely — 350°
read as $+350$ instead of $-10$ commands nearly a full revolution beside a
person. Unit-tested at $0/180/360$ in
`src/srl_teleop/test/test_kortex_convention.py`. **Round-trip error is not
reported as a number** — gap G5.

### 1.5 Calibration

| Item | Form | Provenance |
| --- | --- | --- |
| Per-channel zero and sign | `PotCalibration` with `zeros`, `signs`, `a_hat`, `gyro_bias`; persisted to `config/master_zero_<arm>.txt` | `master_calibration.py:201–300` |
| Relative zero-referencing | `apply(raw)` subtracts stored zero and applies sign | `:305` |
| $\hat{\mathbf{a}}$ capture | $\hat{\mathbf{a}} = -\mathrm{normalise}(\mathbf{a}_{\text{rest}})$, arm hanging | `:262` |
| Gyro bias capture | mean of rest samples; stale if residual $>1$ °/s | `:249`, `GYRO_REST_WARN_DPS` |
| Channel health | `health(raw)` | `:312` |
| Dropout detection | `zero_dropouts(raw)` — firmware clamps sub-ADC-minimum to exactly 0.0, so a dropout is indistinguishable from bottom-of-travel | `:359` |
| Accel gate | $\lVert\mathbf{a}\rVert$ within `ACCEL_GATE_G = 0.15` g of 1 g | `:364`, param `master_pose_node.py:274` |
| Kabsch alignment | used for the axis-map regression | `:383–407` |

### 1.6 Gyro-aided azimuth (implemented, not enabled)

$$\dot{\psi} = (\boldsymbol\omega - \mathbf{b})\cdot\hat{\mathbf{u}}$$

`yaw_rate_from_gyro`, `master_calibration.py:177–197`. Requires no mount
rotation because both vectors are already in the sensor frame. Default
`azimuth_mode = "j1"` (`master_pose_node.py:283`); blend time constant
`azimuth_blend_tau_s = 20.0` (`:288`). Status **IMPLEMENTED, NOT ENABLED**.

### 1.7 Filtering, deadbands and rate limits

| Quantity | Value | Provenance |
| --- | --- | --- |
| EMA on tip position | $\alpha = 0.3$, position only, never orientation | `master_pose_node.py:262` |
| Channel incoherence rejection | `max_channel_rate_deg_s = 800.0` | `:276` |
| Joint velocity sanity | `max_joint_velocity_deg_per_s = 1500.0` | `:249` |
| Joint step cap | `max_joint_step_cap_deg = 180.0` | `:253` |
| Resync after rejects | `force_resync_after = 25` | `:258` |
| Clutch debounce | `0.05` s | `:296` |
| Clutch quasi-static gate | `0.002` m over `0.1` s | `:300–301` |
| Clutch reference averaging | 5 frames | `:303` |
| Button map | left → `btn2`, right → `btn1` | `:311–312` |

### 1.8 Inverse kinematics and safety limits

| Quantity | Value (sim / `real_robot`) | Provenance |
| --- | --- | --- |
| Continuous joint indices | `(0,2,4,6)` = joints 1/3/5/7 | `ik_follower_node.py:62` |
| Max step per cycle | `max_step_rad = 0.35` | `:145` |
| Slew rather than discard | `clamp_towards()`, converges in $\lceil d/\Delta\rceil$ cycles | `:101–113` |
| Pose deadband | `0.002` m, `0.02` rad | `:149–150` |
| Flip reject limit | `10` consecutive, then force-accept | `:155` |
| Posture bias weight | $w = 0.02$, applied to the **seed** only | `:162` |
| Clearance floor | `min_clearance_m = 0.05` (sim) | `:167` |
| Redundancy retries | `redundancy_samples = 6`, span `2.0` rad | `:172–173` |
| Max joint velocity | `max_vel_rad_s = 0.6` | `:177` |
| Min trajectory time | `min_time_s = 0.05` | `:178` |
| Unwind limits | `0.3` rad/s, `0.25` rad step | `:181–182` |

Seed blending: $\mathbf{q}_{\text{seed}} = \mathbf{q}_{\text{cur}} + w\,\mathrm{angdiff}(\mathbf{q}_{\text{home}},\mathbf{q}_{\text{cur}})$.

### 1.9 Clearance model

Point-to-primitive distance against boxes, cylinders and spheres taken from
`human_backpack.xacro`: `_dist_point_box`, `_dist_point_cyl`,
`_dist_point_sphere`, `point_clearance` — `clearance.py:43–82`. Distal links
checked are `forearm_link`, `spherical_wrist_1/2_link`, `bracelet_link`,
`end_effector_link` (`clearance.py:38–40`). Wearer primitives at `:25–34`.

### 1.10 NOT IMPLEMENTED — the compliance mathematics

**There is no impedance controller, no virtual-spring model, no stiffness or
damping gain, and no force-reflection path anywhere in the repository.**
Searched for and absent. This is the largest single divergence from the
planning report and is treated in DIVERGENCE.md. Status **NOT-DONE**.

---

## 2. DATA FLOWS (§1.3)

### 2.1 Serial protocol — master to host

| Property | Value | Provenance |
| --- | --- | --- |
| Transport | USB CDC serial | `serial_port.py` |
| Baud | **115200** | `serial_port.py:21` |
| Framing | newline-terminated ASCII, one frame per line | `virtual_teensy.py:84–90` |
| Format | `k1j1:<deg> … k1j7:<deg> K1IMU:ax,ay,az,gx,gy,gz k2j1:… K2IMU:… fsr1:<n>,fsr2:<n>,btn1:<0\|1>,btn2:<0\|1>` | `virtual_teensy.py:11–13`, parser `master_pose_node.py:196–212` |
| Frame rate | ~50 Hz | measured, `docs/ENGINEERING_LOG.md` |
| **Sensor update rate** | **14.7–17 Hz** — so 82–88 % of adjacent frames are bit-identical | measured, `docs/ENGINEERING_LOG.md` |
| Port discovery | glob `/dev/ttyACM*`, `/dev/ttyUSB*`, sniff for `k1j1` | `serial_port.py:14–15, 44–49` |
| Exclusive access | `flock(LOCK_EX\|LOCK_NB)` on the fd | `serial_port.py:76, 112` |

### 2.2 ROS 2 graph

**57 distinct topics** reconstructed from source (not from a live graph). Full
table to be reproduced in the appendix. Principal chain:

`/master_arm_pose_<arm>` (PoseStamped) → `ik_follower_node` →
`/<arm>_arm_controller/joint_trajectory` (JointTrajectory) → `ros2_control` →
`/joint_states` → `/tf`.

Safety and observability topics: `/estop`, `/estop_state`, `/estop_deadman`,
`/blocking`, `/blocking_summary`, `/motion_unexplained`, `/mount_guard`,
`/recovery_state`, `/arm_link_status`, `/master_channel_state`,
`/master_capability_<arm>`.

Real-arm namespace: `/real/joint_states`, `/real/session_state`,
`/homing_active_<arm>`, `/cascade_active_<arm>`, `/sim_to_real_enabled_<arm>`.

### 2.3 End-to-end path

Teensy frame → `serial_port.find_port` → `master_pose_node.parse_arm` →
validation (mode-scoped) → spherical decomposition → EMA ($\alpha=0.3$) →
clutch → `map_to_workspace` → `/master_arm_pose_<arm>` →
`ik_follower_node` (TRAC-IK via `/compute_ik`, redundancy retry, `clamp_towards`,
clearance floor) → `JointTrajectory` → controller.

**Per-stage latency is not instrumented.** Gap **G3**.

---

## 3. MASTER ARM DESIGN (§1.4) — THE LARGEST GAP

The prompt requires this to be "the most detailed part of the thesis". The
repository does not support that.

| Required item | Present? | Evidence |
| --- | --- | --- |
| CAD models (any format) | **NO — zero files** | searched `.step .stp .stl .f3d .sldprt .sldasm .dxf .iges .3mf .scad`: 0 hits |
| Teensy/Arduino firmware | **NO — zero files** | searched `.ino .pde`: 0 hits |
| Kinematic chain of the miniature arm | **PARTIAL** | link lengths and joint types only, `master_calibration.py:5–6` |
| Scale factor vs Gen3 | **DERIVED, not declared** | $0.272$ m vs $0.902$ m ⇒ $\approx 3.32$ |
| Spring placement A/B/C study | **NO** | exists only in the planning report; nothing in the repo |
| Configuration C mapping (J5–J6, J3–J2, J4–J1) | **NO** | planning report only |
| Pot pin map | **NO** | no firmware, so no pin assignment |
| ADC resolution | **NO** | firmware absent |
| IMU part number / mounting | **PARTIAL** | code implies an MPU-6050/9250-class device at I²C `0x68`/`0x69` (`docs/ENGINEERING_LOG.md`); not in repo source |
| Known channel defects | **YES, quantified** | `recordings/baselines/channels_20260806.json` |
| Electronics, power, current limiting | **NO** | planning report only |
| Fabrication parameters, materials, iterations | **NO** | nothing in repo |

**Status: PARTIAL at best; mostly NOT-DONE as an artefact.** The mannequin
demonstrably exists — recorded potentiometer and IMU traces could not have
been produced otherwise — but its *design record* is not in this repository.

---

## 4. RESULTS INVENTORY (§1.5)

| Claim | Evidence type | File | Status | Confidence |
| --- | --- | --- | --- | --- |
| Continuous reachability, 26 directions × 10 repeats; IQR = 0.000 m | JSON | `recordings/baselines/workspace_n10_20260806.json` | SIM-TESTED | High |
| Isotropy on medians 0.16 (L) / 0.10 (R) | JSON | same | SIM-TESTED | High |
| Two arms share 0 of 63 frontal cells | script output | `scripts/verify_task_scenes.py` | SIM-TESTED | High |
| Mount sweep 3 → 22 shared cells | JSON | `recordings/baselines/mount_overlap_sweep.json` | SIM-TESTED | Medium (kinematic upper bound) |
| 14 channels classified; 6 usable | JSON | `recordings/baselines/channels_20260806.json` | SIM-TESTED (analysis of real recorded data) | High |
| Degraded-mode vibration 54.5 mm (L) / 0.014 mm (R) | script, replays real traces | `scripts/prove_degraded_mode.py` | SIM-TESTED | High |
| No reach observable survives on left ($R^2=0.133$) | analysis | `docs/ENGINEERING_LOG.md`; regression not retained | PARTIAL | Medium |
| 14/14 fault injections handled | logs | `recordings/fault_acceptance/` | SIM-TESTED | High |
| 6/6 experiments invalidate mid-trial and continue | logs | `recordings/fault_acceptance/e{1..6}/` | SIM-TESTED | High |
| Dead-man trips 0.94–0.99 s (stale) / 0.97 s (silent) | **log text only** | `docs/ENGINEERING_LOG.md` | PARTIAL | Medium |
| All 17 scenario + 17 protocol coordinates SOLID at N=10 | YAML | `recordings/baselines/scenario_audit.yaml` | SIM-TESTED | High |
| 87 verification runs, 4 camera angles each | mp4 + JSON | `recordings/verification/` | SIM-TESTED | High |
| 0 of 87 clips pass an arm through the wearer | JSON | per-run `summary.json` | SIM-TESTED | High |
| 45/45 grasp clips: gripper opens, closes on object, reopens | JSON | `recordings/verification/gripper_check.json` | SIM-TESTED | High |
| 45 clips: object carried with the arm; 0 static | JSON | `recordings/verification/attachment_check.json` | SIM-TESTED | High |
| 48 of 87 runs below the 120 mm clearance floor | JSON | per-run `summary.json` | SIM-TESTED | High |
| Lateral half-span 0.155→0.054 m, 0.250→0.134 m clearance | **stdout only** | `scripts/probe_task_band_clearance.py` | PARTIAL | Medium |
| 28 unit test files; style tests fail pre-existing | source | `src/*/test/` | IMPLEMENTED | High |

---

## 5. TASK SET (§1.6)

| Task | Implemented | Sim-tested | Hardware | Evidence |
| --- | --- | --- | --- | --- |
| T2 hold-and-fill (pick and place into a carried container) | yes | **yes**, 12 runs | no | `recordings/verification/t2/`, `blocks_placed` in `rviz_capture.json` |
| T3 rigid coupled carry | yes | yes, 12 | no | `recordings/verification/t3/` |
| T5 handover to the wearer | yes | yes, 9 | no | `recordings/verification/t5/` |
| T6 compliant (sling) carry | yes | yes, 12 | no | `recordings/verification/t6/` |
| T7 bimanual pursuit | yes | yes, 18 | no | `recordings/verification/t7/` |
| T8 wearer-assisted reach | yes | yes, 12 | no | `recordings/verification/t8/` |
| T9 reach under wearer motion | yes | yes, 12 | no | `recordings/verification/t9/` |
| T4 inter-arm handover | **blocked by geometry** | n/a | no | 0 of 16 transfer points reachable by both arms |
| Language-driven grasping (mode 6) | yes | partially | no | detection rate **unmeasured** (G7) |

**"Completed successfully rather than merely ran"** is evidenced for the pick
tasks by the automated checks: gripper transitions from `grip_trace.json`, and
object handling from pixel analysis (orange supply pixels fall, green
in-container pixels rise).

---

## 6. GAP LIST

Ordered by how much each blocks a submittable thesis.

**G1 — NO MASTER-ARM DESIGN ARTEFACTS.** No CAD, no firmware, no pin map, no
ADC specification, no fabrication record, no spring study data. The planning
report names this the core contribution and the prompt requires it to be the
longest section of Chapter 3. *Needed:* export the CAD and commit it; commit
the Teensy firmware; photograph the assembly; record the pin map and ADC
resolution.

**G2 — NO HARDWARE VALIDATION AND NO USER STUDY.** Zero artefacts from a
physical arm or a human participant. Targets E1–E9 are therefore all
unmeasured except where simulation can stand in.

**G3 — NO LATENCY MEASUREMENT.** End-to-end latency (target E8, < 200 ms) is
not instrumented at any stage. *Needed:* timestamp at the Teensy frame, at
`/master_arm_pose`, at trajectory publication and at `/joint_states`, and log
the differences.

**G4 — SCALE FACTOR NOT DECLARED.** The 3.32 ratio is derived here from link
lengths, not designed or documented. *Needed:* confirm the intended scale and
record it.

**G5 — CONVENTION ROUND-TRIP ERROR NOT QUANTIFIED.** The conversion is
unit-tested for correctness but no numerical round-trip residual is reported.
*Needed:* trivially cheap — sweep $[0,360)$ and report max $\lvert$error$\rvert$.

**G6 — SEVERAL KEY FIGURES EXIST ONLY IN THE NOTEBOOK.** The axis-map
regression coefficients, the dead-man trip traces, the degraded-mode
regression, and the task-band clearance probe all print to stdout or appear
only in `docs/ENGINEERING_LOG.md`. *Needed:* re-run with output to file.

**G7 — VISION DETECTION RATE UNMEASURED.** The synthetic renderer was found to
be out of distribution; the real number does not exist.

**G8 — NO FIGURES.** Not one publication-quality figure. Needed: system
architecture (TikZ), node graph (TikZ), master kinematic chain, workspace
envelope (pgfplots from `workspace_n10_*.json`), channel health, clearance.

**G9 — NO IMPEDANCE / FORCE-REFLECTION IMPLEMENTATION.** No stiffness,
damping, or force feedback path exists, so targets E3 and E7 cannot be
approached at all.

**G10 — NO ISAAC LAB SIMULATION.** The planning report's simulation
environment is absent; the delivered simulation is Gazebo-less ROS 2
`mock_components` with MoveIt collision checking.
