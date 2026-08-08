# EVIDENCE INVENTORY

*Phase 1 output. What the repository can actually prove, where each number
lives, and what a thesis will need that is not here.*

**Rule applied throughout:** a claim is listed as EVIDENCED only if a file in
this repository contains the number or the artefact. Everything else is listed
as a GAP with the experiment that would close it. Nothing below is inferred
from the engineering log alone unless the log is explicitly the source, in
which case it is marked as such — `CLAUDE.md` is a lab notebook, not an
independent measurement.

---

## 0. The three classes of claim, and why the distinction matters

Every result in this repository falls into one of three tiers, and the thesis
must label them individually rather than blurring them:

| tier | meaning | how much of the work |
| --- | --- | --- |
| **A — verified in simulation** | measured against a running `move_group` / `ros2_control` stack with mock hardware | the large majority |
| **B — implemented, exercised against mocks only** | code runs, its failure paths are injected, but no physical device was involved | the real-arm bridge, recovery, VR |
| **C — validated on hardware** | a real Kinova arm, a real Teensy, or a human participant took part | **almost nothing** |

**No experiment in this repository has ever been run with a human
participant, and no bimanual task has ever been driven through the master arm
by a person.** That is the single most important sentence for the thesis to
carry, and it constrains what Results may claim.

---

## 1. Repository inventory

| | |
| --- | --- |
| tracked files | 1097 |
| Python | 207 files |
| ROS 2 packages | 12 (`srl_teleop`, `srl_description`, `srl_moveit_config`, `srl_perception`, `srl_autonomy`, `srl_experiments`, `srl_vr_teleop`, `srl_vr_autonomy`, plus 4 vendor) |
| unit tests | 28 files |
| CSV logs | 219 |
| recorded clips | 174 mp4 |
| baseline JSON/YAML | 6 in `recordings/baselines/` |

### 1.1 THE COMMIT HISTORY IS NOT A DEVELOPMENT RECORD — flag this early

`git log` holds **13 commits**. One is `df8cc54 Initial commit` (2026-07-14)
containing essentially the whole system; the other twelve are all dated
2026-08-08. The history therefore **cannot** be used to reconstruct when
anything was built, in what order, or by whom.

The brief for this thesis assumed the commit history would supply the
development narrative. It does not, and no amount of reading will make it.
The usable chronology is instead the dated sections of `CLAUDE.md`
(2026-07-31 through 2026-08-08), which is a lab notebook — good for *what was
measured and why*, and unsuitable as a citation for *when code was written*.

**Consequence for the thesis:** do not write a chronological development
chapter. Structure Method by subsystem instead.

---

## 2. EVIDENCED — claims with a file behind them

### 2.1 Kinematics, workspace and reachability

| claim | evidence |
| --- | --- |
| Continuous reachability from home, 26 directions, 10 repeats | `recordings/baselines/workspace_n10_20260806.json`; script `scripts/measure_workspace.py` |
| Isotropy on medians: left 0.16, right 0.10; worst direction 0.140 / 0.090 m | same file; `CLAUDE.md` §REACHABILITY N=10 |
| IQR = 0.000 m across 10 sweeps — the measurement repeats, the *statistic* did not | same |
| Per-call IK false-negative rate 0.0–0.4% on known-solvable targets | `scripts/measure_ik_stickiness.py`, `recordings/baselines/` (`ik_sticky.json` in scratch — **see GAP G7**) |
| All 17 scenario coordinates and all 17 protocol-table coordinates SOLID at N=10 over densified paths | `recordings/baselines/scenario_audit.yaml` (`repeats: 10`, `path_step_m: 0.02`) |
| The two arms' reachable sets are disjoint: 0 of 63 frontal cells reachable by both | `scripts/verify_task_scenes.py`, `scripts/sweep_mount_overlap.py`; `recordings/baselines/mount_overlap_sweep.json` |
| Mount sweep: 3 → 22 of 63 shared cells for a buildable 0.20 m separation | `recordings/baselines/mount_overlap_sweep.json` |

### 2.2 Master-arm sensing and channel health

| claim | evidence |
| --- | --- |
| 14 pot channels classified DEAD / INCOHERENT / INTERMITTENT / ALIVE | `recordings/baselines/channels_20260806.json` (keys `l_j1`..`r_j7`) |
| 7 of 14 channels incoherent; only 6 usable | same file; `scripts/check_channels.sh` re-runs it |
| The recorder oversampled a 14.7–17 Hz sensor at 50.3 Hz, so 82–88% of adjacent rows are bit-identical | `recordings/teleop_20260731_*.csv`; analysis in `CLAUDE.md` |
| Degraded mode contributes 54.5 mm of commanded-tip vibration on the left arm, 0.014 mm on the right | `scripts/prove_degraded_mode.py` — **replays recorded traces**, so it is reproducible |
| Idle virtual-Teensy test: commanded EE std 0.0000 mm over 60 s | `scripts/virtual_teensy.py` + `CLAUDE.md` |

### 2.3 Safety layer

| claim | evidence |
| --- | --- |
| 14/14 fault injections handled | `scripts/inject_experiment_faults.py`, `recordings/fault_acceptance/` (e1..e6 plus `*_all_invalid`) |
| 6/6 experiments invalidate correctly mid-trial and continue | `recordings/fault_acceptance/e{1..6}/` |
| Dead-man trips on a frozen-but-publishing master in 0.94 s and a silent one in 0.97 s | `src/srl_teleop/srl_teleop/estop_node.py`; measurement in `CLAUDE.md` — **GAP G3: no logged trace file** |
| BlockMonitor three-state semantics (active / expired / clear) | `src/srl_teleop/test/test_block_expiry_is_not_permission.py` (6 tests) |
| Both-button e-stop requires a sustained hold | `src/srl_teleop/test/test_both_button_estop_needs_hold.py` |

### 2.4 Verification recordings — the strongest evidence in the repo

| claim | evidence |
| --- | --- |
| 87 task × scenario × condition runs recorded, four camera angles each | `recordings/verification/<task>/<scenario>/<condition>/rviz_{front,side,top,gripper}.mp4` + `rviz_quad.mp4` |
| Zero clips show an arm link inside the wearer | `recordings/verification/index.json`, per-run `summary.json` |
| 45 clips show the object grasped and carried WITH the arm; 0 show it stationary | `recordings/verification/attachment_check.json`; `scripts/verify_object_attachment.py` |
| Gripper opens, closes on the object, and reopens | per-run `grip_trace.json` (knuckle angle measured from `/joint_states`); `recordings/verification/gripper_check.json` |
| 48 of 87 runs sit below the 120 mm `real_robot` clearance floor | per-run `summary.json` (`below_real_floor`) |
| The clearance limit is lateral, not fore/aft: half-span 0.155 → 0.054 m, 0.250 → 0.134 m, all 6/6 reachable | `scripts/probe_task_band_clearance.py` — **GAP G6: its output is not saved to a file** |

### 2.5 Unit-test evidence

28 test files. Those that encode a *measured* property rather than a coding
convention:

- `test_ik_guard.py` — slew-not-discard convergence, 6 tests
- `test_coupled_metrics_known_answers.py` (21) and `test_t7_targets_known_answers.py` (7) — every T3/T6/T7 metric recovers a known answer
- `test_t2_outcome_known_answers.py` (15) — the T2 discrete outcome
- `test_t9_interaction_known_answer.py` (6) — recovers an injected interaction, and pins that a yoked design is rank-deficient
- `test_right_arm_spherical_survives_dead_rolls.py` (6) — dead roll joints cost ≤20.4 mm of tip motion
- `test_kortex_convention.py`, `test_imu_orientation.py`, `test_degraded_mode.py`, `test_serial_exclusive.py`

`test_flake8` and `test_pep257` **fail, and failed before any of this work** —
the package uses double quotes throughout. Pre-existing, and the thesis should
not present the suite as fully green without that caveat.

---

## 3. GAPS — what a thesis needs that is NOT in this repository

Ordered by how much they constrain what can be written.

**G1 — NO HUMAN PARTICIPANTS, AT ALL.**
There is no participant data, no ethics approval record, no consent form, no
NASA-TLX response, no trust or perceived-safety rating. The entire
experimental design (`docs/research/02`, `05`, `06`, `07`) is *pre-registered
intent*. A Results chapter cannot contain a single human-subject number.
→ `\todo{MEASURE: ...}` on every experimental hypothesis.

**G2 — NOTHING HAS RUN ON THE REAL ARMS IN THIS REPOSITORY'S RECORD.**
`CLAUDE.md` describes real-arm bring-up (21.4 Hz dual-arm, homing residuals,
Kortex session handling), but there is **no logged artefact** — no bag, no
CSV, no plot — from a real arm anywhere in `recordings/`. Every recorded
number comes from `mock_components/GenericSystem`.
→ Method may describe the real-arm path; Results must not claim it was
measured, only that the log reports it.

**G3 — Timing and latency claims have no saved traces.**
Dead-man trip times, e-stop halt latency, the 18.4–18.7 Hz high-level bridge
rate, and the ~26 ms send latency are quoted in `CLAUDE.md` but no
corresponding log file exists in the repository.
→ `\todo{MEASURE: re-run with output captured to recordings/}`.

**G4 — No figures exist.** There is not one publication-quality plot. There
are `plot_metrics.png` files per clip (tracking + clearance against time) and
some `teleop_*_gain.png` from July, but nothing at thesis figure quality: no
system architecture diagram, no node graph, no workspace plot, no
kinematic-chain figure.
→ `\todo{WRITE: figure}` throughout; several can be generated from existing
data, which should be stated as the plan.

**G5 — The perception pipeline's detection rate is UNMEASURED.**
`scripts/measure_detection.py` returned 0–4% on synthetic renders, and that
figure was traced to the renderer being out of distribution, not the model.
The real number does not exist. Mode 6 is therefore not participant-ready and
the thesis cannot quote a detection rate.

**G6 — Several key probes print to stdout and save nothing.**
`probe_task_band_clearance.py`, `probe_collision_response.py`,
`probe_clutch_indexing.py`, `diagnose_front_reach.py` produce numbers quoted
in `CLAUDE.md` with no artefact.
→ Cheap fix: add JSON output and re-run. Worth doing before writing Results.

**G7 — Some analysis outputs live only in the scratch directory** (e.g.
`ik_sticky.json`, `percep.json`) and are outside the repository, so they will
vanish. Re-run and commit before citing.

**G8 — No related-work bibliography.** `bibs/sample.bib` contains one entry
(a CTAN article) which is irrelevant to the subject. Every citation in
Background must be gathered by hand. `docs/research/01_literature_review.md`
§7 contains ~12 verified sources with URLs (Fusion, SRL Proxemics, Zhang
2024, Jian 2000, Godspeed, NASA-TLX) — those are checkable and can seed the
bib. The other ~30 citations referenced in §§1–6 of that document were located
by web search in an earlier session and are **not** independently verified
here.

**G9 — No power analysis, no sample size justification with real variance.**
`02_baseline_and_hypotheses.md` sketches n=12 dyads but the variance estimates
are assumed, not measured.

---

## 4. What the thesis CAN claim strongly

1. **A complete, safety-instrumented dual-arm teleoperation stack exists and
   is reproducible**, with 28 unit tests and a fault-injection suite that
   passes 14/14.
2. **A rigorous verification methodology**, including the standing rule that
   an instrument must be validated before a bad measurement is reported —
   with at least a dozen documented instances where the instrument, not the
   system, was at fault. This is a genuine methodological contribution and it
   is unusually well evidenced.
3. **Reachability and workspace characterisation** of a backpack-mounted
   dual-Gen3 configuration, with a negative result of real value: the two arms
   share no workspace, which blocks inter-arm handover by geometry.
4. **A measured account of degraded-input teleoperation** — what survives when
   7 of 14 sensing channels fail, and what it costs.
5. **87 recorded verification runs** with automated content, attachment and
   gripper checks.

## 5. What it must NOT claim

- Any human-factors result.
- Any real-hardware performance number as a measurement of this work.
- A detection rate for the vision pipeline.
- That the experimental protocols were executed.
