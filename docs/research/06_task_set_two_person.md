# THE TASK SET UNDER THE TWO-PERSON FRAMING

*2026-08-08. Every verdict below is backed by a number from a live
`/compute_ik` with both arms VERIFIED at home, N=10 repeats over the whole
densified path. `FEASIBLE-IF` means it works only with a stated change.*

---

## Summary table

| task | single-person verdict | **two-person verdict** | evidence |
| --- | --- | --- | --- |
| T1 bimanual reach | subsumed by T7 | **DROPPED**, unchanged | T7 measures the same construct continuously |
| T2 hold and fill | IN | **IN, and now stronger** | 4/4 scenarios verified; discrete outcome instrumented |
| T3 rigid carry | IN, respec'd | **IN** | 4/4 verified; **but see the clearance finding** |
| T4 inter-arm handover | BLOCKED | **STILL BLOCKED** | 0 of 16 transfer points, 0 of 63 grid cells reachable by both arms |
| T5 handover to wearer | IN, contested slot | **PROMOTED TO CORE** | 3/3 verified; the canonical Fusion scenario |
| T6 compliant carry | IN | **IN** | 4/4 verified |
| T7 pursuit | IN | **IN, re-verified** | 6/6; centres moved z 1.05 → 1.10 |
| **T8 wearer-assisted reach** | did not exist | **NEW — FEASIBLE** | 4/4 verified, 2 per arm, 3 distinct stances |
| **T9 reach under wearer motion** | did not exist | **NEW — FEASIBLE-IF** | 4/4 verified; **IV capped at 100 mm, not 160 mm** |

---

## What the reframing changes, task by task

### T5 — PROMOTED TO CORE, and the argument has flipped

Under the single-person framing I argued **against** T5's displacement by T2,
on the grounds that T5 is the only task where the robot serves the wearer.
Under the two-person framing that argument becomes much stronger and is no
longer a preference:

- **T5 is the canonical Fusion scenario.** Fusion's stated motivation is
  remote assistance and instruction of the surrogate. T5 is that scenario with
  a measurement attached, which is precisely what Fusion did not have.
- **It is the ONLY task where the wearer has a functional role.** In T2, T3,
  T6 and T7 the wearer is a moving mount. In T5 they receive the object, and
  the "collaborator or platform" item has a referent instead of being
  hypothetical.
- **It is the only single-arm task**, so it survives if the bimanual geometry
  degrades further.

**Verified:** 3/3 scenarios, right arm, whole delivery path densified to
20 mm — approach, grasp, lift, transit, present, retreat. Cradle at
(−0.30, 0.35, 1.05), receive at (−0.16, 0.35, 1.05).

**Still blocked on wiring:** the outcome is the **wearer's button press**, a
foot pedal that does not exist yet. That is correct — only the wearer knows
whether they have the tool — but it means T5's headline metric is not
instrumented. See §"remaining blockers".

### T2, T3, T6, T7 — unchanged as tasks, changed as measurements

The tasks are identical. What changes is that **wearer body motion is now a
measured covariate in every one of them**, from the wearer's torso IMU, and
the wearer's questionnaire battery is taken after every block. A trial that
was previously "the operator tracked badly" can now be attributed.

**T3/T6 carry a finding that recording surfaced and IK verification did not.**
See below.

### T4 — still blocked, and the two-person framing does not rescue it

0 of 16 transfer points and 0 of 63 frontal grid cells are reachable by both
arms; with collisions off only 3 of 63, and the wearer removes all three. This
is geometry, not orientation and not autonomy. **A wearer reposition does not
fix it either** — moving the base moves BOTH arms together, so their mutual
separation is invariant. T4 needs the right arm re-parked in hardware.

---

## T8 — WEARER-ASSISTED REACH (new)

**A target the operator cannot reach at the wearer's nominal stance, and can
reach once the wearer repositions. Neither person can complete it alone:
the operator cannot reach, and the wearer cannot manipulate.**

### Verified scenarios — 4/4

| scenario | arm | target (x, y, z) | wearer must | approach path |
| --- | --- | --- | --- | --- |
| S1_left_lean_forward | left | (+0.55, 0.35, 0.95) | lean forward ~20° | 16/16 |
| S2_left_step_forward | left | (+0.55, 0.45, 0.95) | step forward 0.30 m | 19/19 |
| S3_right_crouch | right | (−0.55, 0.45, 0.95) | crouch 0.20 m | 14/14 |
| S4_right_step_forward | right | (−0.55, 0.55, 0.95) | step forward 0.30 m | 17/17 |

Two per arm, three distinct stances. **The first version of the search
returned four left-arm lean-forward variants** — one situation measured four
times and called a graded task set. Balance is now enforced.

Every target is confirmed **unreachable at nominal** (the defining property)
and every approach path is verified densified to 20 mm after the reposition.

### Metrics

| metric | definition |
| --- | --- |
| `coordination_latency_s` | `t_wearer_completes_reposition − t_operator_requests` |
| `anticipation_s` | `t_wearer_starts_moving − t_operator_requests`. **Negative = the wearer moved before being asked** |
| `initiator` | who acted first: operator request, or wearer volunteering |
| `comm_events` | utterances, coded request / acknowledgement / warning / social |
| `attempts_before_request` | how long the operator tried to reach it alone before asking — an operator-side awareness measure |
| `repositions_per_trial` | over-correction: a wearer who keeps shuffling has not understood the requirement |

### The learning hypothesis, and how it is tested

**H3.** Across repetitions, `coordination_latency_s` falls and `anticipation_s`
turns negative. A negative anticipation means the wearer predicted the
operator's intent **without controlling the limbs and without proprioception
of them** — body-schema-like learning in a person who is not driving.

8 repetitions per condition, so the trend is estimable within a block rather
than only across them.

### Limitation, stated

The wearer's stance is simulated in verification by **transforming targets**,
which is exactly equivalent kinematically but does **not** move the wearer's
own collision geometry. On the real rig a wearer who leans *into* the workspace
brings their torso with them. Every T8 pose is therefore also checked at the
nominal stance with collisions on, and the real-rig clearance must be
re-measured with the wearer actually leaning.

---

## T9 — REACH UNDER WEARER MOTION (new)

**The wearer sways or steps to a metronome while the operator works. Sway
amplitude is the independent variable.**

This is the hypothesis the architecture makes uniquely available, but **not
because it is untested** — see the correction in §7.5 of the literature
review. Zhang et al. (2024) report 1.37 ± 0.58 mm tracking with the human
shoulder as a floating base, so the *control* problem is known to be solvable.
What is untested is the **two-person** case, where the disturbance originates
in a different nervous system: no efference copy, no vestibular access, and
visible to the operator only through a camera mounted on the moving base.

### Verified scenarios — 4/4, and the IV is CAPPED

| scenario | sway amplitude | envelope reachable | what it is |
| --- | --- | --- | --- |
| S1_sway_000mm | 0 mm | 1/1, 1/1 | static control |
| S2_sway_020mm | 20 mm | 8/8, 8/8 | quiet-standing postural sway |
| S3_sway_060mm | 60 mm | 8/8, 8/8 | gentle weight shift |
| S4_sway_100mm | 100 mm | 8/8, 8/8 | deliberate weight shift |

Targets at (±0.35, 0.35, **1.15**) — **not** the T7 centre. Measured: at
z = 1.10 the left arm tolerates only 60 mm of sway before the envelope leaves
its reachable set; z = 1.15 gives 100 mm on both arms.

**160 mm ("step in place") was tested and REMOVED.** The envelope leaves the
reachable set at every centre probed. Keeping it would lose trials to geometry
and score them as disturbance effects, which is exactly backwards. **So the IV
spans quiet sway to a deliberate weight shift, and NOT a step** — recorded as
a limitation rather than quietly dropped, and it does weaken the top of the
range.

Metronome: 30 / 50 / 70 bpm, chosen to straddle plausible visual-tracking
bandwidth so **H2-null** (the operator simply watches and compensates) is
falsifiable rather than assumed away.

### Metrics

`rms_error_mm` and `max_error_mm` against a world-fixed target; `phase_lag_s`
of the EE against the **wearer's** motion (not the target's — that is what
distinguishes compensation from following); `wearer_body_motion` RMS and
dominant frequency, from the torso IMU, as the manipulation check; clearance
minimum; and the operator's awareness-probe accuracy.

**The manipulation check matters.** If the wearer's measured sway does not
track the commanded amplitude, the IV did not happen and the trial says
nothing.

---

## THE CLEARANCE FINDING — RESOLVED, with a measured fix

Recording all 87 clips showed every chest-band scenario at **48-67 mm** of
clearance to the wearer's torso: IK-valid, contact-free, and **below the
120 mm floor `real_robot` enforces**. MoveIt's `avoid_collisions` is binary
contact; the floor is a margin, and a scenario can pass one and be refused by
the other.

`scripts/probe_task_band_clearance.py` then answered the three questions the
clips cannot.

**1. WHICH LINK?** `spherical_wrist_1_link` on 9 of 12 probes,
`spherical_wrist_2_link` on the other 3. **Not the forearm** — so this is not
a path routing problem. The wrist is two links from the gripper, which says
the TARGETS are too close to the torso, not that the arm takes a bad route to
them.

**2. MOVING THE BAND FORWARD DOES NOT WORK.** Clearance and reach are in
direct conflict in y:

| y | worst clearance | reachable | verdict |
| --- | --- | --- | --- |
| 0.35 (current) | 0.051 m | 6/6 | below floor |
| 0.40 | 0.084 m | 5/6 | below floor |
| 0.45 | **0.131 m** | **1/6** | clears, but unreachable |
| 0.50, 0.55 | — | 0/6 | unreachable |

The band that clears the floor cannot be reached; the band that can be reached
does not clear the floor. This is the same trade-off recorded for the mount in
CLAUDE.md, reappearing at the task level.

**3. MOVING THE BAND OUTBOARD DOES WORK, and this is the fix.** Sweeping the
half-span at y = 0.35:

| half-span x | worst clearance | reachable | verdict |
| --- | --- | --- | --- |
| **0.155 (current)** | 0.054 m | 6/6 | **below floor** |
| 0.200 | 0.090 m | 6/6 | below floor |
| **0.250** | **0.134 m** | **6/6** | **CLEARS — the minimum that works** |
| 0.300 | 0.176 m | 6/6 | clears |
| 0.350 | 0.219 m | 6/6 | clears |
| 0.400 | 0.265 m | 6/6 | clears |

**Every lateral position from 0.25 m outward clears the floor and stays fully
reachable.** The recordings independently corroborate it: T7 and T8 work at
|x| >= 0.40 and measured **0.158-0.187 m** across 30 clips, with zero below
the floor.

### What the fix costs, and why it is NOT applied here

A half-span of 0.25 m means a **500 mm tray**, not 310 mm. That is not a free
parameter — it propagates:

* **T6's sling fails.** `SLING_L = 350 mm` retains the ball only to
  `s_max = 340.7 mm`. At 500 mm separation the sling is taut and the ball is
  gone before the trial starts. A 500 mm working separation needs
  **L >= 507 mm** for the same 40 mm ball, so the sling becomes ~520 mm — and
  the whole point of L=350 was that the failure threshold sat INSIDE the
  reachable band. That property must be re-established, not assumed.
* **T3's tilt metric rescales.** `tilt = atan2(dz, span)`, so at 500 mm the
  same 60 mm height difference is 6.8 deg rather than 11.3 deg. The
  ball-rolls-off threshold is a property of the tray, and it moves.
* **T2's container changes.** The opening sits 300 mm from the handle now; at
  a 500 mm separation it would be 500 mm, which is no longer a saucepan.

So the fix is **recorded and not applied**. It changes verified geometry, the
object specifications and two thresholds, and it must be one deliberate pass
with full re-verification — the same discipline applied to the mount sweep.

### T5 is different and must not be "fixed"

T5 measured 0.068-0.095 m, also below the floor — but T5 is a **handover to
the wearer**. The arm is supposed to come to their waist; low clearance is the
task, not a defect. T5 needs a task-scoped exemption with the wearer's
knowledge and consent, not a relocated target. Conflating the two would either
break T5 or quietly lower the floor for everything.

---

## Remaining blockers on wiring

| blocked | on |
| --- | --- |
| T5's headline metric | the wearer's **foot pedal / receipt button**. Not built |
| T8 and T9 wearer motion | a **wearer torso IMU** publishing to ROS. The disturbance is currently commanded, not measured |
| Wearer arousal | an **EDA sensor**. Nothing in the repo reads one |
| Operator awareness probes | an experimenter script to freeze the arms and pose the probe; the freeze path exists (`/estop` + clutch) but the probe UI does not |
| Every DIRECT condition | 7 of 14 master channels **INCOHERENT**. `l_j2`/`l_j4` first |
| T4 | the right arm re-parked in hardware |
| T3/T6 on real hardware | the clearance finding above |
