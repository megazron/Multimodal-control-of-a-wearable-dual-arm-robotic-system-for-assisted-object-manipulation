# Baseline, Pre-Registered Hypotheses, and Threats to Validity

> **REFRAMED 2026-08-08 — TWO PEOPLE.** The arms are worn by one person and driven by
> a **different** person. Sections 1-5 below were written for a single person who both
> wore and drove them; their reasoning about baselines, fatigue and within-subject
> design still holds, but **the unit of analysis is now the DYAD**, and the wearer is a
> **participant with their own measures and their own consent**, not part of the
> apparatus. Section 6 states the two-person design and takes precedence wherever it
> disagrees with what follows.


**System under study:** two Kinova Gen3 7-DOF arms mounted on a wearable backpack,
commanded from an instrumented mannequin master arm (potentiometers per joint plus a wrist
IMU, read by a Teensy 4.1), ROS 2 Jazzy, TRAC-IK through MoveIt `/compute_ik`.

**Status of this document.** This is a pre-registration draft. Every hypothesis below is
stated with its predicted direction, dependent variable, statistical test, assumption
checks and correction *before* data collection. Effect sizes used in the power sketch are
**guesses** and are labelled as such; they must be replaced with pilot estimates before
the design is locked.

**Prerequisite that is not yet satisfied.** `CLAUDE.md` records that the robot's tracking
of a *moving* master has never been measured — all sub-millimetre tracking figures were
taken with the master at rest, and the one recording with a moving master (right arm,
2097 samples, superseded settings) showed an end-effector gain of ~0.1 against a commanded
gain of ~1.0. **No experiment below may use commanded pose as a proxy for end-effector
pose.** All position dependent variables must be measured from `tf2` on
`<arm>_end_effector_link`, and a moving-master gain matrix against real EE must be
recorded and reported as a system-characterisation appendix before E1 is run.

---

## 1. Baseline

### 1.1 The baseline is this system in direct-teleoperation mode, within-subjects

Every experiment compares an autonomy condition against **the same rig, same operator,
same task, same session**, run in direct teleoperation: master pose mapped to commanded
end-effector pose through the existing spherical-mode pipeline, orientation held at the
workspace anchor, no goal inference, no arbitration.

### 1.2 Why cross-paper baselines are uninterpretable here

Comparing this system's numbers against published teleoperation results would be a
category error, for four independent reasons:

1. **Different hardware.** Published shared-autonomy results are overwhelmingly on
   desk-mounted or wheelchair-mounted assistive arms driven by a joystick or a
   sip-and-puff interface (Herlant et al., 2016; Losey et al., 2020; Javdani et al.,
   2018). This system's arms are mounted on the wearer's back at (±0.15, −0.12, 1.25) m
   and are commanded from a body-worn linkage. The reachable workspace, the singularity
   structure, the collision environment (the wearer is *in* the collision model) and the
   command latency are all different in kind, not degree.

2. **Different task.** The comparison tasks in that literature (assistive feeding, bottle
   grasping, cluttered reaching) are not the same task and are not calibrated to a common
   index of difficulty. `CLAUDE.md` records that the volume in this project's own brief is
   not even reachable: 5.6% IK success with collisions on, 11.1% with them off; moving the
   same box to the arm's own side at chest height gives 77.8–100%. A number produced on an
   unreachable volume is not comparable to anything.

3. **Different participants.** Assistive-robotics baselines are frequently collected with
   users who have motor impairments (Kim et al., 2012: spinal cord injury; Gopinath et al.,
   2017: 4 of 17 with SCI), for whom the arm replaces lost function. This project's
   operators are able-bodied and the arm *supplements* function. Motivation, fatigue
   profile, learning rate and tolerance for autonomy all differ.

4. **Different reporting practice.** Amini et al. (2025), reviewing 122 Fitts-law studies
   in 3D XR — a comparatively standardised sub-field — found that over half referenced
   Fitts' law without properly investigating throughput, movement time or error rate, that
   ID reporting is inconsistent (71% report a range, 49% report exact values), and that
   26.1% of studies showing a Fitts regression omit the regression coefficient. If that is
   the state of a standardised sub-field, cross-study numeric comparison in the
   non-standardised teleoperation literature is not defensible.

### 1.3 Why within-subject comparison is the right design

- **It removes the between-subject variance that dominates teleoperation data.** Operator
  skill differences on a novel interface are large; a between-subjects design would need
  several times the participants to detect the same effect.
- **It matches how the question is posed.** The question is "does autonomy recover
  capability *this operator* has lost to a DOF-deficient input", which is a
  within-operator quantity by construction.
- **It matches the field's own practice.** Dragan and Srinivasa (2012) used a
  within-subjects design with 8 participants and a balanced Latin square for order, and
  detected an interaction between arbitration aggressiveness, prediction correctness and
  task difficulty at that n. That is the precedent for both design and rough scale.
- **Its cost is order effects and fatigue**, which are addressed in §4.

---

## 2. E1's Fitts throughput is the bridge to the wider literature

E1 characterises the master arm as a pointing device using the ISO 9241-9 / ISO 9241-411
methodology as codified by Soukoreff and MacKenzie (2004):

```
IDe = log2(Ae/We + 1)        We = 4.133 * SD(endpoint deviation along the movement axis)
TP  = IDe / MT               computed per participant per condition, then averaged
```

Using the **effective** width normalises for the speed-accuracy trade-off, so an operator
who moves fast and overshoots gets no free credit.

**Why this is the number that travels.** Every other quantity produced by this project —
IK success rate, clearance margin, Kabsch residual, tracking RMS — is meaningful only to
someone who has this rig. Throughput in bits/s is not. A reader can place it directly
against:

| interface | throughput | source |
|---|---|---|
| Mouse, ISO 9241-9 studies | 3.7 – 4.9 bits/s | Soukoreff and MacKenzie (2004), Table 5 |
| XR head-mounted displays | median 3.25 bits/s | Amini et al. (2025), 122 studies |
| Stereo displays | median 3.20 bits/s | Amini et al. (2025) |

If this master measures, say, 1.2 bits/s, a reader immediately knows it is roughly a third
to a quarter of a mouse, without ever seeing the hardware. That single number is what makes
the rest of the thesis auditable from outside. It also lets a future group with different
hardware ask whether their master is better, and by how much, on a scale that already has
a literature.

**Two honest caveats, to be printed alongside the number.**

- Fitts' law was derived for planar aimed movement. Its fit to 3D robotic-arm control is
  known to be anisotropic in the literature, and this system has a *known* directional
  defect: `CLAUDE.md` records that up/down and fore/aft track correctly on both arms while
  lateral does not, with a Kabsch residual of 54.8° (left) / 54.7° (right) between measured
  and intended sweep directions. **Throughput must therefore be reported per axis as well
  as pooled**, or the pooled number will silently average a working axis with a broken one.
- A single throughput number from one rig with n participants is a point estimate with a
  confidence interval, not a benchmark. Report the CI.

---

## 3. Pre-registered hypotheses

### 3.0 Conventions applying to all experiments

- **Design:** within-subjects throughout. Condition order counterbalanced by a balanced
  Latin square; where the number of conditions permits, fully counterbalanced.
- **Target n = 20** (see §3.7 for the power sketch). Recruitment stops at 20 completed
  participants; no optional stopping, no peeking at inferential results before n = 20.
- **Alpha:** 0.05, two-tailed, for all tests.
- **Families for correction:** each experiment defines one confirmatory family, listed
  under that experiment. **Holm-Bonferroni** within family. Any test not listed is
  exploratory, reported as such, and not corrected (and not claimed as a finding).
- **Assumption-check policy, fixed in advance:** normality of the paired differences /
  residuals is checked by Shapiro-Wilk plus a Q-Q plot; sphericity for repeated-measures
  ANOVA with more than two levels by Mauchly's test with Greenhouse-Geisser correction
  applied whenever ε < 0.75 (Huynh-Feldt for 0.75 ≤ ε < 1). **The decision rule is
  pre-committed:** if Shapiro-Wilk p < 0.05 or the Q-Q plot shows gross departure, the
  pre-registered non-parametric alternative is used and the parametric result is reported
  in an appendix. Choosing whichever test gives the smaller p is a garden-of-forking-paths
  violation and is forbidden.
- **Effect sizes reported:** Cohen's dz for paired t; partial η² with CI for RM-ANOVA;
  rank-biserial correlation for Wilcoxon; Kendall's W for Friedman; odds ratio for McNemar
  and for logistic mixed models.
- **All raw logs, the analysis script, and the exclusion rules are written and committed
  before the first participant.**

---

### E1 — Fitts characterisation of the master arm

**Purpose.** Establish that the interface obeys Fitts' law well enough for throughput to
be meaningful, and produce the throughput figure and the per-axis breakdown.

**Task.** ISO 9241-411-style 3D reciprocal pointing: spherical targets presented in the
arm's own-side chest-height volume (the volume `CLAUDE.md` measures at 77.8–100% IK, not
the unreachable table volume), at 3 amplitudes × 3 widths, giving IDs spanning roughly
2–5 bits. Directions blocked so that vertical, fore/aft and lateral are separable.

| | statement |
|---|---|
| **H1.1** | Movement time increases linearly with the effective index of difficulty. **Direction:** positive slope. **DV:** MT (s) per trial. **Test:** ordinary least-squares regression of mean MT on IDe per participant; report r² per participant and the group mean r². **Pre-registered acceptance threshold:** group mean r² ≥ 0.85, which is the conventional bar for "the model applies". |
| **H1.2** | Throughput differs by movement axis, and is **lowest for the lateral axis**. **Direction:** TP(lateral) < TP(vertical) and TP(lateral) < TP(fore/aft). **Rationale:** not a guess — `CLAUDE.md` measures azimuth as the broken channel (Kabsch residual ~54.8°, intended-orthogonal directions measured 137.7° apart). **DV:** TP (bits/s) per participant per axis. **Test:** one-way repeated-measures ANOVA, factor = axis (3 levels), followed by two pre-planned contrasts (lateral vs vertical, lateral vs fore/aft). **Non-parametric fallback:** Friedman + Wilcoxon signed-rank on the two contrasts. |
| **H1.3** | Overall throughput of the master arm is **below the published mouse range** (3.7–4.9 bits/s, Soukoreff and MacKenzie, 2004). **Direction:** lower. **DV:** pooled TP per participant. **Test:** one-sample t-test against 3.7 bits/s, one-tailed, as a descriptive anchor only. **This is explicitly labelled a weak comparison** (different task, different participants, different apparatus) and is reported as context, not as a finding. |

**Confirmatory family for E1:** {H1.2 contrast 1, H1.2 contrast 2}. H1.1 is a model-fit
check, H1.3 is descriptive; neither enters the correction family.

**Assumption checks:** Shapiro-Wilk on TP differences; Mauchly on the 3-level ANOVA;
inspection of the Fitts regression residuals for the characteristic fan pattern that
indicates a speed-accuracy drift.

---

### E2 — Autonomy level

**Purpose.** Locate the operating point of the arbitration function, and test whether the
performance/agency trade-off found in the desk-teleoperation literature reproduces on a
wearable supernumerary arm.

**Conditions.** Three levels of arbitration aggressiveness, spanning the Dragan and
Srinivasa (2012) axis: **direct** (α = 0, no assistance), **timid** (assistance grows with
goal confidence but saturates below full authority), **aggressive** (assistance takes
authority at high confidence). Prediction is the project's own goal-inference module,
identical across the two assisted conditions.

| | statement |
|---|---|
| **H2.1** | Task completion time decreases monotonically with autonomy level. **Direction:** direct > timid > aggressive. **DV:** completion time (s). **Test:** one-way repeated-measures ANOVA (3 levels) with a pre-planned linear polynomial contrast. **Fallback:** Friedman + Wilcoxon on the direct-vs-aggressive pair. |
| **H2.2** | Sense of agency decreases monotonically with autonomy level. **Direction:** direct > timid > aggressive. **Rationale:** Collier et al. (2025) found higher autonomy improved performance and *decreased* sense of agency. **DV:** agency score (pre-registered instrument, administered per condition). **Test:** as H2.1. Ordinal Likert data → Friedman is the primary test here, not the fallback. |
| **H2.3** | Preference does **not** simply track completion time. **Direction:** the correlation between per-participant preference and per-participant time advantage is positive but well below 1. **Rationale:** Dragan and Srinivasa (2012) measured Pearson's r(30) = .66 between preference and timing, with users who "prefer the timid mode despite it being slightly less efficient ... they felt more in control". **DV:** forced-choice preference (7-point) and time difference. **Test:** Pearson correlation with a bootstrap CI; the substantive prediction is that the CI excludes 0.90, i.e. preference is *not* reducible to time. |
| **H2.4** | Aggressive assistance is worse than direct teleoperation when the goal prediction is wrong. **Direction:** on trials where the top-ranked goal is not the operator's goal, completion time(aggressive) > completion time(direct). **Rationale:** "Being aggressive and wrong results in large penalties in time and user preference" (Dragan and Srinivasa, 2012). **DV:** completion time on mispredicted trials. **Test:** linear mixed model with condition as fixed effect and participant as random intercept (mispredicted trials are unbalanced across participants, so an LMM is required rather than a paired t). |

**Confirmatory family for E2:** {H2.1, H2.2, H2.4}. H2.3 is a correlational check reported
with its CI and not corrected.

**Assumption checks:** Mauchly + Greenhouse-Geisser for H2.1; Friedman needs no normality
assumption but requires the same participants across all levels (enforced by design);
for H2.4, residual normality and homoscedasticity of the LMM, plus a check that the
random-intercept variance is non-zero (if it is zero, refit as OLS and report both).

---

### E3 — Divided attention

**Purpose.** The central applied question: does the benefit of autonomy survive — or grow —
when the operator's own body is doing something else? This is the neural-resource-
allocation problem of Dominijanni et al. (2021) made operational.

**Design:** 2 × 2 within-subjects. Factor A = autonomy (direct / assisted, using the level
selected by E2). Factor B = attention (single task: the robot only; dual task: the robot
plus a concurrent primary task performed with the operator's own hands). The concurrent
task must load a *different* resource pool where possible so that the comparison is not
trivially about motor interference alone — but note that a manual concurrent task
necessarily competes on the manual-response dimension of Wickens' (2008) 4-D model, which
is the realistic case for a supernumerary limb and is therefore the right choice.
Penaloza and Nishio (2018) used a ball-balancing task with the participant's own two
hands; that is a defensible template.

| | statement |
|---|---|
| **H3.1** | Dual-tasking degrades robot-task performance. **Direction:** performance(dual) < performance(single). **DV:** primary robot-task metric (completion time, or success rate if failures are common). **Test:** main effect of attention in a 2 × 2 repeated-measures ANOVA. |
| **H3.2** | Autonomy improves robot-task performance. **Direction:** assisted > direct. **DV:** as H3.1. **Test:** main effect of autonomy in the same ANOVA. |
| **H3.3** | **The key hypothesis: autonomy buys more under divided attention than under single-task.** **Direction:** a positive interaction — the assisted-minus-direct advantage is larger in the dual-task condition. **DV:** as H3.1. **Test:** the attention × autonomy interaction term in the 2 × 2 RM-ANOVA, followed by the simple-effects contrast (assisted − direct) within each attention level. **Fallback if the residuals fail normality:** aligned rank transform ANOVA (ART), pre-specified, because Friedman cannot test an interaction. |
| **H3.4** | Autonomy protects the *concurrent* task, not only the robot task. **Direction:** concurrent-task performance is higher in the assisted condition than in the direct condition, within the dual-task level. **DV:** concurrent-task score. **Test:** paired t-test (or Wilcoxon signed-rank) on the dual-task rows only. **Rationale:** this is the claim that actually matters for a wearable SRL — an extra arm that costs you your own two arms has not augmented anything. |

**Confirmatory family for E3:** {H3.1, H3.2, H3.3, H3.4}. H3.3 and H3.4 are the two the
thesis rests on; H3.1 and H3.2 are manipulation checks that nevertheless enter the family
because they are stated directionally in advance.

**Assumption checks:** Mauchly is not needed (2 levels per factor, sphericity is trivially
satisfied); Shapiro-Wilk on the interaction contrast; check for a ceiling/floor effect in
the single-task condition, because an interaction can be manufactured by a ceiling. If the
single-task assisted cell is at ceiling, H3.3 is uninterpretable and must be reported as
such rather than claimed.

---

### E4 — DOF recovery

**Purpose.** The core mechanistic question. If the input device cannot observe a degree of
freedom, how much of the capability lost to that deficiency does autonomy restore?

**Design.** DOF availability is manipulated **in software**, by masking channels that are
physically healthy, so that the deficit under test is the structural one and not a
hardware fault (see §4 for why this matters). Conditions:

- **Full:** all channels the pipeline can use are live.
- **Masked-yaw:** the wrist-yaw observable is suppressed. This is the structural deficit —
  gravity cannot observe rotation about gravity, so a wearable accelerometer-based master
  can never supply absolute yaw (see §4.2).
- **Masked-yaw + autonomy:** identical masking, with the autonomy supplying the missing
  DOF from the goal posterior.

The primary outcome is deliberately **categorical**, because that is the honest form of the
question for a manipulation task: did the trial succeed?

| | statement |
|---|---|
| **H4.1** | Masking the DOF reduces success. **Direction:** success(full) > success(masked-yaw). **DV:** binary trial success. **Test (primary, participant-level):** McNemar's exact test on the per-participant dichotomised outcome (participant classified as "succeeded on a majority of trials" or not) — this is the pre-registered categorical test. **Test (pre-specified sensitivity analysis, trial-level):** generalised linear mixed model, binomial family, logit link, condition as fixed effect and participant as random intercept. **Why both:** McNemar assumes independent paired observations; with multiple trials per participant that assumption fails, so the GLMM is the statistically correct model and McNemar is the simple, pre-registered headline. If they disagree, the GLMM is reported as authoritative and the disagreement is discussed. |
| **H4.2** | **Autonomy recovers the loss.** **Direction:** success(masked-yaw + autonomy) > success(masked-yaw). **DV, tests:** as H4.1. |
| **H4.3** | Recovery is partial, not complete. **Direction:** success(masked-yaw + autonomy) < success(full). **DV, tests:** as H4.1. **Note:** this is a hypothesis that the null is *rejected in the stated direction*; if the data instead support equivalence, that must be tested properly with a TOST equivalence test against a pre-registered equivalence margin (proposed: 10 percentage points of success rate), not inferred from a non-significant difference. |
| **H4.4** | Recovery is larger for the structural deficit than for an incidental hardware fault of comparable dimensionality. **Direction:** the autonomy benefit is larger in masked-yaw than in a secondary "simulated dead pot channel" condition. **Rationale:** a structural deficit is *predictable from task context* (grasp yaw is largely determined by the object); a random dropout is not. **DV, test:** difference of differences in success rate, tested in the GLMM as a condition × deficit-type interaction. **This hypothesis is exploratory-confirmatory:** it is stated in advance but is the least well-motivated of the four and should be flagged as such. |

**Confirmatory family for E4:** {H4.1, H4.2, H4.3}. H4.4 is reported separately.

**Assumption checks:** for McNemar, the count of discordant pairs must be reported and, if
below ~10, the exact binomial version used rather than the χ² approximation. For the GLMM:
convergence, singular-fit check on the random-effects variance, and separation check
(a condition with 100% or 0% success makes the logistic model degenerate — pre-register a
Firth penalised-likelihood fallback).

**Mandatory reporting for E4:** for every trial, the vector of which master channels were
live, taken from the recorder's per-run `<csv>.meta.json` sidecar and the per-frame
validity flag. This is not optional; see §4.4.

---

### E5 — Intent inference characterisation

**Purpose.** Characterise the prediction module on its own terms, so that E2–E4 results can
be interpreted. Autonomy that is right 60% of the time and autonomy that is right 95% of
the time are different interventions, and Dragan and Srinivasa (2012) give the crossing
point: aggressive assistance is a net win only if the robot is wrong in **less than about
16%** of cases.

**Data.** Recorded operator trajectories with known ground-truth goals; predictions
replayed offline so that cue sets can be compared on identical data.

| | statement |
|---|---|
| **H5.1** | Prediction accuracy at the moment of first commitment exceeds the 84% operating threshold implied by Dragan and Srinivasa (2012). **Direction:** greater than 0.84. **DV:** per-participant top-1 accuracy at the commitment point. **Test:** one-sample t-test (or Wilcoxon signed-rank) against 0.84, one-tailed. **If this fails, E2's aggressive condition is predicted to be harmful, and that prediction is itself testable against H2.4** — the two experiments constrain each other. |
| **H5.2** | Accuracy increases monotonically with the fraction of the reach completed. **Direction:** positive. **DV:** accuracy binned at 20/40/60/80% of trajectory duration. **Test:** linear mixed model, accuracy ~ fraction-elapsed + (1 \| participant), testing the slope. |
| **H5.3** | Adding a pointing-direction cue to the end-effector-velocity cue does **not** significantly improve early prediction. **Direction: null / no improvement.** **Rationale, stated honestly:** the literature review found no study isolating pointing direction against end-effector velocity, so there is no basis for predicting a benefit; and this rig's azimuth channel is measurably unreliable (Kabsch residual ~54.8°), so a pointing cue inherits that error. **DV:** area under the accuracy-vs-fraction-elapsed curve. **Test:** paired t-test between cue sets, with a pre-registered **TOST equivalence test** (margin: 0.05 AUC) as the primary analysis, because the prediction is of *no effect* and a non-significant t-test does not establish that. |
| **H5.4** | Prediction degrades under divided attention. **Direction:** accuracy(dual-task trajectories) < accuracy(single-task trajectories). **Rationale:** a distracted operator produces less goal-directed, higher-entropy input, which is exactly what a MaxEnt-IOC likelihood is worst at. **DV:** top-1 accuracy at commitment. **Test:** paired t-test / Wilcoxon on per-participant accuracy. **This is the mechanism by which E3's H3.3 could fail**, and stating it in advance protects against post-hoc rationalisation. |

**Confirmatory family for E5:** {H5.1, H5.2, H5.3, H5.4}.

**Assumption checks:** accuracy is a bounded proportion; check for ceiling and, if the
mean exceeds ~0.9, use a logit or arcsine-square-root transform (pre-registered) or a
beta-regression GLMM. LMM residual checks as in E2.

---

### 3.6 Cross-experiment coherence checks (stated in advance, not corrected)

- E5's H5.1 accuracy should predict E2's H2.4 penalty. If accuracy is well above 84% and
  aggressive assistance still loses to direct on mispredicted trials, the arbitration
  function, not the predictor, is the problem.
- E5's H5.4 degradation should partially offset E3's H3.3 interaction. If both hold, the
  net benefit of autonomy under divided attention is the difference of two opposing
  effects and must be reported as such rather than as a clean win.

### 3.7 Target n and power sketch

**Target n = 20 completed participants.**

For a within-subject paired comparison at α = 0.05 (two-tailed) and 80% power, the normal
approximation gives the required n as

```
n ≈ (z_{0.975} + z_{0.80})^2 / dz^2 = (1.960 + 0.842)^2 / dz^2 = 7.85 / dz^2
```

with a small upward correction (typically +1 to +2) for using t rather than z. That gives:

| n | smallest detectable dz (80% power, α = .05, two-tailed) |
|---|---|
| 12 | ≈ 0.86 |
| 16 | ≈ 0.75 |
| **20** | **≈ 0.67** |
| 24 | ≈ 0.61 |

**The effect size assumed is a guess, and is labelled as such.** The reasoning behind
choosing dz ≈ 0.7 as the target sensitivity:

- Dragan and Srinivasa (2012) detected a three-way structure with **n = 8**, which implies
  their effects were large (dz well above 1.0) — but their manipulation was near-total
  (full autonomy vs none) on a task designed to be hard.
- Pan et al. (2024, 2025) used **n = 24** for autonomy-level effects on cognitive load and
  trust in desk teleoperation, and reported significant main effects. That is the closest
  methodological analogue and is the reason n = 20 is the floor rather than n = 8.
- Amini et al. (2025) report that Fitts-law studies in 3D XR average **18 participants
  (SD 8.52)**, so n = 20 is at the modal scale for E1 specifically.

**Honest statement of the limitation:** none of the above is a pilot estimate from *this*
rig. dz = 0.67 is what n = 20 can see, not what the effect is. The pre-registration must
be amended after a pilot of 5–6 participants, with the pilot data excluded from the
confirmatory analysis. If the pilot suggests dz < 0.5, the design must either recruit more
participants, increase trials per condition (which shrinks measurement error and raises dz
without raising n), or drop the weaker hypotheses rather than run underpowered.

**On the multi-level tests.** The RM-ANOVA (E2, E3) and mixed-model (E4, E5) power figures
are *not* derived here. Rules of thumb suggest n = 20 with three repeated levels and
ε ≈ 0.75 detects roughly a medium effect (Cohen's f ≈ 0.3), but that should be computed
properly in G*Power (for the ANOVAs) and by simulation, e.g. `simr`, for the mixed models,
before the design is locked. **[UNVERIFIED — rule-of-thumb, not a computed power analysis]**

---

## 4. The reviewer objection

> *"You have added autonomy to compensate for a broken input device. Fix the device and the
> contribution disappears. What you are calling a research contribution is a workaround for
> dead potentiometers, and the experiment is a demonstration that a working sensor beats a
> broken one — which nobody needed to be told."*

This is the correct objection to raise, it is the one that will decide the viva, and it
deserves a full answer rather than a deflection.

### 4.1 What the objection gets right, conceded without hedging

This rig genuinely has broken sensors, and the record is specific about it (`CLAUDE.md`):

- **LEFT:** j1–j4 live; j5 shows 2.8% exact zeros and 14% railed; j6 is clamped at 0 for
  12.8% of frames; j7 is railed 75% of the time.
- **RIGHT:** j1, j2, j6 live; j3 dead (70% zeros); j5 and j7 dead (99% zeros); j4
  intermittent at 12.9% exact-zero dropouts, concentrated in bursts (51.1% of one sweep).
- **Right IMU gyro** shows intermittent spikes to 72 °/s while stationary, giving +44 °/min
  yaw drift against −5.0 °/min on the left.

These are wiring faults. All three dead right channels are roll joints, which points at one
fatigued harness rather than three independent pot failures. They are repairable with a
soldering iron and an afternoon. **They are not a research contribution, and no hypothesis
in §3 depends on them.**

### 4.2 What the objection gets wrong: the structural limits are not repairable

The objection assumes that "a working input device" means a device that measures the full
task space. For a **wearable, hands-free** master, that device does not exist, and the
reasons are physical rather than budgetary.

**(a) Mass and grounding.** A desk-mounted haptic device (Phantom, Omega, Sigma-7 class) is
*grounded*: its base is bolted to a table, so it can carry an encoder on every joint and
reflect force through a reaction path that terminates in the table. A body-mounted master
has no such ground. Its reaction path terminates in the wearer's own skeleton, so any force
it reflects is a force the wearer must resist, which both loads the operator and corrupts
the measurement. It must also be light enough to be worn for a shift and must leave the
hands free — that is what makes it a *supernumerary* limb interface rather than a desk
interface. Prattichizzo et al. (2021) name this as an unsolved constraint: "achieving a
trade-off between weight and dexterity is still an open challenge". A grounded 6-DOF
force-feedback measurement chain is not something a wearable master will ever carry, at any
budget.

**(b) Gravity gives roll and pitch but never yaw. This is a rank deficiency.** An
accelerometer at rest measures the gravity vector in the sensor frame. Two of the three
rotational degrees of freedom — the ones that tilt the sensor relative to gravity — change
that vector and are observable. The third, rotation *about* gravity, leaves the vector
unchanged and is therefore **unobservable in principle**, not merely poorly calibrated.

This project's own design exploits exactly that structure, which is why it is a clean
demonstration rather than an excuse. Elevation is taken as
`asin(dot(a_hat, normalize(accel)))`, which `CLAUDE.md` describes as mount-independent and
drift-free with no integration — because it uses the observable part. Azimuth is precisely
the unobservable part, and it is taken from a single joint angle (j1) as a substitute. The
consequence is measured, not asserted: fitting the best rotation from measured sweep
directions to intended directions leaves a **54.8° RMS residual (left) / 54.7° (right)**,
and the rotation-invariant pairwise-angle test shows intended-orthogonal directions
measured **137.7° apart**, so no rotation and no permutation can reconcile them. Fixing
every potentiometer in the rig would not move that number, because it is not a
potentiometer problem.

**(c) The three ways out of (b) are all closed, and each was tested.**

- *Integrate a gyro.* Implemented (`azimuth_mode: gyro`), measured, and **not enabled**:
  −5.0 °/min drift on the good arm, +44.0 °/min on the bad one. Even the good arm drifts
  ~1.7° over a 20-second segment. Bias instability in low-cost MEMS gyros is a device-class
  property; a better part improves the constant, not the fact that unbounded integration
  of a biased rate diverges.
- *Double-integrate acceleration for position.* Standard strapdown dead-reckoning error
  growth makes this unusable for a hand-scale workspace over task-length durations: a
  constant accelerometer bias `b` produces position error `½·b·t²`, which for even a
  milli-g bias exceeds the master's 0.272 m arm length within seconds. This is textbook
  inertial-navigation behaviour, and it is why no wearable master in the literature
  surveyed uses it.
- *Add a magnetometer for absolute yaw.* A magnetometer near two 7-DOF robot arms with
  brushless actuators, an aluminium backpack frame and a steel harness measures the robot,
  not the Earth. Not attempted, and correctly so.

An external reference (optical tracking, a second IMU on the torso, UWB) can supply yaw —
but each of those either re-grounds the system to the environment, which defeats the point
of a wearable, or adds a second body-worn subsystem with its own drift and its own mass
budget. This is a genuine design space with genuine trade-offs, which is what makes it
worth studying.

**(d) The consequence is already measured on this rig.** `CLAUDE.md` records IK
feasibility with orientation taken from the wrist versus held at a fixed anchor:

| arm | scale | orientation from wrist | held at anchor |
|---|---|---|---|
| left | 0.6 | 10% | 100% |
| left | 1.0 | 8% | 92% |
| right | 0.6 | 62% | 98% |
| right | 1.0 | 75% | 95% |

The system is currently a **position-only teleoperator** (`orientation_mode: fixed`)
because commanded orientation from an unreliable wrist made position and orientation
jointly unreachable. That is the DOF deficit, quantified, and it is exactly the deficit E4
manipulates.

### 4.3 The reframing: this is a design constraint of the device class

The objection, applied consistently, would delete a substantial body of accepted work.
Muelling et al. (2017) added autonomy because a brain-computer interface produces "noisy
and erratic low-dimensional motion commands". Nobody argues that BCI shared autonomy is a
workaround for a broken electrode. Losey et al. (2020) built latent-action embeddings
because people control 7-DOF arms with 2-DOF joysticks; nobody argues they should have
bought a better joystick. Jain et al. (2015) mapped residual body motion into a
low-dimensional signal for users with motor impairment. In every case the input deficiency
is *intrinsic to the interface class*, and the autonomy that compensates for it is the
research.

The claim here is the same one: **a wearable supernumerary master will always be
DOF-deficient relative to a grounded haptic device, because it must be light, wearable and
hands-free.** That is not a defect of this build; it is the defining constraint of the
device class. Prattichizzo et al.'s (2021) review states the resulting research question in
its own words — "it is fundamental to find the right trade-off between the degrees of
freedom that are under the direct control of the user and the level of robot autonomy" —
and does not cite an answer.

### 4.4 The experimental control that makes this argument empirical rather than rhetorical

The argument above is only worth as much as the experiment that tests it. Two concrete
controls are therefore built into E4:

**Control 1 — the deficit under test is imposed in software, on healthy channels.**
E4's masked-yaw condition is created by **suppressing a channel that is physically
working**, not by using a channel that is broken. The hardware faults listed in §4.1 are
repaired, or the arm with the fault is excluded, before E4 runs. This makes the independent
variable the *structural* deficit — the one that no repair can remove — rather than a
soldering failure. The manipulation is then reversible, calibrated and identical across
participants, which a hardware fault can never be.

The left arm is the candidate for this: `CLAUDE.md` records j1–j4 live, gyro drift
−5.0 °/min (usable), and a clean 109 distinct j1 values over 193.5° during shoulder
rotation. The right arm's j3/j5/j7 harness fault and 12.9% j4 dropout make it unsuitable
for E4 until repaired, and that should be stated in the methods rather than worked around.

**Control 2 — channel liveness is reported for every trial, as data.**
The recorder already writes a `<csv>.meta.json` sidecar per run and a per-frame validity
flag; `CLAUDE.md` shows the validator tracks dropouts to within 0.1% (`valid==0` at 13.0%
against `j4==0` at 12.9%), so no fabricated data reaches the command. E4 extends this into
the reporting requirement:

- Every trial record carries the vector of live channels and the per-channel dropout rate.
- A **pre-registered exclusion rule**: any trial in which a channel the current
  `position_mode` consumes (j1, j2, j4 in spherical mode) drops out for more than 2% of
  frames is excluded from the confirmatory analysis and reported in an exclusion table.
  The 2% threshold is set from the measured baseline — the healthy left channels show 0%,
  the known-faulty right j4 shows 12.9% — so 2% cleanly separates "healthy" from "faulty"
  and is not tuned to the outcome.
- The results table reports N excluded per condition. If exclusions are unbalanced across
  conditions, that is itself a confound and is reported, not smoothed over.
- H4.4 turns the incidental/structural distinction into a *tested* prediction rather than
  an assertion, by including a simulated-dropout condition alongside the masked-yaw
  condition and testing whether autonomy recovers them differently.

**What would falsify the argument.** If E4 shows that autonomy recovers the simulated
hardware-fault deficit just as well as the structural yaw deficit (H4.4 null), then the
distinction between incidental and structural, as far as the *autonomy* is concerned, does
not matter — and the honest conclusion is that this is a general low-DOF shared-autonomy
result, already precedented by Losey et al. (2020), rather than something specific to
wearable masters. That would weaken the novelty claim, and it must be reported if it
happens.

---

## 5. Threats to validity

### 5.1 Fatigue, and its interaction with condition order

**The threat.** Holding a mannequin master arm up is a mid-air interaction, and mid-air
interaction produces monotonically accumulating shoulder fatigue — the "gorilla arm"
effect, quantified as Consumed Endurance by Hincapié-Ramos et al. (2014). Fatigue is
*monotonic in time on task*, so in a within-subjects design it is perfectly confounded with
block order unless the order is broken. If direct teleoperation always ran first, it would
look artificially good; if it always ran last, artificially bad. Worse, fatigue may
interact with condition: the direct condition demands more sustained precise holding than
the assisted condition, so fatigue may accumulate *faster* in one condition, which no
amount of order counterbalancing removes.

**Mitigations, all pre-registered.**
- Balanced Latin square on condition order, with order entered as a covariate (or as a
  fixed effect in the LMM) and its coefficient reported. If order has a significant main
  effect, that is reported as a finding about the interface, not hidden.
- A **fatigue probe** at every block boundary: (i) Borg CR10 perceived exertion, and
  (ii) a fixed 10-second reference hold at a marked pose, from which tremor amplitude and
  drift are computed from the master's own IMU. The reference hold gives an objective,
  interface-native fatigue measure that does not depend on self-report.
- Mandatory rest between blocks, of fixed duration, with the wearer's arm supported.
- Time-on-task recorded per trial and included as a covariate in the mixed models.
- **A pre-registered failure criterion:** if the reference-hold tremor metric shows a
  significant condition × block interaction, the fatigue confound is judged not controlled,
  and the affected hypothesis is reported as inconclusive rather than as supported.

### 5.2 Channel dropouts mid-trial

**The threat.** A right-arm j4 dropout freezes the command at the last good joint vector.
`CLAUDE.md` records the effect precisely: the right command freezes for half of one sweep
and a quarter of another, which "flattens measured displacement, biases the gain toward
zero and adds apparent lag". A frozen command during a Fitts trial inflates movement time
and looks exactly like poor operator performance.

**Mitigations.** The per-trial liveness reporting and the 2%-dropout exclusion rule of
§4.4. In addition: dropout rate is reported per condition so that a reader can see whether
any effect could be an artefact of unequal hardware health; and E1 (which produces the
externally-comparable throughput number) is run on the healthy arm only, with that stated
in the methods.

### 5.3 Bench-mounted versus worn operation

**The threat.** Running the rig on a bench and running it worn are not the same experiment,
and conflating them changes what is being claimed. Worn operation adds: the wearer's own
postural sway coupling into the mount; a real collision risk from the arms to the wearer
(`CLAUDE.md` records a measured 0.126 m clearance to the head at the current right home —
6 mm above the 0.12 m floor); genuine mass on the shoulders; and the psychological effect
of a 7-DOF arm moving near one's own head, which plausibly changes how much authority an
operator is willing to cede.

**Mitigations.** State explicitly, per experiment, whether it was run bench-mounted or
worn. E3 (divided attention) is meaningless bench-mounted and **must** be worn, because the
concurrent task is performed with the wearer's own hands. E1 and E5 can be bench-mounted
and should say so; their conclusions then apply to the *input device*, not to the worn
system. Any generalisation from bench to worn must be labelled as an assumption, and the
sense-of-agency and trust measures must not be transferred across that boundary at all.

### 5.4 Learning effects

**The threat.** This is not a nuisance variable to be counterbalanced away, because the
literature shows learning rates *differ by condition*. Kim et al. (2012) found that over a
three-week study, manual-mode users showed a pronounced learning effect in completion time
and command count while auto-mode users mainly reduced variability. If direct teleoperation
improves faster than assisted teleoperation with practice, then a single-session result
overstates the autonomy benefit — the assisted condition's advantage may simply be a
head-start.

**Mitigations.** A training phase to a pre-registered performance criterion before any
confirmatory trial, in *every* condition (Dragan and Srinivasa, 2012 used a training phase
for exactly this reason). Trial index within condition entered as a covariate, and the
learning slope reported per condition. **A pre-registered secondary analysis:** fit
per-condition learning curves and report whether the slopes differ; if they do, the
single-session result is explicitly labelled as not generalising to trained operators.

### 5.5 Small n

**The threat.** n = 20 detects dz ≈ 0.67 at 80% power. Anything smaller is invisible, and
the study will produce non-significant results for real-but-modest effects. Small samples
also inflate published effect sizes when only significant results are reported.

**Mitigations.** Report confidence intervals on every effect, not just p-values. Report
non-significant results with their CIs so that "no effect detected" is distinguishable from
"effect ruled out". Use equivalence tests (TOST) wherever the hypothesis is of no effect
(H5.3, and H4.3's alternative). Do not interpret a non-significant interaction as evidence
of no interaction.

### 5.6 Experimenter demand effects

**The threat.** Participants will infer which condition the experimenter believes in,
particularly when the assisted condition visibly "helps". Preference and agency measures
are the most vulnerable; completion time is less so but not immune.

**Mitigations.** Scripted, written instructions read verbatim (or played from a recording)
so that phrasing does not vary by condition. Conditions labelled neutrally to the
participant ("Mode A / B / C"), never "assisted" or "manual". Subjective measures collected
via a self-administered form the experimenter does not see during the session. The
experimenter is out of the participant's line of sight during trials where safety permits.
A post-session funnel debrief asking participants what they thought was being tested, with
responses coded and reported.

### 5.7 The same person built the autonomy and runs the study

**The threat.** This is the most serious of the validity threats and the hardest to remove
in a single-student project. The developer of the arbitration function has: (i) an interest
in it working, (ii) tacit knowledge of which conditions are fragile, (iii) the ability to
tune parameters after seeing data, and (iv) discretion over exclusions and analysis
choices. Any of these can produce a real effect out of noise without a single dishonest
act.

**Mitigations, in decreasing order of strength.**
1. **Pre-registration.** This document, timestamped in version control before the first
   participant, with hypotheses, tests, exclusion rules and correction families fixed. That
   is what removes analytic discretion, and it is the only mitigation that is fully within
   the student's control.
2. **Analysis code written and committed before data collection**, run end-to-end on
   simulated data with the true effect set to zero, to confirm it does not produce
   significant results from noise.
3. **Blinded analysis.** Condition labels are replaced with arbitrary codes in the analysis
   dataset; the code-to-condition key is held in a separate committed file and applied only
   after the confirmatory tests have been run and their outputs committed.
4. **Parameter freeze.** All autonomy parameters — arbitration function shape, confidence
   threshold, prediction horizon, the posterior-blend weight — are frozen and committed
   before data collection, with their values printed into every trial log so that a drift
   is detectable after the fact.
5. **A second experimenter runs the sessions** where the department permits it. If it does
   not, this is stated as a limitation, not omitted.
6. **The developer is excluded as a participant.** If the developer must be a participant
   for logistical reasons, that data is analysed separately and reported separately, never
   pooled.
7. **Adversarial review of the analysis script** by someone with no stake in the result,
   before unblinding.

**What remains uncontrolled.** Even with all of the above, the choice of *task*, of
*volume*, of *concurrent task* and of *which DOF to mask* were made by someone who knows
the system's strengths. The reachable-volume choice in particular is documented as being
made because the brief's original volume measured 5.6% IK. That is the correct engineering
decision and it is also, unavoidably, a decision that makes the system look better. It must
be stated in the methods with the measured numbers attached, so that a reader can judge it.

### 5.8 Construct validity of the primary measures

Two smaller but real threats, listed for completeness:

- **Fitts' law may not fit.** H1.1 sets an explicit acceptance threshold (group mean
  r² ≥ 0.85). If it is not met, throughput is not a valid summary of this interface and
  must not be reported as one; the fallback is raw movement time and error rate at fixed
  IDs, which is less comparable but still honest.
- **"Success" in E4 is a designed variable.** The success criterion (position tolerance,
  orientation tolerance, grasp achieved) determines the answer, and a loose orientation
  tolerance would make a yaw deficit invisible by construction. The criterion must be
  fixed in advance, justified against the task's physical requirements, and reported with
  a sensitivity analysis at ±50% of the chosen tolerance.

---

## References

Amini, M., Stuerzlinger, W., Teather, R. J. and Batmaz, A. U. (2025). A systematic review
of Fitts' law in 3D extended reality. *Proceedings of the 2025 CHI Conference on Human
Factors in Computing Systems (CHI '25)*. DOI: 10.1145/3706598.3713623

Collier, M. A., Narayan, R. and Admoni, H. (2025). The sense of agency in assistive
robotics using shared autonomy. *Proceedings of the 2025 ACM/IEEE International Conference
on Human-Robot Interaction (HRI '25)*. arXiv:2501.07462.

Dominijanni, G., Shokur, S., Salvietti, G., Buehler, S., Palmerini, E., Rossi, S.,
De Vignemont, F., d'Avella, A., Makin, T. R., Prattichizzo, D. and Micera, S. (2021). The
neural resource allocation problem when enhancing human bodies with extra robotic limbs.
*Nature Machine Intelligence*, 3(10), 850–860. DOI: 10.1038/s42256-021-00398-9

Dragan, A. D. and Srinivasa, S. S. (2012). Formalizing assistive teleoperation. *Robotics:
Science and Systems VIII*. https://www.roboticsproceedings.org/rss08/p10.pdf

Dragan, A. D. and Srinivasa, S. S. (2013). A policy-blending formalism for shared control.
*The International Journal of Robotics Research*, 32(7), 790–805.
DOI: 10.1177/0278364913490324

Fitts, P. M. (1954). The information capacity of the human motor system in controlling the
amplitude of movement. *Journal of Experimental Psychology*, 47(6), 381–391.

Gopinath, D., Jain, S. and Argall, B. D. (2017). Human-in-the-loop optimization of shared
autonomy in assistive robotics. *IEEE Robotics and Automation Letters*, 2(1), 247–254.

Herlant, L. V., Holladay, R. M. and Srinivasa, S. S. (2016). Assistive teleoperation of
robot arms via automatic time-optimal mode switching. *Proceedings of the 11th ACM/IEEE
International Conference on Human-Robot Interaction (HRI '16)*.

Hincapié-Ramos, J. D., Guo, X., Moghadasian, P. and Irani, P. (2014). Consumed endurance:
a metric to quantify arm fatigue of mid-air interactions. *Proceedings of the SIGCHI
Conference on Human Factors in Computing Systems (CHI '14)*.

Jain, S., Farshchiansadegh, A., Broad, A., Abdollahi, F., Mussa-Ivaldi, F. and Argall, B.
(2015). Assistive robotic manipulation through shared autonomy and a body-machine
interface. *2015 IEEE International Conference on Rehabilitation Robotics (ICORR)*,
526–531. DOI: 10.1109/ICORR.2015.7281253

Javdani, S., Admoni, H., Pellegrinelli, S., Srinivasa, S. S. and Bagnell, J. A. (2018).
Shared autonomy via hindsight optimization for teleoperation and teaming. *The
International Journal of Robotics Research*, 37(7), 717–742.
DOI: 10.1177/0278364918776060

Kim, D.-J., Hazlett-Knudsen, R., Culver-Godfrey, H., Rucks, G., Cunningham, T., Portée, D.,
Bricout, J., Wang, Z. and Behal, A. (2012). How autonomy impacts performance and
satisfaction: results from a study with spinal cord injured subjects using an assistive
robot. *IEEE Transactions on Systems, Man, and Cybernetics — Part A: Systems and Humans*,
42(1), 2–14. DOI: 10.1109/TSMCA.2011.2159589

Losey, D. P., Srinivasan, K., Mandlekar, A., Garg, A. and Sadigh, D. (2020). Controlling
assistive robots with learned latent actions. *2020 IEEE International Conference on
Robotics and Automation (ICRA)*, 378–384.

MacKenzie, I. S. (1992). Fitts' law as a research and design tool in human-computer
interaction. *Human-Computer Interaction*, 7(1), 91–139.
DOI: 10.1207/s15327051hci0701_3

Muelling, K., Venkatraman, A., Valois, J.-S., Downey, J. E., Weiss, J., et al. (2017).
Autonomy infused teleoperation with application to brain computer interface controlled
manipulation. *Autonomous Robots*, 41, 1401–1422. DOI: 10.1007/s10514-017-9622-4

Pan, J., Eden, J., Oetomo, D. and Johal, W. (2024). Effects of shared control on cognitive
load and trust in teleoperated trajectory tracking. *IEEE Robotics and Automation Letters*,
9(6), 5863–5870. arXiv:2402.02758.

Pan, J., Eden, J., Oetomo, D. and Johal, W. (2025). Using Fitts' law to benchmark assisted
human-robot performance. *Proceedings of the 2025 ACM/IEEE International Conference on
Human-Robot Interaction (HRI '25)*. arXiv:2412.05412.

Penaloza, C. I. and Nishio, S. (2018). BMI control of a third arm for multitasking.
*Science Robotics*, 3(20), eaat1228. DOI: 10.1126/scirobotics.aat1228

Prattichizzo, D., Pozzi, M., Lisini Baldi, T., Malvezzi, M., Hussain, I., Rossi, S. and
Salvietti, G. (2021). Human augmentation by wearable supernumerary robotic limbs: review
and perspectives. *Progress in Biomedical Engineering*, 3(4), 042005.
DOI: 10.1088/2516-1091/ac2294

Soukoreff, R. W. and MacKenzie, I. S. (2004). Towards a standard for pointing device
evaluation: perspectives on 27 years of Fitts' law research in HCI. *International Journal
of Human-Computer Studies*, 61(6), 751–789.

Wickens, C. D. (2008). Multiple resources and mental workload. *Human Factors*, 50(3),
449–455. DOI: 10.1518/001872008X288394

**Standards**

ISO 9241-9:2000, superseded by ISO/TS 9241-411:2012 / ISO 9241-411:2012 — requirements and
evaluation methods for non-keyboard input devices. *(revision history from secondary
sources; [UNVERIFIED] in detail)*

**Project-internal source (not a publication)**

`kortex_ws/CLAUDE.md` — engineering log. All numeric claims attributed to "`CLAUDE.md`" in
this document (channel health, dropout rates, gyro drift, Kabsch residuals, IK feasibility
by orientation mode, clearance margins, master reach 0.272 m, real-arm loop rate and
latency) are measurements from this rig, not published results.


---

# 6. THE TWO-PERSON DESIGN

*Added 2026-08-08. Supersedes Sections 1-5 wherever they conflict.*

## 6.1 What changed, and what did not

**Did not change.** The baseline is still *this system in direct teleoperation*,
within-subjects (§1). Cross-paper baselines are still uninterpretable here (§1.2).
Fitts throughput is still the bridge to the wider literature (§2).

**Changed.**

| | single-person framing | two-person framing |
| --- | --- | --- |
| unit of analysis | participant | **dyad** |
| who consents | one | **both, separately and independently** |
| whose fatigue | one curve | **two different curves** — the wearer bears >17 kg, the operator bears an ungravity-compensated master arm |
| the wearer | apparatus | **participant, with their own DVs** |
| base motion | a nuisance | **an independent variable (T9) and a measured covariate everywhere else** |
| task space | one person's reach | **plus tasks neither person can complete alone (T8)** |

## 6.2 The primary hypothesis, restated for two people

The single-person version asked whether autonomy improves task performance. That is
now only half of it, because SRL Proxemics (CHI 2026) found autonomy makes the wearer
*worse* off — higher arousal, lower trust — while the shared-autonomy literature says
it makes the operator *better* off. **Both ends must be reported or the result is not
interpretable.**

> **H1 (DYADIC TRADE).** Increasing autonomy improves OPERATOR task performance and
> degrades WEARER experience, and the two effects are separable and both non-zero.
>
> - **H1a** operator: completion time and tracking error improve with autonomy level.
> - **H1b** wearer: perceived safety and trust do **not** improve, and may decline —
>   **directionally pre-registered as a decline**, following SRL Proxemics rather than
>   intuition.
> - **H1c** the trade is **mediated by predictability**, not by autonomy per se. If
>   wearer-rated predictability of the arms' motion is entered as a covariate, the
>   autonomy effect on H1b is substantially reduced.
>
> H1c is the one worth being right about. If it holds, the design implication is
> "make autonomy legible", not "use less autonomy", and that is an actionable finding.

## 6.3 The hypothesis the architecture makes uniquely available

> **H2 (EXOGENOUS DISTURBANCE).** Autonomy compensates wearer-induced base motion
> better than direct teleoperation does, and the advantage **GROWS with disturbance
> amplitude**.
>
> Interaction, not main effect. A main effect of autonomy would be unsurprising and
> could come from anywhere; the interaction `autonomy x disturbance amplitude` is the
> prediction, and its mechanism is specific: the operator cannot anticipate a
> disturbance originating in another person's body, having neither efference copy nor
> vestibular access to it, and seeing it only through a camera mounted on the moving
> base. Autonomy reads the base state directly.
>
> **This is the strongest hypothesis in the set, but not because it is untested —
> it is partly tested and that is what makes it safe.** Zhang et al. (2024) report
> 1.37 ± 0.58 mm tracking with the human shoulder as floating base, so the control
> problem is known to be solvable. What is untested is the two-person case where the
> disturbance is exogenous to the operator. A null result would therefore be
> *informative* rather than a failure to build the thing.

> **H2-null (stated in advance).** If the operator can see the sway and track it
> visually, direct teleoperation may not degrade with amplitude at these frequencies
> at all, and the interaction will be absent. Metronome rates are chosen (§T9) to span
> both sides of plausible visual tracking bandwidth for exactly this reason.

## 6.4 Coordination, which does not exist in the single-person literature

> **H3 (COORDINATION).** In tasks neither person can complete alone (T8), the dyad
> converges: coordination latency falls across repetitions and initiation shifts from
> the operator toward the wearer.
>
> The shift in **who initiates** is the substantive claim. Early on the operator must
> ask; if the wearer begins to reposition *before* being asked, the wearer has built a
> predictive model of the operator's intent — which is body-schema-like learning by a
> person who has no control over the limbs and no proprioception of them. Measured as
> the sign of `t_wearer_moves − t_operator_requests`.

> **H4 (ROLE ASYMMETRY).** Wearing and operating are not symmetric experiences, and
> the order in which a person does them changes their ratings.
>
> Specifically: having *worn* first will raise an operator's caution (slower, larger
> clearance) relative to operators who have not worn. This is why role order is
> counterbalanced and logged rather than assumed away — and it is a real threat to a
> within-dyad role swap, since it cannot be removed, only balanced and measured.

## 6.5 Design, and the awkward arithmetic of dyads

**Within-dyad on autonomy** (direct / assisted / shared), **within-dyad on role**
(each person wears and operates), **between-dyad on nothing**.

The cost is real and must be stated: **role is crossed with everything**, so a full
crossing of role x autonomy x task doubles the session. §7 of `04_protocol.md`
resolves this by swapping role **only on a subset of tasks** (T5 and T9), which are
the two where the wearer's role differs most, and holding role fixed elsewhere. That
is a deliberate loss of power on the role factor, taken because the alternative is a
session no participant pair can complete without fatigue confounding everything.

**Williams squares are now assigned per DYAD**, and role order is counterbalanced
ACROSS dyads (AB / BA), so with n dyads the role-order imbalance is 0 for even n.

## 6.6 New threats to validity

**6.6.1 The wearer is not blind and cannot be.** They feel every motion. Condition
masking is impossible for them, so wearer-rated measures carry demand
characteristics that operator-rated ones do not. Mitigation: the wearer is never told
the autonomy level, autonomy is never named in their instructions, and **skin
conductance is recorded** as a measure they cannot consciously manage — following SRL
Proxemics, where the physiological and subjective measures dissociated.

**6.6.2 Two fatigue curves that are not exchangeable.** The wearer's is **mass**
(>17 kg, isometric, monotonic, and worst in the shoulders and lumbar spine). The
operator's is **effort** against an uncompensated master arm (dynamic, worst in the
deltoid). They saturate at different rates, so a single session length is wrong for
both; Borg CR10 is taken from **both** at every block boundary and entered as a
covariate, and the wearer's block is the one that sets the ceiling.

**6.6.3 A dyad is one observation, not two.** Ratings within a dyad are not
independent — they talked to each other, and in T8 they had to. Analyses are on
dyad-level aggregates or mixed models with a dyad random effect; treating 12 dyads as
24 participants would roughly halve the true standard errors.

**6.6.4 Pairs who know each other coordinate differently.** Prior acquaintance is
recorded (stranger / colleague / friend) and reported. With small n it cannot be
balanced, only disclosed.

**6.6.5 The wearer can end the session unilaterally, and that is not missing data at
random.** A wearer who stops because they felt unsafe is precisely the observation the
safety measures exist to capture. Stops are recorded with the reason, reported in the
results, and **never** silently dropped.

## 6.7 What this design still cannot answer

- **Embodiment for the wearer.** Our arms are never linked to their limbs (Fusion's
  *Enforced* mode), so ownership language does not apply and is not measured.
- **Long-term adaptation.** A single session cannot separate skill acquisition from
  trust calibration.
- **Whether autonomy would help a wearer who had a control input.** By construction
  they have none; that is the architecture, not an oversight.
