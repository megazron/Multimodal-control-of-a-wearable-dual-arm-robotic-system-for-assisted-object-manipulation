# ALOHA — the methodology, and why its tasks cannot be reused here

**The short version.** ALOHA is the reference point for low-cost bimanual
manipulation benchmarking, and its protocol is the one this project's
colour-matched placement task is modelled on. Its *task set*, however, cannot
be ported to this platform — not because the platform is worse at them, but
because every ALOHA task requires the two grippers to work at or near the same
point, and on a back-mounted supernumerary pair **no such point exists**. That
is a geometric property of the SRL configuration, and it is a finding to
report, not a shortcoming to apologise for.

---

## 1. What ALOHA is

**Zhao, T. Z., Kumar, V., Levine, S. and Finn, C. (2023). *Learning
Fine-Grained Bimanual Manipulation with Low-Cost Hardware*.
arXiv:2304.13705** (submitted 23 April 2023; project page
`tonyzhaozh.github.io/aloha/`). Presented at Robotics: Science and Systems
(RSS) 2023.

ALOHA — *A Low-cost Open-source Hardware System for Bimanual Teleoperation* —
is two things at once, and it matters which one is being cited:

1. **A teleoperation rig.** Two follower arms on a bench, two smaller leader
   arms the operator back-drives by hand, four RGB cameras streaming
   480 × 640, and a ~$20k total budget.
2. **An imitation-learning method.** ACT — *Action Chunking with
   Transformers* — trained on demonstrations collected through that rig. ACT
   predicts a *chunk* of future actions rather than one step, which is the
   paper's answer to compounding error and to the non-stationarity of human
   demonstrations.

The headline result is 80–90 % success on fine-grained tasks from roughly
**10 minutes of demonstration data**, with **50 demonstrations per task**.

### The successor papers

* **Mobile ALOHA** — Fu, Z., Zhao, T. Z. and Finn, C. (2024),
  arXiv:2401.02117 — puts the pair on a wheeled base and adds whole-body
  teleoperation.
* **ALOHA 2** — ALOHA 2 Team (2024), arXiv:2405.02292 — a hardware revision
  aimed at durability and throughput.

> Both successor identifiers are recorded from the arXiv numbering and should
> be checked against the source before they appear in a bibliography; only the
> 2023 paper above was fetched and verified directly for this note.

## 2. The methodology, point by point

What is worth copying, and what this project already does differently.

| ALOHA | here |
| --- | --- |
| **Leader–follower joint-space puppeteering.** The leader is a scaled copy of the follower, so the map is joint-to-joint with no IK in the loop. | **Non-isomorphic master.** A mannequin arm read by potentiometers and IMUs, mapped through a *spherical* position model to a task-space pose and then through TRAC-IK. There is no joint correspondence to exploit, and 7 of 14 master channels are currently incoherent. |
| **50 Hz control.** | 50 Hz master publishing; the real-arm path is capped at ~18 Hz by the Kortex high-level round trip, measured. |
| **4 RGB cameras**, 2 wrist + 2 static. | 2 wrist cameras (Kinova), no static pair. |
| **Randomised object start position** inside a marked region, re-randomised per episode. | The region has now been **measured** rather than assumed — see §4. |
| **50 demonstrations per task; success is a binary per episode.** | This is a **human-factors study, not a policy-learning one**: the unit is a participant dyad, not a demonstration, and the outcome measures are completion time, coordination error, workload, trust and embodiment. |
| **No autonomy comparison.** ALOHA compares *learned policies* against each other. | The independent variable here is the **control mode** — direct teleoperation through to full autonomy — with a human at both ends. |

**The one thing worth copying wholesale** is the randomisation protocol: a
marked region, a re-randomised start position per episode, and a binary
success criterion evaluated at the moment of release. That is what makes a
placement task an experiment rather than a demonstration, and it is what the
colour-matched placement task is being built around.

**The one thing that cannot be copied** is ACT, and that is already decided
and recorded: this project deliberately does not use a VLA or an end-to-end
learned policy, for three reasons set out in `01_literature_review.md` §5.3 —
embodiment mismatch (no back-mounted dual-Kinova SRL exists in Open
X-Embodiment), the fact that a policy emitting actions directly **bypasses
every safety gate** this repository has between a pose and the arm, and
unexplainable failure. None of that changes because ALOHA's protocol is
adopted.

## 3. Why the TASKS cannot be reused — the SRL geometry finding

ALOHA's six tasks are: **slide ziploc, slot battery, open cup, thread velcro,
prep tape, put on shoe.**

Read them as geometry rather than as skills and they have one thing in common.
Every one requires the two grippers to act **on the same object, at the same
time, within a few centimetres of each other**: one hand holds the ziploc
while the other slides the seal; one holds the cup while the other opens it;
one presents the cable tie while the other threads it. ALOHA's two arms face
each other across a shared tabletop precisely so that this is possible — the
overlapping workspace is the design.

**This platform has no overlapping workspace at all.** Measured, over a
7 × 3 × 3 grid of the frontal volume with collision checking on:

| | cells |
| --- | --- |
| reachable by **both** arms | **0 of 63** |
| left arm only | 16 (all at x ≥ +0.4) |
| right arm only | 18 (all at x ≤ −0.4) |
| neither | 29 — including **the entire centreline**, x ∈ [−0.2, +0.2] |

With collisions *off* only 3 of 63 cells are shared kinematically, and the
wearer removes all three. A dedicated search for an inter-arm transfer point
found **0 of 16 candidates**, including the protocol's own nominated transfer
station.

So the correct statement is not "ALOHA's tasks would be hard here". It is:

> **Every task in the ALOHA set is geometrically unavailable on this
> platform, and would remain so with a perfect controller, perfect perception
> and unlimited time.**

### Why that is a finding and not a shortcoming

The disjointness is a consequence of the *configuration*, not the
implementation:

* the arms are mounted on the **wearer's back**, so both must reach forward
  over a shoulder before they can do anything at all;
* the wearer's torso occupies the centreline, which is exactly where a
  bench-top bimanual workspace would be;
* the residual asymmetry `|v_R − M·v_L| = 1.3837 m` is a property of the home
  joint angles and is **provably independent of the mount rotation** (the
  proof is in `docs/ENGINEERING_LOG.md`); it is fixed by re-parking an arm in hardware, not
  by software.

A mount sweep over 108 candidates raises the shared count from 3 to 22 of 63
kinematically — a large improvement, and still not a shared *workspace* — and
the best candidates move the arms inboard, forward and lower, i.e. **towards
the person**. Clearance and shared reach are in direct conflict on a worn
platform in a way they never are on a bench.

That is the reportable claim: **a bench-top bimanual benchmark does not
transfer to a back-mounted supernumerary pair, and the obstruction is
kinematic rather than technical.** It also explains, without any appeal to
control quality, why the SRL literature's bimanual demonstrations are
overwhelmingly *bracing* and *support* tasks rather than the fine in-hand
coordination ALOHA targets.

### What replaces them

Tasks that need both arms **without** needing both at one point. The set
already reflects this:

* **B / T3 coordinated carry** — a rigid body held at two points 500 mm apart
  *spans* the dead band. It needs both arms because no single arm can hold a
  body at two separated points; it never needs them in the same place.
* **C dual pursuit** — two targets in the two disjoint sets, satisfied
  **simultaneously**. Simultaneity, not proximity.
* **0 / A pointing** — deliberately uncoupled, per-arm target sets.

**Inter-arm handover (T4) is dropped and cannot be recovered** by any software
change. It is the one ALOHA-shaped requirement that the geometry refuses
outright.

## 4. The randomisation region, measured

ALOHA randomises the object's start position inside a marked region. The
equivalent region here is not a design choice — it is whatever both arms can
actually pick from — so it was measured before the colour-matched placement
task was designed around it:

    python3 scripts/measure_randomisation_region.py --repeats 3
    -> recordings/baselines/randomisation_region.json

Two orientations are swept, because they are **not the same region** and only
their intersection supports a cross-mode comparison:

* **ANCHOR** — the pinned wrist that `orientation_mode: fixed` gives modes 1–4.
  Its tool axis is measured at (−0.153, +0.846, +0.511): 120.8° from straight
  down and **30.7° above horizontal**, so the hand comes in from the near side
  and *below* the object. This is the only orientation the teleoperated modes
  can command — nothing in the master measures the wrist.
* **TOP-DOWN** — approach along −z, built with `grasp_library`'s own
  `_quat_from_z_and_x`, which is what modes 5 and 6 command.

Results, and the physical constraint the previous marker-based scenes were
hiding, are in `docs/system/12_randomisation_region.md`.

## 5. Citing this honestly

* Cite ALOHA for **the rig and the protocol**, and ACT separately for the
  method. Conflating them overstates what is being adopted: this project takes
  the randomisation protocol and none of the learning.
* Do **not** cite ALOHA's success rates as a comparison point. They are
  policy success rates on a bench-top pair with an overlapping workspace, from
  50 demonstrations per task, with no human in the loop at evaluation time.
  Nothing in this study is measured against that.
* When stating that the tasks were not reused, state the **0 of 63** measure
  and the mount-sweep result with it. Without the numbers it reads as an
  excuse; with them it is a characterisation of the configuration.
