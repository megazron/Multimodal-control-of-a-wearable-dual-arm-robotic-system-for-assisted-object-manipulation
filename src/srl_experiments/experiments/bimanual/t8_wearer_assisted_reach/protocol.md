# T8 — WEARER-ASSISTED REACH

**A target the operator cannot reach at the wearer's nominal stance, and can
reach once the wearer repositions. Neither person can complete it alone.**

This task does not exist in the single-person literature, because in the
single-person case the person who would reposition is the person already
driving the arm — they simply lean, and there is no coordination to measure.
Splitting the roles creates a class of task where the limiting resource is
**communication between two people**, and that is what T8 measures.

---

## Why the task is well-posed

The operator **cannot** reach the target: verified, at N=10 repeats, that IK
fails at the wearer's nominal stance. The wearer **cannot** manipulate: they
have no control input to the arms at all. So the trial can only succeed if
they coordinate, and a trial that succeeds is evidence of coordination rather
than of either person's skill.

---

## Verified scenarios — 4 of 4, two per arm, three distinct stances

| scenario | arm | target (x, y, z) | the wearer must | approach path |
| --- | --- | --- | --- | --- |
| S1_left_lean_forward | left | (+0.55, 0.35, 0.95) | lean forward ~20° | 16/16 |
| S2_left_step_forward | left | (+0.55, 0.45, 0.95) | step forward 0.30 m | 19/19 |
| S3_right_crouch | right | (−0.55, 0.45, 0.95) | crouch 0.20 m | 14/14 |
| S4_right_step_forward | right | (−0.55, 0.55, 0.95) | step forward 0.30 m | 17/17 |

Generated and verified by `scripts/verify_t8_t9_scenarios.py`. Every target is
confirmed **unreachable at nominal** — that is the defining property, not a
side effect — and every approach path is verified densified to 20 mm after the
reposition.

**The first version of the search returned four left-arm lean-forward
variants.** That would have measured one situation four times and called it a
graded task set; the substance of T8 is *who repositions and how*, so both are
now balanced by construction.

---

## Trial structure

One trial = one assisted reach.

1. Both are told the target's location. **Neither is told whose move it is.**
   This is deliberate: who initiates is a dependent variable, and instructing
   it would destroy the measure.
2. The operator attempts the reach. `attempts_before_request` counts how long
   they try alone.
3. At some point the operator requests a reposition, **or** the wearer
   volunteers one. `t_operator_requests`, `t_wearer_starts_moving`,
   `t_wearer_completes_reposition`.
4. **The operator must hold still while the wearer repositions.** Briefed
   explicitly: an arm moving toward a person who is themselves moving is the
   configuration with the least margin, and the wearer cannot see it.
5. The operator completes the reach. Trial ends on the target being touched
   or at `timeout_s` (60 s).

**8 repetitions per condition**, so the learning trend is estimable *within* a
block rather than only across blocks.

**Conditions** — 3, as elsewhere: DIRECT / ASSISTED / SHARED.

---

## Metrics

| metric | definition |
| --- | --- |
| **`anticipation_s`** | `t_wearer_starts_moving − t_operator_requests`. **NEGATIVE = the wearer moved before being asked.** The headline |
| `coordination_latency_s` | `t_wearer_completes_reposition − t_operator_requests` |
| `initiator` | operator \| wearer |
| `attempts_before_request` | operator-side awareness: how long before they recognised they could not reach |
| `repositions_per_trial` | over-correction check — see below |
| `comm_events` | utterances, coded request / acknowledgement / warning / social |
| `wearer_motion_rms_mm` | from the wearer's torso IMU |
| clearance minimum, tracking | hygiene, from the robot |

**Latency alone would mislead.** A dyad can cut coordination latency by the
wearer repositioning constantly and pre-emptively, which is not learning and
is worse for the wearer. A latency improvement accompanied by a rise in
`repositions_per_trial` is reported as over-correction, not coordination.

---

## The hypothesis

**H3.** Across repetitions, `coordination_latency_s` falls and
`anticipation_s` turns negative.

A negative anticipation means the wearer predicted the operator's intent
**while having no control over the limbs and no proprioception of them** —
body-schema-like learning by someone who is not driving. That is the
substantive claim, and it is why `analyse_t8.py` reports the sign before the
magnitude.

---

## Safety

The reposition is **completed and confirmed before the operator is cleared to
approach**. The wearer moves, says so, and only then does the arm move. This
ordering is not a nicety: the wearer cannot see the arms, and a reposition
into a moving arm is the worst configuration the task can produce.

Stances are drawn from a verified list; the wearer is never asked to
improvise a posture.

---

## Known limitation

Verification simulates the wearer's stance by **transforming targets**, which
is exactly equivalent kinematically but does **not** move the wearer's own
collision geometry. On the real rig, a wearer who leans *into* the workspace
brings their torso with them. Every T8 pose is therefore also checked at the
nominal stance with collisions on, and **the real-rig clearance must be
re-measured with the wearer actually leaning** before a participant runs this.
