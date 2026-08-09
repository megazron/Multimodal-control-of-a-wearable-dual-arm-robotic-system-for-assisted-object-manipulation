# Experimental protocol

> **SUPERSEDED IN PART, 2026-08-08 — TWO-PERSON REFRAMING.** The arms are worn
> by one person and driven by a **different** person. This document's task
> descriptions, geometry and metrics remain correct; what changed is who is in
> the room and what is measured about them. Read alongside, and where they
> disagree defer to:
>
> | | |
> | --- | --- |
> | **`10_tasks_abc_and_two_hour_session.md`** | **CURRENT.** Tasks A/B/C, verified N=10 over the full path, and the 116-minute session. Supersedes the five-task set and the 165-minute session for the participant study. |
> | `06_task_set_two_person.md` | the revised task set, **T8** and **T9**, T5 promoted to core, feasibility with numbers |
> | `05_two_person_measures.md` | wearer, operator and dyadic measures; role counterbalancing |
> | `07_session_order_two_person.md` | the 165-minute two-person session |
> | `01_literature_review.md` §7 | Fusion, SRL Proxemics, and the corrected gap |
> | `02_baseline_and_hypotheses.md` §6 | H1-H4 for the dyad |
> | `03_ethics_and_safety.md` | separate consent, the wearer's stop, stops as data |
>
> **The T2-vs-T5 slot argument is resolved and reversed.** This document
> records keeping T5 over T2 as a close call decided on framing. Under the
> two-person reframing it is not close: T5 is the canonical Fusion scenario
> and the only task in which the wearer has a functional role. **Both are in**,
> and the session grew to hold them.

All coordinates are world frame: **+x is the wearer's RIGHT, +y is FORWARD,
+z is UP** (this repo's convention, *not* ROS x-forward). Every position below
was confirmed against a live `/compute_ik` with the arms at home — see
`scenarios_verified.yaml`, 17 of 17 verified.

---

## What survived the geometry, and what did not

| task | status | why |
| --- | --- | --- |
| **T3 rigid carry** | **IN** — respec'd | feasible band is z 1.10–1.30 m, not the 0.885 m table. Transport must be **vertical**; lateral and fore-aft carry leave the band |
| **T6 compliant carry** | **IN** — new | same paths, sling instead of tray |
| **T7 pursuit** | **IN** — new | needs no shared point, no objects, no vision. Works regardless of hardware state |
| **T5 handover to wearer** | **IN** | single arm |
| **T2 hold and fill** | **IN, if the object is redesigned** — see below | |
| T1 bimanual reach | subsumed by T7 | |
| T4 inter-arm handover | **BLOCKED** | 0 of 16 transfer points reachable by both arms. Geometry, not orientation |

### T2 — feasible, and the container spec is now MEASURED

**CORRECTED 2026-08-08.** An earlier version of this section gave the pair as
*"left holds (+0.10, 0.35, 1.15), right releases (-0.10, 0.35, 1.20)"* with a
200 mm / 50 mm container. **All four figures were wrong.** Re-measured with
both arms verified at home (0.0000 rad offset):

* **nobody** reaches (+0.10, 0.35, 1.15) — 0 of 5 attempts, either arm;
* the arm assignment is **swapped**: the RIGHT arm holds;
* `x = ±0.10` is the margin of each arm's span, passing a single IK call and
  failing a repeated one (0/5 to 4/5 by height). `x = ±0.15` is 5/5 at every
  height tested.

The old figures came from a sweep taken with the arms displaced from home —
the same measured-from-the-wrong-pose error that once invalidated a whole
reachability run.

Measured spans at y = 0.35:  **left x 0.10 … 0.55, right x −0.55 … −0.10.**

**Verified geometry:**

```
   RIGHT holds the handle          LEFT drops the block in
        (-0.15, 0.35, 1.10)             (+0.15, 0.35, 1.20)
              |                                |
              |<---------- 300 mm ------------>|
                        100 mm rise
```

**Object spec:** an open-top container whose **opening centre is 300 mm
horizontally from its grasp handle and 100 mm above it** — a box on a long
side handle, like a dustpan. The holding gripper sits well clear of the
opening, and that clearance is what accommodates the arms' separation.

**Four scenarios, 4 of 4 verified**, each with the complete fill path
(pick → lift → transit → above-opening → release → retreat) checked at every
waypoint *while the holding arm is simultaneously solvable at the hold pose*:

| scenario | pick | release | block | tolerance | difficulty |
| --- | --- | --- | --- | --- | --- |
| S1 short reach | (+0.30, 0.35, 1.15) | (+0.15, 0.35, 1.20) | 40 mm | 40 mm | pick close to the opening |
| S2 long reach | (+0.45, 0.35, 1.15) | (+0.15, 0.35, 1.20) | 40 mm | 35 mm | longer transit |
| S3 height change | (+0.35, 0.35, 1.15) | (+0.15, 0.35, 1.30) | 40 mm | 35 mm | container held 100 mm higher |
| S4 tight tolerance | (+0.35, 0.35, 1.25) | (+0.15, 0.35, 1.20) | 26 mm | 18 mm | half the placement margin |

Regenerate with `python3 scripts/verify_t2_scenarios.py` (needs the sim up and
the arms at home).

**Still not instrumented:** the discrete outcome T2 exists for — blocks
placed, blocks dropped. The runner logs trajectories and timing and says so;
do not report a success rate from it yet.

### The T2-vs-T5 trade for the P6 slot — DECISION RECORDED

Adding T2 exceeds 75 min, so it can only take P6, currently T5. Both sides,
so the choice stays visible later:

**For T2.** A fourth bimanual condition with a **discrete success/failure**
outcome (blocks placed, blocks dropped). Every other surviving task —
T3, T6, T7 — yields a *continuous* error signal, so T2 is the only source of
a different KIND of measure, and a discrete outcome is what a reader
unfamiliar with coordination metrics will actually understand.

**For T5.** It is the only task in which the robot **serves the wearer** —
the defining use case of a supernumerary limb, and the one condition where
the embodiment measures have a functional referent rather than an aesthetic
one. It is also the only source of handover latency, and the only single-arm
task, so it is the only block that still runs if the bimanual geometry
degrades further.

**My view: I do not agree that T2 is clearly the better use of the slot, and
I would keep T5.** The reasoning is that the study's framing is *wearable*
supernumerary limbs with embodiment as a headline measure, and dropping the
only wearer-directed task to add a fourth coordination task optimises for the
secondary claim at the cost of the primary one. T2's discrete outcome is
genuinely valuable, but T3/T6 already produce a binary "object retained"
alongside their continuous error, so the discrete measure is not absent
without it.

**Counter-argument, fairly stated:** T5 has no *within-task* manipulation
that is interesting — the three autonomy conditions differ little when one
arm does a scripted pick — whereas T2 is a genuine bimanual coordination
problem with a redesigned object that is cheap to fabricate. If the paper's
claim turns out to be about coordination under divided attention rather than
about embodiment, T2 is the better block and this decision should be revisited.

**Recorded so the choice is auditable:** the session below keeps **T5 at P6**.
If T2 replaces it, note here that it was a deliberate reversal and why.

---

## Design

Within-subjects. Three autonomy conditions per task:

| condition | T3 / T6 | T7 |
| --- | --- | --- |
| **DIRECT** | operator drives both arms | operator tracks both targets |
| **ASSISTED** | autonomy holds one end, operator the other | autonomy tracks one target |
| **SHARED** | autonomy drives both, operator supervises | autonomy tracks both |

**Latin-square counterbalanced WITHIN each task**, not across. Across-task
counterbalancing would confound condition order with the fatigue drift, which
is monotonic and unavoidable here.

## Session order — 75 min without P6

The primary measure must not be taken last.

| | block | min |
| --- | --- | --- |
| P0 | consent, briefing, embodiment baseline, proprioceptive drift | 10 |
| P1 | familiarisation, practice to criterion | 10 |
| P2 | **T7 pursuit** — no objects, warms up, yields real data | 10 |
| P3 | **T3 rigid carry — PRIMARY**, freshest participant | 15 |
| P4 | break, harness off, Borg CR10 | 5 |
| P5 | **T6 compliant carry** — the contrast with T3 | 15 |
| P6 | T5 handover to wearer, *if time* | 8 |
| P7 | post-session measures | 10 |

**Practice criterion (P1):** three consecutive DIRECT trials of T7 S1 with
per-arm RMS error below 60 mm. Stated so it is applied identically.

---

## Verbatim instruction scripts

Read exactly as written. Do not paraphrase, do not add encouragement, do not
comment on performance.

### T7 — pursuit tracking

> "You will see two markers, one on each side. Each arm follows the marker on
> its own side. Keep the gripper as close to its marker as you can, for the
> whole trial. The markers move at different speeds and the speeds change
> between trials. Both arms matter equally — do not favour one. If you lose a
> marker, carry on and re-acquire it. The trial ends on its own."

Unimanual baseline trials (B1, B2):

> "This time only one arm has a marker. Follow it exactly as before. The other
> arm will hold still on its own."

### T3 — rigid coupled carry

> "The two grippers are holding one rigid tray. A ball rests on it. Move the
> tray from the low position to the high position, keeping it level. If the
> tray tilts too far the ball will roll off. Take as long as you need — speed
> is not being measured. Tell me if you want to stop."

### T6 — compliant coupled carry

> "This is the same movement, but the two grippers now hold a flexible sling
> with the ball resting in it. Move it from the low position to the high
> position. The sling will absorb small differences between your hands, but if
> you pull the ends too far apart it will flatten and the ball will escape.
> Take as long as you need."

### T5 — handover to the wearer

> "The arm will pick up the tool and bring it to you. When it stops in front
> of you, take it and press the button. If it does not stop where you can
> reach it, say so and we will stop the trial."

### Unexpected-failure trial (once per participant, not announced)

No script. Autonomy is silently disabled mid-trial in one ASSISTED trial. The
participant is told only afterwards:

> "In one of those trials the assistance was switched off without telling you.
> That was planned, and how you responded is part of what we are measuring."

---

## The T6 object — exact specification

    ┌─────────────────────────────────────────────┐
    │  ◄──────────────  350 mm  ──────────────►   │
    │                                             │
    │  ┌────┐                             ┌────┐  │
    │  │tab │═════════════════════════════│tab │  │   ← stiff fabric or
    │  └────┘         sling body          └────┘  │     0.5 mm polypropylene
    │   80×           40 mm wide           80×    │
    │   25 mm                              25 mm  │
    └─────────────────────────────────────────────┘
                          ●  40 mm ball rests in the sag

- **Sling length 350 mm** end to end, between the inner edges of the tabs.
- **Grip tabs 80 × 25 mm**, one at each end, sized for the 85 mm Robotiq
  gripper closing across the 25 mm dimension.
- **Ball 40 mm diameter**, free-rolling, not attached.
- Body 40 mm wide, stiff enough not to twist but free to hang.

**Why 350 mm, verified against the arms' actual band:**

    sag(s) = sqrt((L/2)^2 - (s/2)^2);  ball retained while sag >= 2r
    geometric retention limit               s <= 340.7 mm
    MEASURED reachable separation           280 - 360 mm
    MEASURED retained AND reachable         280 - 340 mm
    nominal separation for the protocol     310 mm

350 mm is correct **precisely because the 340 mm failure threshold sits inside
the reachable band**. A longer sling would put failure out of reach and the
task could never fail, measuring nothing. The arms cannot close below 280 mm,
so the operator works near the threshold — a thin margin, deliberately.

---

## Metrics — which apply where

The original list came from a pick-and-place framing. Only T5 still does
pick-and-place, so grasp success is **not** forced onto the coordination
tasks.

**All tasks:** completion time; NASA-TLX per condition; Borg CR10 between
blocks; trust-in-automation scale; intervention count; **elapsed wear time per
trial**, so fatigue enters the model as a covariate rather than as noise.

**T3 and T6 (headline: coordination error, continuous):** tilt (T3) and
separation error (T6), RMS and max; gripper height difference, signed,
because the ball escapes to the low side; path efficiency; object retained
(binary); time above the failure threshold.

**T7 (headline: cross-arm interference):** per-arm RMS and max tracking error;
**interference coefficient**; phase lag per arm; time on target within a
40 mm radius.

**T5 only:** grasp success rate; positioning error at grasp; failed attempts;
handover latency from arrival to the operator's button press.

**Embodiment — all tasks.** Ownership/agency questionnaire adapted from the
rubber-hand literature, before and after, plus proprioceptive drift as an
objective proxy.

> **Predicted tension, stated in advance:** autonomy may IMPROVE performance
> while REDUCING ownership. Both outcomes are publishable, and pre-registering
> the prediction is what stops either being presented as the expected one
> after the fact.

**Situation awareness:** one unexpected-failure trial per participant, with
autonomy silently disabled. Measure recovery time. This is the honest
counterweight to a positive result.

---

## The interference measure, in comparable terms

The motor-control literature quantifies bimanual interference as the
difference between a task performed alone and the same task performed
alongside another. Two measures are reported so the result is comparable:

**1. Dual-task cost**, against the B1/B2 unimanual baselines:

    cost = (error_bimanual - error_unimanual) / error_unimanual

**2. Cross-arm interference coefficient**, fitted over trials:

    err_A ~ b0 + b_self * speed_A + b_cross * speed_B
                + b_int * speed_A * speed_B

`b_cross` is the headline. **It is defined as the TOTAL speed sensitivity of
arm A's RMS tracking error to arm B's target speed**, in mm of RMS error per
(m/s) of the other arm's target.

"Total" is load-bearing and must be carried into the write-up. Tracking error
has at least two components and **both grow with target speed**:

- an **amplitude** component — the operator tracks less accurately when
  attention is divided;
- a **lag** component — any fixed reaction delay `L` becomes positional error
  `v·L`, because the target has moved that far by the time the hand arrives.

**From RMS error alone these are not separable**, here or in any other
experiment: they produce the same statistic. So `b_cross` is the total, and
reading it as "the attentional cost" alone would overstate it by whatever the
lag contributes. **`phase_lag_s` is reported alongside it for exactly this
reason:** if `b_cross` is large while the phase lag is flat in the other arm's
speed, the effect is attentional; if both rise together, it is not.

**Pipeline validated end to end.** `scripts/pilot_bimanual.py` runs a
calibration block in which RMS error is linear in the two speeds by
construction, and recovers the injected coefficients exactly:

    injected   b_self = 90.0   b_cross = 60.0   mm per (m/s)
    recovered  b_self = 90.0   b_cross = 60.0   R2 = 1.0000

A unit test recovering 7.5 from 7.5 proves the arithmetic; this proves the
pipeline. The realistic-operator generator in the same pilot fits larger
values (358 / 289) because its error combines amplitude and lag nonlinearly —
that is the definition working as intended, not a discrepancy.

**This is identifiable only because the two speeds vary INDEPENDENTLY across
trials.** With the speeds yoked, `b_self` and `b_cross` are collinear and
neither can be estimated — `test_yoked_speeds_cannot_identify_interference`
pins that, so the design cannot be quietly simplified into
uninterpretability.
