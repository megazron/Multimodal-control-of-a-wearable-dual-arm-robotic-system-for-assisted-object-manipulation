# T9 — REACH UNDER WEARER MOTION

**The wearer sways to a metronome while the operator works. Sway amplitude is
the independent variable. Does autonomy compensate a disturbance the operator
cannot feel?**

---

## Why this task, and an honest correction to its framing

This was briefed to me as "the strongest hypothesis available and no one has
tested it." **The second half is not right, and the task is better for the
correction.**

Motion compensation for supernumerary arms under human-induced disturbance
**has** been studied as a control problem. Zhang et al. (2024, *Advanced
Intelligent Systems*) model SRAs on a floating base and report tracking errors
of **1.37 ± 0.58 mm with the human shoulder as the floating base**; a related
framework (arXiv:2310.10029) reconstructs the arm Jacobian against IMU-derived
skeleton feedback.

**In every one of those, the wearer IS the operator.** The disturbance is
*self-generated*: a person about to sway knows they are about to sway, and
their own motor commands are available as feed-forward.

**What is untested is the two-person case.** Split the roles and the operator
loses all of it — no efference copy, no vestibular access, and the disturbance
visible only through a camera that is *itself mounted on the moving base*.
That is the condition this task creates, and it is created by the architecture
rather than contrived for the experiment.

Because the control problem is known to be solvable, **a null result here is
informative rather than a failure to build the thing** — which is the property
that makes this a good experiment rather than a demonstration.

---

## Verified scenarios — 4 of 4, and the IV is CAPPED

Targets at **(±0.35, 0.35, 1.15)**. Not the T7 centre: at z = 1.10 the left
arm tolerates only 60 mm of sway before the envelope leaves its reachable set.
z = 1.15 gives 100 mm on both arms.

| scenario | amplitude | envelope (L, R) | what it is |
| --- | --- | --- | --- |
| S1_sway_000mm | 0 mm | 1/1, 1/1 | static control |
| S2_sway_020mm | 20 mm | 8/8, 8/8 | quiet-standing postural sway |
| S3_sway_060mm | 60 mm | 8/8, 8/8 | gentle weight shift |
| S4_sway_100mm | 100 mm | 8/8, 8/8 | deliberate weight shift |

**160 mm ("step in place") was tested and REMOVED.** The sway envelope leaves
the reachable set at every centre probed. Keeping it would have lost trials to
geometry and scored them as disturbance effects, which is exactly backwards.
**The IV therefore spans quiet sway to a deliberate weight shift, and NOT a
step** — a real weakening of the top of the range, recorded rather than
quietly dropped.

**Metronome: 30 / 50 / 70 bpm**, chosen to straddle plausible visual-tracking
bandwidth on both sides, so H2-null is falsifiable rather than assumed away.

---

## Trial structure

One trial = 30 s of continuous tracking.

1. The wearer is given the metronome and an amplitude cue (floor marks at the
   commanded excursion), and starts swaying. **10 s of settling is logged and
   not scored.**
2. The operator tracks a **world-fixed** target. The target does not move; the
   base does. In the arm's own frame the target orbits, which is the whole
   task.
3. Trial ends at 30 s.

**Conditions** — 3: DIRECT / ASSISTED / SHARED, crossed with the 4 amplitudes.

**The wearer is seated, or standing with a rail within reach.** Swaying to a
metronome under >17 kg is a balance risk in a way that the same motion unloaded
is not.

---

## Metrics

| metric | definition |
| --- | --- |
| **`rms_error_mm`** | EE against the world-fixed target. The DV |
| `phase_lag_s` | EE against the **WEARER'S** motion, not the target's. This is what separates *compensation* from *following* |
| **`wearer_motion_rms_mm`, `wearer_motion_hz`** | from the wearer's torso IMU. **The manipulation check** |
| `sway_amplitude_mm`, `metronome_bpm` | what was commanded |
| clearance minimum | hygiene — and the reason the amplitude is capped |

---

## The hypothesis is an INTERACTION

```
error ~ b0 + b_auto*autonomy + b_sway*amplitude + b_int*(autonomy x amplitude)
```

**`b_int` is the finding.** A main effect of autonomy would be unsurprising
and could come from anywhere. The prediction specific to a two-person system
is that autonomy's advantage **grows** with amplitude, so `b_int` is negative.

**H2-null is pre-registered:** if the operator can simply watch the sway and
compensate visually at these amplitudes and rates, the interaction is absent.
That is an informative result and `analyse_t9.py` prints it as one.

---

## THE MANIPULATION CHECK CAN VETO THE ANALYSIS

`sway_amplitude_mm` is what the metronome **asked** for. `wearer_motion_rms_mm`
is what the wearer's IMU **measured**. If the second does not track the first,
the independent variable did not happen and nothing downstream means anything.

**A tiring wearer produces a beautiful null result that has nothing to do with
autonomy.** The check requires all three of:

- correlation `r > 0.7` between commanded and measured,
- **slope > 0.5** — measured must actually grow with commanded,
- **span coverage > 50%** of the commanded range.

Correlation alone is not enough, and a known-answer test is what showed it: a
wearer who plateaus at 20 mm while being asked for 20 / 60 / 100 still
correlates **r = +0.71**, because a plateau is monotonic. See
`test_t9_interaction_known_answer.py`.

---

## Blocked on wiring

**The wearer's torso IMU does not exist yet.** Until it publishes,
`wearer_motion_rms_mm` is empty, the manipulation check cannot pass, and the
analyser correctly refuses to call any result confirmatory. **T9 is not
participant-ready without it** — and it is the cheapest of the outstanding
sensors.
