# E6 — Mannequin vs VR: does autonomy recover capability lost to input deficiency?

**Type:** the experiment that isolates the central claim. **Participants:** REQUIRED.

## Why this is the strongest design available here

Same robot. Same task. Same scenarios. Same autonomy code. Same metrics. Two
input devices that differ in exactly one scientifically meaningful way:

| | mannequin master | Quest controller |
| --- | --- | --- |
| position | pots only, 3 load-bearing channels | 6-DOF tracked |
| **orientation** | **roll/pitch from gravity; YAW NOT OBSERVABLE** | full, tracked |
| rate | 50 Hz | 72 Hz |
| dead channels | j3, j5, j7 dead; j4 12.9% dropout | none |
| commands orientation | **no** (`orientation_mode: fixed`) | **yes** |

Everything downstream of the input is byte-identical: the same
`ik_follower_node`, the same collision checking, the same e-stop, and
`srl_autonomy` running unmodified — `vr_intent_source` publishes the same
`/master_pointing_<arm>` topic the IMU path publishes, so the inference code
is not merely equivalent, it is the same code.

## Design

2 (input: mannequin, VR) × 2 (autonomy: off, on), fully within-subjects.
4 blocks. Scenarios reused verbatim from E2/E4 so the numbers are comparable
across experiments, not just within this one.

## Counterbalancing

**A Williams square over the four conditions**, which balances both position
and immediate sequence. n should be a multiple of 4 for exact balance;
`check_balance()` reports the imbalance at the actual n and it goes in the
manifest.

Two additional constraints, both because of asymmetries between the devices:

1. **Never alternate input device trial-by-trial.** Donning and doffing a
   headset takes ~30 s and would dominate the timing. Device is a BLOCK
   factor; autonomy alternates within a device block.
2. **Balance which device comes first across participants**, because the
   second device inherits task learning from the first. With device as a
   block factor and a Williams order, this is automatic at n multiple of 4;
   below that it must be checked and reported.

## Pre-registered hypotheses

- **H6.1 (the central claim)** There is an **input × autonomy interaction** on
  grasp success: autonomy improves the mannequin condition MORE than the VR
  condition. Predicted direction: the interaction is positive. Test: 2×2
  repeated-measures ANOVA, or a logistic mixed model with participant as a
  random effect for the binary outcome; Aligned Rank Transform if residuals
  are non-normal. **This is the hypothesis of record.**
- **H6.2** In the `orientation-critical` scenarios inherited from E4
  (`side_facing`, `inverted`), mannequin-without-autonomy fails at ≈0 while
  all three other cells succeed. Test: McNemar per scenario, Holm-corrected.
  This is the categorical form of the same claim.
- **H6.3** VR-without-autonomy outperforms mannequin-with-autonomy on
  completion time. If it does NOT, that is the more interesting result: it
  would mean autonomy fully compensates the DOF deficit, and it should be
  reported as such rather than buried.
- **H6.4** NASA-TLX is lowest in VR+autonomy and highest in
  mannequin-without. Test: Friedman, then Wilcoxon signed-rank, Holm.

## The confound this design must not have

VR is better in *several* ways at once — 6-DOF, higher rate, no dead channels.
So a raw "VR beats mannequin" result does not isolate the DOF deficit. The
**interaction** does, which is why H6.1 and not a main effect is the
hypothesis of record: an input-quality main effect would be produced by any of
those differences, but "autonomy helps the deficient device more" is
specifically a statement about what autonomy is compensating for.

A secondary control, cheap and worth running: a VR condition with orientation
command **disabled in software** (`command_orientation:=false`), which
reproduces the mannequin's structural deficit on the good hardware. If that
condition behaves like the mannequin, the deficit — not the hardware — is what
the autonomy is compensating.

## Measures

Identical to E2 and E4: completion time, grasp success, positioning error,
failed attempts, path ratio, time to handover, intervention count, the full
intent distribution over time, NASA-TLX, trust, Borg fatigue. Plus, per trial,
the input device and which channels were live.

## Practical notes

- Total ≈ 60 min including two device changes. That exceeds the 45-minute cap
  from the fatigue protocol, so **E6 runs as two sessions on different days**,
  device order counterbalanced across participants.
- The VR condition needs the observer e-stop confirmed before it will enable
  (`vr_safety_node` refuses otherwise). The mannequin condition does not, but
  use one anyway so the two conditions differ only in the input device.
