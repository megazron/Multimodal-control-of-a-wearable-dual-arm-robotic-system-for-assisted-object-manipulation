# E2 — Autonomy level

**Type:** the core comparison. **Participants:** REQUIRED. Within-subjects.

## Conditions
| | what the operator does | what the autonomy does |
| --- | --- | --- |
| `direct` | everything | nothing — `srl_autonomy` is not consulted |
| `shared` | position, always; confirms the grasp | infers intent, offers an IK-validated grasp, servos wrist orientation on confirmation |
| `full_auto` | designates a target, then stops | completes the reach and grasp |

`direct` is **the baseline**, and it is our own system, not another paper's
number. Cross-paper baselines are uninterpretable here: different hardware,
task and participants.

## Why within-subjects
Between subjects, variance from operator skill on a novel wearable device
would swamp the effect. The cost is order effects, which is what the
counterbalancing and the fatigue protocol are for.

## Counterbalancing
Williams (balanced) Latin square over the three conditions — balances both
position AND immediate sequence, so carry-over from one condition cannot load
onto a particular successor. n should be a multiple of 6 for exact balance;
`check_balance()` reports the imbalance at the actual n and it goes in the
manifest.

## Procedure
1. Consent, screening, demographics (age band only).
2. Practice block, `direct`, 6 trials, not analysed.
3. Three blocks in the counterbalanced order, 12 trials each.
4. **NASA-TLX and the trust scale after every block**, before the rest.
5. Borg fatigue probe between blocks. Blocks ≤ 8 min; total session ≤ 45 min.

## Pre-registered hypotheses
- **H2.1** Completion time differs by autonomy level. Test: one-way repeated-
  measures ANOVA on participant means; **Friedman** if Shapiro–Wilk rejects
  normality. Holm–Bonferroni for the three pairwise follow-ups.
- **H2.2** Grasp success is highest in `shared`. Predicted direction: shared >
  direct; shared ≥ full_auto. Test: Cochran's Q, then McNemar pairwise.
- **H2.3** NASA-TLX raw workload is lowest in `shared`. Predicted: shared <
  direct. Test: Friedman + Wilcoxon signed-rank.
- **H2.4** Intervention count is higher in `full_auto` than `shared` — the
  cost of removing the operator from the loop.

## Target n and power
n = 12 (two full Williams squares). With a within-subject correlation of ~0.6,
that detects d ≈ 0.9 at 80% power. **The effect size is a guess until pilot
data exists** and is recorded here so it can be checked against the pilot
rather than quietly revised afterwards.

## Stop / invalidate
45 s timeout. INVALID on channel dropout or e-stop.
