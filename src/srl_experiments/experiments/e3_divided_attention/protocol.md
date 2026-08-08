# E3 — Divided attention

**Type:** the SRL-specific experiment. **Participants:** REQUIRED.

## Question
Does shared autonomy help MORE when the operator's own hands and eyes are
busy? **The interaction term is the result of record**, not the main effects.

## Why this is the contribution
A supernumerary limb is not a better teleoperator's arm. It is an extra limb
used *while the operator is doing something else with their own two*. Every
result in E2 could in principle be obtained with a desk-mounted arm; this one
could not.

## Design
E2's three autonomy levels × {single_task, dual_task}. Fully within-subjects.

## The primary task
**Continuous manual tracking**, not an n-back. It loads the same manual and
visual channel the SRL competes for, which is the competition an SRL actually
creates; an n-back would load working memory instead and would answer a
different question.

Tracking speed is **fixed across conditions**. If it adapted, primary-task
load would differ by condition and the interaction would be uninterpretable.
Primary-task error is logged alongside the SRL task, because an operator can
always protect one task at the other's expense — reporting only the SRL task
would let a participant "improve" by abandoning the primary task.

## Pre-registered hypotheses
- **H3.1 (the hypothesis of record)** There is an autonomy × attention
  interaction on completion time: the benefit of `shared` over `direct` is
  LARGER under dual-task. Test: two-way repeated-measures ANOVA; **Aligned
  Rank Transform** if residuals are non-normal.
- **H3.2** Primary-task error is lower under `shared` than `direct` in the
  dual-task condition. This is the operational claim — the SRL costs less
  attention.
- **H3.3** NASA-TLX shows a main effect of attention and an autonomy ×
  attention interaction in the same direction as H3.1.

## Threat specific to this experiment
Participants may protect the primary task and let the SRL task suffer, or the
reverse, and the strategy may itself differ by condition. Both tasks'
performance is therefore reported together, and any condition where one task
is abandoned is flagged.

## Stop / invalidate
As E2, plus: INVALID if the primary task is not attempted for >20% of a trial.
