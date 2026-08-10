# E1 — Fitts characterisation of the degraded master

**Type:** system evaluation. **Participants:** NOT required — the experimenter
can run it, and `--scripted` runs it with no human at all.

## Question
What does the DOF-deficient wearable master cost, in a unit the wider
literature already uses?

## Why it exists
Every other result here is internal to our own hardware. Throughput in bits/s
is the one figure another group can compare against without owning this rig.
It is also the honest way to state the size of the problem the autonomy
addresses — without it, "the master is bad" is an assertion.

## Design
- D ∈ {0.06, 0.10, 0.16, 0.24} m × W ∈ {0.015, 0.025, 0.040, 0.060} m → 16
  cells; ID = log2(D/W + 1) spans ≈1.0–4.1 bits.
- 3 repeats per cell, de-clustered so a cell never repeats back-to-back.
- Targets are placed on a circle at the requested distance, sampling 8
  directions, so a direction bias cannot masquerade as a difficulty effect.
- **No grasping.** Isolates POINTING cost from GRASPING cost.
- Distances span the MASTER's measured 0.22 m ball, not the robot's workspace.

## Procedure
1. `bash scripts/diagnostics.sh` — controllers active, `/joint_states` ~100 Hz,
   master port present.
2. Seat the operator, master at neutral, arm supported.
3. `bash scripts/run_experiment.sh e1 --participant P01`
4. Blocks of ≤ 8 minutes with 2-minute rests; Borg fatigue probe between
   blocks.

## Measures
Movement time, endpoint error, path length vs straight line, per-trial channel
liveness.

## Pre-registered hypotheses
- **H1.1** MT is linear in ID with R² > 0.8. *If not supported, the throughput
  figure must not be reported* — the master would not be behaving as a
  pointing device at all.
- **H1.2** Throughput is below the 3.7–4.9 bits/s reported for mouse pointing.
  Predicted direction: lower. Reported descriptively against the published
  range, not as a significance test against another study's population.

## Stop / invalidate
20 s timeout. INVALID on any dropout of j1/j2/j4/accel mid-trial, or any e-stop.
