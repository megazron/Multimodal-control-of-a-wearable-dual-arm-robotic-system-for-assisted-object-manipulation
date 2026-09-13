# EVALUATION CRITERIA E'1–E'9

*The thesis's own success criteria, matching what was delivered. The planning
report's E1–E9 are reported separately in the Discussion as a scope analysis,
not as the thesis's criteria.*

Each threshold is justified from the platform's own constraints or from
literature. Where a criterion is met, the evidence file is named.

| # | Criterion | Definition | Method | Threshold and its justification | Status |
|---|---|---|---|---|---|
| **E'1** | Convention round-trip fidelity | $\max\lvert\theta - g^{-1}(g(\theta))\rvert$ over $[0,360)$ for the Kortex↔ROS conversion | Sweep at 0.01° through `kortex_convention` | **< 1e-9 rad.** Pure arithmetic: any deviation is a wrapping defect, and the failure mode is a near-full revolution beside a person. There is no tolerance to spend | **NOT YET RUN** (trivial) |
| **E'2** | Directional fidelity of the mapping | Angle between commanded EE displacement and the operator's intended world axis | Replay the 12 labelled hardware trajectories (27 Jul 2026) through the live mapping | **< 30° per axis**, and reversal pairs within 30° of 180°. 30° is the point beyond which an operator must consciously correct rather than move naturally; the existing reversal figures (12.6° left, 26.2° right) show the bar is achievable on the healthy axes | **PARTIAL** — up/down and fore/aft pass; lateral does not |
| **E'3** | Command tracking in simulation | RMS $\lVert\mathbf{p}_{cmd}-\mathbf{p}_{EE}\rVert$ over a trial | 87 recorded verification runs | **< 5 mm.** The pose deadband is 2 mm and the step guard 0.35 rad; below 5 mm the residual is dominated by the deadband rather than the controller | **MET** — 0.7–4.7 mm |
| **E'4** | Wearer clearance | Minimum distance, any distal arm link to any wearer primitive | `clearance.py` from TF, every frame of every run | **≥ 120 mm** in `real_robot` mode. The platform's own configured floor; corroborated by near-body SRL guidance reporting a palm-width (~100 mm) torso buffer preference [15] | **NOT MET** — 48 of 87 runs below it; see \cref{ch:workspace} |
| **E'5** | Safety response completeness | Fraction of injected faults detected, responded to, and made externally visible | `fault_injector`, 14 faults | **100 %.** A mechanism that handles 13 of 14 is not a safety mechanism; there is no defensible partial credit for a robot beside a head | **MET** — 14/14 |
| **E'6** | Degraded-mode integrity | Commanded tip motion attributable to failed channels from a still hand; and whether the system refuses when no positional channel survives | Replay recorded traces with channels frozen | **< 1 mm** contributed motion (the pose deadband — below it the contribution cannot command motion), **and** refusal must occur when all positional channels are lost | **PARTIAL** — right arm 0.014 mm passes, left arm 54.5 mm fails; refusal implemented |
| **E'7** | Scenario reachability | Every declared coordinate reachable over the whole path | Audit at N=10 repeats, 20 mm densification | **100 % $N/N$.** Measurement showed a 72–82 % feasible pose passes a 5/5 check 19 % of the time, so anything short of $N/N$ is not an established pose; and a scenario failing on the day wastes a participant session | **MET** — 17/17 scenarios, 17/17 protocol coordinates |
| **E'8** | Control-path latency | $t_6-t_0$, per stage and end to end | Seven-point instrumentation (DIVERGENCE.md Part F) | **< 200 ms**, inherited from the planning report and consistent with the ~120 ms reported for direct master–slave mapping [12] | **NOT YET MEASURED** — highest-value gap |
| **E'9** | Object-handling verification | Gripper opens, closes within the holding band, reopens; and the object tracks the gripper | `grip_trace.json` plus pixel analysis of the recorded views | **100 % of grasp clips.** A binary property of a correct demonstration; a partial pass means some clips show a grasp that did not happen | **MET** — 45/45 fingers cycle, 45 objects carried, 0 static |

## Two criteria deliberately absent

**No force or impedance criterion.** `mock_components/GenericSystem` returns
identically zero for every force and effort field. Any quantity derived from a
force reading is therefore **structurally undefined on this platform, not
merely unmeasured** — the number exists but carries no information. Force
reflection (planning-report E3) and closed-loop overshoot (E7) cannot be
defined as criteria until either a dynamic simulator or the physical arm is in
the loop.

**No angular-tracking criterion.** `orientation_mode` is `fixed`: the
commanded orientation is pinned to the anchor and never follows the master's
wrist, because the wrist channels are dead or incoherent. There is no angular
tracking to measure until those channels are repaired.
