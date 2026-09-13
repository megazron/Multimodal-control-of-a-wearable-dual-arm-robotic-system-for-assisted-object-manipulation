# Full autonomy in the study design — what it supports, and what it does not

## Where mode 6 sits

The experimental design has an **autonomy-level axis**. Until now it ran from
DIRECT (mode 1) to SHARED (mode 4), and the top of the scale was still an
operator moving a master arm. Mode 6 gives that axis a genuine **upper
bound**: the same robot, the same safety stack, the same task, with the
operator supplying only a spoken goal.

That matters for E2 specifically. A two-level comparison (direct vs shared)
cannot distinguish "assistance helps" from "any reduction in operator burden
helps". A three-level axis with a fully autonomous endpoint can, because it
brackets the operator's contribution from both sides.

## The claim it supports

> Performance and workload vary with the level of autonomy, and the
> relationship is not monotonic in the operator's favour.

Mode 6 is the condition where the operator contributes least. If workload
falls monotonically from 1 to 6 while task success peaks somewhere in the
middle, that is the interesting result and it needs the endpoint to be
visible. Without mode 6 the curve has no right-hand end.

## The claim it does NOT support

**It is not evidence that autonomy is safer.** Mode 6 is the highest-risk
mode in this project: no operator in the loop, arms mounted beside a person's
head, and a perception system that can be confidently wrong. Its safety rests
entirely on the shared stack plus the mode-6 extras (confidence floor, action
timeout, voice stop, spoken confirmation, world-model agreement, wearer
exclusion, decision log). Those make failures *catchable and explainable*;
they do not make them rarer.

**It is not a claim about generalisation.** The detector is prompted with a
closed vocabulary and the grammar accepts four verbs. Nothing here shows the
system would cope with an unrehearsed object or an unrehearsed phrasing, and
the study should not imply it does.

**It is not a VLA comparison.** This is a modular pipeline by choice (see
docs/ENGINEERING_LOG.md). A paper claiming "modular beats end-to-end" would need the
end-to-end arm of that comparison, which was deliberately not built and for
reasons of safety architecture rather than performance.

## T4 — the result that is NOT available

The design anticipated a headline: *inter-arm handover is impossible under
teleoperation (both wrists share one approach direction under
`orientation_mode: fixed`) and becomes possible under autonomy (each arm gets
its own wrist angle)*. A capability that appears only with autonomy is
exactly the kind of categorical result an autonomy-level axis wants.

**Measured, and it is not available on this rig.** Across 16 candidate
transfer points including the protocol's own transfer station, **zero** are
reachable by both arms; over a 7x3x3 frontal grid, **0 of 63** cells are, and
the entire centreline is reachable by neither. There is no point at which one
arm can present an object to the other.

So the blocker was never wrist orientation, and mode 6 does not lift it. T4
is blocked for teleop and autonomy alike, by the arms' disjoint workspaces,
until the right arm is re-parked in hardware. The home joint angles are
ground truth and were not changed to manufacture the result.

**This is worth reporting as a negative finding.** A design that assumed
handover was an orientation problem would have spent a lab session tuning
wrist angles against a geometric impossibility.

## What mode 6 still needs before a participant sees it

- **Perception models installed and characterised on the real cameras.**
  Detection rate, pose accuracy and latency are currently unmeasured on
  hardware; the pipeline runs against injected detections. **Under 95% at the
  working distance is a blocker for any user study**, and that number does
  not yet exist.
- **Audio.** `/dev/snd` in WSL holds only `timer`; the shipped path is a
  Windows-side sender over UDP and it has not been exercised with a real
  microphone.
- **The wearer keep-out validated against the PARTICIPANT's dimensions**, not
  the mannequin's. The current volumes come from `human_backpack.xacro`.
- **An independent observer e-stop**, as for VR. Mode 6 has no operator
  holding anything at all.
