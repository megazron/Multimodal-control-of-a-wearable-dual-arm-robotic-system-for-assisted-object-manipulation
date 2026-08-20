# 18. Dynamic degradation — every mode keeps working when a source is missing

**DESIGN ONLY. Nothing in this document is built**, except where a row says
BUILT, which means the rung already exists in the code and is named so the
ladder is honest about what is new and what is not.

---

## 0. THE PATTERN ALREADY EXISTS AND IS THE RIGHT ONE

`srl_teleop/capability.py` solves this problem for one capability -- the
master arm's position sensing -- and solves it well enough that the rest of
this document is "do that, for the other seven".

Its design premise is worth restating because it generalises exactly:

> A mapping that is defined only for "all channels healthy" is a mapping that
> is undefined for the condition the hardware will spend most of its life in.

And its three structural decisions:

1. **A LADDER, NOT A FADE.** Each rung uses a structurally different
   observable, not a noisier version of the same one. L1 SPHERICAL takes its
   radius from an FK magnitude; L2 SPH_RATE has no radial observable at all
   and drives radius as a rate off a spare channel. Those are different
   instruments, not the same instrument degraded.
2. **THE BOTTOM RUNG IS NAMED, NOT AN ERROR.** L4 DIR_ONLY and L5 NONE both
   report position UNAVAILABLE, as rungs. "A fabricated position on a
   manipulator bolted to a person is the worst available outcome, and the
   difference between 'no position' and 'a position' has to be visible from
   outside the process."
3. **THE RUNG IS PUBLISHED, WITH ITS COST.** `/master_capability_<arm>` says
   which rung is running and what it cost in metres of commanded-tip error,
   from `scripts/measure_capability_ladder.py`. A rung with no measurement
   carries None and the publisher says "unmeasured" rather than printing a
   number.

There is even a `capability.regain(healthy, imu_ok)` that answers "what would
repairing each currently-unhealthy channel buy" -- which is the second half of
what this brief asks the GUI to show, already written, for one capability.

**So the work is not inventing a scheme. It is applying the existing one to
the other capabilities, and giving the GUI one place to read them all.**

---

## 1. THE LADDERS

Each table reads: rung, what it uses, what is LOST at that rung. The rule for
every ladder is the same -- a rung must be structurally different from the one
above it, and the bottom rung must be a named refusal rather than a guess.

### 1.1 Master arm position -- BUILT (`capability.py`)

| rung | uses | capability lost |
| --- | --- | --- |
| L0 FK | 7 accurate joints | -- |
| L1 SPHERICAL | j1 + one of j2/j4 + IMU | wrist orientation fidelity |
| L2 SPH_RATE | j1 + IMU + a spare channel | radius is a rate, so absolute reach is lost |
| L3 SHELL | j1 + IMU | radius entirely: the command set is a surface |
| L4 DIR_ONLY | IMU | **NO POSITION** -- elevation only |
| L5 NONE | -- | **NO POSITION** |

Currently binding in the lab: 7 of 14 channels incoherent (a soldering
problem), so this ladder is not hypothetical.

### 1.2 Object pose (where the cube is)

| rung | uses | capability lost |
| --- | --- | --- |
| P0 wrist camera colour + depth | the shipped path (`vision_grasp.observe_and_detect`) | -- |
| P1 wrist camera colour + the known work plane | RGB only, ray-plane intersection at `work_surface.WORK_PLANE_M` | height error if the object is not on the plane; no support for stacked objects |
| P2 scene camera prior | doc 16's camera, coarse | grasp-scale accuracy. **Look-to-refine only**, never a grasp source |
| P3 AprilTag on the object | `apriltag_detector.py` | only works on tagged objects; not the task as specified |
| P4 REFUSE | -- | **the task does not run** |

**P4 is where the code is today and it is correct.** `observe_and_detect`
waits ~40 s and raises `DetectionUnavailable`. It does NOT fall back to the
declared coordinate, and TASK_SPEC P-4 forbids that fallback precisely because
a blind pick is indistinguishable from a working perception path. **Any new
rung on this ladder must preserve that: the declared coordinate is never a
rung.** P1 and P2 are admissible because they are still MEASUREMENTS; the
declared coordinate is not.

### 1.3 Wearer body model

| rung | uses | capability lost |
| --- | --- | --- |
| B0 tracked body, live | doc 16's tracker + the measured size prior | -- (not built) |
| B1 measured size, declared posture | tape measurements per participant | live posture: a wearer who moves is not seen (not built) |
| B2 mannequin -- BUILT | `wearer_posture.wearer_model()` with `SRL_WEARER_ARMS` | the wearer's actual size and posture |
| B3 torso only | drop the arm primitives | the wearer's arms, which are what bind the RIGHT arm inboard |

B3 is listed to be rejected: it was the state before 2026-08-15 and it is a
LOSS of safety, so it is not a fallback. The ladder bottoms out at B2, which
is conservative by construction. Per doc 16 section 6.1, B0/B1 may only ever
make the wearer BIGGER than B2.

### 1.4 Real arm state

| rung | uses | capability lost |
| --- | --- | --- |
| R0 `real_*` frames in the shared /tf -- BUILT | the real stack's own publisher | -- |
| R1 FK on `/real/joint_states` -- BUILT | the URDF, computed in the GUI | nothing for viewing; it is the same computation. Named separately because the SOURCE differs and the panel says which |
| R2 NO REAL ARM -- BUILT | -- | the divergence readout carries no number, and says NO REAL ARM rather than 0.000 |

Both R0/R1 and the refusal at R2 are shipped in the operations GUI's ACTUAL
panel and in `divergence.py`'s seven statuses.

### 1.5 Wrist camera view

| rung | uses | capability lost |
| --- | --- | --- |
| C0 live -- BUILT | frames inside the staleness limit | -- |
| C1 STALE / NO SIGNAL / NO CAMERA -- BUILT | `camera_relay.ChannelState` | the view. **The panel is not painted**, because a frozen picture of a workspace cannot be told from a live picture of a workspace that is not moving |

There is deliberately no "dimmed last frame" rung. Under time pressure a dim
picture is read as a picture.

### 1.6 An arm (one of two)

| rung | uses | capability lost |
| --- | --- | --- |
| A0 both arms | -- | -- |
| A1 one arm, single-arm tasks only | T0, T3 | T1 (cubes straddle the centreline, neither arm crosses it), T2 (bimanual by definition), T1S2 |
| A2 no arms | -- | everything |

**A1 is a REFUSAL for T1 and T2, not a degraded run**, and the reason is
measured: 0 of 10 IK solutions at every cross-side pad slot and every
cross-side cube, both arms. A cube can only be delivered to the pad on its own
side. A one-armed T1 is not a worse T1; it is half a task run under a whole
task's name.

### 1.7 Master arm channels -- BUILT (`degraded_mode.py`)

Freezes incoherent channels against the newest
`recordings/baselines/channels_*.json`, and `reach_fallback_plan` gives the
operator a deliberate radial input on a surviving channel. The trap here is
already documented: **a baseline older than the wiring makes the software
freeze channels that now work, and nothing downstream disagrees.** Any rung
that reads a stored baseline has to publish the baseline's age.

### 1.8 The cascade to the real arms

| rung | uses | capability lost |
| --- | --- | --- |
| X0 cascade enabled | the sim drives the real arms | -- |
| X1 sim only | the sim runs, the bridge is disabled | nothing moves in the world |
| X2 REFUSE to enable -- BUILT | -- | the bridge names the joint and the gap and points at the runbook |

X2 is the shipped behaviour and is the right shape: it refuses rather than
commanding a 1.9 rad difference as a jump.

---

## 2. WHAT THE GUI SHOWS

One panel, the same shape as the connection panel built in Part 1, and for the
same reason: a diagnosis in plain words, plus the button that improves it.

    CAPABILITY                RUNG              FIX THIS TO REGAIN
    master arm position       L1 SPHERICAL      repair j4 -> L0 FK, +12 mm accuracy
    object pose               P4 REFUSE         start a camera -> P0, T1 becomes runnable
    wearer body               B2 mannequin      measure the wearer -> B1
    real arm state            R2 NO REAL ARM    start the real stack -> R0
    left wrist camera         C1 NO CAMERA      -> C0
    right wrist camera        C0 live           --
    arms available            A0 both           --
    cascade                   X2 REFUSED        home the arms -> X0

Four rules, each of which exists because of a specific failure in this repo:

1. **The rung is named, always, including the top one.** "OK" hides which of
   three working configurations is running, and `capability.py` exists because
   the difference between L0 and L1 is 12 mm of commanded-tip error.
2. **NEVER GREEN ON UNKNOWN.** A capability that could not be assessed shows
   UNKNOWN in its own colour. G-3, and the reason the GUI once reported "IK
   BLOCKED -- 0% success" computed over zero attempts.
3. **The cost of the rung is measured or it says "unmeasured".** No invented
   numbers. `capability.py` already does this and it is the harder half.
4. **The regain column names ONE action.** `capability.regain()` already
   computes this for the master arm: what repairing each unhealthy channel
   would buy. It is the difference between a status panel and a panel you can
   act on.

**And the rung must be recorded with the trial.** A run at L1 SPHERICAL with a
B2 mannequin and a P1 plane-intersection object pose is not the same
experimental condition as a run at L0/B1/P0, and if the rung is not in the
manifest the two are indistinguishable afterwards. This is the same argument
that put the typed instruction verbatim on the information card.

---

## 3. THE RULE THAT MAKES THIS SAFE RATHER THAN JUST ROBUST

A ladder is a mechanism for continuing. That is only correct when continuing
is correct. Three capabilities must **refuse** rather than descend, and each
already does:

* **object pose** -- never the declared coordinate (TASK_SPEC P-4);
* **the wearer** -- never smaller than the mannequin (doc 16 section 6.1);
* **the cascade** -- never enable across an unexplained home gap (HARD
  CONSTRAINT 1).

The distinguishing question for any proposed rung: *does this rung still
MEASURE the thing, or has it started to assume it?* A rung that measures worse
is degradation. A rung that assumes is a fabrication wearing a rung's name,
and on a machine bolted to a person the second one is how somebody gets hurt.

---

## 4. COST

| item | cost | notes |
| --- | --- | --- |
| `capability.py` generalised: a `Ladder` type, rungs, costs, `regain()` | **1 session** | the master-arm ladder becomes the first instance rather than the only one |
| Object-pose ladder P0/P1 + the P4 refusal kept intact | **1.5 sessions** | P1 is a ray-plane intersection against `work_surface`; the refusal must stay provable |
| Wearer ladder B1/B2 with the more-conservative rule | **1 session** | doc 16 pays for B0 |
| Arm-availability ladder + per-task admissibility | **0.5 session** | the data exists: the cross-side IK result is measured |
| One `/capability/ladders` publisher aggregating all of them | **0.5 session** | |
| GUI panel, in the connection panel's shape | **1 session** | rung, colour, measured cost, one regain action |
| Rung recorded into the trial manifest | **0.5 session** | `write_manifest` already exists and already refuses identifying fields |
| Known-answer tests: every ladder must descend on a deliberately broken input AND refuse where refusing is correct | **1.5 sessions** | the standing rule. A ladder that cannot be shown to descend is a ladder that has never descended |
| **total** | **~7.5 sessions** | |

**What to build first, if only one thing is built:** the aggregator and the
GUI panel over the ladders that ALREADY EXIST (1.7, 1.4, 1.5, 1.8, and 1.1).
That is ~1.5 sessions, adds no new fallback behaviour, and turns five
capabilities that already degrade correctly and silently into five that
degrade correctly and visibly.
