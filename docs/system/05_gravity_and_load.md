# Gravity and load — three separate problems

They get confused because they share the word "gravity", but they have
different causes, different fixes and different consequences for the study.

| | what carries the load | what goes wrong | who it hurts |
| --- | --- | --- | --- |
| (a) robot payload | the Gen3's own controller | tracking error varies with what is held | the DATA |
| (b) master arm | the operator's arm, unassisted | fatigue drifts monotonically with time | the DATA, via condition order |
| (c) worn mass | the operator's back and shoulders | ~8.2 kg per arm limits session length | the PARTICIPANT |

---

## (a) Robot payload — `payload_manager`

The Gen3 compensates gravity internally from a configured payload mass and
centre of mass. If that configuration does not match what is actually in the
gripper, the residual gravity torque appears as a **steady-state tracking
error that varies with the object**.

That is a confound, not just an inaccuracy. Object type is normally crossed
with condition in E2–E4, so "heavy object" and "condition B" become partially
confounded, and no amount of analysis afterwards can separate them.

`payload_manager` sets the payload on every `GRASPED` transition and clears it
on release, from a catalogue keyed by object name, and **publishes the value it
used on `/payload_state_<arm>` so it lands in the trial log** — including when
the hardware call is a no-op.

### Measured tracking error with and without correct payload

**The sim cannot answer this question, and it is important to say so rather
than report a number that means nothing.** The workspace runs
`mock_components/GenericSystem`, which echoes commanded positions back as
state with no dynamics, no gravity and no joint compliance. Payload
configuration therefore has *exactly zero* effect on simulated tracking — the
two conditions are bit-identical, which is a property of the mock, not
evidence that payload does not matter.

What was done instead:

- the harness (`experiments/e1_fitts_characterisation/analyse_fitts.py`
  tracking block, and `srl_teleop/analyse_teleop.py`) already computes
  commanded-vs-actual tracking error from tf2, so the measurement needs no new
  code on the day;
- `payload_manager` logs the configured value per trial, so a
  with/without comparison is a filter on existing logs;
- the sim null result is recorded below so nobody mistakes it for the answer.

```
sim, mock hardware, ±0.10 m Lissajous, 40 s, left arm
   payload configured   0.045 kg      tracking RMS 0.0164 m
   payload configured   0.000 kg      tracking RMS 0.0164 m
   difference                          0.0000 m   <- mock has no dynamics
```

**UNVERIFIED WITHOUT HARDWARE.** The real figure requires a Gen3 holding a
known mass. Expect the uncompensated case to be worse in the distal joints and
worst when the arm is extended horizontally, since the moment arm is longest
there. Run it before the first participant session, and if the difference is
comparable to the positioning accuracy the study measures, payload
configuration becomes a *precondition* for running at all rather than a
refinement.

---

## (b) The master arm has NO gravity compensation

The operator holds an unsupported mannequin arm. Nothing carries its weight
but the operator.

**Why this is a data problem and not just a comfort problem.** Fatigue grows
monotonically with time on task. Condition order also advances with time. So
without intervention, fatigue is **collinear with condition order**, and NASA-
TLX — which is the instrument most sensitive to it — is contaminated in every
condition. A main effect of condition and a main effect of "how tired you were
by then" are not separable from a single ordering.

### Mitigations, in the order they should be applied

1. **Counterbalance condition order** (Latin square, already required by the
   E2/E3 protocols). This does not remove fatigue; it converts a *bias* into
   *variance*, which is the difference between a wrong answer and a noisy one.
2. **Cap the block, not the session.** Blocks of **≤ 8 minutes of continuous
   holding**, with a mandatory **2-minute rest** between blocks, arm down. This
   is the single most effective change and costs only wall-clock time.
3. **Support the arm between trials.** A simple padded arm rest at the neutral
   pose, used during instructions and inter-trial intervals, removes most of
   the *static* holding load without touching the *dynamic* task.
4. **Counterweight the master.** A counterweight at the master's shoulder
   sized to null the static moment of the forearm segment. Cheap, and it does
   not change the kinematics the pots measure. Do this before adding a second
   IMU, since it changes the mass distribution the IMUs sit on.
5. **Total session ≤ 45 minutes** including consent, instructions and breaks.

### The fatigue probe — mandatory, between every block

A one-item Borg CR10 rating of perceived arm exertion, asked verbally, logged
with the block index and the wall-clock time:

> "On a scale of 0 to 10, where 0 is nothing at all and 10 is maximal, how
> hard is your arm working right now?"

It takes five seconds and it turns fatigue from an unmeasured confound into a
**covariate** that can be entered into the model. If the probe rises
monotonically across blocks regardless of condition, that is evidence the
counterbalancing did its job; if it rises faster in one condition, that is a
finding in itself.

The probe is logged by the shared trial logger as `fatigue_borg` on the
session manifest, once per block.

---

## (c) Worn mass — ~8.2 kg per Gen3, plus harness

A Kinova Gen3 7-DOF weighs **8.2 kg**. Two of them plus the harness, mount
plate and backpack is **well over 17 kg** carried on the back and shoulders.

**State plainly in every write-up whether a study was bench-mounted or worn.
It changes what is being claimed.** A bench-mounted study measures the
control interface. A worn study measures the control interface *plus* the
biomechanical cost of carrying the device, and only the second supports a
claim about supernumerary limbs as a wearable technology.

### The position taken here

**E1 and E5 are BENCH-MOUNTED.** They are system characterisations — Fitts
throughput and intent-inference accuracy — and the worn load adds fatigue
noise without adding anything to what is being measured.

**E2, E3 and E4 should be WORN if the claim is about wearable SRLs**, and
bench-mounted otherwise. Whichever is chosen, it is recorded per session in
the manifest as `mounting: worn | bench`, because a mixed dataset with an
unrecorded mounting is unanalysable.

### If worn

- **20 minutes maximum** of continuous wearing, and the rig comes off during
  every inter-block rest.
- Screen for back and shoulder problems in the pre-session questionnaire, and
  exclude on any positive answer. This is in
  `docs/research/03_ethics_and_safety.md`.
- Two people to don and doff the rig, always. A 17 kg backpack being lifted
  onto a seated participant is the largest physical risk in the whole study
  and it has nothing to do with the robot moving.
- Record participant height and mass. The collision model is a 1.75 m
  mannequin, and the clearance figures in `CLAUDE.md` are for that geometry;
  a substantially different participant needs the padded model re-checked
  (`min_clearance_m`, and `REAL_ROBOT_PAD_M` in `srl_teleop/clearance.py`).

**Honest limitation:** with the arms bolted at (±0.15, −0.12, 1.25) on a
1.75 m model, the measured proximal clearance to the wearer's own upper arms
is already a **static interference** (see the mount section of `CLAUDE.md`).
Until the bracket stands the bases off the shoulder, worn operation is not
recommended, and that is a mechanical prerequisite for E2–E4 in worn form.
