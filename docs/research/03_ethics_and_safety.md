# Ethics and safety — protocol, risks, mitigations, consent and data handling

Written to be adapted for an ethics submission. It describes the study as it
is actually built, including the parts that are not ready, because an ethics
application that overstates readiness is the wrong kind of document.

---

## 1. Summary of the study

Participants teleoperate a pair of supernumerary robotic arms — two Kinova
Gen3 7-DOF manipulators mounted on a backpack frame — using an instrumented
mannequin "master" arm. The studies compare direct teleoperation against two
levels of shared autonomy, including under a concurrent manual task.

| | participants | duration | robot moves |
| --- | --- | --- | --- |
| E1 Fitts characterisation | none required | 25 min | yes |
| E2 Autonomy level | 12 | 45 min | yes |
| E3 Divided attention | 12 | 45 min | yes |
| E4 DOF recovery | 12 | 40 min | yes |
| E5 Intent characterisation | none required | 30 min | yes |

E1 and E5 are system evaluations run by the research team. E2–E4 involve
participants.

---

## 2. THE SINGLE LARGEST RISK, stated first

**A 17 kg robot mounted behind a seated person, moving under partly autonomous
control, within reach of their head and torso.**

Everything below follows from that. Two facts about the current build must be
in any submission:

1. **The arms' proximal links statically interfere with the wearer's own upper
   arms in the collision model.** Measured: the Gen3 base tube starts 38 mm
   clear of the wearer's upper-arm cylinder and no mount rotation exceeds
   −15 mm clearance while also placing the hands usefully. This is set by the
   mount POSITION, not by any joint, so no planner can avoid it. It is
   excluded in the SRDF and recorded in `docs/ENGINEERING_LOG.md` as a mechanical action.
   **Until the bracket stands the arm bases off the shoulder, WORN operation
   must not be approved.** Bench-mounted operation is unaffected.
2. **The rig has never been run against real hardware in the gated flow.**
   Every figure in this repository is from simulation or from
   feedback-only hardware sessions.

**Recommendation: seek approval for BENCH-MOUNTED operation only in the first
instance**, with worn operation as a subsequent amendment once the bracket
change and a hardware safety validation are complete.

---

## 3. Risks and mitigations

### 3.1 Robot contacts the participant
- *Likelihood* low; *severity* moderate to high.
- Collision-aware IK (`avoid_collisions=True`) with the wearer in the
  collision model.
- A geometric clearance interlock measured from **TF — where the robot really
  is** — not from where it was commanded. Below the floor the follower holds
  and publishes nothing.
- Participant floor **0.15 m**, inflated by the ratio of the model's 1.75 m
  height to the participant's, because a shorter participant is closer to the
  arm than the model assumes (`participant_safety_node`).
- **The autonomy is never permitted to move the arm toward the wearer**:
  checked continuously as a closing-distance test on the body axis, and any
  violation aborts and latches the e-stop.

### 3.2 Robot moves unexpectedly or too fast
- **A participant velocity cap of 0.05 rad/s**, separate from and stricter
  than the developer cap (0.10 rad/s in `real_robot`, 0.6 in sim). Separate on
  purpose: a shared limit silently inherits whatever the developer last
  loosened.
- Limits are **re-asserted every 2 s**, so a stray `ros2 param set` cannot
  quietly raise them mid-session.
- Motion must be explicitly armed; the system starts unable to move.

### 3.3 E-stop not reachable
- A physical e-stop within the seated participant's reach, plus the
  experimenter's.
- **Verified every session, and the software enforces it**:
  `participant_safety_node` refuses to arm until
  `/estop_reachable_confirmed` is published, which cannot be done without
  performing the check.
- Three independent triggers already exist (service/topic, both master
  buttons within 0.4 s, and a dead-man on master pose going stale) and all
  latch until an explicit reset.

### 3.4 A sensor channel fails mid-trial
- Measured reality: right j4 drops out on 12.9% of frames, in bursts.
- **The trial is ABORTED and marked INVALID. It is never silently continued.**
  A frozen command is indistinguishable in the data from a slow participant,
  which is both a safety and an integrity problem.
- Every abort is logged with its cause to a JSONL file that survives the
  session.

### 3.5 Physical strain from the master arm
- The master has **no gravity compensation**; the participant's own arm
  carries it.
- Blocks of **≤ 8 minutes** of continuous holding, **2-minute rests**, total
  session **≤ 45 minutes**, arm rest available throughout.
- **Borg CR10 fatigue probe between every block**, logged, so fatigue is a
  measured covariate rather than an unmeasured confound.
- Participants are told, in the consent form and verbally, that they may rest
  at any time without giving a reason.

### 3.6 Strain from wearing the rig (worn condition only)
- ~17 kg. **20 minutes maximum continuous wear**, doffed at every rest.
- **Two people** to don and doff, always.
- Screening exclusion on any back, neck or shoulder problem.
- See §2: worn operation is not recommended for approval yet.

### 3.7 Psychological risks
- Low. Task failure may be mildly frustrating; participants are told
  explicitly that the SYSTEM is under evaluation and they are not.
- The autonomy can take an action the participant did not intend. They are
  briefed on this, shown the cancel, and told that cancelling is a normal part
  of use and not an error.

---

## 4. Consent

### Before the session
Participants receive an information sheet at least 24 hours in advance
covering: what the device is, that it moves near them, the weight if worn,
duration, right to withdraw, what is recorded, and how to withdraw data
afterwards.

### Consent is sought for, separately and individually
1. participation;
2. video recording (optional — participation does not depend on it);
3. retention of anonymised data beyond the project;
4. use of anonymised data in publications.

### Withdrawal
Any time, without reason and without consequence. Data can be withdrawn up to
the point of publication, identified by the participant's code, which is why
the code-to-person mapping exists at all (see §5).

### Screening — exclusion criteria
- any back, neck or shoulder injury or pain (worn condition);
- uncorrected visual impairment affecting the task;
- pregnancy (worn condition, on precautionary grounds);
- under 18.

---

## 5. Data handling

### What is recorded
Per trial: completion time, grasp success, positioning error, failed attempts,
path length, time to handover, intervention count, correction time, the full
intent distribution over time, master and end-effector trajectories, gripper
events, channel health, clearance. Per condition: NASA-TLX and a short
trust-in-automation scale. Per block: the Borg fatigue probe.

### Anonymisation
- **Participant codes only** (`P01`, `P02`, …). No name, email, date of birth,
  address or phone number appears in any file.
- This is enforced **in code**: `TrialLogger.write_manifest()` raises if any
  such field is passed, rather than relying on discipline.
- Demographics are collected as **age band**, not date of birth.

### The code-to-person mapping
Kept only if withdrawal-after-the-fact is offered, which it is. Held on
university encrypted storage, **outside this repository**, accessible only to
the named researchers, and destroyed at publication.

### Video
Optional, framed to show the rig and the participant's hands, **not their
face**. Stored on encrypted university storage, retained only as long as
consented.

### Storage and retention
- Working data on encrypted university storage.
- Anonymised data retained per institutional policy (typically 10 years).
- **This repository is not a data store.** `experiments/*/results/` is for
  pilot and scripted runs; real participant data does not live in version
  control.

### Legal basis
Public-task processing under UK GDPR / Data Protection Act 2018, with consent
for the optional elements. Confirm the wording with your institution's DPO —
this section is a starting point, not legal advice.

---

## 6. Session checklist

Do these in order, every session, before the participant enters.

```
[ ] bash scripts/diagnostics.sh          all controllers active, ~100 Hz
[ ] e-stop physically placed in the participant's reach
[ ] press the e-stop and confirm the arms stop; reset
[ ] ros2 topic pub --once /estop_reachable_confirmed std_msgs/Bool "{data: true}"
[ ] ros2 service call /participant/arm_session std_srvs/srv/Trigger
[ ] confirm /participant/safety reports the 0.05 rad/s cap applied
[ ] participant height entered  -> participant_height_m
[ ] mounting recorded in the manifest: worn | bench
[ ] arm rest in position
[ ] consent signed, all four items answered
```

After the session:

```
[ ] all trials written; check the summary CSV row count matches the schedule
[ ] safety JSONL reviewed: any abort or e-stop, and its cause
[ ] data copied to encrypted storage; local copies removed
```

---

## 7. What is NOT yet safe, and must not be glossed

- **The gated real-hardware flow has never run against real arms.** It has
  been exercised end to end only against `mock_real.launch.py`.
- **The mount's proximal interference with the wearer is a real mechanical
  clash**, currently handled by an SRDF exclusion. Excluding a pair makes the
  planner ignore it; it does not make it safe. Bench-mount until the bracket
  is changed.
- **Perception has been characterised only on synthetic images.** Detection
  rate on real cameras, under real lighting and motion blur, is unmeasured.
  The ≥95% study gate has been passed in simulation only.
- **Payload compensation is not applied to hardware.** The node records what
  it would set; the Kortex call is a stub because the arm permits one session
  and the bridge owns it.
- **The right arm's home joint values were never read from hardware.** They
  are documented as unverified, and P_HOME for that arm depends on them.

---

# TWO-PERSON ETHICS — added 2026-08-08 with the reframing

**The wearer is a participant, not apparatus, and they bear a risk they cannot
control.** Everything above was written for one person who chose to move the
arms that were on their own back. That person could stop the danger by
stopping their own hand. The wearer cannot.

## The four changes that are not negotiable

**1. CONSENT IS TAKEN SEPARATELY, IN DIFFERENT ROOMS.**
Two people who arrived together will not decline in front of each other, and
the wearer is consenting to strictly more than the operator: 17 kg of borne
mass, and physical risk from motion they do not command. A joint consent
session makes refusal socially expensive for exactly the person who most needs
to be free to refuse. Separate rooms, separate forms, separate opportunity to
withdraw, and neither is told whether the other consented until both have.

**2. THE WEARER HAS AN UNCONDITIONAL STOP, AND REHEARSES IT BEFORE ANY DATA.**
A physical e-stop in the wearer's own hand, on the dead-man path already built
(`estop_node`, `/estop`). They **physically press it twice and see the arms
halt** during the safety brief, before any recorded trial. A stop mechanism a
person has not personally exercised is not one they trust, and trust is a
dependent variable in this study — measuring it while the wearer is unsure the
stop works would measure the wrong thing.

No justification is ever requested for a stop. No persuasion, not once.

**3. A STOP IS DATA, NOT ATTRITION.**
A wearer who halts because they felt unsafe is the single most informative
observation the safety measures can produce. Stops are logged with the wearer's
own stated reason, reported in the results, and **never** silently dropped.
Sessions that end early are **not** replaced with a fresh dyad to top up n —
that would select for tolerant wearers, whose safety ratings are the least
informative ones available.

**4. THE OPERATOR IS TOLD, EXPLICITLY, THAT THEY CANNOT FEEL THE WEARER.**
This is a briefing item, not a footnote. The operator has no proprioception of
the wearer, no vestibular sense of their sway, and sees them only through a
camera mounted on the moving base. Operators reliably over-estimate their own
awareness — which is why §2.1 of `05_two_person_measures.md` measures it by
probe accuracy rather than self-report. They are told that the wearer's stop
is the real backstop, and that reaching for it is never a failure on their
part.

## Risk that is new to the two-person configuration

| risk | who bears it | mitigation |
| --- | --- | --- |
| Arms move toward the wearer's head or torso on a command they did not make | **wearer** | collision-aware IK; clearance floor (0.12 m in `real_robot`); `mount_guard_node`, which ignores the SRDF deliberately; wearer's own e-stop |
| Wearer loses balance under 17 kg while swaying to a metronome (T9) | **wearer** | T9 is performed **seated or with a rail within reach**; sway amplitude capped at 100 mm, which is a weight shift and NOT a step; the step condition was removed for reachability and is not reinstated for realism |
| Wearer repositions INTO the arms' workspace (T8) | **wearer** | T8 stances are verified poses; the reposition is completed and confirmed **before** the operator is cleared to approach; the operator is briefed never to move during a reposition |
| Social pressure to continue | **wearer** | separate consent; separate debrief; Borg ≥ 7 ends their wearing regardless of what they say; the experimenter, not the operator, calls the break |
| Operator over-confidence about wearer state | wearer | awareness probes with confidence, so **calibration** is measured; poor calibration is reported as a safety finding |
| Fatigue divergence between the two | both | pack **off** during every break, not loosened; 12-minute wearing cap; Borg from both at every boundary |

## The clearance finding, and what it obliges

Recording every task revealed that the T3/T6 carry paths bring the arms within
**51-62 mm** of the wearer's torso — IK-valid, contact-free, and **below the
120 mm margin `real_robot` mode enforces**. MoveIt's check is binary contact;
the floor is a margin, and a scenario can pass one and be refused by the other.

**Ethically this resolves one way only: the floor stands and the task moves.**
T3/T6 are either re-specified with the carry band further forward, or run
bench-mounted (`mounting: bench`, already supported). **Lowering the floor to
make a task run is not available** — that margin is the last thing between the
arms and a person's chest, and the person it protects is the one who did not
choose the motion.
