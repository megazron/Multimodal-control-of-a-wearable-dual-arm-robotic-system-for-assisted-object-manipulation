# EXPERIMENTAL DESIGN — THE MODE COMPARISON

*The arithmetic first, the design second.*

## 1. Why the obvious design does not fit

Four control modes (DIRECT TELEOP via the mannequin, VR, SHARED AUTONOMY,
FULL AUTONOMY) crossed with five tasks is **20 conditions per participant**.

Per-condition cost, from the measured scripted trial durations plus the
handling overheads already established:

| item | per condition |
| --- | --- |
| instruction and mode familiarisation | 2 min (first exposure to a mode: 6 min) |
| trials (3 scenarios x 3 repeats, or 8 pursuit trials) | 6–9 min |
| questionnaires (NASA-TLX + trust) | 3 min |
| reset, re-home, break | 2 min |
| **total** | **13–16 min** |

Twenty conditions is therefore **260–320 minutes of task time alone**, before
consent, baselines, fitting or the role swap. With the two-person overheads
already costed (consent taken separately, the stop drill, fitting, debrief =
57 min), a full within-subjects crossing is a **5.5–6.5 hour session**.

The wearer bears >17 kg. The existing protocol caps pack-on time at 12 minutes
per block and 48 minutes total for one person, on both fatigue and ethical
grounds. **A full crossing exceeds that by a factor of five. It is not
feasible and should not be presented as an option.**

## 2. Three designs that do fit

### Option A — MODE BETWEEN SUBJECTS

Each dyad experiences **one** mode and performs all five tasks under it.

| | |
| --- | --- |
| conditions per participant | 5 |
| task time | 5 x 14 = **70 min** |
| session total | 70 + 57 = **127 min** |
| pack-on per person | 35 min (within the 48 min cap) |
| dyads for n = 12 per mode | **48 dyads = 96 participants** |

*Power.* Between-subjects comparison of four independent groups. For a
one-way ANOVA at $\alpha = 0.05$, $1-\beta = 0.80$, detecting a large effect
($f = 0.40$, Cohen) needs $n \approx 19$ per group = **76 dyads**. A medium
effect ($f = 0.25$) needs $n \approx 45$ per group = **180 dyads**. Only a
large effect is realistically detectable.

*Verdict:* statistically clean, logistically impossible for an MSc. 96
participants is not recruitable in the time available.

### Option B — FULL MODE COMPARISON ON A SUBSET OF TASKS

All four modes on **two** tasks; the remaining three tasks under direct teleop
only.

| | |
| --- | --- |
| conditions | (4 modes x 2 tasks) + (1 mode x 3 tasks) = **11** |
| task time | 8 x 14 + 3 x 14 = **154 min** |
| session total | **211 min** — still too long |

Reduce to **one** task across four modes:

| | |
| --- | --- |
| conditions | (4 x 1) + (1 x 4) = **8** |
| task time | **112 min** |
| session total | **169 min** |
| pack-on per person | 56 min — **exceeds the 48 min cap** |

*Verdict:* fits the clock only if the pack-on cap is broken, which it should
not be.

### Option C — TWO MODES, ALL FIVE TASKS, WITHIN SUBJECTS  ← **RECOMMENDED**

Drop from four modes to **two**: DIRECT TELEOP and SHARED AUTONOMY. Each dyad
performs all five tasks under both, order counterbalanced.

| | |
| --- | --- |
| conditions per participant | **10** |
| task time | 10 x 11 min (shorter: familiarisation amortised) = **110 min** |
| — of which pack-on | **44 min per person**, in blocks of ≤ 12 |
| session total | 110 + 57 = **167 min** |
| dyads | **n = 16** |

*Power.* Within-subjects, paired comparison per task. A paired $t$-test at
$\alpha = 0.05$, $1-\beta = 0.80$ detects $d = 0.73$ with $n = 16$, and
$d = 0.60$ with $n = 24$. Because each dyad serves as its own control, the
between-subject variance that dominates Option A is removed, which is why 16
dyads here beat 48 there.

*What is given up.* VR and FULL AUTONOMY are not compared. That is a real
loss and must be stated: the thesis can then say nothing about VR versus the
mannequin, which was one of the original questions.

*Why those two modes.* DIRECT TELEOP is the baseline every other mode is
defined against, and it is the mode this project actually built and verified.
SHARED AUTONOMY is the mode whose value is contested — SRL Proxemics reports
that autonomy *reduced* wearer trust, while the shared-autonomy literature
reports it improves operator performance. That contradiction is the single
most informative comparison available, and it needs both ends measured in one
system.

## 3. Recommendation

**Option C: two modes, five tasks, within subjects, n = 16 dyads
(32 participants), 167-minute session.**

It is the only design of the three that fits the wearer's fatigue limit
without breaking it, has adequate power for a medium-to-large effect, and
targets the one comparison where the literature actually disagrees with
itself.

**If VR must be included**, the honest extension is a *third* session arm run
on a separate cohort (VR versus direct teleop on Task 1 and Task 5 only),
reported as a secondary study rather than folded into the main design.

## 4. Counterbalancing

- **Mode order** within dyad: AB / BA, alternating across dyads. Imbalance 0
  for even $n$.
- **Task order** within mode: Task 1 always first (it is the warm-up and the
  Fitts characterisation); Tasks 2–5 by Williams square, which balances
  immediate sequence as well as position.
- **Role order** (who wears first): alternating across dyads.

With $n = 16$: mode-order imbalance 0, role-order imbalance 0, task-order
position imbalance 0.

## 5. Session structure (167 min)

```
  0   arrival, both together                              5
  5   CONSENT, separately, in different rooms            10
 15   baselines, EDA fitted, Borg baseline               10
 25   safety brief + STOP DRILL (wearer presses twice)   10
 35   familiarisation, no data                            8
 43   BREAK, pack off                                     5
 48   MODE 1 block A: Task 1 + Task 2                    22   pack ON
 70   BREAK, pack off, questionnaires                     6
 76   MODE 1 block B: Tasks 3, 4, 5                      33   pack ON
109   BREAK, pack off, questionnaires                     6
115   MODE 2 block A: Task 1 + Task 2                    22   pack ON
137   BREAK                                               6
143   MODE 2 block B: Tasks 3, 4, 5                      33   pack ON  [*]
176   debrief                                            10
```

`[*]` Block B is 33 min of pack-on time and **breaches the 12-minute
continuous cap**. It is therefore split into three sub-blocks of 11 min with
the pack removed between them, which is what brings the total to 167 min of
task-relevant time within a ~190 min visit. **This is the binding constraint
of the whole design, and it comes from the wearer's load, not from the
science.**


## Amendment 2026-08-09: Task 2 has no baseline

Task 2 loses its DIRECT and VR conditions, because teleoperated grasping is
not achievable with this master (169.7 deg / 164.6 deg of unmeasurable wrist
rotation). It is not replaced with easier objects.

**The two-mode comparison therefore rests on FOUR tasks, not five.** That is a
real loss of power and is recorded as one rather than absorbed. Task 2 remains
in the programme as a single-condition capability demonstration in modes 4 and
above: it is the only task with a discrete outcome, and the only one that
demonstrates rather than asserts the categorical argument for autonomy.

No comparative statistic may be computed for Task 2.
See `docs/research/09_task2_grasping_finding.md`.
