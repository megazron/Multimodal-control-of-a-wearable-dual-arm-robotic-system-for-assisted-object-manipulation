# TASKS A, B AND C, AND THE TWO-HOUR SESSION

Supersedes the five-task set for the purposes of the participant study.
`src/srl_experiments/experiments/abc/tasks.py` is the specification;
`session_timeline.py` is the timeline **and the arithmetic that checks it**;
`scripts/verify_abc_scenarios.py` is the reachability verification.

The five-task set is not deleted. It remains the verified geometry these three
are cut from, and Tasks A/B/C reuse its coordinates unchanged, so nothing has
to be re-derived.

---

## 1. Why three, and why these three

The binding constraint is **the wearer, not the science**. One person carries
more than 17 kg with no gravity compensation; the protocol caps pack-on at
12 minutes continuous and 48 minutes total; and the session must fit two
hours. Two modes crossed with five tasks was costed at 167 minutes and does
not fit.

Two consolidations, each already justified by a recorded finding rather than
by the clock:

**Tasks 3 and 4 were never two mechanisms.** CLAUDE.md records it plainly:
they are the same mechanism — a coupled object spanning the dead band — run
with a rigid and a compliant object. Rigid-versus-compliant is the scientific
contrast, so it belongs *inside* one task as a within-task factor. Merging
saves a whole condition's familiarisation and questionnaires and loses no
comparison.

**Task 2 carried no comparative measure at all.** It has no DIRECT and no VR
condition, because a top-down grasp needs 169.7° (left) / 164.6° (right) of
wrist rotation from the anchor `orientation_mode: fixed` pins to, and nothing
in the master measures the wrist to command it. A task that cannot be compared
across modes cannot contribute to a mode comparison. It is retained as an
optional demonstration, explicitly outside the timeline and outside every
statistic.

What survives is one task per distinct thing measured:

| | task | what only it measures | bimanual? |
| --- | --- | --- | --- |
| **A** | positioning | individual and divided attention, uncoupled | no — attention contrast |
| **B** | coordinated carry | physical coupling through a shared object | **yes, route (a)** |
| **C** | dual pursuit | simultaneity across disjoint sets | **yes, route (b)** |

### The geometric fact underneath all of it

The two arms' reachable sets are **disjoint**: at y = 0.35, z = 1.10 the right
arm reaches x ≤ −0.10 and the left x ≥ +0.15, with a 0.25 m dead band between.
No task may require both arms at the **same** point — inter-arm handover is
impossible on this platform. A task can require both arms only if

* **(a)** a rigid or tensioned object **spans** the gap and must be held at two
  points at once — Task B; or
* **(b)** two targets in the two disjoint sets must be satisfied
  **simultaneously** — Task C.

A is deliberately neither. It is the uncoupled control the other two are read
against, and it is the one task that still runs if the grippers, the vision or
the objects are unavailable — and the only one that survives every degradation
of the master, since with reach frozen the targets still differ in elevation
and azimuth.

---

## 2. Verification — N = 10 over the full path

`python3 scripts/verify_abc_scenarios.py`

Both arms verified at **0.0000 rad from home** before any sample; the script
refuses otherwise, because a run with the arms parked elsewhere answers a
different question under the same name — which has already invalidated one
full reachability run and half a front-reach run in this project.

```
INSTRUMENT CONTROLS
  far outside reach (1.60, 0.35, 1.15)   -> unreachable   (want unreachable)
  inside the wearer (0.00,-0.10, 1.25)   -> unreachable   (want unreachable)
  a declared Task A target                -> reachable    (want reachable)

TASK A   left  3/3 targets, 0 transit waypoint failures
         right 3/3 targets, 0 transit waypoint failures
TASK B   S1_short_lift   4 waypoints  VERIFIED
         S2_full_lift   11 waypoints  VERIFIED
         S3_detour      10 waypoints  VERIFIED
         sag at 500 mm span = 102.0 mm vs a 40 mm ball -> RETAINED
         s_max = 534.0 mm, declared 534.0 mm, margin 34.0 mm
TASK C   left  centre [0.35, 0.35, 1.15]   26/26 shell directions, 0 radial
         right centre [-0.35, 0.35, 1.15]  26/26 shell directions, 0 radial
OPTIONAL left 3/3 approaches, 3/3 places;  right 3/3, 3/3

  2743 IK calls    TOTAL FAILURES: 0
```

Three properties of that run are the reason it can be believed:

**N = 10, not 1.** TRAC-IK restarts randomly, so a pose at the edge of the
feasible set is a coin flip. Measured on this rig, (0.15, 0.35, 1.36) is
72–82% feasible per call and passes a k = 2 check 19–38% of the time — often
enough to be written into a protocol, rare enough to look like bad luck on the
day.

**The whole path, densified to 20 mm.** The arm flies the segments between
declared waypoints; endpoints prove nothing about the 150 mm between them.
Task A additionally verifies every *transit* between targets, which the spec
does not name but the operator traverses.

**Controls, because the headline is a zero.** "0 failures" and "0 poses
actually tested" print identically. The run is bracketed by a pose past the
0.902 m reach and a pose inside the wearer, both of which must come back
unreachable, and a declared target which must come back reachable. If any
control is wrong the script refuses to report a count at all.

> **N is not a substitute for margin.** No finite N proves a pose reachable;
> it only bounds how often the check lies. What saves this rig is that
> feasibility falls off a cliff rather than degrading — 100% at z = 1.34, 82%
> at 1.36 — so the real defence is keeping every declared figure well inside
> the last pose that passed N/N, and using N to find that boundary.

---

## 3. The two-hour session

`python3 src/srl_experiments/experiments/abc/session_timeline.py`

The timeline is held as **data with a checker**, not as prose. "It fits two
hours" is therefore a result, and the next person to add five minutes to the
safety brief finds out immediately what it costs.

```
   0  arrival; roles assigned (information sheet read beforehand)   4
   4  CONSENT, separately, in different rooms                       8
  12  baselines: EDA fitted, Borg baseline, proprioceptive drift    7
  19  SAFETY BRIEF + STOP DRILL -- the wearer presses twice         8
  27  familiarisation, both modes, no data                          6
  33  MODE 1 / TASK A positioning            [BENCH]                8
  41  MODE 1 / TASK B coordinated carry      [WORN]                 9   PACK ON
  50  questionnaires + pack off + Borg                              5
  55  MODE 1 / TASK C dual pursuit           [WORN]                 9   PACK ON
  64  BREAK, pack off, NASA-TLX + trust                             6
  70  MODE 2 / TASK A positioning            [BENCH]                7
  77  MODE 2 / TASK B coordinated carry      [WORN]                 9   PACK ON
  86  questionnaires + pack off + Borg                              5
  91  MODE 2 / TASK C dual pursuit           [WORN]                 9   PACK ON
 100  final Borg, proprioceptive drift re-test, NASA-TLX + trust    6
 106  debrief, both together                                       10
 116  END
```

| constraint | cap | actual |
| --- | --- | --- |
| session | 120 min | **116 min** |
| pack-on, total, per person | 48 min | **36 min** |
| pack-on, longest continuous | 12 min | **9 min** |

**The binding constraint is pack-on, not the clock**, and a test asserts that
it stays that way — if a change makes the clock bind first, the trade-off
being made has changed and someone should notice.

### Task A is bench-mounted, and that is what makes it fit

The project's own rule is that characterisation runs bench-mounted and only
tasks whose *claim* is about wearable SRLs need to be worn. Task A is an
uncoupled Fitts characterisation: its claim is about pointing, not about
wearing. Bench-mounting it buys 15 minutes of pack-on budget that B and C
genuinely need, and costs nothing the design depends on.

### Block durations are derived, not guessed

| block | trials | s/trial | derived | allotted |
| --- | --- | --- | --- | --- |
| Task A | 54 slots (3 targets × 3 widths × 3 conditions × 2 repeats) | 6 | 6.4 min | 8, 7 |
| Task B | 18 (3 paths × 2 objects × 3 repeats) | 30 | 10.0 min | 9, 9 |
| Task C | 12 (4 speeds + 2 baselines × 2 repeats) | 45 | 10.0 min | 9, 9 |

The checker found a real error here on its first run: Task A at 3 repeats is
81 slots = 9.1 min against blocks of 8 and 7. **The spec was changed, not the
check** — Task A now runs 2 repeats. The cost is stated where it lands: at 2
repeats per ID, per-participant Fitts throughput is thin, so the regression is
a **group-level** fit and must be reported as one. A per-participant
throughput may not be presented from this design.

---

## 4. Counterbalancing

* **Mode order** within dyad: AB / BA, alternating across dyads. Imbalance 0
  for even *n*.
* **Task order** within mode: **A always first** — it is the warm-up and the
  uncoupled reference. B and C alternate, counterbalanced across dyads.
* **Role** (who wears): alternating across dyads.

**Roles are fixed within a dyad.** Swapping mid-session would double the
conditions and no version of that fits two hours. This is a real limitation
and must be stated rather than absorbed: no dyad member experiences both
sides, so any *within-person* operator-versus-wearer comparison is unavailable
and may not be claimed. Role differences are between-subjects.

---

## 5. What this design cannot say

Carried forward so it is not rediscovered as a surprise at write-up:

* **Nothing about VR or FULL AUTONOMY.** Two modes only — DIRECT TELEOP and
  SHARED AUTONOMY — chosen because that is the comparison where the literature
  disagrees with itself (SRL Proxemics reports autonomy *reduced* wearer trust;
  the shared-autonomy literature reports it improves operator performance).
* **Nothing comparative about grasping.** The optional pick-and-place block has
  no baseline, by geometry, not by choice.
* **No within-person role comparison**, per the note above.
* **Nothing yet driven by a human through the master arm.** Every coordinate
  here is IK feasibility in simulation, and the DIRECT condition of all three
  tasks depends on master channels that are currently incoherent. See
  `scripts/check_channels.sh` and the degraded-mode banner.
