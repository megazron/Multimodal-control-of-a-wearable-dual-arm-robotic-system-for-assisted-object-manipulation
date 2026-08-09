# WORK BRIEF — 2026-08-09

The standing multi-part brief. **This file is the resume point.** A fresh
session needs only:

> Read CLAUDE.md, NEXT_SESSION.md and docs/WORK_BRIEF.md. Resume from the first
> incomplete part.

**Constraint carried from the brief:** context runs out on long briefs — it has
every time. Finish each part before starting the next. If running low, STOP
CLEANLY at a part boundary and say exactly where. A finished part plus an
honest note beats a half-done system. **Sim and mock only — the user is not in
the lab.**

---

## START HERE

**First incomplete part: PART 2 — FULL DIAGNOSIS.** Report before fixing.

Part 1 is DONE (`6736ee7`): autonomy moves the arm, 1092 trajectories and
0.0700 / 0.1204 m of real EE displacement, residual 0.0000 m.

---

## Part index

| part | title | status | commit |
| --- | --- | --- | --- |
| 1 | Does autonomy move the arm? | **DONE — YES** | `6736ee7` |
| 2 | Full diagnosis (report before fixing) | **NOT STARTED** | — |
| 3 | Finish outstanding work (lang sweep, gripper, degraded warning, tasks A/B/C) | NOT STARTED | — |
| 4 | Dual-view GUI, everything runs through it | NOT STARTED | — |
| 5 | Re-record, driven from the GUI | NOT STARTED | — |
| 6 | Graphs | NOT STARTED | — |
| 7 | Report and commit / push | NOT STARTED | — |

Rules: set STATUS to DONE with the commit hash **in the same commit as the
work**; update START HERE in the same commit; brief text under each part is the
user's **verbatim** and is not deleted when the part completes — the original
wording is what lets a later session check the work answered the question.
Findings go in `docs/NEXT_SESSION.md`; this file carries instruction + status.

---

## PART 1 — DOES AUTONOMY MOVE THE ARM?

**STATUS: DONE — YES.** Commit `6736ee7`. Full write-up in NEXT_SESSION.md.

Root cause was a missing `import time` in `ik_follower_node.py`, killing the
follower on the first autonomy pose. Discovery was also degraded (four stacks,
188 SHM segments, hung daemon) but was **not** the cause. The verification
probe was separately found to be measuring blocker *registration* rather than
*assertion* and was fixed.

### Brief (verbatim)

```
#########################################################################
PART 1 - DOES AUTONOMY MOVE THE ARM? Everything depends on this.
#########################################################################
Three of four study modes are unusable until this is answered.
Last session connected /autonomy/assist_pose_<arm> into ik_follower_node at
request_ik - the same gate teleop uses, so the whole safety stack is in the
path. Verified: subscriber present, pose reaches the follower, clearance floor
fires when aimed inside the wearer.
NOT verified: motion at a reachable target. Zero trajectories, with
RTPS_TRANSPORT_SHM failures and a graph read showing sub 0 while blockers from
that very subscription were arriving. Discovery was degraded.
Stop everything, clear /dev/shm/fastrtps_*, relaunch, confirm both arms at
home, re-run scripts/verify_autonomy_command_path.py.
Use the arm's own anchor orientation, NEVER identity - identity is not
neutral, it is a specific unreachable pose, and it has produced a false
negative three times.
Report whether autonomy actually moves the arm. If it still does not, that is
a real bug, not a discovery problem - find it.
```

---

## PART 2 — FULL DIAGNOSIS. Report before fixing.

**STATUS: NOT STARTED** — Commit: —

### Brief (verbatim)

```
#########################################################################
PART 2 - FULL DIAGNOSIS. Report before fixing.
#########################################################################
Read every file. Report everything, then fix. Say what you judged not worth
fixing and why.

A. THE FIVE RECURRING BUG CLASSES
 1. silent blocking - a check that can only say no, with no recovery
 2. unreachable clear() - state cleared only where it cannot be reached
 3. wall-clock intervals - time.time() is not monotonic under WSL
 4. unprefixed resources colliding across two arms, INCLUDING C++ literals
    invisible to any URDF check
 5. tests that construct the environment where the bug cannot occur

B. THE FOUR INSTRUMENT-FAILURE MECHANISMS from instrument_validation.md
 absence read as a value; the harness constructing the condition it tests;
 state left from a previous run; order of operations inside the measurement.
 Sixteen cases so far, nine caught only by an independently known quantity
 and three by inspection. Where a measurement has no cross-check, add one.

C. STALE REFERENCES - expect the most here
 Paths, node names and topics moved through the six-package restructure, the
 E-to-T renaming, the mode reorganisation and the task consolidation. The
 failure is always silent: a button that runs nothing, a glob that installs
 nothing, a script writing where nobody reads.
 Verify every setup.py entry point resolves, every module is registered, and
 every path a script writes is a path something reads.

D. KNOWN SPECIFIC FAULTS
 - every use of an identity quaternion as a "neutral" test orientation
 - anything that can start a second stack
 - /ik_status_<arm> publishing an empty data array, which blocks the
   clearance readout and the safety measurement that depends on it
 - stale /dev/shm/fastrtps_* segments degrading discovery - make clearing
   them part of startup, not a thing to remember

E. Any reported number not traceable to a validated measurement.
```

**Head start from Part 1** (already found, fix in Part 2):
- Bug class 5 instance: the autonomy probe's substring match — fixed.
- Part 2D "stale SHM as startup" — confirmed necessary; `sem.fastrtps_*` must
  be cleared too, the `fastrtps_*` glob misses them.
- Followers carry no `respawn` while `master_pose_node` does; a follower that
  dies stays dead. Deliberate decision needed, not a default.

---

## PART 3 — FINISH THE OUTSTANDING WORK

**STATUS: NOT STARTED** — Commit: —

### Brief (verbatim)

```
#########################################################################
PART 3 - FINISH THE OUTSTANDING WORK
#########################################################################
 1. LANGUAGE AND VISION SWEEP, never run: vague, relational, superlative,
    compound, misspelled, polite-with-filler, and objects not present. Report
    correct / asked / refused / MISUNDERSTOOD - the last separately and
    loudly, it is the dangerous category.
 2. GRIPPER OPEN ON SHUTDOWN. It stays part-closed on exit and the next
    startup treats that as its open reference, so the whole travel range is
    wrong from then on.
    - open fully on clean shutdown AND on exception paths
    - startup must not assume the current position is open: command a full
      open and confirm from joint states, or read the true limit from hardware
    - EXCEPT when latched on an object; a latched grip must not drop
      something. Distinguish the cases and say how.
    - verify by killing mid-grasp and confirming the next start is correct
 3. DEGRADED MODE WARNING. The pot repair is invisible to the software:
    degraded_mode reads a stored channels_20260806.json saying 6/14 coherent,
    so eight working channels are still frozen. Make the warning unmissable
    and document check_channels.sh as the first action of every lab session.
 4. TASKS A, B and C fully specified, scenarios verified N=10 over the FULL
    PATH, plus a session timeline that fits two hours.
```

---

## PART 4 — DUAL-VIEW GUI, AND EVERYTHING RUNS THROUGH IT

**STATUS: NOT STARTED** — Commit: —

### Brief (verbatim)

```
#########################################################################
PART 4 - DUAL-VIEW GUI, AND EVERYTHING RUNS THROUGH IT
#########################################################################
Two RViz panels side by side - reparenting is proven at 31 fps.
 LEFT: commanded, the sim arms driven by the active mode.
 RIGHT: actual, the real arms from /real/joint_states and /real/tf.
Divergence readout beneath: per-joint difference and EE distance, colour coded
against the lag trip threshold. The lag is currently a number nobody can see.
Both wrist cameras live, labelled. SUBSCRIBE to the image topic, never open
the device. Show "no camera" explicitly, never a frozen last frame.
Controls alongside: mode selection, precision/speed dial, clutch and
force-engage, e-stop, per-arm scale, existing indicators.
EVERY MODE AND EVERY TASK LAUNCHABLE FROM THE GUI, and every button must
actually work - five experiment buttons once exited 2 immediately while
appearing to launch. Click every one.
Report frame time with both panels and both camera streams. 15 GB machine and
two runs already OOM-killed, so if it degrades say so and propose the
single-RViz fallback with commanded ghosted over actual.
```

---

## PART 5 — RE-RECORD, DRIVEN FROM THE GUI

**STATUS: NOT STARTED** — Commit: —

### Brief (verbatim)

```
#########################################################################
PART 5 - RE-RECORD, DRIVEN FROM THE GUI
#########################################################################
The 87 existing clips are stale: old geometry, pre gripper fix, wrong task set.
Re-record for the CURRENT three tasks and four modes, LAUNCHED THROUGH THE
GUI, so the recording doubles as proof the GUI drives the system.
 recordings/verification/<mode>/<task>/<scenario>/
 one file per angle: front, back, left, right, iso, gripper (tight on fingers
 and object), quad. Overlay on front only.
 Xvfb on :99, never WSLg's :0 - x11grab there records black.
 Resumable sweep, progress written after each clip.
VALIDATE THE VERIFIER against frames you have visually confirmed BEFORE
reporting any clip as failed. It has been miscalibrated twice - RViz shades
markers, and HUD text was once counted as the object.
Per clip: fingers open, close to the object's width, hold, open again;
straight-line approach along a vector; attachment coinciding with the fingers
reaching that width; no penetration.
```

---

## PART 6 — GRAPHS

**STATUS: NOT STARTED** — Commit: —

### Brief (verbatim)

```
#########################################################################
PART 6 - GRAPHS
#########################################################################
Publication quality, consistent style: reachability and isotropy per arm; the
capability ladder with measured costs; grasp orientation requirement against
what each mode can command; static hold drift; the precision/speed dial trade;
per-mode command path latency; scene fingerprint accuracy; language sweep
outcomes. Plus placeholders for every participant result, each captioned with
the analysis script that will produce it.
```

---

## PART 7 — REPORT AND COMMIT

**STATUS: NOT STARTED** — Commit: —

### Brief (verbatim)

```
#########################################################################
PART 7 - REPORT AND COMMIT
#########################################################################
Per part: what you built, what you measured, where results differed from
expectation and what you did, and every number still unverified.
Push to https://github.com/megazron/dococthefinal. Check file sizes first.
```

---

## Session log

| date | parts | commit | note |
| --- | --- | --- | --- |
| 2026-08-09 | brief created | `c6c31fe` | superseded by the 7-part brief below |
| 2026-08-09 | Part 1 DONE | `6736ee7` | missing `import time` killed the follower on the first autonomy pose; probe was measuring blocker registration not assertion |
