# WORK BRIEF — 2026-08-09

The standing multi-part brief. **This file is the resume point.** A fresh
session needs only:

> Read CLAUDE.md, NEXT_SESSION.md and docs/WORK_BRIEF.md. Resume from the first
> incomplete part.

Nothing else needs re-pasting.

---

## START HERE

**First incomplete part: [PART 1](#part-1--clear-the-stale-shared-memory-and-confirm-autonomy-moves-the-arm) — IN PROGRESS.**

The root cause of "autonomy does not move the arm" **has been found** and is a
real bug, not a discovery problem. It is not yet fixed. See Part 1's status
block for the evidence and the next action.

---

## How to use this file

- Every part carries a **STATUS** line: `NOT STARTED` / `IN PROGRESS` / `DONE`.
- When a part is finished, set its STATUS to `DONE` with the commit hash, **in
  the same commit as the work itself**. The status and the code never diverge.
- Update the START HERE pointer in the same commit.
- Brief text under each part is the user's, **verbatim**. Do not paraphrase it,
  do not "tidy" it, and do not delete it when the part is done — a completed
  part's original wording is what lets a later session check whether the work
  actually answered the question asked.
- Findings, measurements and decisions go in `docs/NEXT_SESSION.md` as usual.
  This file carries the *instruction* and the *status*, not the results.

### Part index

| part | title | status |
| --- | --- | --- |
| 1 | Clear the stale shared memory and confirm autonomy moves the arm | **IN PROGRESS** |
| 2 | Language and vision sweep | NOT STARTED |
| 3 | Gripper open on shutdown | NOT STARTED |
| 4 | Degraded mode warning | NOT STARTED |
| 5 | Task A | **TEXT NOT SUPPLIED** |
| 6 | Task B | **TEXT NOT SUPPLIED** |
| 7 | Task C | **TEXT NOT SUPPLIED** |
| 8 | Dual-view GUI | **TEXT NOT SUPPLIED** |

---

## Preamble (verbatim)

```
Fresh session. Read CLAUDE.md and NEXT_SESSION.md.

Start here, in this order. Commit after each.
```

---

## PART 1 — CLEAR THE STALE SHARED MEMORY AND CONFIRM AUTONOMY MOVES THE ARM

**STATUS: IN PROGRESS** — root cause found, fix not yet written. Commit: —

### Brief (verbatim)

```
=== 1. CLEAR THE STALE SHARED MEMORY AND CONFIRM AUTONOMY MOVES THE ARM ===
Last session connected /autonomy/assist_pose_<arm> into ik_follower_node at
request_ik - the same gate teleop uses, so the whole safety stack is in the
path. Verified: subscriber present, pose reaches the follower, clearance floor
fires when aimed inside the wearer.
NOT verified: motion at a reachable target. Zero trajectories, with
RTPS_TRANSPORT_SHM failures and a graph read showing sub 0 while blockers from
that subscription were arriving. Discovery was degraded.
So: stop everything, clear /dev/shm/fastrtps_*, relaunch, confirm both arms at
home, and re-run scripts/verify_autonomy_command_path.py.
Use the arm's own anchor orientation, never identity - identity is not
neutral, it is a specific unreachable pose, and it has produced a false
negative three times now.
Report whether autonomy actually moves the arm. If it still does not, that is
a real bug, not a discovery problem - find it.
```

### Progress so far

Environment work, all done:

- **Four concurrent stacks were running**, not one — 35 stack PIDs across four
  `ros2 launch` trees, plus three orphaned `ros2 bag record` processes left
  over from the 2026-08-08 verification sweep. Killed by explicit PID list
  (SIGINT then SIGKILL), zero remaining.
- **`/dev/shm` held 188 `fastrtps_*` entries**, oldest dated Aug 5. Note that
  `rm /dev/shm/fastrtps_*` alone leaves **39 `sem.fastrtps_*` semaphores**
  behind — the glob does not match them, and a count that only looks at
  `fastrtps_*` reads as "39 still there" and invites a hunt for a live writer
  that does not exist. Both patterns must be removed.
- **The `ros2` daemon was hung**, not merely stale: `ros2 daemon stop` blocked
  and died with `TimeoutError: [Errno 110] Connection timed out` after 120 s.
  Already documented behaviour on this box; use `--no-daemon` for ground truth.
- Relaunched a single clean stack, `gate:=false`.
- **Both arms confirmed at home: max error 0.0000 rad on all 7 joints each.**

### THE FINDING — `ik_follower_node` crashes on the first autonomy pose

**`ik_follower_node.py` has no `import time`.** Its complete import block is
`sys, os, math, home_positions, rclpy, …` — the module is never imported. The
autonomy path added last session calls `time.monotonic()` twice:

- line 713, in `on_autonomy_pose`
- line 733, in `autonomy_is_driving`

So the **first autonomy pose to arrive kills the left follower outright**:

```
File "…/ik_follower_node.py", line 713, in on_autonomy_pose
    now = time.monotonic()
NameError: name 'time' is not defined. Did you forget to import 'time'?
[ERROR] [ik_follower_node-13]: process has died [pid 682827, exit code 1, …]
```

The followers are launched **without `respawn`** (only `master_pose_node` has
it), so the node stays dead for the rest of the session.

This fully explains "0 trajectories at a reachable target", and it is exactly
what the brief anticipated: **a real bug, not a discovery problem.** Discovery
was genuinely degraded too — four stacks and 188 SHM segments — but that was a
second, independent fault, and fixing it did not change the result:
the re-run on a clean single stack still reported `autonomy moves arm : False`.

### Two consequences for what was previously reported as verified

1. **The crash happens at line 713, before `self.autonomy_pose_t = now` at
   line 720.** That assignment is the only thing that can make
   `autonomy_is_driving()` return True. So `autonomy_has_control` can never be
   asserted by the code path as written — which means last session's row
   *"autonomy pose reaches the follower — yes, `autonomy_has_control` is
   asserted, and only `on_pose` can set it"* cannot be right as stated, and
   needs re-deriving after the fix.
2. **`scripts/verify_autonomy_command_path.py` matches blocker names as
   substrings of the `/blocking` payload.** If that payload enumerates
   *registered* blockers rather than only *active* ones, then seeing
   `clearance_floor` in it is not evidence the floor fired. The safety row (c)
   in last session's table rests on that match and must be re-validated against
   a known-good reference before it is believed — standing rule, CLAUDE.md.
   **Do not report "the same floor applies to autonomy" as measured until
   this is settled.**

### Next actions, in order

1. Add `import time` to `ik_follower_node.py`.
2. Add a regression test that fails on the pre-fix code — an autonomy pose
   delivered to the callback must not raise. A test that only checks the
   subscription exists would have passed the whole time.
3. Consider whether the followers should carry `respawn` like
   `master_pose_node` does. A follower that dies silently and stays dead is the
   same silent-stop class this project has paid for repeatedly. Decide
   deliberately — respawn also masks crashes.
4. Re-run `scripts/verify_autonomy_command_path.py` and report whether the arm
   actually moves.
5. Fix the probe's blocker detection if item 2 above confirms it matches
   registrations, then re-establish the safety claim honestly.

---

## PART 2 — LANGUAGE AND VISION SWEEP

**STATUS: NOT STARTED** — Commit: —

### Brief (verbatim)

```
=== 2. LANGUAGE AND VISION SWEEP ===
Never run. Wide phrasings including ones you did not design for: vague,
relational, superlative, compound, misspelled, polite-with-filler, and objects
that are not present. Report correct / asked / refused / MISUNDERSTOOD.
Misunderstood separately and loudly - it is the dangerous category.
```

---

## PART 3 — GRIPPER OPEN ON SHUTDOWN

**STATUS: NOT STARTED** — Commit: —

### Brief (verbatim)

```
=== 3. GRIPPER OPEN ON SHUTDOWN ===
On exit it stays part-closed and the next startup treats that as its open
reference, so the whole travel range is wrong from then on.
 - open fully on clean shutdown AND on exception paths
 - on startup do not assume current position is open: command a full open and
   confirm from joint states, or read the true limit from hardware
 - EXCEPT when latched on an object - a latched grip must not drop something.
   Distinguish the cases and say how.
 - verify by killing mid-grasp and confirming the next start comes up correct
```

---

## PART 4 — DEGRADED MODE WARNING

**STATUS: NOT STARTED** — Commit: —

### Brief (verbatim)

```
=== 4. DEGRADED MODE WARNING ===
The pot repair is invisible to the software - degraded_mode reads a stored
channels_20260806.json saying 6/14 coherent, so eight working channels are
still frozen. Put the warning where it cannot be missed, and make
check_channels.sh the documented first action of every lab session.
```

---

## Closing instruction (verbatim)

```
Stop there. Tasks A/B/C and the dual-view GUI are the session after - each has
exhausted a context when bundled.
```

---

## PARTS 5–8 — TASK A, TASK B, TASK C, DUAL-VIEW GUI

**STATUS: TEXT NOT SUPPLIED.** Do not start these, and **do not reconstruct
them from memory or from context** — no specification for them exists in this
repo or in the conversation they came from.

### What is actually known about them

Only the single line quoted above: they exist, there are three lettered tasks
plus a dual-view GUI, they follow Parts 1–4, and **each exhausts a context when
bundled** — so they are one-per-session work, not a batch.

### Discrepancy to resolve with the user

The instruction to write this file asked for **seven parts, verbatim**. What
was actually supplied was **four fully-specified parts (1–4)** plus a one-line
forward reference naming four more items (A, B, C, dual-view GUI) with no
content — which is eight named items, not seven, and four of them have no text
to be verbatim about.

An earlier, interrupted message in the same session opened with *"Three
blockers, then all three tasks specified in full"* and was cut off after its
first line. The full specifications for A/B/C were most likely in that message
and never arrived.

**Ask for the text of Tasks A, B and C and the dual-view GUI before starting
them.** Paste it into this file verbatim under these headings, set their
statuses to `NOT STARTED`, and fix the part index. Writing a plausible
specification instead would be inventing the requirement and then grading the
work against the invention.

---

## Session log

| date | parts touched | commit | note |
| --- | --- | --- | --- |
| 2026-08-09 | brief created; Part 1 in progress | — | four stacks and 188 SHM segments cleared; `import time` missing in `ik_follower_node` identified as the real cause of 0 trajectories |
