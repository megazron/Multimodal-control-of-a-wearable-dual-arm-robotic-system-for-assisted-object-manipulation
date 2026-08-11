# MODE 06 — VERIFIED AND CITABLE. DO NOT OVERWRITE THIS DIRECTORY.

Frozen 2026-08-11 from `recordings/verification/06_full_autonomy` at commit
`1dcac33`, tagged **`mode06-good-20260811`**.

**These four clips passed BOTH gates: the automatic sweep AND frame-by-frame
inspection.** Every number quoted from mode 06 anywhere in this project should
be traceable to this copy. It is a snapshot, not a working directory: any
future recording writes somewhere else, and nothing in here is ever
regenerated in place. If you need to re-record mode 06, record it into
`recordings/verification/` and leave this alone — that is the whole point of
the copy existing.

Recover the exact tree at any time with:

    git checkout mode06-good-20260811 -- archive/recordings/mode06_GOOD_20260811

## WHAT IS HERE

Four tasks, eight angles each (front, back, left, right, top, iso, gripper,
plus the 2x2 quad), 32 mp4 files, 19 MB, with `scene_events.json` beside each
clip and the sweep's own `abc_sweep_progress.json`.

| task | scenario | EE travel L / R (m) | grasp / release | subjects drawn | worst closing distance |
| --- | --- | --- | --- | --- | --- |
| T0 | D3_three_targets | 1.716 / 1.477 | 0 / 0 | L1, L2, L3, R1, R2, R3 | — |
| T1 | S1_left_arm | 1.999 / 0.240 | 4 / 4 | plane_blue, plane_green | 0.0000 |
| T2 | S2_full_lift | 0.711 / 0.604 | 0 / 0 | ball, tray | — |
| T3 | S1_measure_cycle | 0.249 / 0.361 | 2 / 2 | P1, P2, P3, P4 | 0.0000 |

T0 and T2 have no grasp by design — T0 is reaching only and T2's tray is held
by both grippers from the first frame. `min_pad_obj_closed_m` is the distance
between the finger pads and the object AT THE MOMENT THE FINGERS CLOSED, and
0.0000 means every grasp landed on its object rather than beside it.

**Every capture was gated on MOTION, not on the timeout fallback.** That
matters: the previous mode-06 attempt gated all four on timeout, which means
the arm never moved, and its numbers were about a stationary robot.

## WHAT WAS CONFIRMED BY LOOKING, not by a check

* **T0** — all six spheres in frame and coloured BY COLOUR as `task0`
  names them, not by arm: L1/R1 red at head height, L2/R2 green at chest,
  L3/R3 blue low. Three visibly distinct directions, both arms moving between
  them, no furniture at all, nothing near the wearer.
* **T1** — both coloured mats visible and the pairing correct: green cube onto
  the green mat, blue onto the blue. Four close/open cycles; each cube moves
  only while gripped.
* **T2** — the tray spans BOTH grippers with the ball on it, stays level, and
  stays in frame for the whole lift to z = 1.60.
* **T3** — ONE circuit box, held by the right arm; the yellow meter held by
  the left; both lifted clear of the bench and returned to it.

## TWO RESIDUALS, recorded rather than hidden

1. T1's four cubes cannot all be counted simultaneously in the front view —
   the arm occludes some. The events file records all four grasp/release
   pairs, and the gripper close-up shows the individual grasps.
2. T3's four measurement points are sub-pixel at the front framing. They are
   published and appear in `fixtures`; the gripper view shows them.

## THE CAVEATS THAT TRAVEL WITH THESE CLIPS

* **Objects are FIXTURED, not resting.** No support geometry beneath them is
  compatible with the pinned wrist — see
  `docs/system/13_fixturing_and_the_approach_cone.md`. The cost: an object
  that cannot fall cannot be dropped, so `drops` is not a measurable outcome
  for T1.
* **T2's ball is drawn, not simulated.** The clip shows the geometry, not the
  physics; a tilt past 6.8 deg would drop it on a real tray and does not here.
* **Sim only.** Nothing in this repository has ever run against a real arm,
  and the mock hardware echoes commands with no dynamics.

---

## ADDENDUM — THE SCENE CHANGED AFTER THESE CLIPS WERE MADE

Added 2026-08-11, after the Task-1 rework at commit `a81797d`. **The clips are
untouched and remain valid evidence for what they show; this note exists so
nobody assumes the current scene looks like them.**

Since these were recorded:

* the work surface became **two levels** — a table at z = 0.95 reaching
  forward to y = 0.10, with the objects on the raised surface at the verified
  1.10. In these clips there is only the single slab.
* T1's **planes moved 15 mm outboard**, from y = 0.130 to 0.145, to make room
  for per-cube slots.
* T1's cubes now get **a slot each** at ±30 mm in y, because two cubes share a
  plane and both used to be delivered to its centre — one inside the other. In
  these clips they stack.
* the surface now carries **workspace markings**: each arm's measured
  reachable boundary, painted on it.

All of that is verified at N=10 over the full path, 0 failures. **T1 in these
clips is therefore superseded geometry.** T0, T2 and T3 are unaffected by the
slot and plane changes but do now render with the two-level table.

Re-record into `recordings/verification/`, never in here.
