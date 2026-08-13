# Superseded 2026-08-13, before the TASK_SPEC pre-recording pass

Three clip sets moved here. **Nothing in them may be quoted.** They are kept
because they are the evidence for several of the findings below, not because
any number in them is still current.

    verification/                   the 2026-08-12 one-table set
    verification_20260811_t1rework  the T1 rework set
    verification_20260813/          the 19-of-25-cell sweep

`archive/recordings/mode06_GOOD_20260811/` is untouched, as always.

## What changed under them

Every item here changes what is on screen or what the numbers mean. Any one of
them supersedes a clip on its own.

| | change | effect on these clips |
| --- | --- | --- |
| 1 | **the workspace marking was re-surveyed.** The old `WORKSPACE` was a hardcoded box measured **with the bench in the scene**; the bench has been deleted. The marking these clips show excludes both of T1's coloured planes by 70–130 mm in y | the marking in frame is wrong, and it is the thing a participant is told to work inside |
| 2 | **the marking is now drawn as measured cells, not a bounding box.** At 50 mm the right arm's reachable cells fill 62% of their own box, so the rectangle in these clips asserts a third of a region that was never measured reachable | the marked area is larger than the truth |
| 3 | **a one-arm task now draws one marking.** T1 drew both, which put a large empty rectangle in the middle of the frame — and it is what the front view was centring on while the work sat at the edge of the shot | framing and composition differ |
| 4 | **every clip now opens on the presentation pose**, a joint-space staging move that brings the tool axis from +30.8 / +22.1 deg down to +6.5 / +7.5. These clips open on the home wrist | the opening frames differ, and `opened_on` is not recorded in these |
| 5 | **T2 now logs tilt and separation continuously.** `scene_events.json` in these clips has no `carry_series` and no `carry_summary` | the carry can only be scored pass/fail at the end, which is the one thing T2 exists to measure |
| 6 | **T3's information card described a task the path does not perform** — the meter touching each test point in turn. The card in these clips is wrong about what is being shown | the card contradicts the footage |
| 7 | **`msc_clip_tasks` named the wrong arm.** T1's `expect` text said LEFT while `T1_ARM` is right | any caption or report generated from it names the wrong arm |
| 8 | **the yaw check read the wrong arm's gripper axis** (a leaked loop variable), so `max_yaw_err_deg` in these `scene_events.json` files is measured against the right gripper for left-arm items | that field is wrong for every left-arm object |
| 9 | **the anticipation floor** in the dance routines is now stated in time (0.30 s) rather than as a fraction of a move | the routines render the same today, but the clips predate the guarantee |

## What did NOT change

The task coordinates. T1's cubes and planes, T0's spheres, T2's band and T3's
box and meter are all where they were, and all still verify. These clips are
superseded by what is DRAWN and what is LOGGED, not by where anything is.

## Two open faults these clips demonstrate

Both were found by the 2026-08-13 sweep recorded here and neither is fixed:

* **`02_vr_teleop` loses every grasp** while the arms travel 1.0–2.6 m. Not
  "VR is broken": `04_vr_shared` uses the same controllers and scores 4/4,
  4/4, 2/2. The fault is specific to the 02 path and most likely the gripper
  command rather than the pose command.
* **T1 is flaky across modes** — 4/4 under 01 and 04, 0/4 under 02, 03 and 06,
  with identical layout and identical waypoints. The layout verifies at 0 IK
  failures over N=10, so the geometry is fine and the grasp is marginal in a
  way the path check does not capture. Suspect the grip schedule's timing
  against arrival rather than the coordinates.
