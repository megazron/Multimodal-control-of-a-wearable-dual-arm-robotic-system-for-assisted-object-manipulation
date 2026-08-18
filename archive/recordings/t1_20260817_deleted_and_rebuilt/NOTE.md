# T1 as it stood on 2026-08-17, before it was deleted and rebuilt

Everything in this directory is the T1 that existed up to and including commit
`b782a9b`. It was not superseded by a fix; it was **deleted on purpose** and a
new T1 built in its place. This note says what was here, what was wrong with
it, and which of its evidence still stands.

## What is in here

| | |
| --- | --- |
| `0*/T1/S1_left_arm/` | the five recorded clips, one per mode, eight angles and a card each, recorded 2026-08-16 on the presentation pose |
| `t1_layout.json`, `t1_paths.json` | the layout and its path verification |
| `t1_layout_on_surface.json`, `centre_on_surface.json`, `centre_reach.json` | the three searches that concluded the centre was unreachable and objects could not rest on the table |
| `observe_pose_*.json`, `observe_transit_*.json` | the look-then-grasp poses solved for THAT layout. They are specific to the cube and pad coordinates that no longer exist |

## What the deleted T1 was

Four cubes in a row at `x` 0.560 to 0.740, `y` 0.120; two pads at `x` 0.595 and
0.825, `y` 0.215; all six objects **floating 120 mm above the table**, on the
LEFT arm only, run in all five modes. The pads were 605 mm off the centreline
and both of them sat over the table's left-hand side.

## Why it was deleted rather than adjusted

Two of its properties were not defects in the layout but consequences of the
question it had been asked, and neither could be fixed by moving a coordinate:

1. **The pads were at the table's side edge.** They were placed at the
   innermost column the LEFT arm alone could reach while the whole pad
   footprint stayed inside the marked region. Nothing about that construction
   can produce a pad in front of the person, because one arm cannot work on
   both sides of a centreline.
2. **Nothing rested on the table.** `T1-1` had been recorded `BLOCKED` since
   2026-08-13. The cause was measured repeatedly and always came back the
   same: the **pinned wrist**. Every mode commands
   `master_calibration.WORKSPACE_ORIENT`, which points 30.7 deg above
   horizontal, so the hand arrives from the near side and from below --
   through the volume a table top occupies.

The rebuild changes the question. The new T1 runs under **`06_full_autonomy`
alone**, and 06 is the one mode that does not pin the wrist: under 06 the
system supplies the whole pose. That makes the approach a free variable for
this task, which is what makes objects on a surface and a centred pad pair
worth measuring again rather than worth restating as impossible.

## What its evidence still supports

* The **mode-difference** finding (`docs/system/findings.md`, 2026-08-15) is
  partly measured on these clips. They remain its evidence.
* The **vision fault and its fix** (2026-08-17) were diagnosed against this
  layout. The fix -- pausing the followers for the look, and stamping each
  frame with the pose it was rendered from -- is a property of the pipeline,
  not of the layout, and carries over unchanged.
* `02_vr_teleop`'s **inability to grasp** (88 to 206 mm pad miss) is recorded
  in these clips' `scene_events.json` and is not re-measurable under the new
  T1, which does not run 02 at all.

## What it does NOT support any more

The three "the centre is unreachable" searches in here were all taken **with
the wrist pinned**. They are correct about the pinned wrist and say nothing
about a task that does not pin it. `recordings/baselines/t1_centre.json` is
the measurement that replaces them, and it carries the pinned anchor inside
its own approach fan so the two can be compared rather than swapped.
