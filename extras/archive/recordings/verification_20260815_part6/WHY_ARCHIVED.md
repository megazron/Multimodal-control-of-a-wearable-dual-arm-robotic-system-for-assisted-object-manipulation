# Archived 2026-08-15, before the part 6 re-record

This is the 25-cell set recorded on 2026-08-15 (evening). It is kept because
it is the evidence behind finding 3, and superseded because the scene and the
measurements under it moved after it was shot.

## What changed after these clips were taken

* **T2's tray was elastic.** It was drawn at `span + 0.06` -- the LIVE gripper
  separation -- so it ran 0.487 to 1.211 m against a 0.560 m spec and was held
  at both ends whatever the arms did. T2-1 and T2-2 could not fail. It is
  rigid now, and the ball falls off past 6.8 deg instead of riding to 23.
* **T1's two stages were two tasks.** Stage 2 placed onto coordinates drawn
  from each arm's surveyed cells, with no colour rule and no pads drawn at all.
  Both stages now use the same coloured pads, mirrored per side.
* **The wearer's arms were not in the runtime clearance check**, so the
  follower, the homing node and the bridge were enforcing against a torso, a
  head and a pair of hips only.
* **The published innermost-safe columns were stale**: 0.425 / 0.400 against a
  re-measured 0.325 / 0.450, because the home change moved the IK seed.
* **`record_rviz.py` wrote its render checks into a dead path**, and a blank
  white frame counted as a captured still.

## What these clips are still good for

Finding 3. They are the only measurement of achieved motion per mode with no
operator in the loop, and `scripts/analyse_mode_difference.py` reads them to
show the difference is neither a recording-length artefact nor run-to-run
noise. Do not delete them; the replacement set does not contain the same
comparison until it is re-run.

`extras/archive/recordings/mode06_GOOD_20260811` is a different, older archive and is
NEVER touched.
