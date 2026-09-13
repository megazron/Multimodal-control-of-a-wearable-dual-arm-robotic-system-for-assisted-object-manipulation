# Archived 2026-08-23, before the accuracy pass re-record

This is the clip set as it stood after the 2026-08-16 re-record. It is kept
because ARCHIVE, NEVER DELETE, and because it is still the evidence for every
figure quoted against it before 2026-08-23.

## What changed underneath it, so these clips no longer describe the system

* **The teleop motion generator.** `clamp_towards` clamped each joint
  independently; the hand left the commanded path by up to 51.3 mm. It is
  Ruckig now — jerk-limited, phase-synchronised — at 0.0 mm.
* **The declared pad offset.** `clip_tasks.PAD_OFFSET_BY_ARM` was 13.45 mm
  longer than the FK measurement, so the pads landed that far short of every
  object T0, T2 and T3 declare. Derived from the measurement now.
* **The pad midpoint is a function of the gripper opening**, not a constant.
  T1's grasp was built on the wide-open value and is built at the opening a
  40 mm cube needs now — 11.43 mm apart.
* **The shell bias** reached the live pick path, and T1 now uses the work
  plane it had only been gating on.

Full account: `docs/system/findings.md`, 2026-08-23.
