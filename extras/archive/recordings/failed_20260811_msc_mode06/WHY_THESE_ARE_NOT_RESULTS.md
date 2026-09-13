# 32 clips from mode 06 — DELETED, and why they were never results

Recorded 2026-08-11, first end-to-end run of the sweep on the MSc task set.
**Every one of the four tasks FAILED the sweep's own check** and the files
were removed rather than kept, so they cannot be mistaken for evidence.

    FAIL  run exited 1; no scene_events.json -- the scene never ran   (x4)
    32 mp4 files        0 scene_events.json

Two independent checks agree they are worthless:

* **the sweep** refused all four: no `scene_events.json` means nothing ever
  confirmed an object was in the scene, so a clip showing an arm moving
  proves only that an arm moved.
* **the verifier** failed all four on a different criterion entirely —
  `clip only 1.3 s long`. The runner exits about 1.3 s in, so the files are
  a second and a bit of the start of a run that then died.

They are deleted, not archived as clips, because a directory of mp4s under
`recordings/` is exactly the thing that gets cited by accident. This note is
the record that they existed and what was wrong with them.

**What must be true before any replacement is cited:** a `scene_events.json`
per clip, a duration consistent with the task's own path length, and the
verifier's self-test green FIRST (it refuses to report until it has caught
its constructed broken clips — including an all-black gripper view in an
otherwise-good directory).
