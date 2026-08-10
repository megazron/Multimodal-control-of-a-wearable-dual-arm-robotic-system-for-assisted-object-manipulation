# Verification clips — sweep of 2026-08-10

**The previous set is ARCHIVED, not deleted:**
`archive/recordings/verification_20260810/`, with `GEOMETRY_NOTE.md`. It was
recorded against an **empty planning scene** — the bench was a marker only, so
`avoid_collisions` could not see it and the arm swept through a surface that
looked solid, while every object floated 0.107–0.357 m above it and the wrist
was placed where the object should be. Those clips show motions a
collision-aware planner rejects. They are kept because several findings cite
them (the mode axis, the VR half-scale result, the isolation rule, the
verifier's calibration history), none of which depends on the scene being solid.

This set is recorded against the verified geometry: bench in the planning
scene, objects resting on it at the near edge, wrist derived from the object.

## The five to watch first

| # | file | what it shows |
| --- | --- | --- |
| 1 | `06_full_autonomy/A/S1_single_arm/rviz_gripper.mp4` | the whole grasp close up: fingers open on approach, the 40 mm block **between the two pads**, carried to the bin. Placed 0 mm from target |
| 2 | `01_master_teleop/A/S1_single_arm/rviz_front.mp4` | the same task down a completely different command path — the block travels left to the teal bin, the right arm holds still |
| 3 | `03_shared_autonomy/B/S1_bimanual/rviz_front.mp4` | both arms engaged at once, left holding, right carrying — the longest run, the arbiter blending |
| 4 | `02_vr_teleop/A/S1_single_arm/rviz_quad.mp4` | four angles at once; the VR transport driving the same task |
| 5 | `06_full_autonomy/C/S1_present_probe/rviz_gripper.mp4` | present-and-probe: the left hand holds the instrument throughout, which **is** the task |

## Every angle

`rviz_front / back / left / right / iso / top / gripper / quad`. Captions are
burnt into every angle: mode, task, scenario, what was EXPECTED, and the RESULT
— which turns **red** and says so when the run did not complete.

## Opening them from Windows

The workspace is inside WSL. From Explorer or PowerShell:

    \\wsl$\Ubuntu\home\gausms\kortex_ws\recordings\verification\

or, from a WSL shell, `explorer.exe .` in the clip's directory. They are plain
H.264 mp4 and play in anything. **Do not open them through `/mnt/c`** — that
mount has thrown EIO on this machine and the failure looks like a corrupt file.

## State of the sweep

| mode | A | B | C |
| --- | --- | --- | --- |
| `06_full_autonomy` | OK, 0 mm | OK | OK |
| `01_master_teleop` | OK, 0 mm | OK | OK |
| `03_shared_autonomy` | OK, 0 mm | OK | OK |
| `02_vr_teleop` | OK, 0 mm | OK | **FAIL** — see below |
| `04_vr_shared` | **not recorded** | | |

`scene_events.json` beside each clip carries the grasp/release events and now
the closest-approach diagnostic (`min_pad_obj_m`, `min_pad_obj_closed_m`).
