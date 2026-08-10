# What these clips were recorded against, and why they were superseded

**Moved here 2026-08-10 from `recordings/verification/`. NOT deleted — several
findings in `CLAUDE.md` and `docs/NEXT_SESSION.md` cite them, so they are
evidence.** What they are evidence OF is narrower than their file names
suggest, and that is the whole reason for this note.

## The geometry they were recorded against

| | these clips | the geometry that replaced it |
| --- | --- | --- |
| planning scene | **EMPTY.** `clip_scene.py` published a MarkerArray to `/task_objects` and nothing at all to `/planning_scene` | CollisionObjects via `/apply_planning_scene`, **read back** from move_group |
| the bench | **decoration.** `avoid_collisions` could not see it; the arm swept straight through a surface that looked solid | a real collision object at `BENCH_TOP = 1.10`, near edge `y = 0.245` |
| the objects | **floating.** Bench top at z = 0.935 while objects sat at their EE task coordinates — between **0.107 m and 0.357 m of clear air** under every one | resting on the surface, near face at the edge, `OVERHANG = 0.08` |
| grasp offset | **wrist-centred.** The task coordinates were EE poses used directly, so the wrist sat where the object should be | `ee_for()` — declare the object, derive the wrist, pads `+0.095 m` forward and `+0.057 m` up |
| idle arm | parked at the study band `y = 0.35`, **behind** the bench edge, so each idle hold failed on its own every run | `park(x)`, in front of the edge and above the top |

With a hollow bench the pick pose looked fine. With the bench a real collision
object the same pose is correctly REFUSED — **the geometry had been wrong all
along and the empty scene was hiding it.** That is the single reason for the
re-record: these clips show motions that a collision-aware planner rejects.

## What they ARE still good evidence for

Nothing about geometry, clearance, or whether a grasp is admissible. But these
remain sound, because none of them depends on the scene being solid:

- **The mode axis is real.** Each clip travelled a named mode's own command
  path, started by pressing the GUI's own button — measured 0.16–0.53 m of
  real tf2 EE travel per mode.
- **VR carries exactly HALF** (0.206 m against 0.412 m; 0.062 m against
  0.125 m), because the mapper's scale is 0.5 and the object inherits it.
  That is a property of the transport, not of the scene.
- **The isolation finding.** After a VR run, modes 01/04/06 all recorded
  0.0000 m until `vr_pose_mapper` was killed — the one-source-at-a-time rule
  at the process level.
- **The verifier's calibration history**: RViz shades markers, HUD text was
  once counted as the object, four temporal samples undersamples. The frames
  those conclusions were drawn from are in here.
- `GUI_TUTORIALS/` — six narrated screen recordings of the GUI itself, which
  the geometry does not touch at all.

## Subdirectories that were ALREADY stale when this was archived

- `00_unclassified_legacy_geometry/` (181 mp4) and
  `00_unclassified_scripted_playback/` (304 mp4) — recorded by a harness that
  called `/compute_ik` directly, so no follower, no clutch, no anchor and no
  safety guard was in the path. The mode axis of that tree is empty because
  nothing could fill it.
- `04_shared_autonomy/` — from the SUPERSEDED study-taskset sweep (scenarios
  `S2_full_lift` etc.), not one of the 15. Its B clip is the single verifier
  failure (frozen capture). Stale, not broken evidence.
- `03_orientation_assist/`, `05_supervised_autonomy/` — empty; mode 3 is a
  stub because this `/compute_ik` plugin ignores `OrientationConstraint`.

## Replaced by

`recordings/verification/` re-recorded 2026-08-10 against the verified
geometry: bench in the planning scene, objects resting on it at the near edge,
near-edge aiming, 0 IK failures, every pose at 20 mm of displacement margin or
better in all six axis directions at N=10.
