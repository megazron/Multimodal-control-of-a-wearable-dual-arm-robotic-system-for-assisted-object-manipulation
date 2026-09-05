# Vision gallery — plain-language figures from the real-camera run

All from `recordings/vision_thesis/20260830_073141/` (the run whose left-wrist
frame the report already uses) plus `recordings/baselines/arm_directional_calibration.json`
and `recordings/calibration_frames/`. Regenerate: `python3 thesis_v3/figures/gallery/vision/make_vision_gallery.py`.
Each figure is `.pdf` (vector) and `.png` (150 dpi). No participant appears in any frame
(mannequin only); no participant name appears in any PDF's text (word-boundary grep).

Camera names used in the figures: left wrist camera = `left_gripper` (Kinova vision
module), right wrist camera = `right_gripper`, room depth camera = `scene_rs` (RealSense
D435i across the room), room webcam = `scene_hd` (HD USB webcam, no depth, no calibration).

| file | plain title | sources | takeaway | key numbers | verdict |
| --- | --- | --- | --- | --- | --- |
| `four_cameras` | The four cameras' views, side by side | `raw/*/colour.png` | Two cameras ride on the hands and see the table; two stand across the room and see the whole rig. | — | **MAIN** candidate for Method (the report has never shown all four views together); otherwise APPENDIX F |
| `colour_and_depth` | Each picture beside its depth map | `raw/*/colour.png`, `raw/*/depth.npy`, working bands from `data/*/15_raw_depth.json` | The wrist cameras see depth over most of the table; the room depth camera has readings on only 15 % of its picture; the webcam has no depth at all (shown as such). | left 89 % of pixels have a reading, right 80 %, room depth 15 % | APPENDIX F (tall figure; `four_cameras` + `depth_coverage` carry the same story more compactly) |
| `plane_fit_per_camera` | Where the flat table was found, per camera | `raw/*/depth.npy`, plane normal/offset/6 mm tolerance from `data/*/18_ransac.json`, working band from `15_raw_depth.json` | The wrist cameras find one flat surface under most of what they see; the room camera finds almost none. Mask recomputed from the stored plane over the same depth array. | recomputed inlier fraction: left **73.95 %** (stored 73.95), right **52.79 %** (stored 52.79), room **0.80 %** (stored 3.75 — see note) | **MAIN** candidate for Results §Perception (replaces the words "1.43 mm RMS / 74 % inliers" with the picture); the room-camera panel is APPENDIX-only because its number disagrees with the log |
| `depth_coverage` | How much of each picture has a usable depth reading, and at what distance | `data/*/15_raw_depth.json`, `raw/*/depth.npy` | The room depth camera is a different instrument from the wrist cameras: an eighth of the returns, mostly 2–6 m away. | any reading: 89 / 80 / 15 %; within working range: 42 / 33 / 13 %; wrist medians 2.22 m and 1.84 m (band 0.08–1.5 m), room median 4.71 m (band 0.15–8 m) | APPENDIX F (supports the existing sentence that the scene camera should not be expected to give a support surface) |
| `pipeline_status` | The 33 pipeline steps, per camera: gave a result / declined and said why / not applicable | `manifest.json` stage statuses | Zero steps crashed; every non-result is a named refusal or a by-design absence — the project's own three-state rule, in one strip. | results: left 28/33, right 29/33, room depth 27/33, webcam 12/33 | **MAIN** candidate for Method §Verification or Results §Perception (one figure for "instrumented at 33 stages, refuses rather than guesses") |
| `plan_view_left` | What the left wrist camera believes is on the table, seen from above | `raw/left_gripper/depth.npy`, plane from `18_ransac.json`, boxes from `data/left_gripper/30_boxes3d__every_object__in_the_camera_frame.csv` | The point cloud from above, coloured by height; the pipeline's oriented boxes drawn on it — the box (red, too wide) and the cube (green, graspable) plus three small fragments. | 5 objects posed, nearest 0.539 m, furthest 0.843 m; box footprint 341 × 278 mm; cube 99 × 26 mm as posed (the pipeline's own width estimate for the cube is narrow because the top face is what it saw) | **MAIN** candidate (the "map the robot plans against" picture; CLAUDE.md's calibrate-then-plan story has no figure in the body) |
| `left_vs_right_pipeline` | Picture → cube and box picked out → boxes drawn, for both wrist cameras | `raw/{left,right}_gripper/colour.png`, `scripts/srl_object_detector.py` `blobs()` (cube; square-aspect filter added so the teal robot base in the right view is not taken for the cube), measured-hue threshold (box) | The same scene from the other hand: both cameras isolate the cube and the box. | — | APPENDIX F beside the existing `cv_pipeline.pdf` (which is the left row alone) |
| `calibration_frames` | The arm photographed after each 100 mm test move (six directions, both arms) | `recordings/calibration_frames/*_v025_*.jpg` via `arm_directional_calibration.json` | The 36-run sim-to-real calibration was photographed from the room webcam; frames are near-identical because the moves are 100 mm. | 36 runs = 2 arms × 3 speeds × 6 directions | SKIP for the body (frames carry little visual information); APPENDIX E at most |
| `arm_park_error` | How far short of its target each arm stopped, by direction; and the largest joint error at rest | `arm_directional_calibration.json` (36 runs) | Every move stops 5–8 mm short, the same for both arms and for all three speeds; every joint's rest error sits in a 0.289–0.296° band. | Cartesian shortfall 5.1–8.3 mm; worst joint 0.289–0.296° (mean ≈ 0.293°) | APPENDIX E; **note** the report quotes the park error as 0.305° (from `measure_sim_to_real_gap.py`'s pooled per-joint fit) — this JSON's per-run *worst joint* is 0.289–0.296°, consistent but not identical; do not put 0.305 on this figure |

## Not made, and why

- **Wearer tracking (MediaPipe landmarks over a frame).** No recorded landmark output exists in
  `recordings/` — the tracker's rate/latency numbers in the report came from a live measurement
  script, and stage 32 ("is a person in view") ran on these frames and found nobody (there is
  no person; the wearer is a mannequin). Nothing to plot without synthesising; not drawn.
- **Latency distribution.** No file carries per-frame latency samples (only the summary numbers
  61.6 ms median / 62–562 ms end to end in the docs). Not drawn.
- **Checkerboard / AprilTag calibration grid.** `recordings/calibration_frames/` is not camera
  calibration; it is the arm-move photographs above. Stage 28 found no AprilTag in any frame.

## Disagreements with numbers the report quotes

- Room depth camera plane support: the pipeline log says 3.75 % (appendix F quotes 3.8 %);
  recomputing inliers from the stored plane over the stored depth array with the stored 6 mm
  tolerance gives 0.80 %. Left and right recompute exactly (73.95 %, 52.79 %), so the method is
  right and the room-camera discrepancy is real — most likely the pipeline scored a different
  point set for that camera (its aligned/rotated cloud) than the raw array. Either number says
  "no usable support surface"; cite the logged 3.8 % as the pipeline's figure, not this panel's.
- Park error: see `arm_park_error` above (0.289–0.296° per-run worst joint vs 0.305° quoted).
- Everything else agrees: 1.43 mm RMS / 74 % inliers (left), 144 mm narrowest axis vs 85 mm jaw
  (left, stage 27), 28/29/27/12 stages with a result.
