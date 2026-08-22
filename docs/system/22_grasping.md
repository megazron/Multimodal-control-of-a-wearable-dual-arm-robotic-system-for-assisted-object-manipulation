# Detection and grasping: what is built, what is broken, what to add

Written 2026-08-22. Read the first section before proposing a model.

## DETECTION IS NOT THE BOTTLENECK, AND THIS IS THE MOST IMPORTANT LINE HERE

On 2026-08-21 both real arms were driven repeatedly and the perception stack
found the cube **every time** — FastSAM at score 1.0, 1166 depth points, zero
false positives from the teal robots that had fooled a colour-only filter
three times. Nothing was picked up. The reasons were calibration, kinematics,
and eight defects in the code that commands the arms.

So: a better detector fixes nothing that is currently broken. Anyone reaching
for GroundingDINO, SAM-2 or a VLA should read the next two sections first.

## WHAT IS BUILT AND WORKING

| piece | file | what it does |
| --- | --- | --- |
| open-vocabulary detection | `srl_perception/prompt_detector.py` | YOLO-World (~60M) from a noun phrase, FastSAM-s for masks, HSV+depth as a no-model fallback. **The backend is reported on every result**, so a detection never implies the good one was running |
| range as a discriminator | same | the teal robots' shadowed hue is 76 and the target green cube's is 74 — two apart, inside noise. They sit 2.8 m away and the rig is under 1.6 m, so range separates them and colour cannot. Applied at detection time |
| geometric grasp | `srl_perception/rgbd_grasp.py` | principal axes of the depth cloud, close across the short one, approach along the free one. No model, no GPU, and every refusal names a cause: no depth, wider than the 85 mm jaw, too few points |
| reach + safety check | `srl_perception/grasp_pipeline.py` | seeded IK, wearer floor, ±π seam margin |

**Why the grasp is geometric and deliberately not learned:** a parallel jaw on
a rigid object is a solved problem, a learned model would need GPU budget that
does not exist here, and it could not explain a refusal.

## THE TWO DEFECTS FIXED ON 2026-08-22

**The wearer floor in the grasp pipeline could never fire.**
`scorer.arm_terms()` returns two clearances. `clearance_m` is the WHOLE chain,
including the immobile mount stub `base_link -> shoulder_link` — and that stub
is nearer the wearer than anything the joints can move, so it reads the
constant **0.2202 m in every pose the arm can reach**. `reachable()` compared
that against the 0.15 m floor. `0.2202 >= 0.15` is true always. **Every grasp
this pipeline has ever approved was checked against a test that could not
fail.** It reads `clearance_moving_m` now (0.4031 at home), and a test aims
the hand at the middle of the wearer's torso and requires a refusal that names
the wearer.

**Refusals asserted instead of measuring.** "this point is outside the arm's
902 mm reach" is a conclusion about the arm drawn from a search. It now
reports the distance the seeded search actually reached. The seeding also
changed to a cached nearest-sample table — a hardening, and stated as one: the
test could *not* make the old uniform seeding fail.

## WHAT WAS BUILT ON 2026-08-22: SCENE UNDERSTANDING

The operator's complaint was "it is unable to check what's on the table, it
will just move here and there", and it was accurate. There was a detector
that finds a NAMED thing, a grasp planner that turns ONE given cloud into ONE
grasp, and a task layer commanding coordinates written down in advance.
**Nothing looked at a surface and enumerated what was standing on it.**

`srl_perception/table_scene.py` is that piece:

| step | what it does |
| --- | --- |
| support plane | RANSAC constrained to lie near a given `up`, so it fits a **table** and not the back wall. Ties break to the LOWEST plane, because on a cluttered table the objects' own tops can out-vote the surface |
| objects | every cluster standing between 12 mm and 400 mm above the plane, by voxel flood fill — no scikit-learn, one parameter, O(n) |
| grasp | plane-constrained: approach **is** the plane normal, jaw is the short horizontal footprint axis and is perpendicular to the approach by construction, centre is plane + half the height |
| ranking | jaw slack, grip height and how well the cluster was sampled |

**The plane is what makes all three of the 2026-08-21 corrections
arithmetic.** The object is *resting* on it, so its bottom is known without a
second view — measured on constructed data, the shell bias in height goes
from **10.5 mm to 0.0 mm**. The plane's normal gives the approach, so a jaw
axis cannot be 31° off vertical. The plane's surface is what the pad must not
go below.

`scripts/pick_from_table.py` is the chain around it: cloud → scene → choose →
**reach and wearer floor** → **whole densified path** → execute. Run against
a constructed scene the left arm can reach, it produced this:

```
support surface at 1.100 m, tilt 0.0 deg, 73315/76800 points on it
2 object(s) on it:
  object 1: 48 x 48 mm footprint, 50 mm tall, centre (0.440, 0.260, 1.125)
      grasp ... 48 mm wide, approach [-0. -0. -1.]
  pregrasp  OK  -- reachable, clearance 0.239 m to L-upperarm
  grasp     OK  -- reachable, clearance 0.245 m to L-upperarm
  home -> pregrasp   OK  over 71 samples
  pregrasp -> grasp  NO  over 64 samples -- wearer clearance 0.1462 m at
                         step 29/64, under the 0.150 m floor
REFUSED: ... The endpoints were both clear; the MIDDLE is not.
```

That last refusal is **defect 2 of 2026-08-21 being caught** rather than
executed.

**Naming works too.** `--want "the red one"` intersects the detector's mask
with the 3-D clusters and picks the cluster with the largest fraction of its
own pixels inside the detection. Below 15% overlap it **refuses** — "I could
not find it" is an answer; moving to the nearest thing that is not it is not.

**No camera calibration is involved.** Points are transformed by the WRIST
camera's pose, which is forward kinematics. That is why this path can be
trusted today while the scene camera's extrinsic cannot.

## THE SCENE CAMERA EXTRINSIC: WHY IT FAILS, MEASURED

Run against the 20 real captures in
`recordings/real_calibration/run_20260821_193328/right/`, the solver still
reports *no transform cleared the 10% coverage gate*. The one-line threshold
fix identified last session is **not** the whole story:

* the depth is real — **20 of 20 frames distinct**, not the frozen-frame
  failure;
* raising the foreground threshold from 0.08 m to 0.80 m does shrink the mask
  from ~48 000 px to ~12 500, as predicted;
* **and the mask is still not the arm.** Overlaid on the colour frame, the
  largest connected component sits on the *empty white backdrop* to the left
  of the robot, and the arms themselves are barely selected.

Two causes, both visible in the picture:

1. **The background frame contains the arm.** `background.npz` was captured
   with `q = [122°, −52°, 59°, −94°, 159°, −40°, 160°]` — an arm out in the
   workspace, not stowed. Background subtraction then cancels the arm.
2. **The scene is white-on-white.** A texture-less curved backdrop defeats
   passive stereo, so its depth flickers frame to frame and fires the
   subtraction all over the wall. The rig band reads ~3.09 m median against a
   backdrop at 4.3–4.7 m, so an **absolute depth gate** separates them where
   background subtraction cannot.

So the fix is not a threshold. It is: **capture the background with the arms
stowed out of frame**, and gate on absolute depth rather than on difference.
Until then the scene camera is good for coarse aiming only, and
`pick_from_table.py` deliberately does not use it.

## WHAT IS ACTUALLY BROKEN, IN ORDER

1. **The camera→robot extrinsic still has 0% coverage**, and it is not the
   threshold. See the section above: the background frame contains the arm,
   and the white backdrop defeats stereo. The fix is a re-capture with the
   arms stowed plus an absolute depth gate — not a one-line change.
   `pick_from_table.py` does not depend on it.
2. **One depth view is a shell.** The repo already knows this — there is a
   test named for it — and the hand-built planner still put the grasp centre
   20 mm low. Either fuse the wrist and scene cameras, or take two wrist
   views.
3. **No support-plane awareness.** All three corrections needed on 2026-08-21
   came from ignoring the surface the object rests on: a jaw axis 31° off
   vertical that would have hit the table, a shell-biased centre, an approach
   nearly parallel to the jaw axis. Segment the plane first, then constrain
   the grasp against it.
4. **The arm stops ~7 mm short of any commanded move** — a per-joint terminal
   error of 0.305 deg. Measured, modelled, and compensable; see
   `srl_teleop/sim_to_real_gap.py`. Against a 30 mm grasp gate that is a
   quarter of the budget.

## THE ARCHITECTURE THAT WORKED, AND SHOULD BE KEPT

**Coarse to fine, and the fine stage needs no extrinsic.**

```
scene camera  ->  WHICH object, and roughly where      (±160 mm was ample)
wrist camera + FK  ->  the final grasp pose            (exact; FK, not calibration)
```

This is why it is worth keeping: the wrist camera's pose comes from forward
kinematics, so the second stage sidesteps the extrinsic that is broken. The
scene camera only has to be right to about 160 mm, which it is.

The one thing unchecked in it: `camera_link` in the URDF is assumed to match
the physical module. Nothing has ever verified that, and a 10 mm URDF error is
invisible and lands directly in every grasp. Hand-eye for the wrist camera is
the cheapest high-value calibration left.

## MODELS — HUGGING FACE, AND THE VRAM IS THE CONSTRAINT

**RTX A500, 4 GB, shared with everything else.** That is what decides this
table, not benchmark scores. The existing YOLO-World choice was made the same
way and is documented in `prompt_detector.py`.

### Detection / segmentation — already solved, listed for completeness

| model | HF id | size | verdict |
| --- | --- | --- | --- |
| YOLO-World | `stevengrove/YOLO-World` (weights via ultralytics `yolov8s-world.pt`) | ~60M | **in use.** ~52 FPS. Open-vocabulary from a noun phrase |
| FastSAM-s | ultralytics `FastSAM-s.pt` | ~11M | **in use** for masks |
| GroundingDINO | `IDEA-Research/grounding-dino-base` | 218M | better on open benchmarks, does not fit beside anything else |
| OWLv2 | `google/owlv2-base-patch16-ensemble` | 428M | same, more so |
| SAM 2.1 tiny | `facebook/sam2.1-hiera-tiny` | ~39M | a genuine upgrade on FastSAM for mask quality if masks ever become the limit. They are not |

### 6-DOF grasp synthesis — the real question, if geometry stops being enough

| model | where | honest read |
| --- | --- | --- |
| **Contact-GraspNet** | `NVlabs/contact_graspnet` | **the sane first thing to try.** 4-DoF-reduced representation from a single depth recording, 17M simulated grasps, generalises to real sensors, small enough for 4 GB. Its input is exactly what this rig produces |
| **GraspGen / GraspGen-X** | `graspgen.github.io` | the strongest current answer to "grasp anything", and cross-embodiment matters because the Robotiq 85 is not the gripper most models assume. **Check the VRAM before committing** |
| **AnyGrasp** | licensed | billion-scale, 7-DoF from a point cloud. Licensing is restrictive — settle that before it becomes load-bearing |
| **FoundationPose** | `NVlabs/FoundationPose` | model-based or model-free pose from a few reference images. Useful once the task is *specific* objects rather than *any* object |
| open-vocabulary 6D pose | Corsetti et al., arXiv 2312.00690 | matches the "name it and pick it" goal and needs two RGBD viewpoints — which this rig can produce, scene + wrist |

### The order to do them in

1. **Fix the extrinsic** (one line, then a full sweep with a held-out residual).
2. **Add a physical ground truth** for it. A marker on the gripper for the
   calibration run only. The objection was that a marker is inelegant; the
   cost of not having one was a day, and a marker-free method that cannot be
   checked is not better than a marker.
3. **Hand-eye the wrist camera**, because every grasp inherits it.
4. **Support-plane segmentation**, then re-run the geometric grasp. This is
   likely to close most of the remaining gap on its own.
5. **Only then** try Contact-GraspNet on the wrist RGBD, and score its output
   against the support plane and the wearer floor before any of it reaches an
   arm.

## THE RULE THAT APPLIES TO EVERY MODEL ON THIS PAGE

*A model's output is a measurement, and it is an instrument until it has been
cleared.* A learned grasp is not exempt because it is learned. It gets a
known-answer test, it gets checked against the support plane and the 0.15 m
wearer floor, and a grasp it proposes that fails either is a refusal with a
reason — not a lower score.

## WHAT TO RUN

```
# detection alone, against a recorded frame
python3 -m srl_perception.prompt_detector --help

# the geometric grasp, no ROS and no robot needed
python3 -m srl_perception.rgbd_grasp            # self-test

# grasp + reach + wearer floor
python3 -m pytest -q src/srl_perception/test/test_the_grasp_pipeline_looks_before_it_says_no.py

# the whole look-then-grasp path
python3 scripts/verify_look_then_grasp.py
python3 scripts/verify_vision_drives_grasp.py --vision <staged detections>
```

Everything else: `docs/HOW_TO_RUN.md`.
