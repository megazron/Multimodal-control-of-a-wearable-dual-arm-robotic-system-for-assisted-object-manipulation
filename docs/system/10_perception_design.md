# Perception without fiducials — what vision decides, and what it does not

**No AprilTags.** That is a constraint, not a preference, and it changes the
design rather than the tolerances. This document restates the perception
architecture around what has actually been measured.

## The question that was blocking

The scene fingerprint needs **σ ≤ 2 mm** to hold false-changed under 1% at a
10 mm tolerance. Colour/shape gives σ ≈ 10 mm and re-registers 92% of unchanged
scenes, so it cannot support the design. But that is a *colour/shape* figure,
not a *no-fiducial* figure, and the real question — what σ does YOLO-World plus
RGB-D achieve — had never been asked properly.

## 1. Where the pose error actually lives

**Not in the detector.** The pipeline is detect → crop the depth → fit. The
detection box only has to point at the right *region*; the accuracy comes from
the depth fit. So the two halves can be measured separately, and one of them
can be measured here rigorously.

The project's own rule permits this: *synthetic only where the ground truth is
CONSTRUCTED, not RENDERED*. A rendered image is only as good as the renderer —
which is precisely what made the 0–4% detection figure meaningless. A point
cloud of a box of known size, sampled from a known viewpoint under a known
noise model, is arithmetic. The answer does not depend on any appearance model.

`scripts/measure_depth_pose_accuracy.py`, 300 trials per cell, modelling
stereo depth noise (σ_z = z²·σ_d/(f·B)), a 0.4% systematic depth bias that does
*not* average away, partial visibility, and background contamination from a
sloppy box.

### Result: the estimator matters far more than the sensor

| object | naive centroid | **known-dimension fit** |
| --- | --- | --- |
| 40 mm block | 13.3 mm | **1.2 mm** |
| 45 mm part | 16.8 mm | **1.8 mm** |
| 50 mm multimeter | 44.6 mm | **8.0 mm** |

*(at 0.35 m, clean crop)*

**Taking the centroid of the visible points is wrong by 13–45 mm**, and it is
wrong for a reason that never improves with a better camera: you only see the
faces turned toward you, so the centroid of the visible surface is not the
centroid of the object. This is the single biggest error term in a naive
pipeline and it is pure geometry.

Fitting the **known dimensions** removes it, and lands inside ±10 mm on all
three objects.

### Two findings that change the design

**Range barely matters.**

| range | 40 mm block, fit error |
| --- | --- |
| 0.10 m | 1.2 mm |
| 0.20 m | 1.1 mm |
| 0.35 m | 1.2 mm |
| 0.50 m | 1.5 mm |

Per-pixel depth noise rises from 0.06 mm at 0.10 m to 1.46 mm at 0.50 m — a
24× change — and the fit error moves by 0.4 mm. **The limit is not depth
noise.** Averaging over thousands of pixels crushes the random term; what
remains is the systematic bias and the fit geometry, and neither improves by
getting closer.

**This refutes the premise of option (a).** A close-range look at the grasp
standoff was expected to buy a much better σ. Measured, it buys almost
nothing for a known-dimension fit. If we adopt a close-range look it has to be
justified by something else — occlusion, or seeing a face that was hidden —
not by depth precision.

**A sloppy detection box costs nothing, once you segment.**

| contamination | fit error |
| --- | --- |
| 0% | 1.2 mm |
| 10% | 1.1 mm |
| 25% | 1.1 mm |
| 50% | 1.0 mm |

The first version of this measurement reported 24.5 mm at 10% and 28.5 mm at
50% — *saturating*, which is the signature of an estimator artefact rather
than a sensor limit. It was: background points inflated the apparent span
along an axis, the fit concluded both faces were visible, and it centred
between a real face and a background plane. Segmenting to the nearest surface
first — what any real pipeline does — removes it entirely. **Reporting that
first number as "the accuracy of RGB-D" would have been wrong**, and it is the
same class of error as every other instrument bug in this project.

The consequence is the important part: **the detector does not need to be
precise, only to point at the right region.** That is a far weaker requirement
than 2 mm.

## 1b. The real-RGB-D check: LM-O

Run against LM-O (LineMOD-Occluded) — real RGB-D, ground-truth 6D poses, real
sensor noise, deliberate occlusion. 60 frames, 432 object instances, detector
given nothing but an open-vocabulary prompt.

| | |
| --- | --- |
| detection rate | **9.0%** (39/432) |
| median box IoU **when detected** | **0.91** |
| known-fit pose error | median **11.3 mm**, p90 33.8, 44% within 10 mm |
| excluding 3 gross outliers (>100 mm) | median **10.7 mm**, mean 13.5 |

**Does this revise the 1.2–8.0 mm? Not directly, and here is why the
comparison is weak in both directions.** Two confounds both push the real
number up:

1. **Range.** LM-O's median object range is **0.93 m** against our 0.35 m —
   2.7×, and stereo depth noise goes as z², so **7× the noise**.
2. **Shape.** The estimator fits a **box of known dimensions**. LM-O's objects
   are an ape, a duck, a cat, a drill. **Our objects are boxes.** Fitting a box
   to a duck is not the case we ship.

Per object, where anything was detected at all:

| object | instances | detected | median IoU | median fit error |
| --- | --- | --- | --- | --- |
| duck | 60 | 17 | 0.91 | **6.3 mm** |
| watering can | 60 | 19 | 0.91 | 12.8 mm |
| cat | 48 | 3 | 0.92 | 314.6 mm (3 gross failures) |
| ape, drill, egg carton, glue, hole punch | 274 | **0** | — | — |

The best real cases run **1.0–7.1 mm**, which is consistent with the
constructed figures. So: **the constructed 1.2–8.0 mm is not refuted, and it is
not confirmed either.** It remains the best available estimate for our geometry,
and the lab measurement is what settles it.

### The detection rate is a PROMPT result, not a detector result

9% looked alarming until the instrument was checked. Holding everything else
fixed and varying only the wording, over 25 frames with 7.1 ground-truth
visible objects per frame:

| prompt set | detections/frame | confident (>0.1)/frame |
| --- | --- | --- |
| specific ("ape figurine", "hole puncher") | 8.0 | **2.1** |
| generic nouns ("toy", "can", "drill", "box") | 23.6 | **6.4** |
| one generic prompt ("object on a table") | 1.2 | 1.0 |

**Three times the confident detections from wording alone**, and the generic
set lands at 6.4 against 7.1 objects actually visible. The 9% figure therefore
measures a poor prompt set at least as much as it measures YOLO-World.

### Prompt wording is a DESIGN PARAMETER, not an implementation detail

It has a bigger measured effect than any other knob in the perception stack —
3× — and it costs nothing to change, so it is specified here rather than left
to whoever writes the node.

**The system uses GENERIC SINGLE NOUNS by default** (`"can"`, `"box"`,
`"bottle"`, `"tool"`), and the reasons are:

1. **It is what measures best**, by a factor of three, and the yield lands
   closest to the true object count (6.4 against 7.1).
2. **It matches how the detector was trained.** YOLO-World is grounded on
   web image–caption pairs, where objects are named plainly. "ape figurine"
   and "hole puncher" are description-like phrases that occur rarely as
   captions; "can" and "drill" occur constantly.
3. **Precision is not needed here, and asking for it costs recall.** The
   architecture only requires the box to point at the right *region* — the
   known-dimension fit tolerates 50% contamination — so a loose, high-recall
   prompt is strictly the right trade. A specific prompt buys discrimination
   the pipeline does not need and pays for it in missed detections.

**The failure mode of a generic prompt is over-detection**, not under: 23.6
raw detections per frame against 7.1 objects. That is handled downstream and
cheaply — by confidence threshold, by the scene fingerprint's association
gate, and by the depth segmentation — whereas a missed detection cannot be
recovered at all. **Prefer recall; filter afterwards.**

**One generic prompt for the whole scene ("object on a table") is the worst of
both** at 1.0/frame, so the plural matters: one noun per expected object
class, not one phrase for the scene.

This is a tunable and it should be re-measured in the lab on the real objects,
because the wording that works for `"can"` may not be the wording that works
for `"multimeter"`. The procedure is in `NEXT_SESSION.md`.

## 2. Measuring the detector itself — the part not answerable here

Three routes were considered.

| route | verdict |
| --- | --- |
| **Gazebo / Isaac photorealistic RGB-D** | `gz` and `ros_gz` are installed, so it is *possible*. Gazebo Sim renders PBR through OGRE2 — better than the flat primitives that produced 0–4%, but still synthetic, and **whether it is in YOLO-World's distribution is exactly the unknown we are trying to remove**. Cost: building a textured world with a depth-camera sensor, plus headless GPU rendering on a 4 GB card under WSL. Roughly a day, with a real chance the answer is still "the renderer is out of distribution". **Not recommended as the primary route.** |
| **Public RGB-D dataset with GT poses** | **Recommended and available.** LM-O (LineMOD-Occluded) is 687 MB, reachable, and carries real RGB-D with ground-truth 6D poses for 8 household objects under occlusion. YOLO-World is open-vocabulary, so it can be prompted with the objects' own names. This measures the detector's behaviour on **real images with real depth noise**, which is what we cannot construct. It does not use *our* objects — that limit is stated, not hidden. |
| **Lab measurement** | The only route that uses our objects on our bench, and it is written into `NEXT_SESSION.md`. Photograph each object at marked positions, run detection, report σ. |

**What LM-O answers:** whether an open-vocabulary detector finds real objects
reliably, how well-localised its boxes are, and — because the depth is a real
sensor's — whether the 1–8 mm fit figure survives contact with real depth
noise rather than a modelled one.

**What it cannot answer:** the detection rate on *our* objects in *our*
lighting. That stays unmeasured until the lab session, and **under 95% it
blocks a participant session** regardless of what the pose accuracy turns out
to be.

## 3. The recommendation: (c), with (b) as the safety net

**Adopt (c) — depth-only pose refinement against known dimensions — as the
primary path.** The measurement supports it directly:

- it reaches **1.2–8.0 mm**, inside the ±10 mm the grasp requires;
- it is **insensitive to a sloppy detection box** (1.0–1.2 mm across 0–50%
  contamination), so it demands only that the detector point at the right
  region;
- it needs **no fiducials**;
- it needs **no close-range look**, which the range sweep shows buys almost
  nothing — so the sequence stays simple.

**Do not adopt (a) as stated.** Its premise — that close range gives a much
better σ — is not true here. Keep a close-range look only if occlusion at the
working distance turns out to be the problem, which is a different question and
one LM-O will inform.

**Keep (b) as the fallback, and be honest about its cost.** Objects at
measured marked positions with vision confirming only presence is standard in
shared-autonomy work — Javdani's formulation assumes known goal poses — and it
would let a session run even if detection is poor. **What it costs is the
intent-inference claim**: if goal poses are known a priori and vision only
confirms presence, then the system is not *inferring* which object the operator
means from perception, it is selecting among a fixed known set. That is a
weaker claim than the thesis currently makes, and it must be stated in the
write-up rather than quietly assumed.

## 4. The architecture, restated

| stage | what it decides | tolerance it must meet | status |
| --- | --- | --- | --- |
| **Detection** (YOLO-World, open-vocab) | *which region* of the image holds which object | box good enough to contain the object; contamination up to 50% is tolerable | **rate UNMEASURED on real frames** — blocks participant sessions above nothing else |
| **Segmentation** | nearest surface within the crop | must reject the background plane | measured: removes contamination entirely |
| **Pose fit** (known dimensions → cropped cloud) | the object's **centre** | ≤ 10 mm | **1.2 / 1.8 / 8.0 mm** measured against a constructed cloud; to be re-checked on real depth |
| **Scene fingerprint** | only whether the scene **CHANGED** | `pos_tol` 10 mm, needs σ ≤ 2 mm | met by the fit for the two cubes; **the multimeter at 8 mm does NOT meet it** |
| **Grasp** | capture the object | ±10 mm placement | capture half-windows 22.5 / 20.0 / 17.5 mm |

### What this means for ±10 mm

The budget is not spent once. Placement error and pose error add:

- **40 mm block**: 1.2 mm pose + placement. Comfortable.
- **45 mm part**: 1.8 mm pose + placement. Comfortable.
- **50 mm multimeter**: **8.0 mm pose against a 17.5 mm capture window.** That
  leaves under 10 mm for everything else, and it is the same object that binds
  the capture window and the fingerprint tolerance. **It is the binding object
  three times over.** Narrowing it — or giving it a graspable feature of
  smaller depth — helps in all three places at once, and is the single highest-
  value change available to the object set.

### The honest summary

**Vision decides *where to look* and *whether the scene changed*. It does not
decide the grasp pose to millimetres — the known-dimension fit does that, and
it is the part of the pipeline carrying the accuracy.** Nothing here needs a
fiducial. What is still missing is the detection *rate* on real frames, and no
amount of geometry substitutes for it.
