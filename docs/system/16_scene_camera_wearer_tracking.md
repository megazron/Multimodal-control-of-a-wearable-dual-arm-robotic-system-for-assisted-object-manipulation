# 16. The scene camera, and tracking the wearer's actual body

**DESIGN ONLY. Nothing in this document is built.** It is a plan with costs so
the decision to build it can be made on the numbers.

Scope: one USB camera on the laptop, giving a front view of the table, the
objects and the wearer. Its purpose is not another view of the workspace --
the two wrist cameras already give that. Its purpose is to make the wearer's
body a MEASURED quantity instead of a declared one.

---

## 0. WHY THIS IS WORTH BUILDING, IN ONE PARAGRAPH

Every clearance number in this project is measured against
`wearer_posture.wearer_model()` -- a torso box, a head, hips, two thigh
cylinders, and per side an upper arm (r 0.050, l 0.30), a forearm (r 0.045,
l 0.26) and a hand, placed by a posture name from `SRL_WEARER_ARMS`. That is a
mannequin. It is not the person who will be wearing 17 kg of arms. The 150 mm
clearance floor, the "innermost safe column" of 0.325 (L) / 0.450 (R), the 28
predictive-avoidance refusals that all name `end_effector_link`, T0's ~142 mm
breach: every one of those is a statement about the mannequin, and every one
of them changes if the real person is broader, narrower, taller or simply
standing differently. A camera that measures the body converts that whole
table from an assumption into a reading.

**And that is exactly why it is dangerous.** The clearance floor is the last
thing between the arms and a person's chest (HARD CONSTRAINT 11). Replacing a
conservative fixed model with a live estimate replaces a known error with an
unknown one, and a body estimate that is 100 mm too far away does not fail
loudly -- it produces confident clearance numbers that are wrong in the
direction that hurts someone. Section 6 is the most important section here.

---

## 1. WHAT TRACKS THE BODY

Three candidates, and the recommendation is a combination of the first two.

### (a) 2-D pose estimation from the colour image, lifted with a depth prior

A single RGB frame through a joint detector (MediaPipe Pose, MoveNet, RTMPose)
gives ~17-33 keypoints in image coordinates at 20-30 Hz on a laptop CPU.
Depth is NOT recovered: monocular 2-D pose is scale-ambiguous.

To place the skeleton in the world it needs one distance. Three ways to get
one, best first:

1. **The wearer is standing on a known surface at a known distance.** The rig
   is worn; the table is at a measured height with a measured near edge
   (`work_surface.DECLARED_M` 0.980, `DECLARED_NEAR_Y` 0.100). Solving the
   scale from "the feet are on the floor and the floor plane is known" is a
   one-parameter fit and is stable.
2. **The mount is visible and its size is known.** The backpack frame is a
   rigid body whose dimensions are in the URDF, and `base_link` sits a
   measured 0.1610 m from the torso. Detecting the mount gives both scale and
   the camera-to-robot extrinsic in one step, which is the same trick the
   AprilTag detector already performs (`srl_perception/apriltag_detector.py`).
   **Put a tag on the frame.** This is by far the cheapest reliable answer.
3. Assume an average limb length. Rejected: that is the mannequin again,
   wearing a camera.

### (b) Depth, if the camera has it

An RGB-D camera (RealSense D4xx, Astra, Kinect Azure) removes the scale
problem entirely and gives something better than a skeleton: **the actual
occupied volume**. `srl_perception/surface_from_depth.py` and
`work_surface_node.py` already consume depth, and `mock_rgbd_camera.py`
publishes on exactly the topics a real driver owns, so the substitution is
proven at the interface.

With depth the body model can be a **point cloud carved into a small number of
capsules**, which is the shape `clearance.point_clearance()` already measures
against (`_dist_point_cyl`, `_dist_point_box`, `_dist_point_sphere`). No new
clearance mathematics is needed at all -- only new numbers for the primitives
that are already there.

### (c) Nothing at all -- measure the person once, with a tape

Worth stating because it is the honest baseline and it is nearly free. Ten
minutes with a tape measure per participant gives chest width, shoulder
height, upper arm and forearm length. `wearer_posture.py` already has ONE
source for those primitives and already writes both the URDF block and
`mount_guard_node.WEARER` from it; a per-participant measurement file is a
data change, not a code change.

**This is the recommendation for the first participant session regardless of
what else gets built**, because it removes the mannequin's size error without
adding any new failure mode, and because a live tracker needs it as its
prior anyway.

### RECOMMENDATION

**Tag-on-the-mount for extrinsics + monocular 2-D pose for POSTURE + a
per-participant measured size prior for SCALE.** Depth if a depth camera is
available, which upgrades posture from a skeleton to a volume and removes the
scale fit. Explicitly: the camera measures *how the person is standing*; the
tape measures *how big they are*. Asking the camera for both is what makes
monocular body tracking unreliable.

---

## 2. HOW A TRACKED BODY UPDATES THE PLANNING SCENE

The trap here is already documented and already cost this project a day:
**two clients writing one planning scene** (`docs/system/findings.md`, "two
clients, one planning scene" -- a verifier run overlapping a sweep read 18 of
171 obstructed where two clean runs read 0). A tracker that pushes collision
objects at 20 Hz into the same `move_group` that a sweep is planning against
reproduces that at 20 times a second.

So the tracker does **not** write the planning scene.

    scene camera -> wearer_tracker_node -> /wearer/estimate  (JSON, ~10 Hz)
                                              |
                       wearer_posture.wearer_model(estimate)  <- ONE source
                                              |
                            +-----------------+------------------+
                            |                                    |
                 clearance.Model (geometric,               a SINGLE writer,
                 the check that actually binds)            rate-limited, that
                 read live by the follower, the            maintains ONE
                 homing node and the bridge                collision object
                                                           in move_group

Three properties this shape has and a direct-write does not:

* **The binding check needs no scene write at all.** The wearer check is
  geometric and is HARD CONSTRAINT 11's own check; `avoid_collisions` is not
  the wearer check, because the SRDF excludes the 44 proximal pairs a shoulder
  mount actually threatens. Feeding the estimate to `clearance.Model` is
  therefore the part that changes behaviour, and it fights nothing.
* **The planning scene gets ONE object from ONE writer at a low rate** (1-2
  Hz, and only when the estimate has moved more than a threshold), so MoveIt
  is never re-planning against geometry that changed under it mid-solve.
* **`wearer_posture.py` stays the single source.** It already writes both the
  URDF arm block and `mount_guard_node.WEARER`, with
  `test_wearer_posture_has_one_source.py` enforcing it. A tracker that grew
  its own body model would be the sixth copy of the home pose all over again.

**The estimate is a POSTURE plus a SIZE, not a mesh.** It selects and
parameterises the primitives `wearer_posture` already defines. That keeps the
whole thing inside the existing one-source rule and inside the existing
clearance mathematics.

---

## 3. HOW IT COMBINES WITH THE WRIST CAMERAS AND THE ARMS' OWN SENSING

They answer different questions and must not be merged into one number.

| source | answers | does not answer |
| --- | --- | --- |
| scene camera | where the PERSON is, and the coarse layout of the table | anything at grasp scale -- it is metres away and sees the back of the gripper |
| wrist cameras | where the OBJECT is, at grasp range, from the tool frame | anything about the wearer; from home they do not even see the work surface |
| joint encoders + FK | where the ARM is, exactly | where anything else is |

The composition rule is **the arm's own sensing always wins about the arm**.
The scene camera's view of the gripper is the worst estimate of the gripper's
position in the system and must never be fused into the arm state. Its output
is used for exactly two things: the wearer's body, and a coarse table/object
prior that the wrist camera then refines.

The one genuine overlap is object detection: the scene camera can see cubes
the wrist camera has not looked at yet. That is a **prior for where to look**,
never a substitute for looking -- TASK_SPEC P-4 forbids the substitution, and
for the same reason: a blind pick is indistinguishable from a working
perception path.

---

## 4. WHAT IT CHANGES PER TASK

Numbers below are the current mannequin-based readings from
`recordings/baselines/home_change_applied.json` and CLAUDE.md's clearance row.

| task | today | with a measured body |
| --- | --- | --- |
| **T0** target reaching | breaches the floor by ~142 mm (L) / 137 mm (R), 4 IK failures per arm | the breach is measured against the real chest. If the wearer is narrower it may partly disappear; if broader it gets worse. Either way it stops being a number about a mannequin |
| **T1** stage 1 | the only clean task | should stay clean; the value is CONFIRMING that against the person actually wearing it |
| **T1** stage 2 | seed 0's right arm clean since 2026-08-17 | per-seed clearance becomes a live check rather than an offline sweep, so a seed can be rejected at draw time |
| **T2** coordinated carry | breaches by 150 / 147 mm, left arm fails 1 waypoint | the worst offender. The carry path runs across the chest, which is precisely where a body-size error lands |
| **T3** circuit box | left arm breaches by 88 mm | smallest change |
| **the dance** | 0 of 1298 waypoints failing after the envelope moved to \|x\| 0.47-0.76 | the envelope was chosen against the mannequin. A live body lets it be chosen per wearer, or lets a wearer be told the routine is not safe for them |

The cross-cutting change is bigger than any row: **posture stops being a
launch-time environment variable**. `SRL_WEARER_ARMS` has five settings and
CLAUDE.md already records that two of them (`behind`, `out`) put the wearer's
own limbs inside the floor at HOME. Today nothing notices if the wearer simply
moves their arms mid-run. With tracking, that becomes an observable.

---

## 5. WHAT BREAKS IT

| condition | what happens | what it looks like |
| --- | --- | --- |
| **the arms occlude the wearer** | the robot is mounted on the person and works in front of them, so it is between the camera and the torso for a large part of every task | keypoints drop out or, worse, the detector locks onto the ROBOT's limbs as if they were the person's. This is the dominant failure and it is systematic, not occasional |
| low or side-lit room | keypoint confidence collapses; a 2-D detector fails gracefully to low confidence, which is usable | confidence falls, tracking goes UNKNOWN |
| backlight (window behind the wearer) | silhouette only; depth on a stereo camera fails entirely | confident-looking keypoints on the silhouette edge, i.e. wrong and confident |
| the wearer wears the same colour as the background | 2-D pose is fine (it is learned, not colour-keyed); a depth camera is fine; a colour-segmentation approach fails | this is why colour segmentation is not proposed |
| camera knocked or moved | every body position is wrong in a rotated frame, with no other symptom | **identical in shape to the VR tracking-reference failure already documented**: poses stay valid, rate stays up, and every number after it is wrong. The answer is the same -- latch the extrinsic at start and freeze on movement past a threshold |
| second person walks behind the wearer | the detector may track them instead | the body jumps. Mitigation: keep only the detection nearest the known mount position |
| USB bandwidth | a second UVC camera on the same controller can starve the wrist cameras | frame rate on the WRIST cameras drops, which reads as a perception fault somewhere else entirely |

---

## 6. WHEN IT IS ABSENT, OCCLUDED OR WRONG

**A wrong body model is more dangerous than no body model.** This section is
the design, not a caveat on it.

### 6.1 The tracker is never authoritative on its own

The estimate carries a per-part **validity and age**, and `clearance.Model`
takes the **more conservative** of the tracked part and the mannequin part,
per part, always:

    effective_part = the one whose surface is CLOSER to the arm

That single rule gives the required behaviour in every case without a mode
switch:

* tracker absent -> mannequin, exactly as today;
* tracker says the person is FURTHER away than the mannequin -> the mannequin
  still binds, so a hallucinated retreat buys nothing;
* tracker says the person is CLOSER than the mannequin -> the tracker binds,
  which is the case where it adds safety;
* tracker jitters -> the envelope only ever grows towards the arm, so jitter
  costs workspace and never safety.

The asymmetry is deliberate and is the whole design: **the camera may only
make the wearer bigger, never smaller.** Getting the shrinking case right is
impossible to guarantee from a monocular estimate, so it is not attempted.

### 6.2 Staleness is a hard gate, not a fade

Same rule as the camera panels in the GUI and `camera_relay.ChannelState`: a
frame past its age limit is not used, not dimmed. A stale body pose is
indistinguishable from a live pose of a person standing still, and it is the
"data fresh but never changes" row of CLAUDE.md's instrument table. Past
~300 ms the tracker contributes nothing and the mannequin binds alone.

### 6.3 The extrinsic is latched, and a knock freezes it

Copied wholesale from `vr_safety_node`, which already solves this exact
problem for the headset: latch the first camera pose (from the tag on the
mount), and if it moves past **20 mm / 2 deg**, freeze the tracker's
contribution and say so. The freeze must survive the ordinary unfreeze path,
because valid-looking data flows the whole time it is wrong. A deliberate
`/wearer/rebase_reference` is the only way back, and it tells you to
re-calibrate.

### 6.4 It has to be able to fail on a deliberately broken input

Non-negotiable, per the standing rule. The known-answer test set:

1. a recorded clip with the wearer's real dimensions measured by tape --
   tracked size must match to a stated tolerance;
2. the same clip with the camera image rotated 5 deg -- the extrinsic freeze
   must fire;
3. a clip with the wearer absent from frame -- must read UNKNOWN and fall to
   the mannequin, never "clear";
4. a clip with the robot arm crossing the torso -- must not attribute the
   arm's limbs to the person;
5. an injected offset that moves the estimate 100 mm AWAY from the arm -- the
   effective clearance must NOT improve, which is 6.1 made executable.

Test 5 is the one that matters. If it passes, the feature cannot reduce
safety; if it cannot be written, the feature should not be built.

---

## 7. WHAT IT COSTS TO BUILD

Estimates are working sessions on this codebase, at the pace the rest of it
has actually moved.

| item | cost | notes |
| --- | --- | --- |
| USB camera driver + calibration + one topic | **0.5 session** | `usb_cam` or `v4l2_camera`; intrinsics from a checkerboard. NOTE: this box has no `/dev/video*` at all, so it is untestable in WSL without usbipd attach |
| AprilTag on the mount, extrinsic solve, latch-and-freeze | **1 session** | `apriltag_detector.py` exists; this is a second consumer of it plus the `vr_safety_node` freeze pattern |
| Per-participant measured size prior, into `wearer_posture` | **0.5 session** | data + one loader; the one-source test already exists and must be extended |
| `wearer_tracker_node`: 2-D pose -> `/wearer/estimate` | **1.5 sessions** | MediaPipe is not installed and is not in the ROS index; would be pinned as a Python dependency. `onnxruntime` IS available, so an ONNX pose model avoids a new heavy dependency |
| `clearance.Model` takes a live estimate, more-conservative-per-part | **1 session** | the mathematics is unchanged; this is plumbing plus the asymmetry rule |
| Single rate-limited planning-scene writer | **0.5 session** | must not run while a sweep runs; same rule as HARD CONSTRAINT 3 |
| Known-answer tests 1-5 above | **1.5 sessions** | needs recorded clips WITH tape-measured ground truth. This is the expensive part and it is not optional |
| GUI: the body's rung, its age, and which model is binding | **0.5 session** | one row in the connection panel's pattern; see doc 18 |
| **total** | **~7 sessions** | plus hardware |

**Hardware.** A plain UVC webcam is ~GBP 30 and gives (a). A RealSense D435 is
~GBP 250-300 and gives (b), removes the scale fit, and upgrades the body from
a skeleton to a volume. Given that the scale fit is the least reliable part of
the monocular route and that the depth topics already have a mock and a
consumer in this repo, **the depth camera is the better buy** if the money
exists.

**What to build first, if only one thing is built:** the per-participant
tape-measured size prior (0.5 session, no hardware, no new failure mode). It
removes most of the mannequin error. The camera then improves POSTURE, which
is the smaller half.
