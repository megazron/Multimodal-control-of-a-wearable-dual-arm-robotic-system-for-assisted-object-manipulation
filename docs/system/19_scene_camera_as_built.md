# 19. The scene camera and the wearer tracker, AS BUILT

Doc 16 was the design. This is what exists, what it was measured at, and --
kept separate on purpose -- what is still inferred.

Built 2026-08-20. **No camera was attached to this machine while it was
built.** Everything below that says MEASURED was measured on real data
through the real code; everything that says INFERRED is a number this machine
cannot produce yet and is labelled as such rather than estimated into a table.

---

## 1. THE CAMERA

### Getting it into WSL

There is no USB in WSL. The camera is handed over with `usbipd`, exactly as
the Teensy is, and `scripts/attach_scene_camera.sh` finds the busid and
prints the commands rather than making you remember them.

On this machine the device is
`2-5  30c9:00c1  HP 5MP Camera, HP IR Camera, Camera DFU Device` -- the
laptop's own webcam, which is a UVC device on the USB bus and passes through
like any other.

```
# Administrator PowerShell, once. Survives reboots.
usbipd bind --busid 2-5

# Normal PowerShell, after every reboot or unplug.
usbipd attach --wsl --busid 2-5

# To give the camera back to Windows:
usbipd detach --busid 2-5
```

**The admin step is real and cannot be avoided.** Measured: `usbipd bind`
without elevation answers
`usbipd: error: Access denied; this operation requires administrator
privileges.` The `attach` step does not need admin.

**Windows loses the camera while it is attached.** It is one device and only
one operating system can have it.

The kernel side is ready: this WSL kernel has `CONFIG_USBIP_VHCI_HCD=m` and
`CONFIG_USB_VIDEO_CLASS=m`, so the modules load on attach. What it does not
have is a camera attached, and `scene_camera_node` says which of those is
wrong rather than reporting "no camera" for both.

### The node

`ros2 run srl_perception scene_camera_node`. Two behaviours it has that
`usb_cam` and `v4l2_camera` do not, and they are why it was written rather
than installed:

* **It refuses rather than republishing.** If the device stops delivering it
  publishes NOTHING and says so on `/scene_camera/state`. It never re-sends
  the last good frame. A frozen picture of a workspace cannot be told from a
  live one of a workspace where nobody is moving, and here the consumer is a
  collision model.
* **Its `CameraInfo` is real or empty.** Uncalibrated, it publishes a ZERO
  camera matrix and `calibrated: false`. A plausible guessed focal length
  would scale every body position silently.

It also refuses a device that OPENS but does not DELIVER -- this webcam
presents an IR sensor as a second `/dev/video*` that returns nothing, and a
node that accepted the first thing that opened would sit on it for ever.

`source:=file:<path>` publishes a still, labelled `NOT A CAMERA` in the state
message. The whole path was built and verified on that.

### Calibration

| what | tool | check |
| --- | --- | --- |
| intrinsics | `scripts/calibrate_scene_camera.py` | `--self-test` recovers a known camera matrix from projected chessboards: **fx to 0.43%, cx to 4.5 px over 20 views. MEASURED** |
| extrinsics | `scripts/calibrate_scene_camera_extrinsics.py` | `--self-test` recovers a known camera pose from a marker at three placements: **0.0000 m and 0.000 deg. MEASURED** |

Both tools run their self-test **before** every real calibration and refuse to
continue if it fails, because a calibration tool that is wrong produces a
calibration that is wrong in a way nothing downstream disagrees with.

Two real bugs were caught by those self-tests and would not have been caught
any other way:

1. **The synthetic board had 8x5 inner corners while the tool asked for 9x6**,
   so `findChessboardCorners` correctly refused all twelve views and the
   self-test reported "the tool cannot be certified" -- which was true, and
   about the test rig.
2. **The extrinsic solve ran in the marker's frame and the answer was read as
   a world pose.** `SOLVEPNP_IPPE_SQUARE` assumes marker-local object points;
   it was handed world-frame corners. The tell was a translation error of
   *exactly* 1.0500 m against a marker at z = 1.05: the answer was right, in
   the wrong frame. On a real camera that is a body 400 mm from where the
   person is, with full confidence.

The recommended extrinsic route is `--marker-on-mount`: a marker stuck to the
backpack frame, whose world pose comes from TF because `world -> torso ->
backpack -> mount` is a fixed chain. Nothing is measured by hand, so nothing
is measured wrong.

---

## 2. BODY TRACKING

### The method, and why

**MediaPipe Pose (`pose_landmarker_full`), markerless, plus PnP against the
calibrated intrinsic.** Reasons, in order of weight:

1. **It gives METRIC landmarks, not just image ones.** `pose_world_landmarks`
   are in metres about the hip centre, so the person's actual limb lengths
   come out of the detector. That is what makes "different people are
   different sizes" a measurement instead of an assumption, and it removes
   the scale ambiguity that makes monocular pose lifting unreliable.
2. **Its per-landmark visibility is a real discriminator, measured.** On a
   photograph of a person with folded arms the shoulders and hips read 1.00
   while the elbows read 0.09-0.11 -- and the "upper arm" it reports there is
   0.107 m long. The confidence and the nonsense arrive together, which is
   exactly what a per-segment gate needs.
3. **It runs at 28-34 ms per frame on this laptop's CPU. MEASURED.**
4. Markers on the wearer were the alternative and were rejected: the wearer
   is a participant who has to put the rig on, and five printed markers is a
   thing that can be forgotten, occluded by the harness, or put on backwards.

Absolute position comes from `cv2.solvePnP(SQPNP)` on the metric skeleton
against the image landmarks: the detector knows the SHAPE in metres, the
image knows the DIRECTION, and PnP is the only thing solved. Its residual is
published, so a bad solve is visible rather than silent.

**The robot arms are not detected visually.** They are PROJECTED into the
image from their own joint encoders. The arm's own sensing is a far better
estimate of the arm than a camera two metres away looking at a partly
occluded gripper, so fusing a visual arm estimate would make the arm state
worse. What the projection buys is the only real check on the extrinsic
there is: **if the drawn arm does not sit on the real arm in the picture, the
camera transform is wrong, and every body position is wrong with it.**

### Into the planning scene

The tracker publishes a `PlanningScene` diff holding exactly the wearer's
parts, from ONE writer, rate-limited, and **only when the geometry has
actually changed**. Two clients writing one planning scene cost this project a
day once (a verifier overlapping a sweep read 18 of 171 obstructed where two
clean runs read 0), and a 30 Hz stream of scenes would reproduce that at
thirty times a second.

Verified live: `move_group` holds twelve `wearer_*` collision objects, and the
per-part substitution is visible in them -- `wearer_torso` at
0.45 x 0.28 x 0.499 (the measured person) beside `wearer_L-upperarm` at
0.30 x 0.05 (the mannequin), in the same scene at the same time. **MEASURED.**

### Rate and latency

Measured over 25 s with the whole path running
(`scripts/measure_wearer_tracking_rate.py`):

| | |
| --- | --- |
| scene camera frames | **5.4 Hz** (a file source at `fps:=10`, sharing the CPU with the tracker) |
| wearer estimates | **6.7 Hz** |
| pose model alone | **28-34 ms per frame** |
| frame to estimate | **61.6 ms median, 357.8 ms p95** |
| estimate to planning scene | **500 ms worst case** -- the `scene_hz=2.0` rate limit |
| **end to end** | **62 to 562 ms** |

Two honest caveats on that range. The spread is the rate limit, not jitter --
`scene_hz` is a parameter and raising it moves the upper end. And it does
**not** include the camera's own shutter-to-ROS delay, which is typically
10-30 ms on a UVC webcam and is **not measurable from here**: there is no
timestamp on the frame older than the one this repository puts on it. That
number stays INFERRED until a real camera is attached.

---

## 3. THE FALLBACK IS THE SAFETY CASE

The rule the whole module is arranged around:

> **THE CAMERA MAY ONLY EVER MAKE THE WEARER BIGGER, NEVER SMALLER.**

Implemented in `wearer_tracking.fuse()` as: for each part, keep whichever of
the tracked and mannequin primitives is CLOSER to the robot. There is no mode
switch, because a mode switch is a thing that can be in the wrong mode. A
hallucinated retreat buys nothing; jitter costs workspace and never safety; a
dead tracker leaves exactly today's model.

### The gates

| gate | what it catches | threshold |
| --- | --- | --- |
| per-segment confidence | a part not clearly seen | 0.50, which sits in a real gap: measured 0.93-1.00 when clear, 0.09-0.22 when occluded |
| plausibility | a "limb" that is not a limb, however confident | upper arm 0.20-0.42 m, forearm 0.17-0.38, palm 0.055-0.145, shoulders 0.28-0.55 |
| speed | the detector jumping to a different object | 6 m/s per joint |
| age | a stale body pose | 0.30 s |
| frame gate | dark, blank, nobody, more than one person | luma < 12, span < 15, n != 1 |

**Confidence is the detector's opinion of its own output; plausibility is a
fact about human beings, so it overrules the opinion.**

### Tested by injection, through the live node

`scripts/verify_wearer_fallbacks.py` starts the REAL camera node on a REAL
broken frame and reads the REAL topic, six ways. **7 checks, 7 PASS.**

| injected | result |
| --- | --- |
| a clear body (the control) | PARTLY TRACKED -- 3 of 9 measured |
| lights off (3% exposure) | MANNEQUIN -- "too dark to read a body from" |
| covered lens (uniform mid-grey) | MANNEQUIN -- "blank -- covered lens, or a driver repeating one frame" |
| two people | MANNEQUIN -- "more than one person is in view" |
| an empty room | MANNEQUIN -- "nobody is in view" |
| arms folded | arms tracked: **none**; the torso may still be |
| after all that | the planning scene still holds `wearer_L-upperarm` at [0.3, 0.05] -- the mannequin |

**The check required the refusal to NAME the injected fault, and that caught a
false pass.** With the detector capped at two poses it reported one person in
the two-person photograph, so the two-person gate never ran and the tracker
refused for an unrelated reason -- every box green with the thing under test
unexecuted. The cap is four now.

`src/srl_perception/test/test_wearer_tracking.py` drives the same gates at
the module level, 21 tests, including the one that matters:
`test_a_body_further_away_never_improves_clearance` displaces the body 0.10,
0.30, 1.00 and 5.00 m AWAY and requires the model not to follow it.

---

## 4. WHAT IT CHANGES

Measured with `scripts/measure_wearer_size_effect.py` against a running
stack: whole-arm clearance through IK and FK over every distal link, at
z = 1.120, 0.20 m forward, N = 3, the 150 mm floor.

`config/wearer_sizes/measured_adult.json` is ONE adult measured through this
path from a single clear photograph. **It is not the wearer and n = 1.** It
exists so the question has a real answer rather than an invented one.

| | mannequin | measured adult |
| --- | --- | --- |
| upper arm | 0.300 | **0.256** |
| forearm | 0.260 | **0.247** |
| chest width | 0.360 | **0.430** |
| innermost clear column, arms down | L 0.375 / R 0.350 | **L 0.350 / R 0.350** |
| innermost clear column, arms folded | L 0.325 / R 0.325 | **L 0.350 / R 0.350** |
| what binds | the wearer's own forearm or upper arm | **the TORSO, in every case** |

### What changed

* **Arms down: the left column improves by 25 mm** (0.375 to 0.350). The real
  person's arms are shorter, so the forearm stops being the constraint.
* **Arms folded: the real person is 25 mm WORSE** (0.325 to 0.350), because a
  70 mm wider chest binds where the mannequin's folded arms did.
* **What binds changes identity.** Against the mannequin the constraint is a
  limb; against this person it is the torso, every time.

### What did not

**Asking the wearer to fold their arms is worth 50 mm to the mannequin and
NOTHING to this person** -- 0.375 to 0.325 for the model, 0.350 to 0.350 for
the measured body. That is the finding with an operational consequence: the
posture instruction the workspace measurements were partly justified by does
not help a broad person, because folding your arms does not narrow your chest.

### What is NOT claimed

The published innermost safe columns (0.325 L / 0.450 R) were certified over
the whole densified path at N = 10. The numbers above are a single-point probe
at N = 3. **They are different quantities and the script refuses to print
AGREES or DIFFERS against them.** An earlier version did print it, and duly
reported DIFFERS on three of four rows -- a fact about the comparison, not
about the robot. The valid comparison is mannequin against person, same probe,
same point, which is what is tabulated.

The reachable band itself is unchanged and cannot change: reachability is a
property of the arm and the IK solver, and `avoid_collisions` is blind to the
pairs a shoulder mount threatens (HARD CONSTRAINT 11). The wearer moves only
the clearance half of the safe column.

---

## 5. THE GUI

A **Wearer** tab, beside Instruct. Top line, in the largest text on the panel
and coloured:

    PLANNING SCENE: PARTLY TRACKED -- 3 of 9 measured: head, hips, torso

That line is the safety state. Everything else on the panel is diagnosis:

* the camera picture, **never painted when stale** -- the same rule the wrist
  camera panels obey, for the same reason;
* the tracked skeleton drawn ON it, coloured per segment by the confidence of
  its weaker end, so a torso seen at 1.00 with elbows at 0.10 is visible as
  such;
* **the robot arms projected from their own encoders**, in a third colour --
  the extrinsic check, on screen, continuously;
* a nine-row table: per segment, MEASURED with its confidence, or `mannequin`
  with the reason in plain words;
* `what would fix this`, which names the single most useful next action.

---

## 6. WHAT IS STILL INFERRED

Listed rather than buried, because this is the half a reader should not trust.

| | why |
| --- | --- |
| everything about real optics | no camera has been attached. Intrinsics, distortion, field of view, exposure behaviour, rolling shutter: none measured |
| shutter-to-ROS latency | not measurable without the device; 10-30 ms is a typical figure, not a reading here |
| the extrinsic in the lab | the self-test is exact on constructed geometry. A real marker on a real mount has not been solved |
| detection at working distance | the person in the reference photograph fills the frame. A wearer 2-3 m away in a lab with the robot arms crossing their torso is the case that matters and it is unmeasured |
| `measured_adult` | one adult, from one photograph, not the wearer |
| limb RADII | a single camera cannot measure limb thickness at all. The radii are the mannequin's, deliberately -- assuming a thinner limb is the direction that would make the model too small |
| the arms occluding the wearer | doc 16 named this the dominant failure. It cannot be exercised without a camera pointed at the real rig |

**The first lab session with the camera attached should re-run, in order:**
`scripts/attach_scene_camera.sh`, `calibrate_scene_camera.py --capture`,
`calibrate_scene_camera_extrinsics.py --live --marker-on-mount`,
`verify_scene_camera.py --live`, then `verify_wearer_fallbacks.py`, and only
then measure a real wearer into a size profile.
