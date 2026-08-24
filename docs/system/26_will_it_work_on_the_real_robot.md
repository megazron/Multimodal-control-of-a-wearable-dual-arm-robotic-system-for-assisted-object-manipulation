# Will this work on the real robot, in any condition?

Asked directly, so answered directly. **No — not yet, and here is every reason
in the order it will bite you.** Each row is checked against the source, not
remembered.

Nothing in this repository has ever run against a real Kinova. That is not a
disclaimer at the bottom of this page; it is the first line of it.

---

## THE BLOCKERS — things that will stop you on the day

### 1. The pick drives the SIMULATION, not the arm

`pick_from_map --execute` published to `/<arm>_arm_controller/joint_trajectory`.
`kortex_highlevel_bridge` subscribes under its `real_ns` parameter, which
defaults to **`/real`**. A bare topic never reaches it.

So every "it picked the object" in this repository is the simulated arm. Fixed
so far as it can be without hardware: `--drive-real` publishes under `/real`,
it is **not** the default, and the tool now prints which robot it is talking
to on every run. The bridge's own arming gate and home refusal still sit on
top.

**Untested against hardware.** It cannot be otherwise here.

### 2. Sim home and real home are 1.9 rad apart, by design

Hard constraint 0. Sim home is the presentation pose; the physical arms are at
the legacy Kortex home with the wrists pointing up. The bridge **refuses to
enable** on the difference rather than commanding it — correctly, because
commanding it is a 110° wrist snap.

The new pose has to be captured on the arms before any real session. Values in
Kortex degrees are in `NEXT_SESSION.md`.

### 3. The gripper camera topic is a guess

The window and the whole calibration path read
`/<arm>_camera/color/image_raw`, which is what `mock_rgbd_camera` publishes.
**What the real Kinova wrist camera publishes has never been observed here.**
The window subscribes to three candidate names and prints which one delivered,
so the first real session will tell you in one glance instead of one
afternoon — but the name itself is unverified.

### 4. `camera_link` has never been checked against the physical module

The camera pose is forward kinematics: exact given the URDF, and the URDF's
`camera_link` has never been compared with where the camera actually sits. A
10 mm error there lands directly in every point of every map, and nothing
downstream can see it. It is the cheapest high-value calibration left and it
is not done.

### 5. Seven of fourteen master channels are incoherent

A soldering problem, `l_j2` and `l_j4` named by regression. Master teleop is
the mode most likely to refuse.

### 6. One Kortex session, ever

Hard constraint 2. SIGINT the bridge; SIGKILL leaks the session and the next
run cannot connect.

---

## THE THINGS THAT WILL BE WRONG BUT NOT STOP YOU

### The arm parks 0.305° short, every joint, every time

Measured: 211 of 252 recorded per-joint errors at ±0.25–0.30°, sign a coin
flip. **7.2 mm at the end effector on a 100 mm move.** It is a constant
terminal offset, not a gain — so it does not shrink on short moves, it
dominates them.

Against the map's own 11.3 mm and a 30 mm capture gate, that is most of the
remaining budget. `terminal_overshoot` and the tightened deadband exist for it
and **neither has been tried on hardware**.

### The wearer check is not what MoveIt calls collision

Hard constraint 11. The SRDF excludes the 44 proximal pairs a shoulder mount
actually threatens, so a pose MoveIt calls valid can put the tube inside the
person. Every bound this calibration sets is clipped to columns measured
geometrically, never to what IK will accept — but the *pick* itself is checked
by IK, and IK is not that check.

### The scene camera has no depth and may not exist

It is a monocular colour camera and is used only to notice change. If it is
absent, QUICK CHECK refuses and says so; FULL SCAN is unaffected.

### Speed has never been varied

The "speed sweep" was three replicates of one condition — `vmax_rad_s` was
read once at bridge start-up and 7 of 36 runs moved faster than the limit they
were commanded with. **How the error varies with speed is unmeasured.**

---

## WHAT SHOULD SURVIVE CONTACT

Stated as expectation, not fact, because none of it has met hardware.

* **The perception**, because it measures rather than assumes. Moving the
  table 350 mm and dropping it 300 mm changed nothing about the answer.
* **The refusals.** Out-of-reach tables, stale maps, absent cameras, moving
  captures and unreachable objects are all refused by name. On a real cell
  these fire more often, which is the point.
* **The reachability cache**, which is keyed on geometry and re-probes itself
  when anything moves.
* **The stillness gate**, which matters *more* on real hardware: a real camera
  has an exposure time and a moving arm smears the cloud, where the simulated
  one does not.

---

## IN WHAT ORDER TO FIND OUT

1. Capture the presentation pose on the arms. Nothing else can start.
2. Attach a wrist camera. Read the topic name off the window's camera panel.
3. Hand-eye the wrist camera — `camera_link` against the real module.
4. FULL SCAN with the arms **powered but the bridge not armed**. Perception
   only, no motion. Compare the map with a tape measure.
5. Only then a pick, `--drive-real`, one object, hand on the e-stop.
6. Then the 0.305° park: four runs settle whether the overshoot or the
   deadband wins.
