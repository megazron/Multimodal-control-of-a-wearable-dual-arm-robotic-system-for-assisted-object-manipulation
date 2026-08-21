# NEXT SESSION — 2026-08-22

## THE ONE-PARAGRAPH VERSION

Both real arms were driven repeatedly on 2026-08-21 and the perception stack
found the cube every time. Nothing was picked up. The reasons were **never**
detection: they were calibration, kinematics, and eight defects in the code
that commands the arms — seven of them written that same day, one of them
sitting in `real_homing_node` since the wearer guard was added. The rig
`scripts/real_calibration/` now exists to stop that class of failure, and
every module in it must recover a known answer AND fail on a deliberately
broken input before it is trusted. **The next session is a redesign, not a
retry**: one calibration that is measured rather than assumed, one RViz
master that serves every control mode, one GUI that works in all of them, and
a pick-and-place path that does not care what the object is.

---

## READ FIRST, IN THIS ORDER

1. `scripts/real_calibration/README.md` — what exists, what it measured, and
   what is explicitly still unverified.
2. This file.
3. `docs/NEXT_SESSION_2026_08_21.md` — the previous session, whose "why the
   cube was not picked up" section is now partly answered and partly wrong.

Run `python3 scripts/real_calibration/check_all.py` before touching hardware.
Four modules, all passing. It proves the INSTRUMENTS work and says nothing
about the arms.

---

## WHAT REAL HARDWARE TAUGHT US

Eight defects, each found by the arm behaving differently from the model.
They are listed because the redesign has to be immune to all of them, not
because they are fixed — most are fixed only inside `real_calibration/`, and
the rest of the repo still has the same shapes.

| # | defect | how it presented | where it lives now |
| --- | --- | --- | --- |
| 1 | **the wearer guard could never resolve** | `0 of 9 parts checked` in every run ever made. The arm link was looked up as `real_<arm>_<link>` and the wearer part as a bare `torso`; the real stack publishes everything under `real_` | FIXED in `real_homing_node.py`, one named `REAL_FRAME_PREFIX` used on both operands |
| 2 | **only waypoints were checked, never the path** | a pick cleared three poses at 0.4 m and commanded a joint-space sweep between them that passed through the wearer | `safe_motion.check_path` densifies to 0.05 rad and checks every sample |
| 3 | **`goto()` reported success after a timeout** | a move that stalled 39.89 deg short was logged "reached" and the next waypoint commanded from it | `safe_motion.goto` RAISES |
| 4 | **liveness detected our own CPU load** | `no joint state for 2.4 s` with ZERO bridge errors and joint states flowing at 17 Hz — the optimiser holds the GIL and the spinner starves | `assert_live` stops computing and waits before concluding |
| 5 | **continuous joints differenced across the seam** | `288.39 deg from home` for an arm 69.5 deg from home; would have commanded the long way round | `safe_motion.delta` wraps joints 1/3/5/7 |
| 6 | **±π treated as a joint limit** | `joint limit violated at step 10/46`, direction recorded as `reached 0.000 m`. `srl_fk.limits()` says in its own docstring that this is a SEARCH BOX | `check_path` enforces limits only on non-continuous joints |
| 7 | **a thin search declared a workspace boundary** | `unreachable ... 9 seeded starts` became the recorded extent of the arm | full 200-seed search before any limit is recorded |
| 8 | **the run summary was written once, at the end** | the left arm completed four directions and saved **zero files** | `points.jsonl`, appended and fsynced per point |

**The meta-lesson, and it is the one worth carrying:** in six separate cases
the instrument was wrong and the hardware was fine. Twice I told the operator
to go check a power fault that did not exist. CLAUDE.md's standing rule
already covers this — *a surprising failure is evidence about the INSTRUMENT
until the instrument has been cleared* — and it was violated by writing new
instruments without known-answer tests. Every new module below ships with one
or it does not ship.

---

## WHAT IS ACTUALLY MEASURED, AND WHAT IS NOT

**Measured on the real arms, 2026-08-21:**

* Home is reachable and repeatable: left 0.10–0.26 deg, right 0.12–0.75 deg,
  EE within 1.9 / 3.7 mm of FK.
* Commanded-vs-achieved EE error over 19 recorded points: **9–21 mm**. This is
  the real tracking error and nothing in the repo had ever recorded it.
* The two arms are **not symmetric in what stops them**. Direction −1−1−1:
  the right arm runs to 0.750 m and stops on kinematics (202-seed search); the
  left stops at 0.450 m on the **wearer**, hand placeable to 3.7 mm but
  clearance 0.0962 m against the 0.15 floor.
* The mount cap is **0.2202 m** — the whole-chain clearance reads that
  constant for every pose ever tried. Always use `moving_chain_m`.
* RealSense over usbip: depth **424×240 + colour 640×480 @15** is the only
  configuration that delivers fresh depth. 640×480 depth **silently
  re-delivers a frozen frame** at what looks like full rate.
* The camera is mounted upside down: rotated principal point
  **cx′=320.13, cy′=247.78** against native 318.87 / 231.22. Using the native
  value on a rotated image displaces a ray **69.2 mm at 2.5 m**.

**NOT measured, and do not let anyone quote it as if it were:**

* **The camera→robot extrinsic.** The solver is validated on synthetic data
  (0.124 deg, 1.6 mm) and has **failed on every real recording** — 0%
  coverage. Cause identified on the last run and not yet fixed: the
  foreground threshold was 0.08 m while the sensor's own noise at 4 m is
  0.082 m, so the "arm" mask was 55 000 pixels of noise across the whole
  frame. A threshold sweep says 0.80 m gives a clean 7 426-pixel blob at
  2.91–4.82 m. **One line, untested.**
* Anything about the left arm's reliability. It dropped six times with real
  `read failed` / `Broken pipe` / lost ARP — but four other "drops" were
  defect 4, so treat the count as suspect until it is re-observed with the
  fixed instruments.

---

## THE FOUR THINGS TO BUILD

### 1. CALIBRATION THAT IS MEASURED, NOT ASSUMED

This is the blocker for everything else and it comes first.

* **Fix the foreground segmentation** (threshold ~0.8 m + largest connected
  component + a minimum blob area), then run `solve_extrinsic.py` against a
  full sweep. Gate: **held-out** residual, not fit residual, and the coverage
  gate stays.
* **Add a physical ground truth.** The whole extrinsic effort has no
  independent check. Cheapest honest one: drive the gripper to N known poses
  and have a human mark where the *tip* actually is, or put a marker on the
  gripper for the calibration run ONLY. The objection to a marker was that it
  is inelegant; the cost of not having one was a day. A marker-free method
  that cannot be checked is not better than a marker.
* **Hand-eye for the wrist camera too.** Its pose is FK, which is exact only
  if `camera_link` in the URDF matches the physical module. Nothing has ever
  checked that. A 10 mm URDF error is invisible and lands directly in every
  grasp.
* **Finish the workspace sweep**, both arms, all 26 directions, with
  `points.jsonl` on. That replaces `arm_reach_extents.json`, which collapses
  three different stop reasons into "unreachable".
* **Re-measure the tracking error as a function of speed.** 9–21 mm at
  `vmax 0.15`. Dexterity claims mean nothing without this curve.

### 2. THE RVIZ MASTER, REDESIGNED

Today's GUI embeds RViz showing COMMANDED beside a native ACTUAL panel. That
was built before any real-arm experience. What real arms showed it needs:

* **Three states, not two.** COMMANDED, ACTUAL, and **PLANNED-BUT-REFUSED** —
  the path the checker rejected and why. Every refusal today was invisible
  unless you read a log.
* **The wearer floor as geometry, drawn.** The clearance number is
  meaningless next to a robot; the 0.15 m shell around the tracked body is
  not. Colour the arm segment that is nearest and label the part it is
  nearest to — `moving_chain_to` already carries this.
* **The whole path, not the endpoints.** Draw the densified sweep. Defect 2
  existed because nobody could see the middle of a motion.
* **Liveness on the face of it.** Joint-state age per arm, session state,
  and the bridge's own error count. Four of today's "arm dropped" calls would
  have been settled in one glance.
* **Workspace envelope overlay**, from the sweep — reachable, wearer-limited,
  and out-of-reach as three distinct volumes per arm.

### 3. A GUI THAT WORKS IN EVERY CONTROL MODE

The mode list is `m0 m1 m1s2 m2 m3`, plus VR/desk teleop. The audit
(`verify_gui_buttons.py`) passes at 203/203 but it audits buttons, not modes.

* One panel per mode, each with the same four things: what it will command,
  what it refuses and why, live divergence, and a stop.
* **Every refusal reaches the screen with its reason string.** The refusal
  text in `real_calibration` is written to be read by an operator; nothing
  currently shows it.
* The audit must **exercise each mode**, not just press each button. An audit
  that cannot fail is not an audit — and this one already hung on a modal
  dialog for weeks without anyone noticing.

### 4. UNIVERSAL PICK AND PLACE

Today's grasp was hand-built for a cube on a stool and still needed three
corrections (jaw axis 31° off vertical would have hit the surface; the centre
was shell-biased 20 mm low; the approach was nearly parallel to the jaw
axis). That does not generalise. What does:

* **Support-plane awareness is not optional.** Every one of those three bugs
  came from ignoring the surface the object rests on. Segment the plane
  first, then constrain the grasp against it.
* **One depth view is a shell.** The repo already knows this
  (`test_a_SHELL_from_one_depth_view_is_not_gripped_along_the_line_of_sight`)
  and my hand-built planner still put the centre 20 mm low. Either fuse the
  wrist and scene cameras, or take two wrist views.
* **Coarse-to-fine is the right architecture and it worked.** Scene camera to
  aim (±160 mm was ample), wrist camera + FK for the final pose (exact, no
  calibration). Keep it.

---

## MODELS — WHAT WOULD ACTUALLY HELP

**Read this before downloading anything: detection was never the bottleneck.**
FastSAM + the colour/depth ensemble found the cube at score 1.0 with 1166
depth points and zero false positives from the teal robots. What failed was
3D localisation, calibration, and the motion code. A better detector fixes
nothing that broke on 2026-08-21.

**The hard constraint is VRAM: this machine has an RTX A500 with 4 GB.**
CLAUDE.md already records that YOLO-World was chosen over GroundingDINO
(218M) and OWLv2 (428M) for exactly this reason. Anything below must fit or
run on CPU.

Where a model genuinely earns its place, for *universal* pick-and-place:

| need | candidate | honest assessment |
| --- | --- | --- |
| 6-DOF grasp synthesis on arbitrary objects | [**GraspGen**](https://graspgen.github.io/) (NVIDIA, diffusion + discriminator, 2026) and [**GraspGen-X**](https://arxiv.org/html/2606.00998v1) (cross-embodiment, trained on 2B grasps, open-sourcing announced) | the strongest current answer to "grasp anything". Cross-embodiment matters here because the Robotiq 85 is not the gripper most models assume. **Check the VRAM before committing** |
| same, smaller and proven | [**Contact-GraspNet**](https://github.com/NVlabs/contact_graspnet) | 4-DoF-reduced representation from a single depth recording, 17M simulated grasps, generalises to real sensors. Older, lighter, well-trodden — the sane first thing to try on 4 GB |
| same, best-in-class but licensed | **AnyGrasp** | billion-scale, 7-DoF from a point cloud. Licensing is restrictive — check before it becomes load-bearing |
| known-object pose + tracking | [**FoundationPose**](https://nvlabs.github.io/FoundationPose/) | model-based or model-free from a few reference images, no fine-tuning. Useful once the task is *specific* objects rather than *any* object |
| text-prompted 6D pose, no CAD | [**open-vocabulary 6D pose**](https://arxiv.org/html/2312.00690v3) (Corsetti et al.) | matches the "name it and pick it" goal, needs two RGBD viewpoints — which this rig can produce, scene + wrist |
| segmentation to feed the above | **SAM-6D**, Grounded-SAM2 | standard glue; FastSAM already works here and is far cheaper |

**Recommended order:** fix the extrinsic → finish the sweep → try
Contact-GraspNet on the wrist RGBD → only then evaluate GraspGen. And apply
the repo's own rule to every one of them: **a model's output is a
measurement, and it is an instrument until it has been cleared.** Score its
grasps against the support plane and the wearer floor before any of them
reach an arm.

---

## STANDING TRAPS (do not rediscover these)

* Two Kortex sessions is one too many. SIGINT the bridge; a dead session does
  **not** heal when the network returns — it sits there with a broken pipe
  looking alive.
* `pkill -f <pattern>` matches the shell running it. Kill explicit PIDs.
* The ros2 CLI daemon wedges; `ros2 topic list` returns nothing against a
  fully running stack. Use a small rclpy script instead.
* Never `rm -rf` a recording directory to "start clean". 43 captures and
  42 MB were destroyed that way. Runs are timestamped now — use a new one.
* The bridge zero-speeds after 0.5 s without a target. Any pause — a camera
  grab, a solve — needs a holder thread feeding the last target.
* `vmax_rad_s` and friends are live parameters now, but they were read once
  at startup for a long time. If a speed sweep gives identical results,
  suspect that first.
