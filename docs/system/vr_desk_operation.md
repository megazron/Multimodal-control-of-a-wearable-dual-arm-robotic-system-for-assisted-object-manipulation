# Desk operation — the controllers as motion capture, headset worn by nobody

**This is not a VR experience.** The operator sits across the room facing the
wearer, holds the two controllers as a 6-DOF input device, and watches the
real robot with their own eyes. The headset stands on a shelf as the tracking
reference. Nobody wears it, nobody sees a display, and the in-headset overlay
is irrelevant.

Everything below replaces the head-worn assumptions in `vr_teleop.md` and
`vr_bringup.md` Route B. The **transport** and the **robot side** are
unchanged; what changes is who is where, and what that does to the frames and
the failure modes.

---

## 1. THE TWO ROUTES, AND WHY (b) WINS

### (a) SteamVR / OpenVR over Quest Link or Air Link

| | |
| --- | --- |
| needs installed | **Meta Quest Link PC app** (not present — checked), **SteamVR** (not present — Steam is installed, SteamVR is not), an OpenXR runtime (**none registered**), `pip install openvr` |
| borrowed headset | **NO.** Link pairs the headset to the PC app through a **Meta account**, and the headset is on its owner's. Air Link additionally needs enabling in the headset's own Settings → Beta, which is a settings change. USB Link still needs the PC app signed in. |
| pose rate | ~90 Hz. `GetDeviceToAbsoluteTrackingPose` is poll-driven and predicted, so **no better than what WebXR already delivers** — the floor is the headset's frame period either way. |
| carries over | **All of `vr_pose_mapper`, `vr_gripper_node`, `vr_safety_node`, the clutch, the scaling and the reset logic.** They consume `/vr/controller_pose_*` and `/vr/controller_joy_*`; only the *source* changes. A new `openvr_bridge` would replace `quest_bridge_node` and nothing else. Conveniently OpenVR uses the same handedness as WebXR (+x right, +y up, −z forward), so `webxr_to_world_*` would apply unchanged. |
| other costs | SteamVR runs on **Windows**, so the client is a Windows-side Python process bridging into WSL. That hop is already proven (`networkingMode=mirrored` lets Windows reach a socket bound on WSL's `127.0.0.1`). Several GB of download, on a machine currently on a phone hotspot. |

### (b) Headset on a tripod, proximity sensor taped, WebXR — **RECOMMENDED**

| | |
| --- | --- |
| needs installed | **nothing.** Already built, already verified end to end. |
| borrowed headset | **YES.** No sideloading, no account, no settings changed. The proximity sensor is defeated *physically* — a folded scrap of paper — which is reversible in one second and leaves no trace. |
| pose rate | **89.85 / 89.89 Hz measured**, 0 dropped of 17267 frames, one-way latency estimate **9.88 ms** against a 30 ms budget. |
| carries over | everything, because it *is* the current system. |

**Recommendation: (b).** Not on preference — on the fact that (a)'s first
prerequisite is a Meta account on a headset you do not own, which is the exact
thing that ruled out the adb route in the first place. (a) also buys no rate:
both are bounded below by the headset's own frame period.

**Keep (a) written down** for the case where the lab acquires its own headset.
The porting cost is genuinely one node, because the topic contract
(`/vr/controller_pose_<hand>`, `/vr/controller_joy_<hand>`,
`/vr/controller_valid_<hand>`) is the seam.

---

## 2. WHAT THE DESIGN CHANGES

### 2.1 No display, no overlay

The operator has line of sight to the robot. The in-headset panel is dead
weight — the headset is on a shelf pointing at the operator's hands.

Status goes on a monitor beside them:

    python3 scripts/vr_desk_monitor.py

link, latency, per-arm clutch, scale, gripper, lag, and **TRACKED / OCCLUDED**
per controller. It publishes nothing; it is a viewer.

The overlay code is **kept, not deleted**: it costs nothing, and a head-worn
session is still the right way to debug a tracking problem.

### 2.2 The tracking reference is now a physical object that can be knocked

This is the failure mode desk operation *introduces*, and it is the nastiest
one in the system because **it announces nothing**. Every other failure has a
signature — a dropout stops the frames, an occlusion invalidates a pose, a
dead link stalls the rate. A nudged headset leaves the poses valid, the rate
at 90 Hz and the controllers tracked, and silently rotates the frame every
subsequent pose is expressed in.

**Where it sits:** on a shelf or tripod, lenses facing the operator, 1–1.5 m
away, at roughly chest height, on something that will not be leaned on. Its
cameras track the controllers, so it needs a clear view of the operator's
hands — the FOV is wide and the placement is not critical. What *is* critical
is that it does not move afterwards.

**What happens if it moves:** `vr_safety_node` latches the first HMD pose it
sees as the reference and freezes on **> 20 mm or > 2°**. A 10° twist barely
moves the headset and rotates the operator's entire frame — that is the case
the rotation threshold exists for. The freeze **survives the ordinary unfreeze
path**, because poses keep flowing the whole time the reference is wrong.
Clearing it is deliberate:

    ros2 service call /vr/rebase_reference std_srvs/srv/Trigger

and it tells you to re-run the yaw calibration, because the heading you
measured belonged to the old reference.

Switch it off with `watch_tracking_reference:=false` for a head-worn session,
where the HMD moves constantly by design.

### 2.3 The operator faces the wearer, so the frames are opposed

`vr_pose_mapper`'s docstring has always promised

    p_cmd = p_anchor + scale * R_align * (p_controller - p_ref)

and the code never had an `R_align` — the displacement was added raw, so the
controller frame *was* the world frame by assumption. That assumption is true
only for an operator standing behind the wearer facing the same way.

**`align_yaw_deg` is now real, and it is a YAW.** The intuitive correction —
"they're facing me, so mirror it" — is a **reflection**, det = −1. Applied to
position alone it looks right; applied to the pose it mirrors every
**orientation**, so the gripper rolls the wrong way while the positions look
correct. That is the hardest possible thing to see in a video, and it is why
the alignment is exposed as **one angle** rather than per-axis sign flips:
**no value of `align_yaw_deg` can produce a reflection**, and
`test_operator_alignment_is_a_rotation.py` pins that over ±720°.

The yaw is applied to the position *and*, conjugated, to the orientation
delta — `qz · dq · qz⁻¹`. Rotating one and not the other puts them in two
different frames and IK is asked for a pose that does not exist.

**Measure it, do not guess it:**

    python3 scripts/calibrate_operator_yaw.py --apply

The operator moves their hand ~40 cm **straight towards the robot** — a
direction both people in the room can identify without ambiguity. In the
robot's world frame that is known (the wearer faces the operator, so
operator→wearer is the wearer's −y), and one known direction is exactly enough
to fix one unknown angle. A second motion, the operator's own right, is
recorded as a **cross-check, not an input**: after the solved yaw is applied
the two must still be perpendicular and horizontal. If they are not, the
operator's frame is not a pure yaw — a tilted shelf, or the wrong motion — and
the script **refuses to write a number** rather than returning a confident
wrong one. It also refuses motions under 100 mm and motions more than 35° off
horizontal, which carry too little heading information.

Nothing moves during calibration: it reads the controller pose directly and
never engages the clutch.

**A note on why the axis question was not settled by eye.** There is a real
ambiguity in this repo. The docs say world **+x = the wearer's right**, and
the arms sit the other way round — `left_end_effector_link` at x = +0.117,
`right_end_effector_link` at x = −0.160, and `t1` places "the green pad at
x = −0.290 on the RIGHT arm". Either +x is the wearer's *left*, or the arm
named `right_` is physically on the wearer's left. Those need different fixes
and only one of them is an axis-map change, so the heading is **measured**
rather than argued from the naming. **This ambiguity is still open** and is
worth settling separately.

### 2.4 Safety

The operator **can** see the arm now, which is strictly better than the
head-worn case. **The observer e-stop requirement still stands** and
`require_observer_estop` stays `true`. The justification weakens; the
requirement does not. The operator is across the room, watching a 17 kg rig on
a person who did not choose the motion, and the person best placed to stop it
is next to the wearer, not next to the controllers.

Two gaps that desk operation turned from theoretical into likely, both now
closed:

* **Controller-level tracking loss did not freeze anything.**
  `/vr/tracking_ok` is derived from `last_rx`, stamped on every frame
  *arrival* before the per-controller validity check — so it caught a network
  dropout, a sleeping headset and a backgrounded app, but not the case it is
  named after. Measured with the real headset: covering a controller produced
  **zero** tracking-loss transitions while the stream ran on at 90 Hz. The
  bridge now publishes `/vr/controller_valid_<hand>`; the mapper freezes that
  arm and **refuses to engage** it until the controller is visible. Off-head,
  occlusion is routine rather than exceptional.

* **Nothing consumed `/vr/freeze`.** `vr_safety_node` computed a freeze and
  published it and no one listened, so "the observer e-stop was withdrawn" and
  "the tracking reference moved" were both states the system could be in while
  still commanding the arm. The mapper now honours it, drops the clutch, and
  refuses to re-engage while it stands. It defaults off and is only ever set
  by a message actually received, so a stack with no `vr_safety_node` — the
  recording sweep — behaves exactly as before.

---

## 3. RUNNING IT

    # 1. sim FIRST, so TF exists before the mapper starts
    ros2 launch srl_teleop teleop.launch.py gate:=false

    # 2. the VR input chain
    bash scripts/start_vr_wifi.sh

    # 3. status, on the monitor beside the operator
    python3 scripts/vr_desk_monitor.py

    # 4. headset on its shelf, proximity sensor taped, session entered,
    #    then measure the operator's heading
    python3 scripts/calibrate_operator_yaw.py --apply

Order matters: `vr_pose_mapper` refuses to engage without robot TF, and the
symptom is a clutch that silently does nothing.

`vr_pose_mapper` **resets between runs** — `reset()`, `/vr/reset` (Trigger) and
`/vr/reset_request` (Empty), called by `mode_upstreams.isolate()` on every run.
`align_yaw_deg` is deliberately **not** cleared by a reset: it is a calibrated
property of where the operator is standing, not per-run state. `scale` **is**
cleared, because the thumbstick moves it live.
