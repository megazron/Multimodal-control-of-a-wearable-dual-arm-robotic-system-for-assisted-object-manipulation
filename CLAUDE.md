# kortex_ws — shared autonomy for a wearable supernumerary limb

Two Kinova Gen3 (7-DOF) on a backpack frame, driven from an instrumented
master mannequin arm over a Teensy 4.1, with a shared-autonomy layer.
ROS 2 Jazzy. **The arms are worn by one person and driven by a different
person**; both are participants.

**This file is the standing rules and the current state. It is deliberately
short.** The history, the worked examples and the reasoning behind every rule
live in `docs/system/`; each rule below points at where its detail is.

## READ THIS BEFORE YOU START READING

* **Read with `offset` and `limit`. Do not re-read a file you have already
  seen this session.** Read results have hit a third of the context window,
  which is time and money spent on text already in front of you.
* Prefer `grep -n` to opening a file. Prefer opening 40 lines to opening 4000.
* This file was 98k tokens on 2026-08-12 and is now under 15k. Keep it that
  way: new findings go in `docs/system/findings.md`, not here.

---

## HOW TO RUN THINGS

Source first, in every terminal:

```
source /opt/ros/jazzy/setup.bash && source ~/kortex_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=SHM        # NOT optional. See wsl.md
```

```
bash scripts/start_gui.sh               # THE ONE COMMAND. Sources everything,
                                        # refuses a second window, opens the GUI
ros2 run srl_teleop gui                 # same window, if the env is already set
ros2 run srl_teleop teleop_gui          # curses, SSH / no display only
bash scripts/run_teleop.sh              # teleoperation only
bash scripts/run_autonomy.sh            # + perception + shared autonomy
bash scripts/run_experiment.sh m1 --participant PILOT --scripted
bash scripts/calibrate.sh zero|gyro|imu-mount|max-position
bash scripts/diagnostics.sh             # before blaming the code
python3 scripts/sim_session.py --stack teleop --keep-up -- true
```

Three-terminal split for real work:

```
terminal 1:  bash scripts/run_teleop.sh gate:=false
terminal 2:  ros2 run srl_teleop live_monitor
terminal 3:  bash scripts/start_real.sh          # --mock to rehearse
```

**`gui` is primary.** `console` and `launcher` are superseded and absorbed;
do not start new work in them. Task keys are `m0 m1 m1s2 m2 m3` (displayed
T0–T3); `t*`/`e*` are the archived sets and are refused by name.

### First action of every lab session

```
bash scripts/check_channels.sh          # ~3 min, block A only
```

Which master channels are FROZEN is decided from the **newest**
`recordings/baselines/channels_*.json`. A baseline older than the wiring makes
the software freeze channels that now work, and nothing downstream disagrees.
Target: 12 or more of 14 coherent.

---

## CURRENT STATE, 2026-08-15

Re-measure rather than trust this table; each row names its command.

| | state | check |
| --- | --- | --- |
| unit tests | 906 pass, 2 allowed failures (`test_flake8`, `test_pep257`, pre-existing: the package uses double quotes). `check_tests.py` is the gate and knows the allowlist; `python3 -m pytest -q src/*/test` is the raw run | `python3 scripts/check_tests.py` |
| **home pose** | **the presentation pose since 2026-08-15**: wrists level, hands in front of the chest. Task set re-measured first — with the ANCHOR unchanged every task performs exactly as before. **The real arms are still at the legacy home**; the bridge refuses to enable at 1.94 rad and says why | `recordings/baselines/home_change.json` |
| **where home is stored** | **FIVE places, not two.** `config/home_positions_*.txt` is the SOURCE (5 live consumers incl. the bridge gate and the real homing target); the URDF's two blocks are where the sim spawns; `pot_bridge.py` and `srl_teleop_node.py` had their own hardcoded copies and now load it; `real_home_reference.txt` is reference-only and read by nothing | `test_home_has_one_source.py` |
| **a run starts from HOME** | **it did not, from the SECOND run of a session onward.** Nothing returned the arms to home when a run ended and nothing checked at the start, so run 2 began 1.2338 rad (left) / 0.6248 (right) out — measured at the first commanded waypoint. Boot and run 1 both read 0.0000. `run_abc.require_home()` now stages before the task and REFUSES rather than running from an unknown pose; `--allow-unhomed` overrides, loudly. The sweep always staged, so the clips were never affected | `test_run_starts_from_home.py` |
| **T1 from a typed sentence** | **works, and it does what you type.** `run_abc --instruct`, `scripts/instruct_t1.py`, or the GUI's **Instruct** tab. 75 phrasings: **34 correct / 14 asked / 27 refused / 0 MISUNDERSTOOD**, against **21 / 14 / 37 / 3** for the grammar at b782a9b on the same cases (checked out of git and run in its own interpreter). 10 of the 14 asks carry a scored reply and all 10 resolve CORRECTLY, in 12 replies -- two need a second turn -- scored on their own line and never counted as CORRECT. Selectors, compounds, bare picks, politeness and misspellings; ambiguity ASKS and the ask is answerable in the same box | `scripts/sweep_t1_instructions.py` |
| **the GUI prompt box** | **mode 06 from a typed sentence, in the existing window** — an `Instruct` tab: text box, voice button on the existing `/voice_transcript` path, what the parser understood shown BEFORE anything moves, what the camera detected with the classifier's own HSV margin per cube, the announced intention and a confirm click, live state from the runner's own `[progress]` lines, and questions answerable in the same box. The check that matters: CONFIRM hands the runner an argv carrying this task, mode 06, the detections it was planned from, and the sentence VERBATIM | `scripts/verify_prompt_panel.py` |
| **the scene camera** | **BUILT 2026-08-20, and NO CAMERA HAS EVER BEEN ATTACHED.** `scene_camera_node` (refuses rather than republishing; publishes a ZERO K until calibrated), both calibration tools with self-tests that pass exactly (**fx 0.43%, cx 4.5 px; extrinsic 0.0000 m / 0.000 deg**), and `attach_scene_camera.sh`, which finds the busid and prints the two `usbipd` commands. The device is `2-5 30c9:00c1`, the laptop webcam; `usbipd bind` needs Administrator and is refused without it | `docs/system/VR_CHECKLIST.md` | **THE ONE TO PRINT.** One action per line, tick as you go, usable by somebody who has read nothing else. The button is `START VR TELEOP` on the RUN tab; everything else in it is what to do when that stops |
| `docs/system/VR_WHAT_IS_VERIFIED.md` | what on the VR path is verified against mocks (twelve rows) and what is impossible without the hardware (eight), plus **which real-arm surprises the GUI tells you about and which six you find by watching the arm** |
| `docs/system/VR_REAL_ARM_AS_RUN.md` | **THE FIRST REAL-ARM VR SESSION, 2026-08-20, AS IT ACTUALLY RAN — follow this one.** The ordered sequence that worked (teardown, `master:=false`, prove discovery, ONE arm, VR chain, observer LAST, headset), and a trap table where every row cost time: the launch sweeping the ros2 daemon's own shared memory, `timeout` SIGTERMing DDS participants, a deaf observer, `require_observer_estop` not gating the freeze, and **two arms measured at 8-12 Hz with 519 ms spikes against 21 Hz for one** |
| `docs/system/VR_REAL_ARM_RUN.md` | **THE LAB-DAY PROCEDURE.** Numbered, with a pass condition and a failure branch at every step: the discovery partition, stale SHM and the five-second gap after clearing, the hung daemon, a second stack, orphaned nodes, mirrored networking and 192.168.3.x, the leaked Kortex session, the home refusal and the recapture, the observer e-stop, proving the e-stop BEFORE teleop, and §14 -- what is still unverified, ranked by how likely it is to bite |
| `docs/system/19_scene_camera_as_built.md` |
| **the wearer is TRACKED now** | **MediaPipe Pose, markerless, metric landmarks lifted by PnP.** 28-34 ms/frame measured; frame to estimate **61.6 ms median**; end to end **62-562 ms**, the spread being the `scene_hz` rate limit. `move_group` holds twelve `wearer_*` objects and the substitution is PER PART -- a measured torso beside a mannequin upper arm, in one scene | `scripts/measure_wearer_tracking_rate.py` |
| **the camera may only make the wearer BIGGER** | the whole safety case, in one rule: `fuse()` keeps whichever of the tracked and mannequin primitive is CLOSER to the robot, per part, with no mode switch. A body injected 0.10 / 0.30 / 1.00 / 5.00 m FURTHER away changes nothing. Six faults injected through the LIVE node -- dark, blank, two people, empty, occluded arms, plus a control -- **7 of 7**, and each refusal must NAME the injected fault, which caught a false pass where the two-person gate had never run | `scripts/verify_wearer_fallbacks.py`, `test_wearer_tracking.py` |
| **the wearer's SIZE is a variable** | posture became one on 2026-08-15 and size did not, so every figure since described a 300 mm upper arm and a 360 mm chest. `SRL_WEARER_SIZE` + `config/wearer_sizes/`, through the SAME one source the posture uses, and `mannequin` reproduces the shipped body byte for byte | `test_wearer_posture_has_one_source.py` |
| **what a real body costs** | measured whole-arm through IK at z=1.120: arms down the left column IMPROVES 25 mm (0.375 -> 0.350); arms folded it is 25 mm WORSE (0.325 -> 0.350) because the chest is 70 mm wider. **What binds changes identity: a limb for the mannequin, the TORSO every time for the person.** And the operational one -- **folding the arms is worth 50 mm to the mannequin and NOTHING to this person**. NOT comparable with the published 0.325/0.450, which were certified at N=10 over the whole path | `recordings/baselines/wearer_size_effect.json` |
| **ONE BUTTON STARTS VR** | **`START VR TELEOP`, top of the RUN tab.** Twelve steps in an order that is load-bearing -- the simulation before the mapper, the certificate before the bridge, the port before the bridge -- each with a plain-words failure and the fix as a button. Idempotent: pressing it on a running system says "already done". **It cannot reach a real arm, by construction and by test.** `verify_vr_one_button.py` runs it for real with nothing typed: **22 checks, 0 failed**, 10 of 10 machine-side steps, stopping honestly on the absent headset | `test_vr_bringup_sequence.py` (27), `scripts/verify_vr_one_button.py` |
| **"READY TO OPERATE" with no headset in the building** | the last step checked the pose topic was LISTED, and the bridge creates its publishers at start-up -- so the topic exists the moment the bridge does. The operator would have squeezed the grip, watched nothing happen, and had a window still saying ready. **It counts ARRIVALS over a window now.** CLAUDE.md's own "feature present but does nothing" row, in the step whose whole job is to say ready | `test_vr_bringup_sequence.py` |
| **clearing stale SHM wedges the daemon** | and the bring-up then blamed the SIMULATION -- false, specific and confident, which is the worst combination because it sends you to the wrong log. Measured: after a clear, `ros2 topic list` returned 2 topics against a fully running stack. The clear now restarts the helper, and the simulation step consults the process table before blaming the simulation | `verify_vr_one_button.py` |
| **three repairs recreated their own fault** | found only by pressing the button repeatedly from a dirty machine. Clearing shared memory restarts the daemon, and that ORPHANS THE DAEMON'S OWN SEGMENTS -- four repairs in a row, 14 blocks each. Sweeping between the kill and the start cleared nothing (the old daemon had not finished dying). And the sweep DESTROYED A STARTING SIMULATION: a process coming up has segments it has not mapped yet, so they look unowned -- 46 blocks swept a second after the sim spawned. Stale is now asked of the kernel (`/proc/*/maps`) with a 20 s age floor, not inferred | `test_vr_bringup_sequence.py` |
| **the test gate could invent a failure** | `check_tests.py` matched `^FAILED (\S+)` over the whole pytest output, and pep257 ECHOES the source lines it objects to -- so `FAILED = "failed"` in a source file became a failing test called `=` on a suite where everything passed. A gate that cries wolf is a gate somebody bypasses | `test_the_test_gate_cannot_invent_a_failure.py` |
| **the observer e-stop was unsatisfiable, and silence read as presence** | **nothing in this repository ever published `/vr/observer_estop_present`** -- `start_vr_wifi.sh` warned about a publisher that could not exist, so the interlock was a permanent refusal somebody would eventually route around with `require_observer_estop:=false`. `scripts/observer_estop.py` is the observer's station and it is a HEARTBEAT: 5 Hz, re-confirmed every 30 s, and `vr_safety_node` now treats 2 s of silence as WITHDRAWAL. It was a latch, so a publisher that stopped left the system believing somebody was standing there -- the one state the interlock exists to detect | `test_observer_estop_silence.py` |
| **passthrough is BUILT, and it is an ADDITION** | `immersive-ar` in the Quest's own browser: **no sideloading, no account, no settings changed**, so it works on a borrowed headset. It adds **nothing** to the arm's latency, and that is structural rather than measured -- ONE `requestSession` call site and ONE frame loop serve both modes, pinned by a test. It reports `environmentBlendMode`, because a granted AR session that composites `opaque` shows the operator a black room while the page believes it showed them the lab. **In desk operation nobody wears the headset, so it is worth nothing there** | `test_passthrough_is_an_addition.py` |
| **the clearance floor follows the measured body now** | the tracker reached MoveIt's planning scene and NOT `mount_guard_node`, which builds its wearer at IMPORT time -- so the body MoveIt planned against and the body the 150 mm floor was enforced against were two different bodies, and the floor had the mannequin. It subscribes now, through `fuse()`, so it cannot be loosened: silent, stale, gated-out or further-away all return the mannequin unchanged. `/wearer/enforced` says which | `test_clearance_floor_follows_the_tracked_body.py` |
| **the mapper was silent while disengaged** | `/vr/mapper_<hand>` published only inside the engaged branch, so "disengaged", "reset", "frozen" and "the process died" were one observation -- and `/vr/reset` could not be checked, because the thing it clears is only visible when it is not cleared. A 2 Hz idle heartbeat now carries the same keys with `engaged: false` | `vr_measure_session.sh reset` |
| **the GUI, 2026-08-20** | **one command, and the two views are side by side.** `bash scripts/start_gui.sh`. RViz (COMMANDED) is EMBEDDED where a window manager exists -- detected from the X root window, not assumed -- beside a native ACTUAL panel drawn from the `real_*` frames, with the divergence readout under both. Geometry dump on WSLg: overlay 1005-1554, actual 1566-1913, divergence 1002-1918. Left column grouped SET UP / RUN / STATUS. A **connection panel** names eight faults in plain words with the fix as a button | `scripts/verify_gui_buttons.py` (163 checks, 0 failed), `test_real_arm_doctor.py` |
| **the GUI button audit** | **passes for the first time: 91 checks, 0 failed** — and getting there found four defects. It had HUNG on the modal consent dialog since that dialog was added, so it never reached the presses that would have shown: `Gui` has no `log` method (five session buttons raised inside a Qt slot, which PyQt swallows, and did nothing silently); `procscan` was never imported, so the stray-`robot_state_publisher` preflight — HARD CONSTRAINT 3's own guard — had never run; and the Launch panel rendered three of its four groups, so the dance had no buttons | `scripts/verify_gui_buttons.py` |
| **the teleop motion generator** | **it was a per-joint step clamp and it is Ruckig now, jerk-limited and synchronised.** `clamp_towards` clamped EACH JOINT independently to `max_step_rad`, so the joints sat at different fractions of their own travel and the hand left the line the IK solution implies by **51.3 mm** (left) / 39.2 (right) against a 30 mm grasp gate -- **0.0 mm** now, all seven joints arriving on ONE cycle instead of two. It also never read `joint_limits.yaml`: 17.5 rad/s commanded against 1.3963, **12.53x**, now 1.00x. Not a tuning value -- swept against its own step the excursion SETTLES at 68 mm rather than tending to zero. Velocity limits are the robot's own; acceleration and jerk are **ASSUMED, labelled, and parameterised as ramp times**, because the yaml declares `has_acceleration_limits: false` for all fourteen joints. `motion_generator:=legacy` reproduces any recording made before 2026-08-23 | `scripts/measure_teleop_motion.py`, `test_motion_generator.py` (36) |
| task layer | the same waypoints are **COMMANDED** under every mode — `run_abc` builds them with no mode argument. **The robot does NOT perform identically**: measured 2026-08-15 with no operator, T0's left path 1.407 m under 01 against 2.034 m under 03 | `docs/system/findings.md` |
| **02_vr_teleop** | **cannot grasp.** All three of its grasping tasks fail; the pads miss by 88 → 206 mm against a 30 mm gate, and the miss ACCUMULATES. The other four modes close at 0.0000 m | `recordings/verification/02_vr_teleop/T1/*/scene_events.json` |
| **the VR mapper** | `vr_pose_mapper` **holds the arms**, so the staging move cannot execute while it runs. Isolated with a control either side. The sweep stops it for staging and re-isolates. **The other defect — it did not RESET between runs, which cost 02 every grasping task — is FIXED IN THE NODE as of 2026-08-19**: `reset()`, a `/vr/reset` Trigger service and a `/vr/reset_request` topic clear references, anchors, the filter and the thumbstick's scale, and `mode_upstreams.isolate()` calls it EVERY run and refuses the mode if it will not answer. The accumulating term was `filt`, the rate limiter, and it is now published as `lag_m` | `test_vr_mapper_resets_between_runs.py` |
| **desk operation (the real setup)** | **the operator sits ACROSS THE ROOM FACING THE WEARER, holds the controllers as 6-DOF motion capture, and watches the robot directly. Nobody wears the headset** -- it stands on a shelf as the tracking reference. `align_yaw_deg` is REAL now (the docstring promised an R_align for months and the code added the displacement raw) and it is a YAW, never a mirror: facing someone and copying them is a REFLECTION, det -1, which mirrors every ORIENTATION while positions look right. No value of the parameter can produce one. MEASURED, not guessed, by `calibrate_operator_yaw.py`, which refuses a motion under 100 mm or 35 deg off horizontal. **Open: the repo disagrees with itself on whether world +x is the wearer's right -- the arms sit the other way round** | `docs/system/vr_desk_operation.md` |
| **the tracking reference can be knocked** | in desk operation the headset IS the reference, and a nudge **announces nothing**: poses stay valid, rate stays 90 Hz, controllers stay tracked, and every pose after it is in a rotated frame. `vr_safety_node` latches the first HMD pose and freezes past **20 mm / 2 deg**; the freeze SURVIVES the ordinary unfreeze path, because poses flow the whole time it is wrong. `/vr/rebase_reference` is deliberate and tells you to re-calibrate | `test_tracking_reference_stability.py` |
| **controller tracking loss froze NOTHING** | `/vr/tracking_ok` is derived from `last_rx`, stamped on every frame ARRIVAL *before* the per-controller validity check -- so it caught a dropout, a sleeping headset and a backgrounded app, but **not the case it is named after**. Measured with the real Quest: covering a controller gave **zero** loss transitions while the stream ran on at 90 Hz. Now `/vr/controller_valid_<hand>`, and the mapper freezes that arm and refuses to engage it. **And nothing consumed `/vr/freeze` at all** -- the safety node computed freezes nobody listened to | `test_tracking_reference_stability.py` |
| **the VR path over WIFI** | **works end to end, 2026-08-19, and needs NO adb.** WebXR in the Quest's own browser; `quest_bridge_node` serves the client page AND the `wss` socket from ONE TLS origin, so one cert exception covers both. Cert with the IP in `subjectAltName`, mirrored WSL so this box's LAN address is Windows'. Running it for the first time found that the node had **never started at all** (`self.clients` shadows a read-only `Node` property) and that its axis map was for the ROS frame, not this repo's — a hand 3 m forward commanded 3 m to the wearer's RIGHT | `scripts/verify_vr_wifi_route.py`, `scripts/start_vr_wifi.sh` |
| **T2's tray** | **FIXED 2026-08-16.** It was elastic — drawn as the LIVE `separation + 0.06`, so it ran 0.487–1.211 m against a 0.560 m spec and T2-1 and T2-2 could not fail. Now drawn at the spec length and oriented along the gripper line, and the ball falls off past 6.8 deg and STAYS off | `src/srl_experiments/test/test_t2_tray_is_rigid.py` |
| accuracy table | reproducible from committed data; grasp NOT uniformly 100% (06 reads 50%, VR 75%) | `scripts/accuracy_table.py` |
| status | 25 clip dirs, 19/19 planned data cells | `scripts/status_table.py` |
| **T1 under 06, both stages** | **RECORDED 2026-08-18 (later), from a typed sentence.** `06_full_autonomy/T1/S1_both_arms_centre` and `.../T1S2/S2_both_arms_random` (seed 3, a 3/1 draw -- seed 0 draws 2/2 and is indistinguishable from stage 1). Both: 4 grasps, 4 releases, every cube on the pad of its own colour, started AT home 0.0000 rad. Each clip runs **card -> look -> task**: the observe move is IN the footage now, and the card carries the sentence verbatim and stage 2's seed | `recordings/verification/06_full_autonomy/T1*/*/clip_meta.json` |
| **the scene measured stage 2 at the wrong approach** | `clip_scene._pad_offset` was guarded `if self.task == "t1"`, and stage 2 -- rebuilt on stage 1's geometry, commanding stage 1's approach -- fell through to the ANCHOR. **99.1 mm out on the left arm, 83.3 mm on the right, against a 30 mm capture gate.** The runner commanded all four picks and both grippers closed; the scene recorded TWO grasps and filed "2 OBJECT(S) NEVER MOVED". A broken measurement that read as a flaky task | `test_scene_measures_both_t1_stages_at_t1s_approach.py` |
| **the frame extractor never extracted a grasp** | it mapped event wall-clocks through `scene_events.t0_wall`, the SCENE NODE's start -- under 06 that is ~300 s before the capture, so every grasp landed at +380 s inside a 48 s clip and was skipped in silence. It printed "10 frames" and exit 0: openings and finals only. The sweep now writes `clip_meta.json` (capture clock + card + look), and the extractor REFUSES a clip it cannot map and FAILS when no grasp frame comes out | `test_clip_frames_land_on_the_grasp.py` |
| **the mislabel control** | **re-measured 2026-08-18 (later): cube 0 declared green, renders blue -> declared path sends it to the green pad at x=-0.290 on the RIGHT arm, the camera's path to the blue pad at x=+0.290 on the LEFT. 0.580 m and a different ARM.** It had been REFUSING since the rebuild -- its own single-arm look cannot see a row that straddles the centreline -- so it now takes `--vision`, the staged two-arm detections the clip itself was planned from | `scripts/verify_vision_drives_grasp.py` |
| clips | **RE-RECORDED 2026-08-16**: 25 cells (5 tasks x 5 modes) plus the three routines, eight angles and a card each, every one opening on the presentation pose, on the current geometry. 31 clip dirs, 19/19 planned data cells, no real gaps. The old set is in `archive/recordings/verification_20260815_part6` and is still the evidence for finding 3 | `scripts/status_table.py` |
| **T2 in the pixels** | **the tray is rigid now and NEITHER GRIPPER IS ON IT.** The grip trace reads 0.00 knuckle for all 992 samples on both arms while the schedule commands `grip_for(30)` on every waypoint. T2-1 "held by BOTH grippers" is false and has been for the life of the task; the elastic tray hid it by stretching to meet whichever hands existed | `recordings/verification/*/T2/*/grip_trace.json` |
| **the dance** | **d1 and d3 could not be filmed** until 2026-08-16: 45 of 382 and 49 of 554 waypoints unreachable, because the envelope was \|x\| 0.26–0.50 against per-arm limits of 0.325 and 0.450. Moved to 0.47–0.76 as ONE symmetric band; re-verified 0 of 1298 failing | `scripts/verify_dance_paths.py` |
| **the table** | **WHITE in the picture, not just in the request**: renders 238 of 255 against the old 118. A marker's ambient is 0.5 x colour and RViz does not clamp, so the top asks for 1.88. The risers are gone from every MSc task | `scripts/probe_marker_shading.py` |
| **grasp approach** | every OTHER task uses the **pinned near-side** wrist — `run_abc.send()` writes the anchor into every waypoint. **T1 does not**: it runs under 06 only and carries its own approach, elevation -10 deg and heading -50 inboard, and `set_orient()` rebuilds the pad offset from it. **T3's circuit box presents 195 mm of box to an 85 mm hand and cannot close**; T2's right grip 114 mm | `recordings/baselines/grasp_approach.json` |
| **wearer clearance, per task** | only T1 stage 1 is clean. **T0 breaches by ~142/137 mm (and 4 IK failures per arm), T2 by 150/147 and its LEFT arm now fails 1 waypoint, T3's left arm by 88**. T1 stage 2 seed 0's right arm is CLEAN as of 2026-08-17 | `recordings/baselines/home_change_applied.json` |
| **predictive avoidance** | **the null space is now REAL** — Jacobian by finite differences, N = I − J⁺J, clearance gradient, closed-loop task-space correction (open-loop drifted the hand 5.5 mm; closed-loop is under 0.1 mm). **And it is still worth 0.0 to 5.6 mm.** Every one of 28 refusals names `end_effector_link`: the HAND is inside the person, and the null space holds the hand fixed by definition. Lookahead 0.30 s, decision rate ~7 Hz through the services, EE residual 0.00001 m, never locked up. Tangential reroute rescues 1 step per target, sliding 47–57 mm | `recordings/baselines/predictive_avoidance.json` |
| **the home pose, in PIXELS** | **the render was right and the numbers were about something else.** Measured from TF at boot: elbows **0.0793 (L) / 0.2715 (R)** below their shoulders, the LEFT elbow **188 mm outboard of its own hand**, hands 1.10 m apart against a 0.36 m torso, forearm mirror residual **0.2826 m**. The wrists ARE level (−0.01 deg) and always were. NOT a cached config: the joint state matches the source file to 7e-5 rad | `recordings/baselines/home_render.json` |
| **can the pose be fixed** | **partly.** Innermost hand column that works for both arms: **\|x\| = 0.450** (torso half-width 0.18), so "hands at torso width" is the TORSO again. Best mirror residual **0.161–0.164 m and it does not improve with more branches** (4 vs 31 changes it by 1 mm), so symmetry is structural: the mounts' base axes mirror exactly but differ by **168 deg of roll**, and two identical arms on mirrored mounts cannot mirror. Left elbow can drop 91 mm. **NOT applied** — home moves only after the task set is re-measured | `recordings/baselines/symmetric_home.json` |
| **the centre, and WHY** | **the TORSO, not the wearer's arms.** The arm posture is now a variable (`SRL_WEARER_ARMS`, five settings, one source). With the wearer's arms deleted entirely the centre is still shut: min \|x\| 0.300 (L) / 0.375 (R), every column 0.125–0.300 binding on the torso, and −0.007/−0.013 m at \|x\| = 0.10. Arms clear buys 25 mm (L) and 75 mm (R). `folded` costs 75 mm; `behind` and `out` breach the floor at HOME | `recordings/baselines/centre_vs_wearer_posture.json` |
| **the mount caps clearance** | `base_link` sits **0.1610 m** from the torso and no joint moves it, so the 150 mm floor has 11 mm of headroom before the arm does anything. A clearance reading of exactly 0.1610 is the MOUNT, not the arm | `wearer_posture.py` |
| **innermost safe column** | **left 0.325, right 0.450** at z = 1.120, N=10 over the full path. The stored 0.425/0.400 in `centre_vs_height.json` is STALE: re-running that script unchanged returns 0.325/0.450 today, because the home change moved the IK seed | `scripts/measure_centre_vs_height.py` |
| **the runtime wearer had no arms** | `clearance.py` — what the follower, homing node and bridge enforce — modelled torso, head and hips only. The wearer's arms are what bind the RIGHT arm inboard. Added, with a known-answer test | `test_wearer_posture_has_one_source.py` |
| **centre, ON the surface** | **searched properly, 3360 cells, table height AND distance swept with objects resting on it: no configuration puts \|x\| <= 0.10 on the surface.** Closest confirmed at N=10: **x = 0.350** (left arm, table top 1.00, near edge 0.530, clearance 0.1524). Right arm reaches only 0.450, reach-only. **Every survivor needs the object at the very FRONT EDGE** — overhang 0.00 — because the pinned wrist arrives from below | `scripts/search_centre_on_surface.py` |
| **top-down on a surface** | **0 of 840 cells**, any table height 0.70–1.10, either arm. Not a trade against centre-on-surface: centre-on-surface is achievable at 350–450 mm off centre, top-down-on-surface is not achievable at all | `recordings/baselines/centre_on_surface.json` |
| workspace marking | **an OUTLINE, not filled tiles**, drawn at the plane it BOUNDS, per arm (`work_top_for`). **For T1 and T1S2 it is the task's OWN cells** (`t1_marking_cells`), not the global survey: `REGION_CELLS` was measured at the pinned anchor on the work plane 120 mm above the table over y = 0.075–0.300, and not one of those cells is over this task's work | `clip_scene.t1_marking_cells`, `marking_z` |
| **T1-1, objects on the table** | **SATISFIED since 2026-08-18, from the shipped scene.** The mounts moved 150 mm outboard and 15 deg of yaw, the table went to 1.250 with its near edge at 0.430, and the cubes stand on it: `verify_objects_on_table` reads gap 0.0 and exit 0, which is reserved for RESTING. Every earlier "0 of 3360 cells" was measured at the ANCHOR, on the old mount | `scripts/verify_objects_on_table.py`, `scripts/audit_task_spec.py` |
| **the on-table check can fail AND pass** | `--inject-float-mm` displaces the scene so one instrument answers three ways: 0 → BLOCKED exit 3, +5 mm → DRIFT exit 1, **−120 mm → PASS exit 0**. Exit 0 is reserved for RESTING. It also read T1's cubes when asked about T1S2 (`T1_CUBES if task=="t1" else T1_CUBES`) and now reads stage 2's own seeded layout | `test_objects_on_table_can_fail.py` |
| **a render check that could not fail** | removing the bounding box left the label reading its corners, so `tick()` raised on frame 1 and the scene came up with **no table, cubes, pads or marking — filed OK at 18% ink, because the WEARER is 18% of the frame** (46–50% when the scene is really there). `render_t1_scene` now SUBSCRIBES to `/task_objects` and refuses to shoot without the task's namespaces | `scripts/render_t1_scene.py` |
| **two clients, one planning scene** | a verifier run that overlapped a still-running sweep read **18 of 171** where two clean runs read 0 — the sweep was applying and removing its own slab in the same scene. HARD CONSTRAINT 3 one level down: never run a sweep and a verifier against one `move_group` | `docs/system/findings.md` |
| T1 | **BOTH arms, pads symmetric at ±0.290 straddling the centreline, cubes at ±0.420/±0.480 resting on the table.** Walked three times sequential AND once independent (the pessimistic branch case it had never survived): 0 IK failures, 0 breaches, worst clearance 0.1920 (L) / 0.2148 (R), pad miss 0.00–0.01 mm. Stage 2 is the SAME geometry with the side drawn: eight seeds walked, all clean | `scripts/verify_t1.py`, `recordings/baselines/t1_stage2_paths.json` |
| **the grasp was 13.47 mm short** | **`grasp_frames.PAD_MID_EE` was DERIVED, not measured** — the magnitude of a world-frame vector recorded at the anchor. FK on the two finger-tip links says 0.09833 m, both arms agreeing to 0.000 mm. It showed as a pad miss of 13.48 mm on all four cubes of both arms, identically. Correcting it cost the shipped heading: at -65 deg the gripper base fouls the table and 0 of 40 grasps solve, so the heading is -50 | `scripts/measure_pad_mid_ee.py` |
| wearer clearance | **the region and every T1 coordinate are now checked against the 150 mm floor GEOMETRICALLY.** `avoid_collisions` cannot see it — the SRDF excludes the pairs that matter — and the previous layout spent 70 of 143 waypoints inside it at 0 IK failures | `scripts/measure_clearance_region.py` |
| **the observe pose** | **re-solved 2026-08-18 for the geometry that exists.** The old one was solved for the OLD MOUNT and the OLD cubes, and the camera looks along the tool axis, so those joints pointed it elsewhere. Now 12 points in frame — both stages' six cube columns and six pad slots — clearance 0.2202, straight transit 0.2130. **The first solve was wrong in a way the COUNT HID**: 0.335 m near, 1.17 m far, and a pad fragment passed the size gate while a real cube was rejected. Range floor 0.62 fixed it; the staged look now checks WHERE each detection is | `scripts/solve_observe_pose.py`, `scripts/stage_observe_and_detect.py` |
| what limits the workspace | forward and outboard: the **pinned wrist**. Inboard: **the wearer**. Down: the **table**. No direction is bound by a joint limit | `docs/TASK_SPEC.md` §2A |
| **what the pinned wrist COSTS** | **a factor of seven at the point where the work happens, and it was never sized.** Mean reach over the direction walk, 0.15 m wearer floor enforced under every policy: from the HOME EE, pinned 0.460 m against free 0.549 (+89 mm); **from the WORK POINT (0.4, 0.175, 1.12), pinned 0.065 m with 11 of 12 walks WEARER-bound, against cone45 0.440 and free 0.450 (+385 mm) with 4**. The mechanism is not the wearer: a fixed 6-DOF pose on a 7-DOF arm spends the whole redundancy, and the redundancy is the joint that moves the elbow out of the person while the hand stays put. **Freeing the ROLL is worth +0 mm** and is refused by name. `orientation_policy` defaults to `exact`, so nothing changes until a mode opts in; HARD CONSTRAINT 1 untouched | `scripts/measure_orientation_cost.py`, `docs/system/21_what_makes_the_workspace_small.md` |
| **the sim-to-real difference, measured** | **every joint parks 0.305 deg short of its target, on the side it came from** -- 211 of 252 recorded per-joint errors at +/-0.25-0.30 deg with the sign a coin flip, which is a BOUND and not a spread. That is 7.2 mm RMS at the end effector on a 100 mm move, both arms. Pushing the recorded joint errors through the Jacobian reproduces the Cartesian error to **90%, r=0.923** -- so it is a CONSTANT terminal offset, not a gain, and the two are indistinguishable at 100 mm but differ 5x at 20 mm. ONE parameter per arm, removing **74% / 76% held out on an AXIS it was never fitted on**, where a Cartesian 3x3 scores 93.5 mm. **The data already existed in `arm_directional_calibration.json` and nothing read it.** Compensation is "overshoot each joint by EPS in the direction of travel" -- no matrix, no basis, no step length | `scripts/measure_sim_to_real_gap.py`, `srl_teleop/sim_to_real_gap.py` |
| **the "speed sweep" was not one** | I reported the error as speed-independent across a 5x range. **`vmax_rad_s` was read once at bridge start-up and 7 of 36 runs moved FASTER than the limit they were commanded with** -- `kortex_highlevel_bridge` records this in its own source and I asserted the opposite. The three speeds are three REPLICATES of one condition; **how the error varies with speed has never been measured**. A control asserts the sweep still looks inert, so a genuine sweep makes it fail and the stale sentences get rewritten | `test_the_arm_stops_short_and_the_bridge_knows.py` |
| **the bridge deadband was 3.3x the error it caused** | 1.0 deg, inside which the proportional term is off by design, against a measured 0.305 deg park. Now **0.10 deg**, plus an opt-in `terminal_overshoot` that applies EPS at the setpoint and REFUSES to run "on" with a zero overshoot. **Neither has been tried on hardware**; both are live parameters and four runs settle which wins | `kortex_highlevel_bridge.py` |
| **the RViz master** | **three states, not two.** COMMANDED, ACTUAL and **PLANNED-BUT-REFUSED with its reason string, which cannot be drawn without one**; the wearer inflated by the floor from the body the guard is ENFORCING, with the nearest segment coloured and LABELLED with the part; the densified path one point per sample, worst called out by index; liveness per arm; the envelope as three volumes. `ros2 run srl_teleop rviz_master`. Pure marker builder, 32 checks, no display needed. `/viz/path` IS fed: `safe_motion.check_path(..., trace=[])` now records one entry per densified sample and keeps going PAST the first breach, because the half after the breach is where the arm was going -- 47 of 47 samples, verdict byte-identical with and without the trace. `/viz/envelope` and `/viz/refused_in` are built and tested but not yet fed | `srl_teleop/rviz_master.py`, `rviz_master_node.py` |
| **LeRobot** | **the FORMAT is worth having and has been taken; the policies are blocked three ways.** 23 episodes / 20 440 frames of the real teleop recording exported as a `LeRobotDataset`, with the 8 channels the newest baseline calls INCOHERENT or DEAD dropped and named in the dataset's own metadata. Policies: no corpus (ethics), 4 GB VRAM, and a policy learns AROUND an unmeasured extrinsic rather than fixing it. **`.venv_lerobot` is separate on purpose** -- installing it into `.venv_vision` took `real_calibration/check_all.py` from 4/4 to 2/4 | `scripts/lerobot_export.py`, `docs/system/20_lerobot.md` |
| grasp pose | **the wrist-to-pad offset is 0.09833 m along the tool axis, MEASURED from FK.** `clip_tasks.PAD_OFFSET_BY_ARM` is 13.45 mm longer and is deliberately unchanged: T0, T2 and T3 declare their coordinates through it and the scene draws through it, so task and picture agree — but in those three the declared coordinate is 13.45 mm from where the object is drawn and grasped | `recordings/baselines/pad_mid_ee.json` |
| **the wrist every sweep measured at** | **it was the HOME wrist, not the anchor the task sends — 32.26 deg apart on T1's arm.** `measure_what_binds.Rig` read the live EE orientation off TF at construction; `run_abc.send()` writes `WORKSPACE_ORIENT`. EVERY reachability sweep in the repo solves through `Rig`. Now `anchor="workspace"` by default. T1's stage-1 layout survives the fix (0 failures, N=10, twice); `centre_on_surface.json`'s 22 hits do not | `docs/system/findings.md`, 2026-08-17 (later still) |
| **two surface heights, 150 mm apart** | **`BENCH_TOP` 1.10 positioned every object, `TABLE_TOP` 0.950 was the only geometry, and nothing compared them.** One owner now: `work_surface.WORK_PLANE_M` 1.100 / `DECLARED_M` 0.980 / `DECLARED_NEAR_Y` 0.100 / `FLOAT_GAP_M` 0.120. Surface raised 30 mm, gap 150 → 120 mm | `test_one_work_surface_height.py` |
| **neither arm crosses the centreline** | **0 of 10 IK solutions at every cross-side pad slot and every cross-side cube, both arms.** Not marginal — nothing solves. So a cube can only be delivered to the pad on its own side; stage 2 draws the SIDE and the colour follows, and `t1_task.build()` REFUSES a cube whose colour names the other side's pad | `recordings/baselines/t1_stage2_paths.json` |
| **T1S2** | **stage 1's geometry with the side drawn**, since 2026-08-18: same table, same two pads, same row, same builder. Eight seeds walked at N=10 over the composed path, every one clean; splits 1/3, 2/2 and 3/1, both arms always worked. The pads sit at 290 mm and not 260 because seed 3's third left slot read 0.1219 m to the wearer's upper arm | `scripts/verify_t1.py --stage2 SEED` |
| table height | owner exists and detects ±20 mm; **nothing calls `set_measured()` from depth** | `srl_experiments/work_surface.py` |
| silent faults | 3–4 open, listed at the top of `03_real_robot_bringup.md` | `scripts/inject_lab_day_faults.py` |
| real hardware | **nothing in this repo has ever run against a real arm** | |

**Blocked on the lab:** 7 of 14 master channels incoherent (a soldering
problem; `l_j2` and `l_j4` named by regression); the two arms' reachable sets
are disjoint so T4 handover needs the right arm re-parked in hardware; the
right arm's home joint values were never read from hardware; two real Kortex
sessions have never been opened; detection rate at working distance is
unmeasured; **speech recognition accuracy against a REAL SPEAKER is unmeasured
— `/dev/snd` has only `timer`, so no microphone can be opened in WSL and every
voice number here was produced from Piper-SYNTHESISED audio (WER 25.2%, zero
exact transcripts, one voice, no room, no noise). A voice score in this repo is
a PARSER score, never a transcription score — TASK_SPEC 4B/U-4**; no `adb`;
this machine is off the lab network.

**Blocked on ethics:** no human data has been collected at all. Operator and
wearer are different people and consent separately. Worn operation is >17 kg
with no gravity compensation on someone who did not choose the motion.

---

## HARD CONSTRAINTS. Violating one of these causes real damage.

0. **SIM AND REAL HOME DISAGREE ON PURPOSE, since 2026-08-15.** Sim home is
   now the presentation pose (wrists LEVEL, hands in front of the chest); the
   physical arms are still at the legacy Kortex home and the wrists there
   point UP by +85/+79 deg. The bridge REFUSES to enable on the ~1.9 rad
   difference rather than commanding it — verified in `enable()`. **Capture
   the new pose on the arms before any real session**; values in Kortex
   degrees are in `NEXT_SESSION.md`. → `home_wrist_is_real.md`
1. **Home joint angles are ground truth, and the REAL arm's are the ones that
   count.** The sim's were changed deliberately on 2026-08-15, after
   re-measuring the whole task set; the arms' were not. Never change either to
   make a pose look right without re-measuring first — the sim→real bridge
   replays sim angles onto the real arm and any difference it does not catch
   is commanded as a jump. **The ANCHOR is a DIFFERENT quantity**: the 30.7
   deg approach direction is a stored constant that nothing recomputes from
   home. Moving home cost nothing measurable; re-deriving `WORKSPACE_ORIENT`
   to match a level home costs **T2 its right arm**, so never do it.
   → `home_wrist_is_real.md`
2. **The arm permits exactly ONE Kortex session.** SIGINT the bridge; SIGKILL
   leaks the session and the next run cannot connect. Grep for
   `kortex session closed cleanly`. → `wsl.md`
3. **Never run two stacks.** Two `master_pose_node` split the serial stream
   and invalidated a full day of measurements. The GUI refuses and names the
   PIDs. A stray `robot_state_publisher` does the same damage and is not
   caught by the count. → `architecture.md`
4. **The Kortex cyclic path is unusable over WSL.** Use the high-level API
   (`kortex_highlevel_bridge`). Every cyclic write is a network round trip.
   Do not spend another day on the ros2_control driver. → `wsl.md`
5. **`FASTDDS_BUILTIN_TRANSPORTS=SHM`.** UDP discovery is dead on this host.
   Clear `/dev/shm/fastrtps_*` with the stack STOPPED, then relaunch.
6. **Never `wait_for_service` in `estop_node.trip()`.** It once blocked the
   e-stop for 4.0 s. Park first, then talk to the driver. → `findings.md`
7. **The dependency arrow runs one way.** `srl_teleop` imports nothing
   in-repo. It is the baseline condition of every experiment; if it imported
   `srl_autonomy` the comparison would be circular.
8. **`real_robot` mode must be armed.** `motion_enabled=false` until set by
   hand, and it refuses to start with a continuous joint wound beyond ±π.
9. **Never `pkill` broad patterns while a stack you want is running.** Kill an
   explicit PID list, and kill the process GROUP.
10. **Wrap ROS `setup.bash` in `set +u` / `set -u`** — it reads unbound
    variables and dies otherwise.
11. **The wearer is in the collision model and the clearance floor is the last
    thing between the arms and a person's chest.** Do not lower it globally.
    An SRDF exclusion silences an alarm; it does not move the metal.
    **`avoid_collisions` IS NOT THE WEARER CHECK** — the SRDF excludes the 44
    proximal pairs a shoulder mount actually threatens, so a `valid` pose can
    have the tube inside the person. Measure clearance geometrically, and do
    not cite a workspace figure without checking
    `docs/system/clearance_gap_ledger.md` for whether it survived.
    **The WEARER'S POSTURE is part of the model and it is a variable now**
    (`SRL_WEARER_ARMS`, default `down`). It has ONE source, `wearer_posture.py`,
    which writes the URDF's arm block and builds `mount_guard_node.WEARER`; a
    posture that reaches one and not the other measures the old wearer under a
    new name. Do not tell a wearer to clasp their arms behind their back or
    hold them out to the sides: with a BACK-mounted rig both put their own
    limbs inside the floor at the HOME pose. → `findings.md`, 2026-08-15
12. **Anonymity is enforced in code.** `write_manifest()` raises on `name`,
    `email`, `dob`, `address`, `phone`.
13. **A demonstration is not evidence.** Demo clips carry that caveat in the
    caption and produce no trial data, no condition and no metric.

---

## THE GUI RULE, ADDED 2026-08-22 AT THE OPERATOR'S REQUEST

**The window is the interface. Every capability reaches it, and every change
is checked in it.**

1. **If you add or change a capability, it gets a control in the GUI in the
   same change.** A feature reachable only from a terminal does not exist for
   the person running the session. `docs/HOW_TO_RUN.md` is the contract: two
   commands, and everything else is in the window.
2. **Run `python3 scripts/verify_gui_buttons.py` before you claim anything
   works.** 257 checks. It presses every button, and since 2026-08-22 it also
   checks what the presses DID -- the e-stop must reach `/estop`, service
   buttons with no stack must say so, the experiment panel's own defaults
   must produce a command the dispatcher accepts.
3. **And LOOK at it.** Launch it and screenshot the window
   (`ffmpeg -f x11grab -window_id <id> -i :0 -frames:v 1 out.png`; a grab of
   the X ROOT records black on WSLg). Three defects lived for weeks behind a
   passing audit: RViz embedding an invisible window, a control column capped
   below its own content, and every mode button below the fold. None of them
   is visible from a return code.
4. **A press must leave a trace, and a control that changes what the next run
   does must SAY so.** That is the audit's own rule and it is why every new
   toggle, drop-down and spin box in this window logs its new value.
5. **After pressing everything, the window must still work.** Level 2c
   re-checks the layout, the tabs, the e-stop and the instruction box at the
   end of the sweep. A control that works on a fresh window and stops working
   after somebody explored the panel is a glitch, and the audit is the only
   thing positioned to catch it.

## THE STANDING RULE

**Before reporting a bad measurement, validate the measuring tool against
ground truth.** A surprising failure is evidence about the INSTRUMENT until
the instrument has been cleared. This has been the fault **seventeen times**.

* Every analysis script gets a known-answer test.
* **A check that cannot fail on a deliberately broken input is not a check.**
* Prefer real data with known ground truth. Synthetic only where the ground
  truth is CONSTRUCTED (arithmetic, geometry), never RENDERED.
* A result that contradicts an earlier measurement is an instrument check, not
  a finding, until the two are reconciled.
* Repeat before believing. TRAC-IK restarts randomly; one call is one flip.
* **N repeats over the WHOLE PATH.** A pose passing one IK call is not
  reachable. N=10, densified to 20 mm, and stay 20 mm inside the last pose
  that passed N/N. MARGINAL is not usable.

### Instrument failure modes seen here. Check this list first.

| symptom | mechanism |
| --- | --- |
| statistic is 0.000 everywhere | recorder oversamples a slower sensor; adjacent rows identical |
| zero variance | dead channel, or a cached value republished |
| data fresh but never changes | stale republished with a NEW timestamp; key on the SOURCE stamp |
| learned model scores near zero | synthetic renderer out of distribution |
| metric swings between runs | min-over-directions; use medians |
| everything matches | substring/prefix matching (`t0` inside `t001`; `t1s2`.startswith(`t1`)) |
| results depend on run order | scene or state persisted; `remove_furniture()` clears only ITS ids |
| every pose returns one value | FK evaluating the CURRENT state, ignoring the solution given |
| feature "present" but does nothing | checked that a field is STORED, not that a consumer READS it |
| a gap that is not a gap | by-design absence rendered identically to a defect; use `by_design.py` |
| clip verifier fails a good clip | matched requested RGB, not RENDERED colour; RViz shades everything |
| test breaks with no behaviour change | test sliced source "before the first def" |
| success count identical either way | a tolerance so loose it binds nothing; measure the ACHIEVED value |

---

## LAYOUT

| package | role |
| --- | --- |
| `srl_teleop` | teleoperation ONLY: master sensing, IK follower, clutch, scaling, sim→real bridge, e-stop |
| `srl_perception` | AprilTag, 6-DOF object pose, mock RGB-D |
| `srl_autonomy` | grasp generation, intent inference, handover arbitration |
| `srl_experiments` | tasks, logging, conditions, analysis, work-surface owner |
| `srl_description` | `srl_dual.urdf.xacro` — arms, mount, wearer |
| `srl_moveit_config` | TRAC-IK, controllers, SRDF |

Build: `colcon build --symlink-install`, 16 packages, no flags.
`robotiq_driver` and `robotiq_hardware_tests` are `COLCON_IGNORE`d (missing
`serial`); nothing in `srl_*` depends on them. All `srl_*` are
symlink-installed, so `.py` edits take effect **on node restart** — if a change
seems not to apply, suspect a stale PROCESS, not a stale install.

---

## WHERE THE DETAIL IS

| file | holds |
| --- | --- |
| `docs/system/findings.md` | the diagnostic history and every worked example: mount geometry, hardening passes, the reachability and clearance measurements, the experiment programme, the two-person reframing |
| `docs/system/hardware.md` | Teensy and serial, channel health, spherical position and AXIS_MAP, gyro azimuth, calibration, clutch, grippers, degraded mode, the virtual Teensy |
| `docs/system/wsl.md` | mirrored networking, the cyclic-path finding, `/mnt/c`, `/dev/shm`, the Quest transport, x11grab and Xvfb capture |
| `docs/system/architecture.md` | packages and topics, the IK follower, operating modes, VR stack, the three GUIs |
| `docs/system/clearance_gap_ledger.md` | **which earlier workspace and clearance numbers the SRDF gap invalidated and which stand. Read before citing any workspace figure.** |
| `docs/system/home_wrist_is_real.md` | **the home pose CHANGED on 2026-08-15.** What moved, what deliberately did not, why the real arms are still at the old pose and what stops that becoming a jump |
| `docs/system/03_real_robot_bringup.md` | **the lab-day fault table and the camera framing note. Read before hardware.** |
| `docs/system/06_troubleshooting.md` | keyed by SYMPTOM |
| `docs/system/19_scene_camera_as_built.md` | **AS BUILT.** The scene camera and the wearer tracker: the usbipd commands, both calibration self-tests and the two real bugs they caught, the method and why, the rate and latency, the six injected faults, what a real body changes, and -- kept separate -- **what is still INFERRED because no camera has been attached** |
| `docs/system/16_scene_camera_wearer_tracking.md` | **DESIGN ONLY.** A scene camera that measures the WEARER instead of assuming a mannequin: what tracks the body, how it reaches the clearance check without fighting the planning scene, and the rule that it may only ever make the wearer BIGGER. ~7 sessions |
| `docs/system/17_vr_passthrough_and_camera_feed.md` | **DESIGN ONLY.** Seeing the environment from inside the headset: Quest passthrough (one client line, no latency, works borrowed) against streaming a camera in (35-145 ms). Both are ADDITIONS -- the shipped setup needs nobody to wear it. ~1 and ~3 sessions |
| `docs/system/18_dynamic_degradation.md` | **DESIGN ONLY.** Every mode keeps working when a source is missing: the ladder per capability, what is lost at each rung, and the three capabilities that must REFUSE rather than descend. ~7.5 sessions, of which the first 1.5 only make existing behaviour visible |
| `docs/system/20_lerobot.md` | **LeRobot, assessed against what this project needs.** What was exported and how, the channel gate that drops DEAD columns rather than exporting a repeated value as data, and the three independent reasons a learned policy is not the next thing |
| `docs/system/21_what_makes_the_workspace_small.md` | **WHY THE ARMS BARELY MOVE, sized.** The pinned wrist per direction per policy, the reconciliation of two baselines that walked from different points, why freeing the roll is worth nothing, and what is still not measured |
| `docs/NEXT_SESSION_2026_08_23.md` | **the current resume point.** What 2026-08-22 measured and what it changed about the plan |
| `docs/HOW_TO_RUN.md` | **START HERE TO OPERATE IT.** One page: the two commands, every mode and where its button is, full autonomy end to end, how much the arms can move and why, what to run before hardware, and a symptom table |
| `docs/system/24_motion_planning.md` | **THE MOTION PLANNING RESEARCH.** What we actually run (nothing planned), every candidate scored against 4 GB of VRAM and a safety case MoveIt's collision model is blind to, what was built instead and what it measured, the Kinova sensor surface we were not reading, and the order to do the rest in. Sources cited |
| `docs/system/23_how_good_is_it.md` | **THE HONEST ASSESSMENT.** The control budget, the reaction budget, what losing tracking costs, and the six things that make it hard for an operator across the room |
| `docs/system/22_grasping.md` | **detection and grasping.** What is built, the two defects fixed on 2026-08-22 (a wearer floor that could never fire, refusals that asserted instead of measuring), what is actually broken in order, the coarse-to-fine architecture that works, and a Hugging Face model shortlist scored against 4 GB of VRAM |
| `docs/NEXT_SESSION.md` | what to do next, in order |
| `docs/WORK_BRIEF.md` | the standing multi-part brief |
| `docs/research/` | literature, hypotheses, protocol, ethics, the grasping and approach-geometry findings |

**Starting fresh?** This file, then `docs/NEXT_SESSION.md`. Go to the others
only when a rule above sends you.
