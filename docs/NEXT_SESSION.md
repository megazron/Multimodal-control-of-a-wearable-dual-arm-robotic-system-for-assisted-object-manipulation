# LAB SESSION: DO THESE, IN THIS ORDER

Everything below needs hardware. Nothing above this line does. The order is
not arbitrary: each step's result changes whether the next one is worth doing.

---

## 0. BEFORE YOU TOUCH ANYTHING: exactly one stack

Two `master_pose_node` instances split the serial stream and invalidated a
full day of measurements. It is now impossible at the file-descriptor level,
but confirm the refusal never appears:

```bash
pgrep -fc "lib/srl_teleop/master_pose_node"     # must be 0 or 1, never 2
ls /dev/shm | grep -c fastrtps                  # if in the hundreds, see below
```

Stale shared-memory segments from killed stacks silently kill NEWLY LAUNCHED
nodes: empty logs, no processes, no error at all. This cost hours on 08-09.
Clear them **with the stack stopped**:

```bash
rm -f /dev/shm/fastrtps_*
```

Kill by explicit PID, never `pkill -f <pattern>` -- the pattern matches the
shell running it, which killed the working shell three times in one session.

---

## 1. REPAIR LEFT j2 AND j4. Nothing else on the master is worth soldering.

**These two, and only these two.** Two independent routes agree, which is why
this is stated flatly rather than as a suggestion:

* the observability regression: with j1 and j7 alone the left arm retains
  **no reach observable at all**, $R^2 = 0.133$ with a residual that is
  \SI{93}{\percent} of the true spread. The right arm needs no help
  ($R^2 = 0.986$).
* `capability.regain()`, which was never told the above, independently names
  **j2 and j4 as WORTH IT** and j3, j5 and j6 as **no change**.

Fixing a roll buys nothing while both bends are dead, because a roll carries
no radial information. That is why the health table alone cannot tell you what
to repair, and the regain map can.

Expected gain: the left arm moves from rung **SPH_RATE** (radius driven as a
rate off the wrist, cost \SI{0.076}{\metre} mean) back to **SPHERICAL**
(radius measured, cost \SI{0.000}{\metre}).

## 2. Re-test AFTER EACH ATTEMPT, not once at the end

```bash
bash scripts/check_channels.sh          # ~3 min, block A only
```

It diffs automatically against `recordings/baselines/channels_20260806.json`.
**Target: 12 or more of 14 coherent**, which is where `degraded_mode:=auto`
stops engaging.

Run it after *each* attempt. A repair that breaks a working channel should be
caught immediately, not discovered at the end of the session when you cannot
tell which attempt did it.

Watch for the verdict **INCOHERENT** specifically. A failing pot does not go
quiet: it returns a wide spread of values that are not a trajectory, and range
alone calls that healthy. That is how a broken channel keeps getting trusted.

## 3. ONE 35-minute recapture, gate fix live

```bash
terminal 1:  bash scripts/run_teleop.sh gate:=false
terminal 2:  ros2 run srl_teleop live_monitor
terminal 3:  bash run_teleop_capture.sh
```

The 2026-08-06 capture lost **41 of 42 directional segments** to a clutch
gating bug and latched the e-stop through the whole of block F. Both are
fixed, but confirm from the recorder's own output rather than assuming:

* each segment must record with the clutch **ENGAGED** on the arm under test
  (each arm is gated by the OPPOSITE arm's button, because an arm's own button
  also toggles its clutch);
* the e-stop must not latch. The both-button trigger now needs a sustained
  \SI{0.30}{\second} hold, so gating taps can no longer fire it.

This capture is what unblocks: the gyro-azimuth decision, any smoothing
parameter, and the first real gain matrix with a **moving** master. Every
tracking figure in the thesis was taken with the master at rest.

## 4. Verify the cascade rate limit end to end

`clamp_towards` now cascades `max_step` as well as `max_vel`. It is
unit-tested and has **never been measured with a stack up**.

```bash
bash scripts/start_real.sh --mock
# then vary the trip threshold and watch the divergence
ros2 param set /sim_to_real_bridge_left lag_trip_rad 0.3
ros2 param set /sim_to_real_bridge_left lag_trip_rad 0.5
ros2 param set /sim_to_real_bridge_left lag_trip_rad 0.8
```

**EXPECTED: divergence stays roughly CONSTANT as `lag_trip_rad` changes.**
If it SCALES with the threshold, the fix is wrong: that would mean the sim is
still running ahead and only the trip point moved, which is the bug the
cascade was meant to remove.

Record the actual numbers either way. This is a prediction with a clear
falsifier, which is the only kind worth testing.

## 5. Unblock VR: platform-tools on WINDOWS

```
https://dl.google.com/android/repository/platform-tools-latest-windows.zip
```

Unzip to `C:\platform-tools`. No install, no admin rights. Verify from WSL:

```bash
/mnt/c/platform-tools/adb.exe devices
```

`unauthorized` instead of `device` means the **Allow USB debugging** prompt
has not been accepted, and that prompt appears **inside the headset** -- put
it on to answer it. This traps everyone.

It must be the **Windows** build: the headset enumerates as a Windows USB
device, so `apt install adb` inside WSL gives a binary that cannot see it.

Then the whole connection is one command, which also supervises the tunnel:

```bash
bash scripts/vr_connect.sh
```

## 6. While you are there, two cheap measurements

* **Read the right arm's home joint angles from hardware.** They have NEVER
  been read (`config/home_positions_right.txt` says so), and $P_{\text{HOME}}$
  for that arm, the workspace anchor and every clearance figure depend on
  them.
* **Photograph the task objects**, 25 images each, for the detection measure.
  The current detection figure characterises a renderer, not a detector, so
  the \SI{95}{\percent} gate is neither passed nor failed.

---

# ITEM 3 - THESIS: instrument-validation chapter written

The master arm chapter (2799 words), the MuJoCo-to-ROS development path
(1631), and all four control-mode flowcharts ALREADY EXISTED from the earlier
rewrite. Checked before writing rather than duplicated.

The genuinely missing piece was the methods contribution, now written:
`thesis_report/methods/instrument.tex`, Chapter 6, "When the instrument is the
fault". Sixteen cases in one table (looked like / actually was / caught by),
four mechanisms, and the finding that **9 of 16 were caught by an
independently known quantity and only 3 by inspection** -- which is the
argument for building the cross-check in rather than trying harder.

76 pages, 0 errors, 0 undefined references.

### Thesis work still outstanding
* The Results chapter predates Parts 1-9 and does NOT yet carry: the aperture
  curve validated against the vendor stroke, the capability ladder costs, the
  scene-fingerprint numbers, the broken-then-fixed VR path, the free-form
  language results, or the 39/39 clip sweep.
* Placeholders: 5 images, 2 reproduced figures needing permission.
* 13 todo markers (10 CONFIRM, 2 MEASURE, 1 FIGURE).

---

# ITEM 2 - WRIST CAMERA RELAY: LOGIC MEASURED, GUI WIDGET UNCONFIRMED

## Measured and working
`srl_teleop/camera_relay.py` holds the staleness rule, shared by the GUI and
the VR publisher so the two cannot drift apart. Against a synthetic 15 Hz
camera on both arms:

    measured rate      14.2 Hz  (reported from ARRIVALS, not configured)
    transport latency  median 2.7 / 3.1 ms, p95 7.6 ms  (header stamp -> receipt)
    after the source stops:  live -> NO SIGNAL, paint disabled

**It degrades visibly by construction**: past `stale_after_s` the widget is
told NOT to paint at all, so a frozen frame can never be shown as live. That
is the failure class this project keeps meeting (frozen /real/joint_states
reading as "holding", the dead pot reading as "steady").

`camera_vr_publisher` relays the same streams as JPEG on
`/vr_camera_<arm>/compressed` with state on `/vr_camera_state`, rate-capped,
and publishes NO FRAME when the channel is not live.

## NOT CONFIRMED: the GUI widget renders
The widget is written and the module compiles, but every screenshot attempt
captured the reparented RViz window covering the GUI instead of the panel.
**Do not report the GUI camera panel as working until a screenshot shows it.**
Suggested: launch with RViz embedding disabled, or capture the GUI's own
window id with `xwininfo` rather than the whole display.

Also fixed on the way: `srl_gui.py` referenced `cap` and `cr` with NO IMPORT
AT ALL. It survived only because the branch dereferencing `cap` never ran with
no data. It would have crashed the moment a capability message arrived.

---

# ITEM 1 COMPLETE - 2026-08-09 overnight run

## ALL 38 CLIPS RE-RECORDED, 39/39 PASS THE VALIDATED VERIFIER

Five-task geometry (500 mm tray, 540 mm sling), single-owner gripper fix,
7 angles per clip, 266 files, 132 MB, largest file 5.3 MB.
`recordings/verification/INDEX.md` carries the Task 2 caveat at the TOP.

### THE VERIFIER WAS MISCALIBRATED FOUR TIMES. Every one caught BEFORE any
### clip was reported as failed.
1. `block_orange` fired 495 px on a frame with no orange object: RViz lighting
   pulls the tan tray into the orange band. Fixed with R-B > 155.
2. `tray_tan` fires 220 px and `ball_yellow` 21 px on the WEARER'S SKIN. At the
   shared 12 px floor every f3/f4 clip would have passed whether or not the
   object rendered.
3. Per-object floors calibrated on 800 px captures, while `frames()` rescales
   every sample to **600 px** -- wrong by (600/800)^2 = 0.56, which rejected
   nine sling clips whose object I had already confirmed by eye.
4. My own expectation was wrong twice, not the detector: the f2 scene
   legitimately renders a tan table.

### A REAL STALE-GEOMETRY BUG, which is what the re-record existed to find
`record_rviz.SLING_L` was still the retired 350 mm while the spec is 540 mm.
At the respec'd 500 mm separation a 350 mm sling is GEOMETRICALLY IMPOSSIBLE
((L/2)^2 - (s/2)^2 = -0.032), so it rendered as a flat line with the ball
instantly fallen, and all nine f4 clips failed correctly. The length now comes
from the scenario. Confirmed from pixels afterwards: `sag=102mm`, matching the
predicted 0.102 m, with the ball retained in the V.

### KNOWN AND ON SCREEN
f3/f4 show a CARRY, not a grasp: `GRASP REFUSED: pre-grasp standoff not
solvable`. Correct -- the tray edge sits at |x| = 0.25 m and top-down grasping
is infeasible below 0.30 m (0/9 against 9/9). The refusal is in the overlay.

## ITEMS 2, 3, 4: NOT STARTED
2 wrist camera relay, 3 thesis sections, 4 lab list.

---

# STOPPED HERE - 2026-08-09 overnight run

## ITEM 1 RE-RECORD CLIPS: INFRASTRUCTURE DONE + VERIFIER VALIDATED.
## THE SWEEP ITSELF HAS NOT BEEN RUN. 4 of 38 clips exist.

### What is finished and committed
* `scripts/final5_runs.py` builds the sweep from the CURRENT five-task spec:
  **14 runs, 38 clips, 266 files at 7 angles**. Output under `f1..f5` so it
  cannot collide with the 87 stale nine-task clips.
* `record_rviz.py --final5 --resume`: per-run conditions (f2 is shared-only),
  progress written to `recordings/verification/sweep_progress.json` AFTER
  EVERY CLIP, and `--resume` skips what is already done.
* **The Task 2 caveat is burned into the overlay in red** and confirmed
  visible in a captured frame: "DIRECT-IK: not operator-commandable. Grasp
  needs 169.7/164.6 deg from the pinned wrist anchor."
* Overlay label bug fixed: clips were labelling themselves by the RENDERER
  code, so an f3 clip said "T3". A clip that misnames itself looks
  authoritative; it now shows the run's own name.

### THE VERIFIER WAS MISCALIBRATED IN TWO WAYS. Both caught BEFORE any clip
### was reported as failed, which is what the brief required.
1. `block_orange` fired **495 px on an f3 frame with no orange object** -- RViz
   lighting brings the tan tray into the orange band. Now requires R-B > 155,
   calibrated against a confirmed positive AND a confirmed negative.
2. Worse: **`tray_tan` fires 220 px and `ball_yellow` 21 px on the WEARER'S
   SKIN** with no task object present. At the shared 12 px floor, every f3 and
   f4 clip would have passed whether or not the object rendered. Per-object
   floors now sit above the skin and far below the object:
   tray 220 (absent) vs 828 (present), floor 400; ball 21 vs 649, floor 120.

### TO RESUME: one command
```
python3 scripts/record_rviz.py --final5 --all --resume
```
It skips the 4 clips already recorded and writes progress after each. Then
`python3 scripts/verify_rviz_clips.py` and rebuild INDEX.md with the Task 2
caveat at the TOP.

### Known, unresolved
* **f3/f4 grasp is REFUSED**: "pre-grasp standoff not solvable". Correct and
  expected -- the tray edge sits at |x| = 0.25 and Part 1 measured top-down
  grasping infeasible below |x| = 0.30. The clips show the carry, not a grasp.
  Decide whether to widen the tray or accept a carry-without-grasp clip.
* **Repo size not yet checked**: 266 files at roughly 1.3 MB each is about
  350 MB. Check before committing the sweep; consider tracking quad+gripper
  only and keeping the rest local.

## ITEMS 2, 3, 4: NOT STARTED
2 wrist camera relay, 3 thesis sections, 4 lab list. Stopped at the item
boundary rather than half-doing item 1's sweep.

---

# SESSION 2026-08-09 - NINE-PART BRIEF, PARTS 1-2 DONE

Sim and mock only. No lab.

## PART 1 - grasp investigation: DONE, committed ae68df1
Headline: **the grasp is NOT real under teleoperation.** It is real under the
scripted and autonomous paths. 169.7 deg (left) / 164.6 deg (right) of wrist
rotation is needed from the anchor that `orientation_mode: fixed` pins to, and
no channel commands it. The clips work because the recorder calls
`/compute_ik` directly, bypassing the teleop orientation lock.

Also: the approach IS a real straight-line approach from a 100 mm standoff
(0.01 mm max deviation); t2/t5 close on their objects, **t3 and t6 stop
1.5 mm short**; precision is NOT measurable against the mock and is reported
as such; dexterity is 5 of 8 approach directions per arm, mirror-symmetric,
with no from-behind approach on either.

Three instrument bugs were found before any number was trusted. See the
commit message.

## PART 2 - dynamic degradation as architecture: DONE, this commit

`srl_teleop/capability.py` (pure, testable) + `capability_node`.

Six rungs, selected live from whatever channels pass health:

    L0 FK        7 healthy                       cost 0.000 m
    L1 SPHERICAL j1 + at least one of j2/j4      cost 0.000 m
    L2 SPH_RATE  j1 + any spare alive channel    cost 0.076 m mean / 0.114 p95
    L3 SHELL     j1 only, radius FROZEN          cost 0.151 m mean / 0.232 p95
    L4 DIR_ONLY  IMU only                        NO POSITION
    L5 NONE      IMU down                        NO POSITION

Costs are LOADED from `recordings/baselines/capability_ladder.json`, measured
on 20440 recorded frames. A rung with no measurement prints "unmeasured"
rather than a plausible number.

Verified live against a synthetic master: DIR_ONLY at startup (conservative,
"unknown" is not "healthy") -> SPHERICAL -> FK -> fell to SPH_RATE when both
bends went incoherent -> RECOVERED to SPHERICAL when j4 came back. Recovery
is automatic and needs no restart.

`scripts/verify_capability_degradation.py` sweeps all 128 channel subsets.
From the measured left-arm health it independently names **j2 and j4** as the
only repairs worth making, and j3/j5/j6 as no change, which matches the
observability regression (R^2 = 0.133 on j1+j7 alone).

18 known-answer tests.

## PART 3 - scene fingerprinting: DONE, this commit

`srl_perception/scene_fingerprint.py` (pure matcher) + `scene_fingerprint_node`.

Verified end to end against synthetic detections, four stages:

    1st sweep   FIRST FINGERPRINT: 3 objects in 0.9 s
    2nd sweep   SCENE MATCHES (max delta 0.0 mm) -- calibration SKIPPED
    3rd sweep   block_b moved 60.0 mm -> re-registered 1, kept 2 unchanged
    4th sweep   block_b vanished -> dropped 1, store now holds 2

Measured (matcher only; the DETECTOR is not measurable here):
  * decision accuracy at AprilTag noise (sigma 0.8 mm): 100% on all four of
    same / moved / appeared / vanished, 2000 trials each.
  * smallest detected displacement: 20 mm at 95% (model predicts 15 mm).
  * compare() costs 0.12 ms for 5 objects, 6.7 ms for 50. The decision is
    free; a sweep's cost is arm motion.

**The fingerprint is only useful with a tag-grade detector.** At the
colour/shape fallback's 10 mm noise the "unchanged" verdict is right 2.6% of
the time -- it would re-register every start. Widening the tolerance fixes the
false alarms and costs sensitivity:

    pos_tol   false "changed"   min detected move
     15 mm       97.3%            30 mm
     30 mm       13.0%            50 mm
     40 mm        0.8%            60 mm

## PART 4 - four-mode path verification: DONE, this commit

**THE VR COMMAND PATH WAS BROKEN AND IS NOW FIXED.** `quest_vendor_bridge` --
the transport the project adopted -- published `/vr_pose_<side>` and
`/vr_gripper_<side>`, which NOTHING in the workspace subscribes to.
`vr_pose_mapper`, the stage that turns a controller pose into
`/master_arm_pose_<arm>`, listens on `/vr/controller_pose_<side>` and
`/vr/controller_joy_<side>`. Measured on a live graph: `/vr_pose_left` had
1 publisher and 0 subscribers; `/vr/controller_pose_left` had neither. VR
could not reach an arm at all through the vendor transport.

Fixed and verified: all three hops now `pub=1 sub=1`, and the mapper publishes
`/master_arm_pose_left` as a result.

Other findings:
  * **two detectors write one topic.** apriltag_detector and
    colour_shape_detector both publish `/perception/detections/<arm>`.
  * **no camera-device contention inside `srl_*`.** Nothing opens a device;
    every consumer subscribes. The device owner is the vendor
    `ros2_kortex_vision` node; two instances of THAT would contend.
  * **the e-stop is a designed second writer** on the arm controller topics.
    The first version of the checker flagged it as contention wrongly.
  * `/blocking` had 1 publisher, 0 subscribers: `blocking_aggregator` is not
    started by `teleop.launch.py`.

NEEDS THE LAB: real Kortex sessions, the vision driver on real cameras, and
mode switching with the real cascade up.

## PART 5 - free-form language: DONE, this commit

`srl_autonomy/referring.py`. **Opens the NOUN, not the VERB.** The recorded
decision against an LLM still holds: mis-hearing a verb makes the robot do the
wrong ACTION, while mis-grounding a noun makes it do the right action to the
wrong THING, which reachability, the keep-out and confirmation all bound.

Measured on 30 phrasings, most never designed for (politeness, filler,
misspellings, superlatives, compound clauses, relational, and phrases that
sound specific while constraining nothing):

    CORRECT        17   56.7%
    ASKED           7   23.3%
    REFUSED         6   20.0%
    MISUNDERSTOOD   0    0.0%   <-- the dangerous category

Verb handling unchanged and still closed: 0 mismatches over 8 cases, with
"stop" winning even inside "grab it but stop if it slips".

TWO REAL BUGS, found by the undesigned phrasings:
  * **relational references grounded to the ANCHOR.** "the one behind the red
    block" returned the red block with full confidence -- the only dangerous
    outcome in the set. Relational phrases are now REFUSED by name.
  * **verb words leaked into the noun list**, so "grab the leftmost object"
    was scored against the word "grab" and refused.

Superlatives are DEFINITE (argmax, not graded scoring), which is semantics
rather than threshold tuning and converted four wrong ASKs into CORRECT.

Reachability is checked BEFORE announcing and names why:
"I can see the far cup, but I can't reach it: it is 1.40 m in front of me,
past my reach." Keep-out objects are filtered BEFORE grounding, so a forbidden
target is never offered as an option.

12 known-answer tests.

## PART 6 - GUI: DONE (485fad5)
Qt5, Helvetica via a user-level fontconfig alias, **RViz embedded as a panel**
by X11 reparenting (verified from pixels, 31 fps), five indicator groups, and
a frame time of 0.45 ms median / 0.76 p95 against a 100 ms budget. Three
embedding routes costed in `docs/system/07_gui_rviz_embedding.md`.

## PART 7 - VR setup and README: DONE, this commit
`scripts/vr_connect.sh` does the whole connection in one command and then
SUPERVISES the tunnel, because `adb reverse` dies with the USB connection and
opening it once leaves the headset silently unable to reach a running bridge.
Verified: with no adb it refuses with the exact install instruction and exits
non-zero.

`docs/img/vr_controls.svg` is the control diagram, drawn rather than tabulated.
Trigger = gripper, grip = clutch, thumbstick y = live scale, face buttons
deliberately unbound. README section added.

**BLOCKED ON: Android Platform-Tools, installed on the WINDOWS side.** Not
present. Nothing in this path has run against a headset.

## PART 8 - operator aids: PARTIALLY DONE, this commit
Six candidates evaluated in `docs/system/08_operator_aids.md`; four rejected
with rig-specific reasons (FSR conflicts with the gripper twice; the IMU is
the primary direction sensor and one unit spikes 72 deg/s while stationary;
snap-to-object would make the DIRECT baseline not a baseline).

BUILT: `boundary_feedback_node` -- predicts the workspace wall ahead along the
direction of travel and names which wall it is. **Live verification did NOT
complete**: /compute_ik was not being served when the test ran. The node
correctly published nothing rather than inventing a boundary, but that is not
evidence it works. FIRST JOB NEXT: one run against a healthy stack to measure
how far the warning leads the wall.

NOT BUILT: the wrist camera relay. Evaluated as the second-best candidate and
still is; it is a subscriber and a widget.

## PART 9 - audit DONE, re-recording NOT DONE, this commit

`docs/system/09_results_audit.md` audits every number in Parts 1-8: how it was
measured, whether the instrument was validated, verdict.

    VALIDATED  14      SOUND 15      UNVERIFIED 4      STRUCTURAL 1

The four UNVERIFIED, none of which is reported as a result anywhere:
end-to-end fingerprint accuracy (no cameras), smallest grippable object (the
mock cannot produce positioning error), anything against a real headset (no
adb), and the boundary warning's lead distance (below).

**BOUNDARY NODE STILL NOT VERIFIED.** It publishes and warns, but reports the
wall at 0 mm along a line a direct IK probe reaches at 6/6 with the same
orientation. Two measurements contradict; at least one instrument is wrong.
Candidates: the node probes before its cached orientation is populated (and
identity is unreachable at 0/6 on that line), or the velocity estimate is
still near zero when the first probe fires. FIRST JOB NEXT.

**CLIP RE-RECORDING NOT DONE.** The 87 clips still use the OLD nine-task
geometry and predate the single-owner gripper fix. Re-recording needs a
healthy stack for hours and was not attempted rather than attempted badly.


6 GUI with embedded RViz, 7 VR setup + README, 8 new operator features,
9 re-record + self-audit. Each is a session's work; they were not begun
rather than begun badly.

---

# SESSION IN PROGRESS — 2026-08-08 (five-task verification + grasp)

Scope of this session, deliberately narrow: finish the N=10 verification,
confirm the five-task set, finish the grasp sequence, commit. **Clip
re-recording, figures and the thesis are NOT in scope** — each has run the
context out when bundled with anything else.

## 1. N=10 full-path verification — DONE, 0 failures

Both arms verified at **0.0000 rad from home** before any sample was taken
(the script refuses otherwise, and that refusal is what invalidated the
previous attempt at 0.87/0.59 rad off home).

    task 1 positioning     left 3/3, right 3/3 targets
    task 2 pick and place  left 3/3 approaches + 3/3 places, right the same
    task 3/4 coupled carry S1 4 wp, S2 11 wp, S3 10 wp -- all VERIFIED
    task 5 dual pursuit    26/26 shell directions, both arms
    TOTAL FAILURES: 0

**No figure changed.** The result is byte-identical to the run committed in
`abd0c74` (`detail` compares equal), which is the first time a reachability
figure in this project has been reproduced across runs rather than merely
re-asserted. The reason it reproduces is that the 500 mm respec moved every
coupled waypoint off the feasibility boundary; the earlier 310 mm geometry
failed 10 of 25 and would not have.

## 2. Grasp sequence — a real defect found in the RECORDED TRACE

The previous session proved the gripper was commanded (109 samples,
0.050–0.424 rad). That was true and still insufficient: **two owners were
writing the gripper on the same cycle.** The scripted grasp called
`send_gripper()`, then `tick()` re-issued the frac schedule immediately after,
so the recorded knuckle sawed through the lift and reached **0.05 (fully
open) at the start of transit** — the object was grasped, dropped, and
re-grasped in mid-air when the schedule's fraction crossed its own threshold.

    BEFORE  lift  0.42 -> 0.275 -> 0.181 -> 0.238 -> ... -> 0.116
            task  0.05 at t=8.79   (dropped)   0.424 at t=10.44 (re-grasped)

    AFTER   lift  0.424 flat, n=18, min == max
            task  0.424 held to t=18.50, then 0.05 once, at the placement

Fixed by giving the gripper a single owner (`grip_hold`): the plan owns it
from pre-grasp to the release fraction, the schedule owns it after, so the
place still happens in one place in the code.

Second defect, same area: the marker attached on the 0.10–0.74 band, so a
40 mm block (needing 0.42) snapped to the gripper at knuckle **0.157**, while
the fingers were still visibly open. `holding()` now takes the object's width
and requires 0.90 of it.

## 3. Five-task set — confirmed, with two honest limits

All five are specified, internally consistent and verified at N=10. The
arithmetic reproduces: sling `s_max = 0.5340` matches the spec exactly, the
33.6 mm length margin is real, and `fail_tilt_deg = 6.8` is `atan(60/500)`.
Two stale figures were found and corrected — the Task 3 docstring still
described a 310 mm span, and `time_above_11_3_s` still named the tilt
threshold of that superseded span.

**LIMIT 1: there are two genuinely bimanual MECHANISMS, not three.** Tasks 3
and 4 are the same mechanism — a coupled object spanning the dead band — run
with a rigid and a compliant object. Task 5 is the second mechanism, two
simultaneous targets in the disjoint sets. That is what the geometry supports.
It is not a shortfall: rigid-versus-compliant is the scientific contrast, so
two objects on one mechanism is deliberate. But the set should not be
described as three independent bimanual demands, because it is not.

**LIMIT 2: nothing runs this spec.** `final5/tasks.py` is consumed by
`verify_final5.py` and by nothing else — no runner, no analyser, no recorder.
The five-task set is verified as a GEOMETRY and unbuilt as an EXPERIMENT. The
superseded nine-task package under `experiments/bimanual/` is what the runner
and the recorded clips still use, and it carries a different span (300/310 mm
against 500 mm) — see the trap comment in `tasks.py`.

## 4. Wrist alignment under `orientation_mode: fixed` — stated plainly

**Under direct teleoperation the wrist cannot align to a grasp, and the
grasp clips do not demonstrate that it can.**

`master_pose_node` declares `orientation_mode` default `"fixed"`: the
commanded orientation is pinned to the home anchor for the whole trial.
Measured, for the Task 2 block at (0.32, 0.35, 1.15):

    anchor quaternion (home EE, what "fixed" pins to)  (-0.0896, 0.4861, 0.8693, 0.0032)
    top-down grasp quaternion required                 ( 1.0,    0.0,    0.0,    0.0)
    WRIST ROTATION NEEDED                              169.7 deg

169.7 deg is very nearly a reversal. There is no channel to command it: the
master's wrist is unmeasurable (left j7 railed and j6 clamped, right j3/j5/j7
dead), which is *why* the mode defaults to fixed, and mode 3
(ORIENTATION_ASSIST) — which exists to supply it — is a STUB because this
`/compute_ik` plugin ignores `OrientationConstraint`.

The grasp sequence works in the recordings because the recorder is a
**scripted path that calls `/compute_ik` directly** with the grasp
quaternion, bypassing the teleop orientation lock entirely. So the clips
demonstrate the ROBOT can execute an aligned grasp; they do **not**
demonstrate an OPERATOR can command one. The three recorded conditions
(direct / assisted / shared) differ only in trajectory smoothing and all use
the same scripted wrist, so they do not distinguish this either.

This is the one place where the autonomy earns a categorical rather than a
quantitative advantage, and it should be claimed exactly that narrowly.

---

# CHECKPOINT — 2026-08-08 (updated after the reachability audit)

Repo: https://github.com/megazron/dococthefinal (`main`). Verified: a clean
clone plus the README's vendor steps builds **21 packages, 0 failures**.

## THE STANDING CAVEAT — read before trusting any number below

**Every figure in the protocols is IK feasibility in SIMULATION.** Nothing in
the bimanual programme has been driven by a human through the master arm. The
DIRECT condition of every task — the baseline the whole design rests on —
depends on channels that are currently **INCOHERENT**: 7 of 14 fail the
coherence test, and degraded mode freezes `l_j2` and `l_j4`, which costs the
left arm its entire radial dimension.

The reachability figures, the verified scenarios, the sling geometry and the
pilot all say what the ROBOT can do. None of them say what an OPERATOR can do
through this master arm today.

## WHAT WORKS — measured, not assumed

| | evidence |
| --- | --- |
| **Real dual-arm control** | two simultaneous Kortex sessions, **21.4 Hz per arm**, no halving |
| **Homing** | all 7 joints inside tolerance; residuals scatter 0.037–0.126 deg after the integral + settle fix (was 7 joints parked at the 0.99 deg deadband edge) |
| **Grippers** | on the Kinova **internal bus**, driven over the existing session — no second session opened |
| **Clutch indexing** | **unbounded**: 338.8 mm over 6 cycles, re-engage jump **mean 0.29 mm / max 0.35 mm**, drift while frozen 0.000 mm |
| **Boot reference** | latches at the master's ACTUAL pose — no calibrated neutral needed |
| **Collision avoidance** | graduated; held at 0.072–0.079 m with **zero** hard-floor blocks, and resumed on retreat |
| **E-stop** | trips on a frozen-but-publishing master in 0.94 s and a silent one in 0.97 s; halt is immediate |
| **Tracking** | up/down and fore/aft correct on both arms |
| **Autonomy pipeline** | modes 1–6, **10/10** mode-6 checks: refusals, ambiguity, wearer keep-out, spoken confirmation, voice stop in **2 ms** |
| **Console** | Dear PyGui, frame time **median 0.80 ms / p95 6.76 ms** against a 33 ms budget |
| **Experiment runner** | `run_bimanual.py` — T2/T3/T5/T6/T7 with real `--participant/--condition/--scenario` |
| **Every protocol coordinate** | re-verified at **N=10 repeats over the whole densified path**: **17/17 scenarios and 17/17 protocol-table figures SOLID**, 0 MARGINAL. `scripts/audit_scenario_reachability.py` |
| **T2 discrete outcome** | blocks placed / missed / dropped, from gripper TRANSITIONS with the opening tracked off the holding arm's live pose. 15 known-answer tests |

## WHAT IS BLOCKED, AND ON WHAT

| blocked | on |
| --- | --- |
| **Lateral tracking** | azimuth comes from j1 alone and couples with arm bend. Gyro azimuth halves the residual (25.6° → 13.5°) but was NOT shipped — three directions from one segment each is not a triad |
| **Left-arm radial motion** | `l_j2` + `l_j4` incoherent. Measured: **no reach observable survives** (R² = 0.133, residual = 93% of the true spread). The j7 rate fallback is a workaround, not a fix |
| **T4 inter-arm handover** | **geometry**, not orientation: 0 of 16 transfer points reachable by both arms |
| ~~**T2 scenarios**~~ | **DONE** — 4 graded scenarios, verified whole-path at N=10, and the discrete outcome is instrumented |
| **Mode 3 orientation assist** | this `/compute_ik` plugin ignores `OrientationConstraint` (measured: identical IK success with and without). Needs a constraint-aware plugin or explicit yaw sampling |
| **Mode 6 participant-readiness** | **detection rate unmeasured.** Models fit (2012/4096 MiB) but the synthetic renderer is out of distribution and measures itself |
| **Voice input** | `/dev/snd` holds only `timer`. The Windows-side UDP sender is written, never exercised |
| **VR on hardware** | `adb` is installed on neither WSL nor Windows |
| **T5 handover success rate** | needs the WEARER'S BUTTON (a foot pedal). Correctly not inferred from joint states — only the wearer knows whether they have the tool |
| **Grasp success in modes 5/6** | unmeasured — execution stops at PLAN |

## THE LAB ORDER

**0. Nothing here needs re-verifying first.** Every protocol coordinate was
re-audited on 2026-08-08 at N=10 over the whole path and all 17 are SOLID.
Re-run `python3 scripts/audit_scenario_reachability.py` only if the mount,
the home pose or a layout changes — all three invalidate it.

**1. Repair `l_j2` and `l_j4`.** These two buy the most: with both frozen the
left arm's commanded set collapses from a 135 mm-thick shell to a *surface*.
Right arm needs no help (R² = 0.986).

**2. `bash scripts/check_channels.sh` after EACH attempt.** Diffs
automatically against `recordings/baselines/channels_20260806.json`.
**Target 12+ of 14 coherent**, where `degraded_mode:=auto` stops engaging.
Run it after each attempt, not once at the end — a repair that breaks a
working channel should be caught immediately.

**3. One 35-minute recapture, gate fix live, EXACTLY ONE STACK.** The
2026-08-06 capture lost 41 of 42 directional segments to a clutch-gating bug
and latched the e-stop through block F. Both are fixed. Two `master_pose_node`
instances are now impossible at the fd level, but confirm the refusal never
appears.

**4. Verify the cascade rate limit end to end.** `clamp_towards` now cascades
`max_step` as well as `max_vel`; unit-tested, **never measured with a stack
up**. **Expected: divergence stays roughly CONSTANT as `lag_trip_rad`
changes rather than scaling with it.** If it still scales, the fix is wrong.

**5. Photograph the task objects** for the detection measure — 25 images per
object at 0.25 / 0.35 / 0.50 m. Pass = **≥95% at working distance**. Until
then mode 6 is not participant-ready.

---

# NEXT SESSION — LAB ORDER

## READ THIS FIRST

**Every number in the protocols is IK FEASIBILITY IN SIMULATION.** Nothing in
the bimanual programme has been driven by a human through the master arm.
The DIRECT condition of every task — the baseline the whole design rests on —
depends on channels that are currently **INCOHERENT**: 7 of 14 fail the
coherence test, and degraded mode freezes l_j2 and l_j4, which costs the left
arm its entire radial dimension.

So the reachability, the scenarios, the sling geometry and the pilot all say
what the ROBOT can do. None of them say what an OPERATOR can do through this
master arm today.

## In order

### 1. Repair l_j2 and l_j4 — these two buy the most
Degraded mode freezes them, and with both frozen the left arm's commanded set
collapses from a 135 mm-thick shell to a SURFACE: no in/out motion at all.
Measured: regressing true reach on every still-coherent channel plus the IMU
gives **R² = 0.133, residual 51 mm = 93% of the 55 mm true spread** — there is
no reach observable left on that arm. The right arm needs no help (R² = 0.986).

### 2. `bash scripts/check_channels.sh` after EACH repair attempt
Diffs automatically against `recordings/baselines/channels_20260806.json`.
**Target: 12+ of 14 coherent**, which is where `degraded_mode:=auto` stops
engaging. Re-test after each attempt, not once at the end — a repair that
breaks a working channel should be caught immediately.

### 3. ONE 35-minute recapture, gate fix live, exactly ONE stack
The 2026-08-06 capture lost 41 of 42 directional segments to a clutch-gating
bug (each arm gated by its OWN button) and the e-stop latched through block F.
Both are fixed. **Exactly one stack**: two `master_pose_node` instances split
the serial stream and invalidated a whole day; that is now impossible at the
fd level (`claim_exclusive`), but confirm the refusal message never appears.

### 4. Cascade rate limit, end to end with a stack up
`clamp_towards` now cascades `max_step` as well as `max_vel`; it is
unit-tested and **has never been measured with a stack running**.
**Expected result: divergence stays roughly CONSTANT as `lag_trip_rad`
changes, rather than scaling with it.** If divergence still scales with the
threshold, the cascade is still not rate-limiting and the fix is wrong.

### 5. Photograph the task objects — the mode-6 detection gate
Per the Part 9 decision below. 25 images per object at 0.25 / 0.35 / 0.50 m.
Pass = **≥95% detection at working distance**. Until then mode 6 is not
participant-ready.

---

# NEXT SESSION

# ============================================================
# BLOCKER — NO PARTICIPANT MAY BE RUN UNTIL THIS CLEARS
# ============================================================

**7 of 14 master channels are INCOHERENT.** Only 6 of 14 are usable
(5 ALIVE + 1 INTERMITTENT). An incoherent channel returns values that span a
wide range but are not a trajectory — consecutive updates jump tens of degrees
in 60 ms, which no hand produces.

**Running participants on that measures the harness, not the research
question.** Every number a session produced would be a property of the wiring.

**Acceptance test — the objective gate:**

```bash
bash scripts/check_channels.sh          # ~3 min, block A only
```

**Target: 12+ of 14 coherent.** It diffs automatically against
`recordings/baselines/channels_20260806.json`, so each repair attempt shows as
a verdict change. Re-run it as often as you like with the iron in hand.

This is a soldering problem. Do not attempt to filter an incoherent channel
into usefulness: the sim is a RELAY, so whatever the pots feed in the sim
reproduces and the real arm copies a second later.

## SECOND BLOCKER — the task layouts are not reachable

`python3 scripts/check_layout_reachable.py` reports **12 of 12 layout points
UNREACHABLE** as the protocols currently specify them. This is independent of
the wiring and must be fixed before T2/T3/T5 can run at all. See
"Front reach" below.

---

Priority order. Item 1 is first because a number derived from it currently
shapes the whole experiment design and **may be a measurement bug**.

Read `CLAUDE.md`'s final section — "HARDENING PASS — CONSOLIDATED SUMMARY" —
before touching anything. The single most important line in it:
**nothing in the hardening pass has ever run against a real arm.**

## Before anything, every session

```
bash scripts/recover.sh                  # known-good from any state
bash scripts/diagnostics.sh              # before blaming the code
ros2 run srl_teleop preflight            # refuses to start, and names why
```

`preflight` now fails on an EXPIRED blocker as well as an active one. An
expiry means the unit asserting it stopped running, so the condition is
unknown — do not start a session on it. See the consolidated summary for the
three-state semantics.

If the graph looks empty, suspect the **ros2 daemon**, not the network:
`ros2 daemon stop && ros2 daemon start`, and use
`ros2 node list --no-daemon` as ground truth.

---

## 1. Reconcile the two IK harnesses — 95.8% vs 0%

**Why this is first.** The 18.8% figure in "The seven constraints" is what
supports the conclusion that *these arms cannot reach a table*, and that
conclusion drives the task volume, the home poses, the grasp library and every
experiment's scenario set. If it is a measurement artefact, a large part of
the design is built on it. Two harnesses disagreed by essentially the entire
range, which is not the signature of a marginal geometry — it is the
signature of a harness bug.

**START HERE, and it is not what you expect: neither harness exists in the
repo.** Both numbers came from ad-hoc scripts written into a scratchpad and
discarded. `grep -rl compute_ik` over `src/` returns only production nodes
(`ik_follower_node`, `grasp_generator`, `master_calibration`) — there is no
committed IK-sweep tool. So this cannot begin by "re-running" either side.

### What to run

1. **Write ONE harness and commit it**, e.g. `scripts/ik_sweep.py`, and make
   it print its own configuration in the header of every run: pose list, seed
   policy, `avoid_collisions`, orientation constraint, group name, timeout,
   and which frame the target is expressed in. Every disagreement below is a
   configuration difference that a printed header would have made obvious.
2. Sample the frontal task volume on each arm's own side and report, per
   sample, the FAILURE REASON, not just a pass rate:

```
bash scripts/run_teleop.sh gate:=false          # move_group + TRAC-IK live
python3 scripts/ik_sweep.py --arm left  --volume frontal --report-reasons
python3 scripts/ik_sweep.py --arm right --volume frontal --report-reasons
```

3. Check these five differences explicitly, because they are the ones that
   can produce 95.8% vs 0% from the same geometry:
   - **`avoid_collisions` true vs false**, and whether the wearer links are in
     the planning scene at all when it is true;
   - **seeded from the current joint state vs from a fixed seed** — the
     continuous-reachability work already showed pointwise IK and
     seeded-from-previous IK answer different questions;
   - **orientation exact vs yaw-free vs an `OrientationConstraint`** — this
     `/compute_ik` setup **ignores the `constraints` field entirely** (measured:
     identical to the digit, 79.3%/68.9% both ways), so a harness that thinks
     it relaxed orientation did not;
   - **target frame** — `world` vs the arm's `base_link`. A frame error gives
     0% cleanly and looks like a reach limit;
   - **a stale `move_group`** holding the pre-mount-fix URDF. Read
     `/robot_description` off the TOPIC and compare the
     `backpack_to_*_mount` origins against the xacro, as was done for the
     mount fix.

### What a pass looks like

One committed harness, run twice with the two configurations, reproducing
BOTH numbers and explaining the gap in one sentence — e.g. "95.8% was
collisions-off with yaw free; 18.8% was collisions-on with an exact pose."
Then a single agreed figure, with its configuration recorded, replacing the
18.8% in `CLAUDE.md`'s constraint 7.

A pass is **not** "the new number is 60%". It is knowing which of the two old
numbers was wrong and why.

### What it blocks

The table-reach conclusion, the mount trade-off between constraints 2 and 7,
the task volume, the home-pose work, and the scenario sets in E1–E6. Do not
re-optimise the mount or the homes until this is settled — both previous
mount searches were invalidated by scoring the wrong thing.

---

## 2. Verify the mount fix against the real arms at the legacy home angles

**Why.** The mount was DERIVED, not searched, and verified only in sim — the
xacro parses, the rpy pair are exact mirrors to 0.000e+00, `/robot_description`
read back off the topic matches to 3.89e-16, and MoveIt reports
`valid=True, contacts=0` at home. None of that is evidence about the physical
bracket.

**GROUND TRUTH, and do not negotiate with it:** at the legacy Kortex home
angles the REAL arms point FORWARD. The home joint angles in
`config/home_positions_{left,right}.txt` and the two `initial_positions` in
`srl_dual.urdf.xacro` are correct and **must not be changed to fix geometry**.
If sim and real disagree, the mount is the suspect.

### What to run

```
bash scripts/check_arm_network.sh                    # latency, loss, ports
bash scripts/start_real.sh --mock                    # rehearse first
ros2 launch srl_teleop real_drivers_readonly.launch.py   # no motion at all
ros2 topic echo /real/joint_states --once
ros2 run tf2_ros tf2_echo world left_end_effector_link
ros2 run tf2_ros tf2_echo world right_end_effector_link
```

Compare the real joint angles against
`config/home_positions_{left,right}.txt` through `kortex_convention`, and the
real arm's physical pose against the sim's P_HOME:

```
left  P_HOME (-0.5195, 0.2138, 1.3928)
right P_HOME (-0.8596, 0.3169, 1.3456)
```

**Read the right arm's home angles off the hardware while you are there.**
`config/home_positions_right.txt` and `config/real_home_reference.txt` both
record that they were NEVER READ and are UNVERIFIED, and right P_HOME depends
on them.

### What a pass looks like

Both arms come over the shoulders to the FRONT at roughly chest height, with
`half_arm_1` crossing in front of the chest — matching the "What you should
SEE in RViz" list. Both hands land on the wearer's left with the arms
crossing; **that is expected**, not a bug — it follows from the home angles
and is provably independent of the mount (`|v_R − M·v_L| = 1.3837 m` for every
mirror-symmetric mount).

Fail signature: arms splayed sideways, reaching up and back over the
shoulders, one hand above the head. That is the OLD mount — suspect a stale
`move_group` process, not a stale install (`srl_description` is
symlink-installed).

**Also check the thing sim can no longer warn you about:** the Gen3 base tube
is 46 mm in radius and starts ~38 mm from the wearer's upper-arm surface, so
`{base_link, shoulder_link, half_arm_1_link}` vs `human_*_upper_arm` are
SRDF-excluded. On the real rig the bracket must stand the base off the
shoulder, or the mounts move outboard/up. **An exclusion silences the alarm;
it does not move the metal.**

### What it blocks

Every clearance figure, the workspace anchor, all grasp poses, and wearing the
rig at all. Worn operation is **not recommended** until the proximal
interference is resolved mechanically.

---

## 3. The wiring repair — seven compromised channels

**Why.** Seven of fourteen pot channels are dead, railed, clamped or
intermittent, and they are the reason the whole system is built around
IMU-primary sensing:

| arm | live | compromised |
| --- | --- | --- |
| LEFT | j1 j2 j3 j4 | **j5** (2.8% zeros, 14% railed), **j6** (12.8% clamped 0), **j7** (75% railed) |
| RIGHT | j1 j2 j6 | **j3** (70% zeros), **j5** (99% zeros), **j7** (99% zeros), **j4 intermittent (12.9% zeros)** |

Two priorities, in this order:

1. **Right j4 — the most consequential.** It is a channel spherical mode
   USES, and its dropouts are bursty: 51.1% of `right_A_back_to_forward`.
   The validator handles it correctly (`valid==0` tracks `j4==0` to within
   0.1%), so no fabricated data reaches the command — the damage is
   **staleness**, a frozen command for half of one sweep. **Check the j4
   connector first.**
2. **All three dead right channels are ROLL joints** (j3, j5, j7). Suspect
   ONE wiring fault fatigued through the rotation, not three independent pot
   failures. Look for a single common-mode cause before replacing pots.

Repairing the wrist channels (left j7/j6, right j3/j5/j7) is what unlocks
`orientation_mode: anchored` — currently `fixed`, i.e. position-only teleop,
because the wrist cannot be measured at all.

### What to run — the gain matrix IS the acceptance test

```
terminal 1:  ros2 launch srl_teleop teleop.launch.py
terminal 2:  bash scripts/run_teleop_capture.sh
             ros2 run srl_teleop analyse_teleop <csv>
```

The recorder is **button-gated** — it waits, you press to start, sweep at your
own pace, press to end. Every segment is gated by the **OPPOSITE** arm's
button, because an arm's own button also toggles that arm's clutch and would
disengage it exactly when the sweep starts. A press under 0.5 s discards the
segment and re-prompts.

**Before capturing, check** `ros2 control list_controllers` shows
`joint_state_broadcaster` and both arm controllers ACTIVE, and
`ros2 topic hz /joint_states` is ~100 Hz. A recording with 0/20440 EE rows has
happened, and the cause was dead `/tf`, not the recorder.

### What a pass looks like

`d(ACTUAL ROBOT EE)/d(master)` — not `d(command)/d(master)` — approaching
`diag(+1,+1,+1)` with a **moving** master. The distinction is not pedantic:
the one existing moving-master measurement showed the command sweeping
correctly at gain ~1.0 while the robot EE gain was **~0.1**.

Specific targets:
- **lateral finally works.** Up/down and fore/aft already track; lateral does
  not, and AXIS_MAP cannot fix it — azimuth comes from j1 alone, and j1 only
  produces lateral motion once a downstream bend offsets the tip from the roll
  axis, so azimuth and elevation are not independent. The reversal test is the
  cleanest metric (it assumes only that you reversed each motion): left is
  12.6° mean error from 180°, right 26.2°.
- right j4 dropouts gone: `valid==0` fraction near zero on the right arm.
- **Do not re-try hybrid azimuth** (`atan2(p_fk.y, p_fk.x)`) until the pot
  ANGLE calibration is fixed — it was tested and made every axis worse
  (left 12.6° → 33.3°), because FK azimuth compounds four suspect angles while
  j1 is one directly-measured zero-referenced quantity. Calibrated q3 spanned
  336° and q6 ±130–170°, which is physically impossible.

### What it blocks

`orientation_mode: anchored`, the two-IMU elbow path (needs a second I²C bus —
`Wire1` on Teensy pins 16/17 — because 0x68 and 0x69 are both already taken),
E4's DOF-recovery manipulation, and any claim that the teleop baseline is a
fair comparison rather than a broken one.

---

## 4. `start_real.sh` end to end

**Why.** It reached `REAL ARMS LIVE` once, then the lag monitor tripped at
0.152 rad. The tuning issue behind that was fixed and **never retested**, so
the gated flow has never completed against real hardware.

### What to run

```
terminal 1:  bash scripts/run_teleop.sh gate:=false
terminal 2:  ros2 run srl_teleop live_monitor
terminal 3:  bash scripts/start_real.sh --mock      # rehearse the identical sequence
terminal 3:  bash scripts/start_real.sh             # then for real
```

`start_real.sh` refuses with a NAMED reason and exit 1 when the sim stack is
down, the e-stop is latched, homing finishes outside tolerance, or the bridge
will not enable.

### What a pass looks like

- Homing: all 7 joints within tolerance (mock reference: ≤0.05 rad, max error
  0.0174 rad = the 1° deadband), and joint_5 taking the short way round.
- `REAL ARMS LIVE`, and the lag monitor **does not trip** through a full
  sequence of deliberate moves.
- On exit, grep for `kortex session closed cleanly`. **The arm permits exactly
  one session and a leaked one blocks the next run.** SIGINT the bridge
  directly if you kill things by hand; SIGKILL leaks the session.
- Read the LOGGED achieved rate, never assume `rate_hz`: expect ~18.4–18.7 Hz,
  not 30. A cycle is two sequential round trips and back-to-back RPCs cost
  ~26 ms each. That is fine — at vmax 0.05 rad/s a joint moves 2.8 mrad per
  cycle against a 17 mrad deadband.

### Expectations worth setting

- **The cyclic path is unusable over WSL** and this is settled, not worth
  re-litigating: every cyclic write is a ~10 ms network round trip, which
  consumes the entire 100 Hz budget. Use the HIGH-LEVEL API
  (`kortex_highlevel_bridge`, `real_arms_highlevel.launch.py`). If you find
  yourself debugging `ros2_control`'s Kortex driver, stop.
- **Mirrored networking is mandatory** (`networkingMode=mirrored` in
  `%UserProfile%\.wslconfig`, then `wsl --shutdown`). Under NAT, TCP 10000
  connects fine and the UDP realtime channel on 10001 times out — 3–6 SECOND
  writes, feedback frozen, controllers reporting "active" while the hardware
  deactivates underneath them.
- **A fast read is the dead-channel signature**, not an improvement. Once a
  write error deactivates the component, `read()` returns a cache. Same
  zero-variance pattern as the frozen `/real/joint_states` and the dead j7 pot.

### What it blocks

Every real-arm experiment, and the **untested Kortex session recovery** —
`/real/session_recover` closes the one permitted session and opens a fresh
one, and that sequence has only ever run against a mock. It is the riskiest
thing in the recovery layer and a real run will hit it first. Exercise it
deliberately once the bridge is up:

```
ros2 service call /real/session_recover std_srvs/srv/Trigger
```

Pass: `fresh session created without a relaunch (recovery #1)`, the arm still
responds, and no leaked session on the next start.

---

## Things that will waste your day if you forget them

- **Never `pkill -f` a node name.** The pattern matches the shell running the
  command and kills your own terminal. Kill an explicit PID list.
- **Never clean up with broad name patterns while a stack you want is
  running.** That is what killed `joint_state_broadcaster` mid-capture and
  produced a recording with zero EE rows.
- **`/dev/shm` fills with stale `fastrtps_*` segments** from killed stacks and
  discovery goes intermittent. Clear them with the stack STOPPED.
- **`ros2 param set` hangs on this box** (the daemon). Use a parameter client.
- **Never use `time.time()` for intervals** — the WSL wall clock steps
  backwards and once produced a send latency of −2321 ms. `time.monotonic()`.
  The audit found six files still doing it; all fixed, keep it that way.
- **`/estop_reset` is a SERVICE, not a topic.** Publishing a Bool at it does
  nothing, which once made six consecutive fault injections report NOT HANDLED
  for that single reason.
- Shell scripts sourcing ROS must wrap it in `set +u` / `set -u`.

---

# FRONT REACH — measured, and it constrains every task

`python3 scripts/diagnose_front_reach.py --arm both`

The arms work far better BEHIND the wearer than in front, and in front is
where every task volume is.

| direction | LEFT | RIGHT | binding constraint |
| --- | --- | --- | --- |
| front | **0.160 m** | **0.100 m** | **IK INFEASIBLE** |
| back | 0.900 m (sweep limit) | 0.900 m | wearer collision |

**(a) The front limit is NOT the wearer, and NOT orientation.**

- Re-solved with `avoid_collisions=False`: **still fails**. So it is not the
  wearer.
- Re-solved with yaw sampled over 8 angles: gains only **+0.010 m** (left) and
  **+0.040 m** (right). So it is not `orientation_mode: fixed` either.
- The limited joints are nowhere near their stops at the failure point:
  **joint_2 at 62% / 57%**, joint_6 at 41%. So it is not a joint limit.

It is genuine kinematic infeasibility for that arm base pose. By contrast
**back** IS wearer collision, and there orientation matters enormously —
yaw-free extends it +0.390 m on both arms.

**(b) Mount rotation DOES trade in our favour, and back does not pay for it.**
`python3 scripts/sweep_mount_tradeoff.py --arm both` (pitch about the mount
x-axis; **nothing applied**):

| delta | left front | left back | left down | right front | right back |
| --- | --- | --- | --- | --- | --- |
| **−40°** | **0.280** | 0.900 | 0.400 | **0.200** | 0.900 |
| −20° | 0.220 | 0.900 | 0.320 | 0.160 | 0.900 |
| **0° (current)** | **0.160** | 0.900 | 0.220 | **0.100** | 0.900 |
| +20° | 0.080 | 0.900 | 0.140 | 0.020 | 0.900 |
| +40° | 0.000 | 0.900 | 0.020 | 0.000 | 0.000 |

−40° nearly doubles front reach (left +75%, right +100%) and improves down
reach (0.220 → 0.400 / 0.180 → 0.360) while **back stays pinned at 0.900**.

**Caveat, and it matters:** back is measured only to the 0.90 m sweep limit,
so it is saturated and cannot show degradation. The trade-off is measured
against a truncated back. Re-run with a larger `--max` before acting.

**NOT APPLIED.** A mount change invalidates P_HOME, the workspace anchor and
every clearance figure, and would force re-deriving the orientation lock.

**(c) 0.160 / 0.100 m is NOT enough. The layouts do not work.**

`python3 scripts/check_layout_reachable.py` → **12 of 12 points UNREACHABLE**,
walking from home exactly as the follower does:

| point | arm | needed | reached |
| --- | --- | --- | --- |
| T2 box lip | right | 0.556 | 0.520 |
| T2 block A/B/C | left | 0.59 / 0.51 / 0.44 | 0.48 / 0.45 / 0.41 |
| T3 tray grips | both | 0.68 / 0.61 | 0.56 / 0.46 |
| T3 place | both | 0.69 / 0.61 | 0.56 / 0.46 |
| T5 cradle, receive | left | 0.48 / 0.56 | 0.41 / 0.44 |

Raising the work surface by 0.15 m and 0.25 m was tested: **still 12 of 12**.

### What has to change — layout, not mount or posture

Three separate problems, and only the first is about height:

1. **The stations are simply too far.** 0.44–0.69 m required against ~0.41–0.63 m
   achieved. Stations must sit within roughly **0.35 m of each arm's home EE**:
   left `(0.697, 0.248, 1.146)`, right `(−0.763, 0.298, 1.179)`.
2. **The naming quirk was applied wrongly in my own protocols.** `left_*` links
   sit at **+x**. The T2 draft put the box at x=−0.28 and assigned it to the
   left arm, i.e. reaching across the body. Corrected in the checker; the
   protocol layout tables still need updating.
3. **T3 is geometrically impossible as specified.** The two home EEs are
   **1.46 m apart in x**. A 300 mm tray on the midline requires each arm to come
   ~0.65 m inward — far beyond the ~0.4 m typical reach. T3 needs either the
   tray placed off-midline within one arm's reach of each grip point, or a
   different home parking pose. **Do not attempt T3 until this is resolved.**

Mount change and wearer posture are NOT the first lever here: the mount buys
+0.12 m of front reach at best, and the shortfall is 0.03–0.23 m across the
points, so layout alone can close it for T2 and T5. T3 needs more.

---

# PARTICIPANT SESSION ORDER

**Gated on the blocker above. Do not run until `check_channels.sh` shows 12+
of 14 coherent, and the layouts are re-sited.**

| | phase | min | notes |
| --- | --- | --- | --- |
| **P0** | Consent, briefing, baselines | 10 | embodiment questionnaire + proprioceptive drift, **BEFORE any system use** |
| **P1** | Familiarisation and practice | 10 | free play, then practice to a stated criterion. **Log whether the criterion was met** |
| **P2** | **T1 Bimanual Reach** | 10 | no objects; doubles as extended familiarisation while producing real characterisation data |
| **P3** | **T3 Coordinated Carry** | 15 | **PRIMARY TASK, deliberately early on the freshest participant** |
| **P4** | Break — harness OFF | 5 | **Borg CR10** fatigue probe |
| **P5** | **T2 Hold and Fill** | 15 | less demanding, forgiving of moderate fatigue |
| **P6** | **T5 Handover to Wearer** | 8 | **OPTIONAL — cut this first if running long** |
| **P7** | Post-session measures | 10 | embodiment + drift repeated, NASA-TLX per condition, trust scale, demand-characteristics question |

**Full session 83 min, which is over the ceiling for wearing unpowered mass.
Without P6 it is 75 min. Treat 75 as the target and P6 as a stretch.**

**T3 goes early on purpose.** The master arm has no gravity compensation, so
the operator's own arm carries its weight throughout and fatigue drifts
monotonically through the session. The headline measure must not be taken
last.

**T4 Inter-Arm Handover does not appear: it is BLOCKED** pending wrist
orientation.

## Counterbalancing and fatigue

- Three autonomy conditions — **DIRECT / ASSISTED / SHARED** — Latin-square
  counterbalanced **WITHIN each task, not across tasks**. Each task is its own
  square, so a task's three conditions are order-balanced against each other.
- **Log elapsed wear time per trial** (`wear_time_s` from harness-on) so
  fatigue enters the model as a covariate.
- **Order cannot remove fatigue, only measure it.** Counterbalancing converts
  a systematic bias into variance; the Borg probe at P4 and the wear-time
  covariate are what make it estimable.

---

# BLOCKED-ON-WHAT (2026-08-07)

| item | blocked by | note |
| --- | --- | --- |
| **T4 inter-arm handover** | **GEOMETRY, not orientation** | corrects the earlier note. 0 of 16 transfer points and 0 of 63 grid cells are reachable by both arms. Wrist angle was never the blocker; there is nowhere to hand anything over. Needs the right arm re-parked in hardware |
| **T2 hold and fill** | geometry | no (hold, release-above) pair exists |
| **T3 coordinated carry** | layout only | feasible at **z 1.10-1.30 m**, not the protocol's 0.885 m table. Put the tray on a stand |
| **mode 3 orientation assist** | this `/compute_ik` plugin **ignores `OrientationConstraint`** | measured: identical IK success with and without it, 79.3%/68.9% both ways over 270 calls. To make it real needs EITHER a constraint-aware IK plugin (e.g. bio_ik or a TRAC-IK build with constraint support) OR explicit yaw sampling inside `ik_follower_node` — sample N yaw values about the commanded approach axis and accept the first that solves. The scaffolding for the second already exists in the home-pose scorer |
| **mode 6 participant-readiness** | **detection rate unmeasured** | models install and fit (2012/4096 MiB) but the synthetic harness is out of distribution and measures itself. Needs the real cameras |
| **voice input** | audio routing, needs the lab | `/dev/snd` holds only `timer`. `scripts/win_mic_sender.py` (Windows-side UDP) is written and NOT exercised with a real microphone |
| **grasp / handover success rates** | the above | unmeasured because execution stops at PLAN: the objects that would be grasped are not reachable by the arms that would need to cooperate, and there is no trustworthy detection to drive them |

## DETECTION RATE — the gate on mode 6 (decision: option 1)

**Chosen: photograph the real objects on the real table and run detection on
those images.** Justified against the alternatives:

- A photorealistic renderer would take a physically-based pipeline, HDRI
  lighting and real material parameters to be worth more than flat shading —
  and it would still be a model of the Kinova camera, not the camera. The
  effort buys a number that must be re-measured on hardware anyway.
- "Accept as unmeasurable" is where it already stands and yields nothing.
- Photographs need only the lab, a phone or the wrist camera itself, and 20
  minutes. It is the honest measurement.

### Procedure

1. Place the task objects on the table in the verified positions.
2. Capture **≥ 25 images per object** at each of 0.25, 0.35 and 0.50 m,
   varying pose and lighting; use the **wrist camera** if the arm is live,
   otherwise a phone at the same working distance.
3. Label the true bounding box once per image (a rectangle drag).
4. Run `scripts/measure_detection.py --images <dir>` (the renderer is
   replaced by a file loader; the metric code is unchanged and already
   known-answer tested).

### What a PASS looks like

| | threshold |
| --- | --- |
| detection rate at 0.25–0.35 m | **≥ 95%** per object |
| position error | ≤ 15 mm mean, ≤ 30 mm p95 |
| latency | ≤ 100 ms per frame |
| false positives on an empty table | ≤ 1 in 100 frames |

**Below 95% at working distance, mode 6 stays not-participant-ready.** It is
there now, and this is the only measurement that moves it.

### Already established, so it need not be redone

- models install and coexist: faster-whisper small/int8 + YOLO-World-s =
  **2012 MiB of 4096**, ~23 ms/frame
- the model is not the problem: **0.89–0.91 confidence on a real photograph**
- ultralytics 8.4.116 + torch 2.13: call `set_classes()` BEFORE the first
  CUDA predict, or it raises a device-placement error
