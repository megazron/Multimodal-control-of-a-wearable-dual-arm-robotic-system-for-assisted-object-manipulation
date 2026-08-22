# NEXT SESSION — 2026-08-23

Supersedes `docs/NEXT_SESSION_2026_08_22.md`, which is still the right
description of the four things to build. This page records what was done on
2026-08-22 and what it changed about the plan.

## READ `docs/HOW_TO_RUN.md` FIRST IF YOU WANT TO OPERATE IT

One page: the two commands, every mode and where its button is, full autonomy
end to end, and a symptom table. This page is the engineering record.

## THE ONE-PARAGRAPH VERSION

Three questions were answered with measurements rather than opinions. **Why
the arms move in such a small space**: the pinned wrist, and it costs a
factor of seven at the point where the work happens — 0.065 m mean reach
against 0.450 m, with eleven of twelve directions refused by the wearer floor
under the pin and four under a free wrist. **What the sim-to-real difference
is**: every joint parks 0.305 deg short of its target on the side it came
from, which is 7.2 mm at the end effector on every move — one parameter,
the same on both arms, removing 74–76% held out on an axis it was never
fitted on, and nothing in the repository was subtracting it. **What LeRobot is
worth here**: the dataset format, today; the policies, not until there is a
corpus, a bigger GPU, and a solved extrinsic. Nothing in any of it ran on a
real arm, and the RViz master that would have shown all three now exists.

---

## 1. THE WORKSPACE — MEASURED, AND THE MECHANISM NAMED

Full write-up: `docs/system/21_what_makes_the_workspace_small.md`.
Instrument: `scripts/measure_orientation_cost.py`.

| policy | from HOME EE | from the WORK POINT |
| --- | --- | --- |
| **pinned (today)** | 0.460 m, 6/28 wearer-bound | **0.065 m, 11/12 wearer-bound** |
| spin (roll free) | +3 mm | **+0 mm** |
| cone 45 deg | +71 mm | **+375 mm** |
| free | +89 mm | **+385 mm** |

The wearer floor was 0.15 m under every policy in every run and was never
relaxed. The mechanism is not "the wearer is in the way": a fixed 6-DOF pose
on a 7-DOF arm spends the whole redundancy, and the redundancy is the joint
whose purpose is to move the elbow out of the person while the hand stays
put.

**Built:** `srl_teleop/orientation_policy.py`, one source consulted by both
`ik_follower_node` and the measuring script. Default `exact` — byte-for-byte
today's behaviour, so no recorded mode comparison changes. `cone(deg)` tries
the commanded orientation first and only tilts after every redundancy seed
has failed. Relaxations are logged and published on `/ik_status_<arm>`.

**Refused by name:** `spin`. Freeing the roll while holding the axis measured
+0 mm, and it is the relaxation that sounds safest.

**Found on the way:** `WORKSPACE_ORIENT` is not stored unit (norms 0.99995844
and 1.00000561), so a rotation built from it is not orthonormal and a 25 deg
cone leaked to 24.9996 deg. Normalised on the way in; the constant is not
rewritten.

### WHAT TO DO NEXT WITH IT

1. **Do not switch a mode to a cone yet.** HARD CONSTRAINT 1's own reasoning
   applies: the task set must be re-measured first. The 385 mm is what the IK
   can reach, not a claim that any task performs better.
2. Run `--as-follower` for all 26 directions from both start points, so the
   number quoted is the discrete candidate search the robot runs rather than
   a continuous bound.
3. Then pick ONE mode — 06 is the honest candidate, it already declares its
   own approach — give it `orientation_cone_deg`, and re-record T1 both
   stages against the existing clips.

---

## 2. SIM TO REAL — THE ARM STOPS 0.305 DEG SHORT ON EVERY JOINT

Instrument: `scripts/measure_sim_to_real_gap.py`.
Read side: `srl_teleop/sim_to_real_gap.py`.
Model: `recordings/baselines/sim_to_real_gap.json`.

**The measurement already existed and nothing read it.**
`recordings/baselines/arm_directional_calibration.json` — 36 real runs, both
arms, six directions, 100 mm each, taken on the rig
(`recordings/calibration_frames/*.jpg` are the photographs).

* Commanded-vs-achieved error **7.2 mm RMS on both arms** for a 100 mm move.
* In joint space, 211 of 252 per-joint terminal errors sit at ±0.25–0.30 deg
  with the sign a coin flip. Not a spread — a BOUND. A joint parking just
  inside a band, on the side it arrived from.
* Pushing those recorded joint errors through the arm's own Jacobian
  reproduces the Cartesian error to **90%, r = 0.923**.

### THE SHAPE MATTERS MORE THAN THE SIZE, AND I GOT IT WRONG FIRST

Every run is 100 mm, so "the arm travels 93.5% of what it is told" and "the
arm stops 7.2 mm short" fit the Cartesian data **identically**. On a 20 mm
move they differ by a factor of five. I shipped a Cartesian 3×3 gain first;
the Jacobian result above says it is the wrong shape, and a gain applied to
short moves would have been confidently wrong.

The model is one number per arm: **EPS = 0.3052 deg (left) / 0.3059
(right)**, and the compensation is *overshoot each joint by EPS in the
direction of travel*. No matrix, no direction basis, no step length.

| | identity | fitted | **held out on an unseen AXIS** |
| --- | --- | --- | --- |
| joint model, 1 parameter | 7.24 mm | 1.83 mm | **1.88 mm** |
| Cartesian 3×3, 9 parameters | 7.24 mm | 0.98 mm | **93.5 mm** |

The 3×3 is kept in the model file and is not used.

### AND A CLAIM I HAD TO WITHDRAW

I reported the error as "speed-independent across a 5× range". It is not a
speed sweep: `vmax_rad_s` was read once at bridge start-up, and **7 of 36
runs moved FASTER than the limit they were commanded with**.
`kortex_highlevel_bridge` records this in its own source and I asserted the
opposite. The three speeds are three replicates of one condition, any
leave-one-speed-out figure is a fit residual, and **how the error varies with
speed has never been measured.** A control now asserts the sweep still looks
inert, so if a future recording really varies speed the test fires and the
stale sentences get rewritten.

### THE FIX, IN THE BRIDGE

* `deadband_deg` **1.0 → 0.10**. Inside the deadband the proportional term is
  off, so only the integrator can close the last fraction of a degree — and
  1.0 deg is 3.3× wider than the residual it leaves behind. **Not yet tried
  on hardware**; it is a live parameter and the first session must watch for
  dither.
* `terminal_overshoot` (default **off**) applies EPS at the setpoint. Two
  corrections for one error is one too many, so it stays off until the
  deadband change is measured. On with a zero overshoot is refused by name —
  that state logs as compensated and moves the arm as if it were not.

Four runs settle which one wins.

## 3. THE GUI -- THREE DEFECTS, FOUND BY RUNNING IT AND LOOKING

The audit passed 203 of 203 the whole time these were live, because it
presses buttons and none of them are about pixels or processes.

**RViz was never actually embedded.** `_rviz_windows()` matched any X window
with "rviz" in its line and the caller took `sorted(new)[0]`, the LOWEST id.
rviz2 creates several windows besides the one it draws in, and the lowest is
`Qt Selection Owner for rviz2` -- invisible. So the GUI reparented a helper
into the COMMANDED panel, the geometry dump measured the CONTAINER and read
1005..1554 exactly as designed, every check passed, and the real RViz sat in
its own window on top of the GUI. Now filtered by WM_CLASS and size, largest
wins, and the chosen window is checked for viewability before it is used.

**rviz2's stdout and stderr went to /dev/null.** An RViz that started and
died was a zombie and a blank panel, and the fallback message blamed X11 --
"RViz did not present an X window in 45 s. Embedding needs an X11 session" --
for a process that had already exited. Now `.scratch/rviz_<panel>.log`, and
the panel reports the exit code and the last lines.

**`host.setMaximumWidth(392)` capped the control column below its own
content.** Measured offscreen: 378 px of content in a 374 px viewport, with
horizontal scrolling off, so every label and button label was cut at the
right edge and no splitter change could ever help. `_fit_left_column()`
measures the shortfall from the laid-out widget and tops it up (three
formulas missed first, by 20, 25 and 4 px); `clipped_pages()` is now asserted
by the audit, which reads *all pages fit; column 398 px*.

**And RViz leaked.** `closeEvent` terminated it, which covers exactly one
exit path; a crash or a SIGKILL left it running. Three orphans accumulated
over three restarts. `PR_SET_PDEATHSIG` now makes the kernel kill it with the
parent -- verified by SIGKILLing the GUI and watching rviz2 go.

## 3b. EVERY MODE IS ON THE RUN TAB

It opened on `START VR TELEOP` with the mode launchers the FIFTH panel down,
so the honest answer to "what can this window do" was "VR teleoperation".
**OPERATE -- how the arms are driven** is now the first panel: master teleop,
VR/desk teleop, shared autonomy, and **FULL AUTONOMY -- type what you want**,
which brings the Instruct prompt to the front of the middle column.

Every button runs a Spec that already existed in `gui_launch_specs`; what
changed is that you can see them. `live mode:` beside them is read from live
publishers, never from which button was pressed.

## 3c. THE ARMS MOVE IN EVERY DIRECTION NOW

`ik_follower_node` defaults to `orientation_policy:=cone`,
`orientation_cone_deg:=15.0`, in every mode. Measured through the follower's
own discrete candidate list at the work point, floor enforced throughout:

| policy | mean reach, 12 directions |
| --- | --- |
| pinned (was the default) | 0.065 m |
| **cone 15 (is the default)** | **0.304 m** |
| cone 45 | 0.325 m |
| free | 0.362 m |

15 deg buys 239 mm of the 260 mm a 45 deg cone buys, at a third of the tilt.
It cannot change a motion that already worked -- the commanded orientation is
candidate 0 and a tilt is only reached after every redundancy seed has
failed. `orientation_policy:=exact` reproduces any earlier recording.

## 3d. THE GRASP PIPELINE'S WEARER FLOOR COULD NEVER FIRE

`grasp_pipeline.reachable()` read `clearance_m` -- the WHOLE chain, which
includes the immobile mount stub and therefore reads the constant 0.2202 m in
every pose the arm can reach. Against a 0.15 m floor that is true always, so
**every grasp this pipeline ever approved was checked against a test that
could not fail.** It reads `clearance_moving_m` now, and a test aims the hand
at the middle of the wearer's torso and requires a refusal that names the
wearer.

Its refusals also asserted "outside the arm's 902 mm reach" -- a conclusion
drawn from a search -- and now quote the distance the seeded search actually
reached. The seeding changed too, and that one is honestly a hardening: the
control could NOT make the old uniform seeding fail.

Detection was never the bottleneck and the model shortlist is in
`docs/system/22_grasping.md`, scored against 4 GB of VRAM.

## 4. THE RVIZ MASTER

`srl_teleop/rviz_master.py` (pure, self-tested, no ROS) and
`rviz_master_node.py` (the shell). `ros2 run srl_teleop rviz_master`.

The five things `NEXT_SESSION_2026_08_22.md` asked for:

| topic | what |
| --- | --- |
| `/viz/states` | COMMANDED, ACTUAL, and PLANNED-BUT-REFUSED with its reason string — and a refused pose **cannot be drawn without one** |
| `/viz/wearer` | the body inflated by the floor, from the body the guard is ENFORCING; the nearest segment coloured by margin and labelled with the part |
| `/viz/path` | the densified sweep, one point per sample, coloured by that sample's own clearance, worst point called out by index |
| `/viz/liveness` | joint-state age per arm, IK failures, guard pairs checked, and whether the wrist has been unpinned |
| `/viz/envelope` | the measured workspace as three volumes — reachable, wearer-limited, out of reach |

32 self-test checks, all passing, with no display. The colour function is
pure arithmetic with a known answer, an UNKNOWN clearance never renders
green, and an empty wearer says so rather than drawing nothing.

**`/viz/path` is now fed.** `safe_motion.check_path(..., trace=[])` fills one
entry per densified sample — end-effector point, that sample's own clearance,
and the part it is nearest to — and continues past the first breach instead
of stopping there, because the half after the breach is the half that shows
where the arm was going. Three controls in `safe_motion.self_test`: the trace
covers every sample (47 of 47 on the breaching case), the verdict with a
trace is byte-identical to the verdict without one, and the trace's own
minimum equals the reported worst. The node subscribes to `/viz/path_in` and
draws the checker's own sentence beside the worst point.

**Still to do:** `/viz/envelope` is built and tested but nothing feeds it —
it wants the finished 26-direction sweep. And nothing publishes a
PLANNED-BUT-REFUSED pose to `/viz/refused_in` yet; `safe_motion` refuses
in-process and would need to publish when it does.

---

## 5. LEROBOT

Full assessment: `docs/system/20_lerobot.md`. Short version: the format is
worth having and has been taken; the policies are blocked three ways
(no corpus — ethics; 4 GB VRAM; and a policy learns around an unmeasured
extrinsic rather than fixing it).

`scripts/lerobot_export.py` exported 23 episodes / 20 440 frames of the real
teleop recording, with the 8 channels the newest baseline calls INCOHERENT or
DEAD dropped and named in the dataset's own metadata.

**`.venv_lerobot` is separate on purpose.** Installing lerobot into
`.venv_vision` upgraded numpy to 2.2.6 and broke system scipy, taking
`scripts/real_calibration/check_all.py` from 4/4 to 2/4. The vision venv has
been restored and re-verified at 4/4. Do not install lerobot into it.

---

## WHAT IS STILL TRUE AND UNCHANGED

* **Nothing in this session ran on a real arm.** The workspace work is
  offline FK/IK; the sim-to-real work is analysis of a recording taken on
  2026-08-21.
* The camera→robot extrinsic still has 0% coverage on real recordings, and
  the one-line threshold fix identified last session is still untested.
* HARD CONSTRAINT 0 stands: the real arms are at the legacy Kortex home and
  the presentation pose has never been captured on them.
* All the standing traps in `NEXT_SESSION_2026_08_22.md` still apply.
