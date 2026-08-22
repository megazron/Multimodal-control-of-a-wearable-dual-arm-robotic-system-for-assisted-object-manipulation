# Motion planning: what we run, what exists, and what to use

Researched and written 2026-08-22. Every external claim below carries its
source. Every claim about *this* project is measured.

---

## FIRST, THE THING NOBODY HAD WRITTEN DOWN

**This project had no motion planner.**

`ik_follower_node` solves **one IK per commanded pose** through MoveIt's
`/compute_ik` and clamps the joint step to `max_step_rad` per cycle. That is
pose streaming with a rate limiter. There is no OMPL in the command path, no
`move_group.plan()`, no `compute_cartesian_path`; the only OMPL reference in
the tree is a vendored Kinova launch file this project does not run.

MoveIt is used for exactly two things: the `/compute_ik` service (TRAC-IK,
`solve_type: Distance`, 5 ms timeout) and the planning scene.

**For teleoperation that is correct.** The operator's hand is the input,
there is no goal to plan to, and the right problem is smooth safe following.

**For autonomy it is not**, and the cost is measured: a pick whose pregrasp
and grasp are both clear, streamed as a straight joint-space line, breaches
the wearer floor at **0.1462 m at step 29 of 64**. `safe_motion.check_path`
was written to *catch* that. Catching it means the run stops.

---

## THE CANDIDATES, AND WHY EACH IS OR IS NOT FOR US

Constraints that decide everything: **RTX A500, 4 GB VRAM** shared with the
detector; WSL2; CPU-only torch by deliberate choice; and a safety case that
turns on a clearance definition MoveIt's own collision checking is **known to
be blind to** (HARD CONSTRAINT 11 — the SRDF excludes the 44 proximal pairs a
shoulder mount actually threatens).

| candidate | what it is | verdict here |
| --- | --- | --- |
| **[cuRobo](https://github.com/nvlabs/curobo)** | CUDA motion generation: collision-free IK at >9000 queries/s, trajectory optimisation, motion generation in ~53 ms average, 62× faster than prior trajectory optimisation; paths in ~20 ms on an RTX 4090 | **The strongest thing in this space, and it does not fit.** It uses ~2.5 GB VRAM at batch 1024 and ~4.5 GB at 2048, measured on a 48 GB card. We have 4 GB *total*, shared. It also wants a CUDA PyTorch stack where this project runs CPU torch on purpose. **Right answer on a bigger machine. Revisit if the GPU changes.** |
| **[Isaac Lab](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)** | GPU sim + RL training, sim2real | **No.** Minimum spec is an RTX 3070 with **8 GB**; current docs list an RTX 4080 with **16 GB** as minimum, 48 GB ideal. GPUs without RT cores are unsupported. We have 4 GB. Even setting VRAM aside, it answers a question we do not have: we are not training policies, we have no corpus, and the ethics block on human data is upstream of all of it. |
| **[Ruckig](https://github.com/pantor/ruckig)** | Time-optimal, **jerk-limited** online trajectory generation. Type V; 100% robustness over a billion trajectories; **19.8 µs mean for 7 DoF**; C++17, no dependencies, MIT | **ADOPTED 2026-08-23 and it is the default in every mode.** `srl_teleop/motion_generator.py`; it replaces `clamp_towards`. Measured: the hand's excursion off the commanded path went **51.3 mm → 0.0 mm** on the left arm and 39.2 → 0.0 on the right, all seven joints arrive on the SAME cycle instead of spread over two, and peak joint speed went from **12.53x** the limit in `joint_limits.yaml` to 1.00x. See §"THE STREAMING PATH" below. |
| **MoveIt 2 + OMPL** | RRTConnect and friends, already installed | **Already available and unused.** Better tested than anything we would write and it knows more constraint types. Its problem here is the collision model, not the planner: its checking is the SRDF's, and the SRDF is blind to the pairs the safety case turns on. Usable for *free-space* moves; not trustworthy as the wearer guard. |
| **MoveIt Servo** | Real-time Cartesian/joint jogging with collision slowdown | **The right shape for teleop** and worth evaluating against the current follower. It brings singularity handling and collision-proximity slowdown, both of which we do by hand or not at all. |
| **Octomap / MoveIt occupancy monitor** | Depth camera → voxels → FCL collision | **The idea is right and we should not use the implementation.** MoveIt's octomap is well known for voxels that do not clear, making plans fail for ever against obstacles that are gone. We build a per-frame `VoxelWorld` instead and deliberately do not carry occupancy between frames. |
| **Pinocchio** | Fast rigid-body dynamics, analytic derivatives | Worth having for exact Jacobians and nullspace projection; we compute the Jacobian by finite differences today. Not urgent. |

---

## WHAT WAS BUILT INSTEAD, AND WHY

`srl_perception/joint_planner.py` — **RRT-Connect in joint space**, with the
choices made for this robot:

* **Joint space, not Cartesian.** The binding constraint is the distance from
  the whole *arm* to the wearer, not from the hand. A Cartesian planner would
  solve IK at every node anyway.
* **The wearer floor is the collision check**, through the same
  `clearance_parts(...)["moving_chain_m"]` the guard enforces — not the SRDF,
  which is blind to the pairs that matter.
* **Every edge densified at 0.05 rad**, the same as `safe_motion`. An RRT that
  checks only its nodes has the old pick's defect one level up.
* **Straight line tried first.** Most moves are fine; planning a clear move
  turns 60 ms into seconds.
* **Shortcut, then resample to the follower's step.** Handing the follower
  waypoints further apart than `max_step_rad` means it invents the
  intermediate motion with an unchecked straight interpolation — the defect
  the planner exists to remove, reintroduced at the last step.
* **A `VoxelWorld` from the observed scene**, so the arm avoids the table and
  the objects and not only the person — with a self-filter, because the wrist
  camera sees the gripper and voxelising it puts a permanent obstacle exactly
  where the hand is.

### The measured result

The move that the straight-line check refused:

```
STRAIGHT LINE pregrasp->grasp:  wearer clearance 0.1462 m at step 29/64,
                                under the 0.150 m floor
PLANNED:  RRT-Connect, 4 iterations, 3 waypoints
          worst clearance ANYWHERE on it: 0.2392 m
          safe_motion agrees on every leg: True
```

A 4 mm breach became **89 mm of margin**, and the planner's output is
re-checked by the same checker that rejected the straight line — a planner
does not mark its own homework.

**Cost: 6.9 s for that plan**, dominated by the clearance call. That is fine
for a pick and far too slow for anything reactive. It is the number cuRobo
would fix.

---

## THE OTHER HALF: WE WERE FLYING BLIND ON REAL HARDWARE

"Everything works in simulation and then real hardware messes up" has a
concrete cause that is not the planner.

**The bridge read one thing from the arm: `GetMeasuredJointAngles()` — seven
numbers.** That was the entire sensory input this project has ever had from
real hardware.

The [Gen3 exposes far more](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_cpp/doc/markdown/messages/BaseCyclic/ActuatorFeedback.md)
through `BaseCyclic.RefreshFeedback()`, **for the same one round trip**:

| per actuator | at the base |
| --- | --- |
| position, velocity | `tool_external_wrench_force_{x,y,z}` |
| **torque** (each joint has a torque sensor) | `tool_external_wrench_torque_{x,y,z}` |
| **current_motor**, voltage | `imu_acceleration_*`, `imu_angular_velocity_*` |
| **temperature_motor**, temperature_core | `fault_bank_a`, `fault_bank_b` |

In simulation there is no friction, no gravity sag, no contact and no thermal
derating. On the arm there are all four, and nothing could see any of them. A
system that cannot tell its gripper has hit something keeps pushing.

`srl_teleop/arm_telemetry.py` parses all of it and decides three things:

* **`in_contact`** — is something pushing back at the tool. This is the one
  that matters: it is the difference between "the gripper is closing on a
  cube" and "the gripper is pushing a cube across the table because the grasp
  was 20 mm off".
* **`faulted`** — a fault bank is set, so the arm will refuse to servo. From
  upstream that looks exactly like an arm that has stopped following.
* **`overheating`** — the arm **derates itself before it faults**, and a
  derated arm also reads as a control problem.

**The wrench needs a baseline and the code says so.** The tool wrench carries
the arm's own gravity model and the payload, so its zero is not zero and
depends on pose. `contact_baseline()` records free-space motion and gates on
the residual. Without one, `in_contact` compares against a fixed number **and
says in its own reason string that the gate is uncalibrated** — tested: a
constant 15 N gravity load is correctly *not* contact with a baseline, and 7 N
on top of it *is*.

HARD CONSTRAINT 4 is untouched: this is the high-level API's feedback call,
not the cyclic *write* path that is unusable over WSL.

---

## THE STREAMING PATH — DONE 2026-08-23

Full account: `docs/system/findings.md`, 2026-08-23.
Instrument: `scripts/measure_teleop_motion.py` (validated against `srl_fk`'s
own control and four constructed known answers before it reports anything).
Baseline: `recordings/baselines/teleop_motion.json`.

`clamp_towards` had three separable faults, and only the first had a name:

| | |
| --- | --- |
| **not synchronised** | each joint clamped INDEPENDENTLY, so the joints sit at different fractions of their own travel and the hand leaves the line the IK solution implies — **51.3 mm**, against a 30 mm grasp capture gate |
| **not continuous** | from rest the first cycle commands a whole `max_step`; its implied acceleration step is **1343x** the jerk-limited one |
| **it never read `joint_limits.yaml`** | `max_step_rad` / dt is 17.5 rad/s against 1.3963 / 1.2218 rad/s — **12.53x** — and the limits differ per joint, which one `max_step` cannot express |

The excursion is not a tuning value: swept against `max_step` it settles at
68.0 mm rather than tending to zero, because the desynchronisation is
proportional.

Three decisions worth carrying forward:

* **`Synchronization.Phase`, not `Time`.** Both make every joint ARRIVE
  together; only phase keeps them on the straight line BETWEEN the endpoints.
  Phase 0.02 mm, time 9.5 mm, clamp 51.3 mm.
* **Ride ONE trajectory; do not `calculate()` every cycle.** A re-plan from a
  mid-profile state cannot reproduce the profile it is halfway through —
  measured at 0.0264 rad off the line against 8.9e-17 riding one.
* **The velocity limits are the robot's; the acceleration and jerk limits are
  ASSUMED and labelled.** `joint_limits.yaml` declares
  `has_acceleration_limits: false` for all fourteen joints, so there is
  nothing to read. They are two named ramp times, and a test requires the
  label to follow the file if it ever declares one.

**Installed in the system user site, on purpose** — the followers run in the
system interpreter, and ruckig has no dependencies at all:
`pip install --user --no-deps --break-system-packages ruckig`. It cannot
repeat the `.venv_vision` numpy incident, and `dependency_check` now carries
a row for it.

Still unmeasured: everything about it on a real arm, the acceleration and
jerk limits themselves, and the `time_from_start` change (the point is now
one cycle ahead and is timed as one cycle).

## THE ORDER TO DO THE REST IN

1. ~~**Wire Ruckig into the follower**~~ — **DONE 2026-08-23**, above.
2. **Record a free-space wrench baseline on each arm** — one minute of motion
   — and turn contact detection on. Until then the gate is a guess and says so.
3. **Use contact to stop the push.** The grasp pipeline should abort on
   unexpected contact rather than driving through it.
4. **Feed the `VoxelWorld` from the wrist camera in the live path.** It is
   built and tested; `pick_from_table` does not yet pass one.
5. **Solve the scene-camera extrinsic** (re-capture with the arms stowed —
   see `22_grasping.md`) and the same `VoxelWorld` gets a workspace-wide view
   instead of a wrist-cone one. This is the operator's idea and it is right;
   the blocker is calibration, not the planner.
6. **Evaluate MoveIt Servo** against the current follower for teleop.
7. **Revisit cuRobo if the GPU changes.** Nothing else on this page would
   change; it plugs in where `joint_planner` is.

---

## SOURCES

- [cuRobo — CUDA Accelerated Robot Library](https://github.com/nvlabs/curobo)
- [cuRobo technical report](https://curobo.org/reports/curobo_report.pdf)
- [Isaac Sim / Isaac Lab requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)
- [Ruckig — jerk-constrained online trajectory generation](https://github.com/pantor/ruckig)
- [Ruckig paper, RSS 2021](https://arxiv.org/pdf/2105.04830)
- [Kinova Kortex2 ActuatorFeedback](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_cpp/doc/markdown/messages/BaseCyclic/ActuatorFeedback.md)
- [Kinova Kortex2 BaseFeedback](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_cpp/doc/markdown/messages/BaseCyclic/BaseFeedback.md)
- [Gen3 specifications — integrated torque sensors](https://hotrobotics.co.uk/wp-content/uploads/2021/07/Gen3-Specifications-R01.pdf)
- [MoveIt Planning Scene Monitor](https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html)
- [MoveIt perception pipeline / octomap](https://moveit.github.io/moveit_tutorials/doc/perception_pipeline/perception_pipeline_tutorial.html)
- [MoveIt octomap voxels not clearing](https://github.com/moveit/moveit2/issues/444)
