# LeRobot, assessed against what this project actually needs

Written 2026-08-22, after installing it and exporting real data through it.
`scripts/lerobot_export.py` is the working code; this is the honest reading
of what the library is worth here and what it is not.

## THE ONE-PARAGRAPH VERSION

`lerobot` 0.6.1 installs cleanly, its dataset format is a genuinely good fit
for this repository's recordings, and **its policies cannot help with
anything that is currently broken**. The 2026-08-21 session did not fail on
perception — FastSAM found the cube at score 1.0 with 1166 depth points and
zero false positives. It failed on an unmeasured camera→robot extrinsic, on
kinematics, and on eight defects in the code that commands the arms. No
behaviour-cloning policy repairs an extrinsic; it learns around one and then
breaks when the camera moves. So the thing to take from LeRobot today is the
**format**, and that has been taken: 23 episodes and 20 440 frames of real
recorded teleoperation now exist as a `LeRobotDataset`, with the six columns
that are trustworthy exported and the eight that are not refused by name.

## WHAT WAS ACTUALLY DONE

```
python3 scripts/lerobot_export.py --list        # what is exportable, and why not
.venv_lerobot/bin/python scripts/lerobot_export.py --self-test
.venv_lerobot/bin/python scripts/lerobot_export.py \
    --csv recordings/teleop_20260731_133310.csv \
    --repo-id srl/kortex_master_teleop \
    --out recordings/lerobot/kortex_master_teleop
```

23 episodes, 20 440 frames, 48 fps, 752 KB. One episode per labelled segment
from the recorder's own sidecar — `left_A_down_to_up`, `clutch_reengage` and
so on — with the segment label as the LeRobot task string.

**It lives in its own virtualenv, `.venv_lerobot`, and that is not tidiness.**
Installing `lerobot` into `.venv_vision` upgraded numpy to 2.2.6, which broke
the system scipy that `scripts/real_calibration/` depends on, and
`check_all.py` went from 4 of 4 passing to 2 of 4 with
`cannot import name 'Inf' from 'numpy'`. The vision venv has been restored
(numpy 1.26.4, scipy 1.11.4, cv2 4.11.0, torch 2.13.0+cpu, `check_all.py`
back to 4 of 4) and lerobot must never be installed into it again.

## THE CHANNEL GATE, WHICH IS THE INTERESTING PART

`recordings/teleop_*.csv` is the only recorded HUMAN-driven data in the
repository, and the newest `recordings/baselines/channels_20260806.json` says
what its master-arm columns were doing:

| verdict | channels |
| --- | --- |
| ALIVE | `l_j7`, `r_j1`, `r_j2`, `r_j4`, `r_j6` |
| INTERMITTENT | `l_j1` (2.2% dropouts) |
| INCOHERENT | `l_j2` `l_j3` `l_j4` `l_j5` `l_j6` `r_j3` |
| DEAD | `r_j5`, `r_j7` (range 0.0 deg, 0 updates) |

A DEAD channel is a column of one repeated value. Exported without comment it
becomes a zero-variance feature that a network learns to ignore — or, worse,
a column somebody later reads as "the wrist did not move". That is two of
this repository's own listed instrument failure modes at once: *zero
variance — dead channel*, and *a gap that is not a gap*.

So the exporter reads the newest baseline and **drops** DEAD always,
INCOHERENT by default, and writes `meta/srl_channel_health.json` into the
dataset itself so the verdicts travel with the data onto the Hub.

**Dropped, not aborted, and that distinction cost a rewrite.** The first
version refused the entire export when any channel was DEAD, which on the
only real recording in the repository meant the tool did nothing at all. The
lie would be *including* a dead column, not omitting it; a gate applied one
level too high protects nothing and guarantees the pipeline is never
exercised. `--self-test` now asserts both halves: a dead channel is dropped
and named **and** the other thirteen still export.

## WHAT THE EXPORTED DATASET IS AND IS NOT

It is a **format demonstration and a pipeline test**. It is not a
demonstration corpus, and that sentence is written into the dataset's own
metadata so it cannot be separated from it:

* These are MASTER-ARM potentiometer channels on an instrumented mannequin —
  a leader device, not robot joint encoders.
* 7 of 14 were not coherent when it was recorded, so 6 of 14 survive the gate.
* No policy trained on it means anything.

## THE POLICIES: THREE INDEPENDENT BLOCKERS

**1. There is nothing to train on.** Imitation learning needs human
demonstrations and this project has collected none — an ethics block, not a
code one. Operator and wearer are different people and consent separately,
and worn operation is >17 kg with no gravity compensation on somebody who did
not choose the motion.

**2. This machine cannot train one.** RTX A500, 4 GB, and `.venv_vision`'s
torch is deliberately the CPU wheel. ACT at ~80M parameters is borderline;
SmolVLA at 450M is not happening locally. CLAUDE.md already records YOLO-World
being chosen over GroundingDINO (218M) and OWLv2 (428M) for exactly this
reason.

**3. A policy would learn around the calibration error, not fix it.** This is
the one that would still apply with a bigger GPU and a corpus. The
camera→robot extrinsic has **0% coverage on real recordings**; the
commanded-vs-achieved Cartesian error is 7.2 mm and systematic
(`docs/system/findings.md`, and `scripts/measure_sim_to_real_gap.py`). A
behaviour-cloning policy trained through those errors encodes them, and the
first time the camera is knocked or the arm is re-mounted the policy is
wrong in a way that has no error signal.

## WHERE LEROBOT WOULD EARN ITS PLACE LATER

In rough order of when it becomes worth doing:

| when | what | why then |
| --- | --- | --- |
| now | `LeRobotDataset` as the recording format for every new session | costs nothing, and the day demonstrations exist they are already in a standard shape. Done. |
| after the extrinsic is solved and the sweep finished | export the real-arm sweeps too, with camera frames as video | gives a corpus with known ground truth, which is the only kind this repository accepts |
| after ethics clearance and a corpus | ACT or Diffusion Policy, trained OFF this machine | 4 GB is the binding constraint, not the algorithm |
| probably never here | SmolVLA / pi0 | a 450M VLA on a two-arm wearable rig with 5 alive master channels is not the next problem |

## THE STANDING RULE APPLIES TO A POLICY TOO

*A model's output is a measurement, and it is an instrument until it has been
cleared.* Any grasp a learned policy proposes must be scored against the
support plane and the wearer floor before it reaches an arm, exactly as
`fuse()` and `mount_guard_node` already require of the camera. A policy is
not exempt because it is a policy.

## FILES

| file | what |
| --- | --- |
| `scripts/lerobot_export.py` | the exporter, with four controls that must pass before anything is written |
| `.venv_lerobot/` | isolated, because installing it elsewhere broke the calibration gate |
| `recordings/lerobot/kortex_master_teleop/` | 23 episodes, 20 440 frames |
| `.../meta/srl_channel_health.json` | which columns are trustworthy, travelling with the data |
