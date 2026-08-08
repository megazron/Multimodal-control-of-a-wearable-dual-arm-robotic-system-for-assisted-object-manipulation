# VERIFICATION CLIP INDEX

Every task x scenario x autonomy condition, driven in sim and recorded. **87 clips.**


## How to review one

Each folder holds `clip.mp4`, `plot_metrics.png`, `summary.json` and a
rosbag2 in `bag/`. The clip has two panels:

* **left** — 3-D view, wearer drawn as the collision primitives from
  `human_backpack.xacro`;
* **right** — **front view** (x-z), wearer facing you. This is the panel to
  judge from: the 3-D view cannot show whether two grippers are level, which
  is the whole of the T3/T6 metric.

The title bar carries live **minimum clearance** and, for T3/T6, live
**|dz|** between the grippers.

**Read the front view knowing y is projected away.** The wearer outline and
the arms overlap on screen whenever the arms are in front of the body — which
is where the tasks are. Clearance in the title bar is the real 3-D number;
trust it over the picture.

**The first ~1 s of every clip is the approach from home**, tagged
`[approach]` in the title. It is included because it is the largest motion and
the most collision-relevant, and it is **excluded from the tracking metric**
(the command is deliberately far ahead of the arm during it). `summary.json`
reports both: `tracking_rms_mm` (task phase) and
`tracking_rms_incl_approach_mm`.

## What the conditions are, honestly

`direct`, `assisted` and `shared` here differ **only in how much the commanded
path is smoothed** (direct: no smoothing -- the command is the raw path, assisted: 30% smoothing on the command, standing in for assistance, shared: 55% smoothing, the most machine-shaped motion). **The autonomy stack did not run.** These clips
verify GEOMETRY, MOTION and CLEARANCE for every scenario; they are not a
measurement of assistance, and no autonomy claim should be read off them.


## Automatic checks across all 87 clips

| check | result |
| --- | --- |
| any arm link inside the wearer | **0**  |
| below the 120 mm `real_robot` clearance floor | **48 of 87** |
| clip shows no motion (EE travel < 50 mm) | **0** |
| clips / plots / bags present | 87 / 87 / 87 |

---

## T2 — 12 clips

**What it should show.** The FILL arm picks at the pick point, carries across, and opens over the container opening (green square). The HOLD arm stays put the whole time. The two arms never swap sides.

| scenario | cond | EE travel | min clearance | tracking RMS | clip |
| --- | --- | --- | --- | --- | --- |
| S1_short_reach | assisted | 1.417 m | 0.050 m *(below 120 mm floor)* | 0.7 mm | `recordings/verification/t2/S1_short_reach/assisted/clip.mp4` |
| S1_short_reach | direct | 1.469 m | 0.049 m *(below 120 mm floor)* | 0.7 mm | `recordings/verification/t2/S1_short_reach/direct/clip.mp4` |
| S1_short_reach | shared | 1.345 m | 0.051 m *(below 120 mm floor)* | 0.7 mm | `recordings/verification/t2/S1_short_reach/shared/clip.mp4` |
| S2_long_reach | assisted | 1.442 m | 0.050 m *(below 120 mm floor)* | 0.8 mm | `recordings/verification/t2/S2_long_reach/assisted/clip.mp4` |
| S2_long_reach | direct | 1.434 m | 0.049 m *(below 120 mm floor)* | 0.8 mm | `recordings/verification/t2/S2_long_reach/direct/clip.mp4` |
| S2_long_reach | shared | 1.340 m | 0.051 m *(below 120 mm floor)* | 0.7 mm | `recordings/verification/t2/S2_long_reach/shared/clip.mp4` |
| S3_height_change | assisted | 1.456 m | 0.049 m *(below 120 mm floor)* | 0.8 mm | `recordings/verification/t2/S3_height_change/assisted/clip.mp4` |
| S3_height_change | direct | 1.501 m | 0.048 m *(below 120 mm floor)* | 0.8 mm | `recordings/verification/t2/S3_height_change/direct/clip.mp4` |
| S3_height_change | shared | 1.438 m | 0.053 m *(below 120 mm floor)* | 0.7 mm | `recordings/verification/t2/S3_height_change/shared/clip.mp4` |
| S4_tight_tolerance | assisted | 1.457 m | 0.050 m *(below 120 mm floor)* | 0.7 mm | `recordings/verification/t2/S4_tight_tolerance/assisted/clip.mp4` |
| S4_tight_tolerance | direct | 1.401 m | 0.048 m *(below 120 mm floor)* | 0.8 mm | `recordings/verification/t2/S4_tight_tolerance/direct/clip.mp4` |
| S4_tight_tolerance | shared | 1.416 m | 0.051 m *(below 120 mm floor)* | 0.7 mm | `recordings/verification/t2/S4_tight_tolerance/shared/clip.mp4` |

right holds, left fills


---

## T3 — 12 clips

**What it should show.** Both grippers rise together with the brown tray between them STRAIGHT and level -- |dz| should stay near 0 mm. Motion is a VERTICAL lift; nothing travels toward the wearer.

| scenario | cond | EE travel | min clearance | tracking RMS | clip |
| --- | --- | --- | --- | --- | --- |
| S1_short_straight | assisted | 1.302 m | 0.052 m *(below 120 mm floor)* | 2.1 mm | `recordings/verification/t3/S1_short_straight/assisted/clip.mp4` |
| S1_short_straight | direct | 1.314 m | 0.052 m *(below 120 mm floor)* | 2.4 mm | `recordings/verification/t3/S1_short_straight/direct/clip.mp4` |
| S1_short_straight | shared | 1.280 m | 0.052 m *(below 120 mm floor)* | 2.2 mm | `recordings/verification/t3/S1_short_straight/shared/clip.mp4` |
| S2_long_height | assisted | 1.546 m | 0.051 m *(below 120 mm floor)* | 2.1 mm | `recordings/verification/t3/S2_long_height/assisted/clip.mp4` |
| S2_long_height | direct | 1.570 m | 0.051 m *(below 120 mm floor)* | 2.0 mm | `recordings/verification/t3/S2_long_height/direct/clip.mp4` |
| S2_long_height | shared | 1.551 m | 0.051 m *(below 120 mm floor)* | 2.1 mm | `recordings/verification/t3/S2_long_height/shared/clip.mp4` |
| S3_curved_obstacle | assisted | 1.465 m | 0.052 m *(below 120 mm floor)* | 2.1 mm | `recordings/verification/t3/S3_curved_obstacle/assisted/clip.mp4` |
| S3_curved_obstacle | direct | 1.525 m | 0.051 m *(below 120 mm floor)* | 1.8 mm | `recordings/verification/t3/S3_curved_obstacle/direct/clip.mp4` |
| S3_curved_obstacle | shared | 1.442 m | 0.056 m *(below 120 mm floor)* | 2.0 mm | `recordings/verification/t3/S3_curved_obstacle/shared/clip.mp4` |
| S4_tight | assisted | 1.340 m | 0.058 m *(below 120 mm floor)* | 1.8 mm | `recordings/verification/t3/S4_tight/assisted/clip.mp4` |
| S4_tight | direct | 1.391 m | 0.058 m *(below 120 mm floor)* | 1.5 mm | `recordings/verification/t3/S4_tight/direct/clip.mp4` |
| S4_tight | shared | 1.337 m | 0.059 m *(below 120 mm floor)* | 1.5 mm | `recordings/verification/t3/S4_tight/shared/clip.mp4` |

rigid tray, 310 mm span, vertical lift


---

## T5 — 9 clips

**What it should show.** One arm (right, -x side) reaches the cradle, lifts, carries inboard toward the wearer and stops at the receive point (green star) at the wearer's side. The other arm never moves.

| scenario | cond | EE travel | min clearance | tracking RMS | clip |
| --- | --- | --- | --- | --- | --- |
| S1_near | assisted | 0.868 m | 0.068 m *(below 120 mm floor)* | 2.3 mm | `recordings/verification/t5/S1_near/assisted/clip.mp4` |
| S1_near | direct | 0.906 m | 0.068 m *(below 120 mm floor)* | 2.5 mm | `recordings/verification/t5/S1_near/direct/clip.mp4` |
| S1_near | shared | 0.820 m | 0.068 m *(below 120 mm floor)* | 2.0 mm | `recordings/verification/t5/S1_near/shared/clip.mp4` |
| S2_far | assisted | 0.838 m | 0.095 m *(below 120 mm floor)* | 2.3 mm | `recordings/verification/t5/S2_far/assisted/clip.mp4` |
| S2_far | direct | 0.879 m | 0.095 m *(below 120 mm floor)* | 2.5 mm | `recordings/verification/t5/S2_far/direct/clip.mp4` |
| S2_far | shared | 0.795 m | 0.095 m *(below 120 mm floor)* | 2.4 mm | `recordings/verification/t5/S2_far/shared/clip.mp4` |
| S3_busy | assisted | 0.886 m | 0.068 m *(below 120 mm floor)* | 2.1 mm | `recordings/verification/t5/S3_busy/assisted/clip.mp4` |
| S3_busy | direct | 0.922 m | 0.068 m *(below 120 mm floor)* | 2.7 mm | `recordings/verification/t5/S3_busy/direct/clip.mp4` |
| S3_busy | shared | 0.850 m | 0.068 m *(below 120 mm floor)* | 2.3 mm | `recordings/verification/t5/S3_busy/shared/clip.mp4` |

right arm delivers to the wearer


---

## T6 — 12 clips

**What it should show.** Same paths as T3 but the object is a SLING: it sags between the grippers. The ball sits at the lowest point and stays brown while retained; it turns RED if the sag drops below the ball diameter (separation past 340 mm).

| scenario | cond | EE travel | min clearance | tracking RMS | clip |
| --- | --- | --- | --- | --- | --- |
| S1_short_straight | assisted | 1.296 m | 0.052 m *(below 120 mm floor)* | 1.9 mm | `recordings/verification/t6/S1_short_straight/assisted/clip.mp4` |
| S1_short_straight | direct | 1.326 m | 0.052 m *(below 120 mm floor)* | 1.9 mm | `recordings/verification/t6/S1_short_straight/direct/clip.mp4` |
| S1_short_straight | shared | 1.292 m | 0.052 m *(below 120 mm floor)* | 1.8 mm | `recordings/verification/t6/S1_short_straight/shared/clip.mp4` |
| S2_long_height | assisted | 1.549 m | 0.051 m *(below 120 mm floor)* | 2.3 mm | `recordings/verification/t6/S2_long_height/assisted/clip.mp4` |
| S2_long_height | direct | 1.608 m | 0.051 m *(below 120 mm floor)* | 1.7 mm | `recordings/verification/t6/S2_long_height/direct/clip.mp4` |
| S2_long_height | shared | 1.534 m | 0.051 m *(below 120 mm floor)* | 2.0 mm | `recordings/verification/t6/S2_long_height/shared/clip.mp4` |
| S3_curved_obstacle | assisted | 1.505 m | 0.064 m *(below 120 mm floor)* | 2.2 mm | `recordings/verification/t6/S3_curved_obstacle/assisted/clip.mp4` |
| S3_curved_obstacle | direct | 1.529 m | 0.051 m *(below 120 mm floor)* | 2.2 mm | `recordings/verification/t6/S3_curved_obstacle/direct/clip.mp4` |
| S3_curved_obstacle | shared | 1.454 m | 0.052 m *(below 120 mm floor)* | 2.0 mm | `recordings/verification/t6/S3_curved_obstacle/shared/clip.mp4` |
| S4_tight | assisted | 1.371 m | 0.058 m *(below 120 mm floor)* | 1.6 mm | `recordings/verification/t6/S4_tight/assisted/clip.mp4` |
| S4_tight | direct | 1.345 m | 0.058 m *(below 120 mm floor)* | 1.9 mm | `recordings/verification/t6/S4_tight/direct/clip.mp4` |
| S4_tight | shared | 1.303 m | 0.058 m *(below 120 mm floor)* | 1.6 mm | `recordings/verification/t6/S4_tight/shared/clip.mp4` |

compliant sling, 310 mm span


---

## T7 — 18 clips

**What it should show.** Each arm traces a small closed loop about its own centre -- a Lissajous on an 80 mm sphere. The two arms are INDEPENDENT. In the B1/B2 baselines only ONE arm moves.

| scenario | cond | EE travel | min clearance | tracking RMS | clip |
| --- | --- | --- | --- | --- | --- |
| B1_left_only | assisted | 1.712 m | 0.182 m | 4.1 mm | `recordings/verification/t7/B1_left_only/assisted/clip.mp4` |
| B1_left_only | direct | 1.759 m | 0.182 m | 4.7 mm | `recordings/verification/t7/B1_left_only/direct/clip.mp4` |
| B1_left_only | shared | 1.582 m | 0.187 m | 3.9 mm | `recordings/verification/t7/B1_left_only/shared/clip.mp4` |
| B2_right_only | assisted | 1.761 m | 0.161 m | 4.3 mm | `recordings/verification/t7/B2_right_only/assisted/clip.mp4` |
| B2_right_only | direct | 1.802 m | 0.172 m | 4.5 mm | `recordings/verification/t7/B2_right_only/direct/clip.mp4` |
| B2_right_only | shared | 1.634 m | 0.182 m | 4.0 mm | `recordings/verification/t7/B2_right_only/shared/clip.mp4` |
| S1_both_slow | assisted | 1.411 m | 0.178 m | 2.0 mm | `recordings/verification/t7/S1_both_slow/assisted/clip.mp4` |
| S1_both_slow | direct | 1.434 m | 0.178 m | 2.1 mm | `recordings/verification/t7/S1_both_slow/direct/clip.mp4` |
| S1_both_slow | shared | 1.371 m | 0.180 m | 1.9 mm | `recordings/verification/t7/S1_both_slow/shared/clip.mp4` |
| S2_one_fast | assisted | 2.448 m | 0.182 m | 2.6 mm | `recordings/verification/t7/S2_one_fast/assisted/clip.mp4` |
| S2_one_fast | direct | 2.506 m | 0.181 m | 2.4 mm | `recordings/verification/t7/S2_one_fast/direct/clip.mp4` |
| S2_one_fast | shared | 2.297 m | 0.187 m | 2.5 mm | `recordings/verification/t7/S2_one_fast/shared/clip.mp4` |
| S3_both_fast | assisted | 3.472 m | 0.175 m | 4.0 mm | `recordings/verification/t7/S3_both_fast/assisted/clip.mp4` |
| S3_both_fast | direct | 3.581 m | 0.160 m | 4.1 mm | `recordings/verification/t7/S3_both_fast/direct/clip.mp4` |
| S3_both_fast | shared | 3.206 m | 0.181 m | 3.7 mm | `recordings/verification/t7/S3_both_fast/shared/clip.mp4` |
| S4_asymmetric | assisted | 3.026 m | 0.184 m | 2.7 mm | `recordings/verification/t7/S4_asymmetric/assisted/clip.mp4` |
| S4_asymmetric | direct | 3.227 m | 0.180 m | 3.0 mm | `recordings/verification/t7/S4_asymmetric/direct/clip.mp4` |
| S4_asymmetric | shared | 2.722 m | 0.186 m | 2.4 mm | `recordings/verification/t7/S4_asymmetric/shared/clip.mp4` |

pursuit, left 0.20 / right 0.00 m/s


---

## T8 — 12 clips

**What it should show.** The arm reaches out to a target (green star) far on its own side, at a position that is only reachable because the wearer has repositioned. The clip shows the arm AFTER the reposition.

| scenario | cond | EE travel | min clearance | tracking RMS | clip |
| --- | --- | --- | --- | --- | --- |
| S1_left_lean_forward | assisted | 0.699 m | 0.158 m | 1.7 mm | `recordings/verification/t8/S1_left_lean_forward/assisted/clip.mp4` |
| S1_left_lean_forward | direct | 0.695 m | 0.158 m | 1.8 mm | `recordings/verification/t8/S1_left_lean_forward/direct/clip.mp4` |
| S1_left_lean_forward | shared | 0.691 m | 0.158 m | 1.7 mm | `recordings/verification/t8/S1_left_lean_forward/shared/clip.mp4` |
| S2_left_step_forward | assisted | 0.757 m | 0.158 m | 1.8 mm | `recordings/verification/t8/S2_left_step_forward/assisted/clip.mp4` |
| S2_left_step_forward | direct | 0.758 m | 0.158 m | 1.6 mm | `recordings/verification/t8/S2_left_step_forward/direct/clip.mp4` |
| S2_left_step_forward | shared | 0.747 m | 0.158 m | 1.8 mm | `recordings/verification/t8/S2_left_step_forward/shared/clip.mp4` |
| S3_right_crouch | assisted | 0.718 m | 0.170 m | 1.9 mm | `recordings/verification/t8/S3_right_crouch/assisted/clip.mp4` |
| S3_right_crouch | direct | 0.727 m | 0.170 m | 1.9 mm | `recordings/verification/t8/S3_right_crouch/direct/clip.mp4` |
| S3_right_crouch | shared | 0.715 m | 0.170 m | 1.7 mm | `recordings/verification/t8/S3_right_crouch/shared/clip.mp4` |
| S4_right_step_forward | assisted | 0.767 m | 0.170 m | 1.7 mm | `recordings/verification/t8/S4_right_step_forward/assisted/clip.mp4` |
| S4_right_step_forward | direct | 0.785 m | 0.170 m | 1.9 mm | `recordings/verification/t8/S4_right_step_forward/direct/clip.mp4` |
| S4_right_step_forward | shared | 0.769 m | 0.170 m | 1.9 mm | `recordings/verification/t8/S4_right_step_forward/shared/clip.mp4` |

left arm; wearer must lean forward


---

## T9 — 12 clips

**What it should show.** The arm traces a circle whose radius is the sway amplitude. This is the arm HOLDING a world-fixed point while the base moves underneath it -- in the arm's own frame the target orbits. Radius should visibly grow across S1->S4 (0, 20, 60, 100 mm).

| scenario | cond | EE travel | min clearance | tracking RMS | clip |
| --- | --- | --- | --- | --- | --- |
| S1_sway_000mm | assisted | 0.776 m | 0.204 m | 0.0 mm | `recordings/verification/t9/S1_sway_000mm/assisted/clip.mp4` |
| S1_sway_000mm | direct | 0.784 m | 0.202 m | 0.0 mm | `recordings/verification/t9/S1_sway_000mm/direct/clip.mp4` |
| S1_sway_000mm | shared | 0.776 m | 0.202 m | 0.0 mm | `recordings/verification/t9/S1_sway_000mm/shared/clip.mp4` |
| S2_sway_020mm | assisted | 1.014 m | 0.183 m | 1.2 mm | `recordings/verification/t9/S2_sway_020mm/assisted/clip.mp4` |
| S2_sway_020mm | direct | 1.021 m | 0.183 m | 1.2 mm | `recordings/verification/t9/S2_sway_020mm/direct/clip.mp4` |
| S2_sway_020mm | shared | 0.971 m | 0.183 m | 1.0 mm | `recordings/verification/t9/S2_sway_020mm/shared/clip.mp4` |
| S3_sway_060mm | assisted | 1.505 m | 0.145 m | 1.9 mm | `recordings/verification/t9/S3_sway_060mm/assisted/clip.mp4` |
| S3_sway_060mm | direct | 1.517 m | 0.145 m | 2.0 mm | `recordings/verification/t9/S3_sway_060mm/direct/clip.mp4` |
| S3_sway_060mm | shared | 1.457 m | 0.145 m | 1.8 mm | `recordings/verification/t9/S3_sway_060mm/shared/clip.mp4` |
| S4_sway_100mm | assisted | 2.006 m | 0.117 m *(below 120 mm floor)* | 2.1 mm | `recordings/verification/t9/S4_sway_100mm/assisted/clip.mp4` |
| S4_sway_100mm | direct | 2.027 m | 0.117 m *(below 120 mm floor)* | 2.3 mm | `recordings/verification/t9/S4_sway_100mm/direct/clip.mp4` |
| S4_sway_100mm | shared | 1.967 m | 0.117 m *(below 120 mm floor)* | 2.3 mm | `recordings/verification/t9/S4_sway_100mm/shared/clip.mp4` |

wearer sway 0 mm -- arm must HOLD a world-fixed point


---

## Regenerate

```bash
ros2 launch srl_moveit_config demo.launch.py      # sim, arms AT HOME
python3 scripts/record_verification.py --all
python3 scripts/make_clip_index.py
```

The recorder refuses nothing about home pose, but every scenario it replays
was verified from home — see `scripts/audit_scenario_reachability.py`.

