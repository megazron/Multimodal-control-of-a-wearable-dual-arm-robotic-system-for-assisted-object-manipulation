# VERIFICATION CLIP INDEX

**87 runs**, every task x scenario x autonomy condition, driven in sim and recorded two ways.


## START HERE -- how to play them from Windows

The files live in the WSL filesystem. From Windows, paste this into Explorer
or into a media player's Open dialog:

```
\\wsl.localhost\Ubuntu\home\gausms\kortex_ws\recordings\verification
```

VLC, MPC-HC and the built-in Films & TV app all open that path directly. Or
from PowerShell:

```powershell
start \\wsl.localhost\Ubuntu\home\gausms\kortex_ws\recordings\verification\t6\S4_tight\direct\rviz.mp4
```

If `\\wsl.localhost` does not resolve, the older form `\\wsl$\Ubuntu\...`
works on the same machine.

## The two recordings, and which to watch

| file | what it is |
| --- | --- |
| **`rviz.mp4`** | **REAL SCREEN CAPTURE of RViz.** What you would see sitting at the machine: the robot, the wearer, and the task objects moving with the arms. **Watch this one.** |
| `clip.mp4` | a TF-rendered 3-D + front-view plot. Uglier, but drawn from exactly the samples that produced `summary.json`, so the numbers and the picture cannot disagree. Kept for automated checking. |
| `plot_metrics.png` | tracking error and clearance against time |
| `summary.json` | the metrics, and the automatic pass/fail checks |
| `bag/` | rosbag2 of /tf, /joint_states and the commands (regenerable; not in git) |

Every `rviz.mp4` carries a burnt-in overlay: task, scenario, condition,
elapsed time, phase, and the live task metric (tilt for T3, separation and sag
for T6, tracking error elsewhere). The overlay is drawn as scene text inside
RViz rather than composited afterwards, so the number on screen is the number
from that frame.

The first ~1.5 s of every clip is the approach from home, tagged
`approach` in the overlay. It is included because it is the largest and most
collision-relevant motion, and it is EXCLUDED from the tracking metric.


## THE FIVE TO WATCH FIRST

**1. T6 / S4_tight / direct** — 6.2 s

   THE SLING. Live separation and sag in the overlay; the ball sits in the V. S4 is the tight 330 mm case, 10 mm from the documented failure threshold, so it is the one where the object nearly fails.

   `\\wsl.localhost\Ubuntu\home\gausms\kortex_ws\recordings\verification\t6\S4_tight\direct\rviz.mp4`

**2. T2 / S1_short_reach / direct** — 9.4 s

   PICK AND PLACE. A block leaves the slab, travels with the gripper and ends up inside the container. This is the clip that proves the gripper is not closing on nothing.

   `\\wsl.localhost\Ubuntu\home\gausms\kortex_ws\recordings\verification\t2\S1_short_reach\direct\rviz.mp4`

**3. T5 / S1_near / direct** — 8.1 s

   THE HANDOVER -- the canonical Fusion scenario. Tool off the cradle, carried inboard, delivered at the wearer's side.

   `\\wsl.localhost\Ubuntu\home\gausms\kortex_ws\recordings\verification\t5\S1_near\direct\rviz.mp4`

**4. T3 / S2_long_height / direct** — 8.4 s

   THE RIGID CARRY. Tray straight, tilt near zero through a full-band lift. Compare directly against the T6 clip above: same path, different object.

   `\\wsl.localhost\Ubuntu\home\gausms\kortex_ws\recordings\verification\t3\S2_long_height\direct\rviz.mp4`

**5. T9 / S4_sway_100mm / direct** — 10.7 s

   WEARER MOTION at the top of the IV. The arm holds a world-fixed point while the base sways 100 mm underneath it.

   `\\wsl.localhost\Ubuntu\home\gausms\kortex_ws\recordings\verification\t9\S4_sway_100mm\direct\rviz.mp4`


## Automatic checks across all 87 runs

| check | result |
| --- | --- |
| any arm link inside the wearer | **0** |
| below the 120 mm `real_robot` clearance floor | **48 of 87** (see the clearance finding in docs/research) |
| shows no motion (EE travel < 50 mm) | **0** |
| RViz screen capture present | 87 of 87 |
| TF clip / plot / bag present | 87 / 87 / 87 |

---

## T2 — 12 runs

**What it should show.** The FILL arm picks an orange block off the slab, carries it across, and drops it into the TEAL container that the HOLD arm is carrying. Blocks left in the container turn GREEN. The hold arm holds station throughout; the two arms never swap sides.

| scenario | cond | rviz.mp4 | duration | EE travel | min clearance | tracking | object outcome | attachment |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1_short_reach | assisted | yes | 9.5 s | 1.42 m | 0.050 m *(below floor)* | 0.7 mm | 1 block(s) placed | object CARRIED |
| S1_short_reach | direct | yes | 9.4 s | 1.47 m | 0.049 m *(below floor)* | 0.7 mm | 1 block(s) placed | object CARRIED |
| S1_short_reach | shared | yes | 9.5 s | 1.34 m | 0.051 m *(below floor)* | 0.7 mm | 1 block(s) placed | object CARRIED |
| S2_long_reach | assisted | yes | 8.8 s | 1.44 m | 0.050 m *(below floor)* | 0.8 mm | 1 block(s) placed | object CARRIED |
| S2_long_reach | direct | yes | 8.9 s | 1.43 m | 0.049 m *(below floor)* | 0.8 mm | 1 block(s) placed | object CARRIED |
| S2_long_reach | shared | yes | 8.9 s | 1.34 m | 0.051 m *(below floor)* | 0.7 mm | 1 block(s) placed | object CARRIED |
| S3_height_change | assisted | yes | 10.9 s | 1.46 m | 0.049 m *(below floor)* | 0.8 mm | 1 block(s) placed | object CARRIED |
| S3_height_change | direct | yes | 11.0 s | 1.50 m | 0.048 m *(below floor)* | 0.8 mm | 1 block(s) placed | object CARRIED |
| S3_height_change | shared | yes | 11.0 s | 1.44 m | 0.053 m *(below floor)* | 0.7 mm | 1 block(s) placed | object CARRIED |
| S4_tight_tolerance | assisted | yes | 10.0 s | 1.46 m | 0.050 m *(below floor)* | 0.7 mm | 1 block(s) placed | object CARRIED |
| S4_tight_tolerance | direct | yes | 10.0 s | 1.40 m | 0.048 m *(below floor)* | 0.8 mm | 1 block(s) placed | object CARRIED |
| S4_tight_tolerance | shared | yes | 10.1 s | 1.42 m | 0.051 m *(below floor)* | 0.7 mm | 1 block(s) placed | object CARRIED |

---

## T3 — 12 runs

**What it should show.** Both grippers rise together with the TAN RIGID TRAY between them and a yellow ball on top. The tray stays straight and level -- |tilt| in the overlay should stay near 0 deg. The ball rolls to the low side and would fall off past 11.3 deg.

| scenario | cond | rviz.mp4 | duration | EE travel | min clearance | tracking | object outcome | attachment |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1_short_straight | assisted | yes | 4.7 s | 1.30 m | 0.052 m *(below floor)* | 2.1 mm | ball retained | object CARRIED |
| S1_short_straight | direct | yes | 4.7 s | 1.31 m | 0.052 m *(below floor)* | 2.4 mm | ball retained | object CARRIED |
| S1_short_straight | shared | yes | 4.6 s | 1.28 m | 0.052 m *(below floor)* | 2.2 mm | ball retained | object CARRIED |
| S2_long_height | assisted | yes | 8.3 s | 1.55 m | 0.051 m *(below floor)* | 2.1 mm | ball retained | object CARRIED |
| S2_long_height | direct | yes | 8.4 s | 1.57 m | 0.051 m *(below floor)* | 2.0 mm | ball retained | object CARRIED |
| S2_long_height | shared | yes | 8.3 s | 1.55 m | 0.051 m *(below floor)* | 2.1 mm | ball retained | object CARRIED |
| S3_curved_obstacle | assisted | yes | 7.1 s | 1.47 m | 0.052 m *(below floor)* | 2.1 mm | ball retained | object CARRIED |
| S3_curved_obstacle | direct | yes | 7.1 s | 1.52 m | 0.051 m *(below floor)* | 1.8 mm | ball retained | object CARRIED |
| S3_curved_obstacle | shared | yes | 7.0 s | 1.44 m | 0.056 m *(below floor)* | 2.0 mm | ball retained | object CARRIED |
| S4_tight | assisted | yes | 6.2 s | 1.34 m | 0.058 m *(below floor)* | 1.8 mm | ball retained | object CARRIED |
| S4_tight | direct | yes | 6.1 s | 1.39 m | 0.058 m *(below floor)* | 1.5 mm | ball retained | object CARRIED |
| S4_tight | shared | yes | 6.1 s | 1.34 m | 0.059 m *(below floor)* | 1.5 mm | ball retained | object CARRIED |

---

## T5 — 9 runs

**What it should show.** The RIGHT arm (screen right) picks the tool off its cradle, carries it inboard, and stops with it at the GREEN SPHERE -- the receive point at the wearer's side. The left arm never moves.

| scenario | cond | rviz.mp4 | duration | EE travel | min clearance | tracking | object outcome | attachment |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1_near | assisted | yes | 8.1 s | 0.87 m | 0.068 m *(below floor)* | 2.3 mm | tool delivered | object CARRIED |
| S1_near | direct | yes | 8.1 s | 0.91 m | 0.068 m *(below floor)* | 2.5 mm | tool delivered | object CARRIED |
| S1_near | shared | yes | 8.1 s | 0.82 m | 0.068 m *(below floor)* | 2.0 mm | tool delivered | object CARRIED |
| S2_far | assisted | yes | 9.0 s | 0.84 m | 0.095 m *(below floor)* | 2.3 mm | tool delivered | object CARRIED |
| S2_far | direct | yes | 9.0 s | 0.88 m | 0.095 m *(below floor)* | 2.5 mm | tool delivered | object CARRIED |
| S2_far | shared | yes | 9.0 s | 0.79 m | 0.095 m *(below floor)* | 2.4 mm | tool delivered | object CARRIED |
| S3_busy | assisted | yes | 8.4 s | 0.89 m | 0.068 m *(below floor)* | 2.1 mm | tool delivered | object CARRIED |
| S3_busy | direct | yes | 8.4 s | 0.92 m | 0.068 m *(below floor)* | 2.7 mm | tool delivered | object CARRIED |
| S3_busy | shared | yes | 8.4 s | 0.85 m | 0.068 m *(below floor)* | 2.3 mm | tool delivered | object CARRIED |

---

## T6 — 12 runs

**What it should show.** Same paths as T3, but the object is a SLING: an orange catenary between the grippers with the ball sitting in the bottom of the V. The overlay shows live separation and sag; the ball turns RED and drops if sag falls below one ball diameter (40 mm).

| scenario | cond | rviz.mp4 | duration | EE travel | min clearance | tracking | object outcome | attachment |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1_short_straight | assisted | yes | 4.7 s | 1.30 m | 0.052 m *(below floor)* | 1.9 mm | ball retained | object CARRIED |
| S1_short_straight | direct | yes | 4.7 s | 1.33 m | 0.052 m *(below floor)* | 1.9 mm | ball retained | object CARRIED |
| S1_short_straight | shared | yes | 4.7 s | 1.29 m | 0.052 m *(below floor)* | 1.8 mm | ball retained | object CARRIED |
| S2_long_height | assisted | yes | 8.3 s | 1.55 m | 0.051 m *(below floor)* | 2.3 mm | ball retained | object CARRIED |
| S2_long_height | direct | yes | 8.3 s | 1.61 m | 0.051 m *(below floor)* | 1.7 mm | ball retained | object CARRIED |
| S2_long_height | shared | yes | 8.3 s | 1.53 m | 0.051 m *(below floor)* | 2.0 mm | ball retained | object CARRIED |
| S3_curved_obstacle | assisted | yes | 7.1 s | 1.50 m | 0.064 m *(below floor)* | 2.2 mm | ball retained | object CARRIED |
| S3_curved_obstacle | direct | yes | 7.1 s | 1.53 m | 0.051 m *(below floor)* | 2.2 mm | ball retained | object CARRIED |
| S3_curved_obstacle | shared | yes | 7.1 s | 1.45 m | 0.052 m *(below floor)* | 2.0 mm | ball retained | object CARRIED |
| S4_tight | assisted | yes | 6.1 s | 1.37 m | 0.058 m *(below floor)* | 1.6 mm | ball retained | object CARRIED |
| S4_tight | direct | yes | 6.2 s | 1.34 m | 0.058 m *(below floor)* | 1.9 mm | ball retained | object CARRIED |
| S4_tight | shared | yes | 6.2 s | 1.30 m | 0.058 m *(below floor)* | 1.6 mm | ball retained | object CARRIED |

---

## T7 — 18 runs

**What it should show.** Each arm chases its own GREEN TARGET SPHERE around a small closed loop -- a Lissajous on an 80 mm sphere. The arms are INDEPENDENT. In the B1/B2 baselines only ONE arm moves.

| scenario | cond | rviz.mp4 | duration | EE travel | min clearance | tracking | object outcome | attachment |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| B1_left_only | assisted | yes | 10.5 s | 1.71 m | 0.182 m | 4.1 mm | - | N/A (no carried object) |
| B1_left_only | direct | yes | 10.5 s | 1.76 m | 0.182 m | 4.7 mm | - | N/A (no carried object) |
| B1_left_only | shared | yes | 10.4 s | 1.58 m | 0.187 m | 3.9 mm | - | N/A (no carried object) |
| B2_right_only | assisted | yes | 10.4 s | 1.76 m | 0.161 m | 4.3 mm | - | N/A (no carried object) |
| B2_right_only | direct | yes | 10.4 s | 1.80 m | 0.172 m | 4.5 mm | - | N/A (no carried object) |
| B2_right_only | shared | yes | 10.4 s | 1.63 m | 0.182 m | 4.0 mm | - | N/A (no carried object) |
| S1_both_slow | assisted | yes | 8.0 s | 1.41 m | 0.178 m | 2.0 mm | - | N/A (no carried object) |
| S1_both_slow | direct | yes | 8.0 s | 1.43 m | 0.178 m | 2.1 mm | - | N/A (no carried object) |
| S1_both_slow | shared | yes | 8.0 s | 1.37 m | 0.180 m | 1.9 mm | - | N/A (no carried object) |
| S2_one_fast | assisted | yes | 10.7 s | 2.45 m | 0.182 m | 2.6 mm | - | N/A (no carried object) |
| S2_one_fast | direct | yes | 10.7 s | 2.51 m | 0.181 m | 2.4 mm | - | N/A (no carried object) |
| S2_one_fast | shared | yes | 10.7 s | 2.30 m | 0.187 m | 2.5 mm | - | N/A (no carried object) |
| S3_both_fast | assisted | yes | 10.7 s | 3.47 m | 0.175 m | 4.0 mm | - | N/A (no carried object) |
| S3_both_fast | direct | yes | 10.7 s | 3.58 m | 0.160 m | 4.1 mm | - | N/A (no carried object) |
| S3_both_fast | shared | yes | 10.7 s | 3.21 m | 0.181 m | 3.7 mm | - | N/A (no carried object) |
| S4_asymmetric | assisted | yes | 11.0 s | 3.03 m | 0.184 m | 2.7 mm | - | N/A (no carried object) |
| S4_asymmetric | direct | yes | 11.0 s | 3.23 m | 0.180 m | 3.0 mm | - | N/A (no carried object) |
| S4_asymmetric | shared | yes | 11.0 s | 2.72 m | 0.186 m | 2.4 mm | - | N/A (no carried object) |

---

## T8 — 12 runs

**What it should show.** The arm reaches out to a GREEN TARGET far on its own side, labelled with the stance the wearer had to adopt. The clip shows the reach AFTER the wearer has repositioned. The other arm is parked at home.

| scenario | cond | rviz.mp4 | duration | EE travel | min clearance | tracking | object outcome | attachment |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1_left_lean_forward | assisted | yes | 8.1 s | 0.70 m | 0.158 m | 1.7 mm | - | N/A (no carried object) |
| S1_left_lean_forward | direct | yes | 8.0 s | 0.70 m | 0.158 m | 1.8 mm | - | N/A (no carried object) |
| S1_left_lean_forward | shared | yes | 7.9 s | 0.69 m | 0.158 m | 1.7 mm | - | N/A (no carried object) |
| S2_left_step_forward | assisted | yes | 8.8 s | 0.76 m | 0.158 m | 1.8 mm | - | N/A (no carried object) |
| S2_left_step_forward | direct | yes | 8.8 s | 0.76 m | 0.158 m | 1.6 mm | - | N/A (no carried object) |
| S2_left_step_forward | shared | yes | 8.8 s | 0.75 m | 0.158 m | 1.8 mm | - | N/A (no carried object) |
| S3_right_crouch | assisted | yes | 7.3 s | 0.72 m | 0.170 m | 1.9 mm | - | N/A (no carried object) |
| S3_right_crouch | direct | yes | 7.3 s | 0.73 m | 0.170 m | 1.9 mm | - | N/A (no carried object) |
| S3_right_crouch | shared | yes | 7.3 s | 0.71 m | 0.170 m | 1.7 mm | - | N/A (no carried object) |
| S4_right_step_forward | assisted | yes | 8.2 s | 0.77 m | 0.170 m | 1.7 mm | - | N/A (no carried object) |
| S4_right_step_forward | direct | yes | 8.2 s | 0.79 m | 0.170 m | 1.9 mm | - | N/A (no carried object) |
| S4_right_step_forward | shared | yes | 8.2 s | 0.77 m | 0.170 m | 1.9 mm | - | N/A (no carried object) |

---

## T9 — 12 runs

**What it should show.** The arm traces a circle whose radius is the sway amplitude -- it is HOLDING a world-fixed point while the base moves underneath, so in the arm's own frame the target orbits. The radius should visibly grow across S1->S4 (0, 20, 60, 100 mm).

| scenario | cond | rviz.mp4 | duration | EE travel | min clearance | tracking | object outcome | attachment |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1_sway_000mm | assisted | yes | 3.5 s | 0.78 m | 0.204 m | 0.0 mm | - | N/A (no carried object) |
| S1_sway_000mm | direct | yes | 3.5 s | 0.78 m | 0.202 m | 0.0 mm | - | N/A (no carried object) |
| S1_sway_000mm | shared | yes | 3.5 s | 0.78 m | 0.202 m | 0.0 mm | - | N/A (no carried object) |
| S2_sway_020mm | assisted | yes | 8.3 s | 1.01 m | 0.183 m | 1.2 mm | - | N/A (no carried object) |
| S2_sway_020mm | direct | yes | 8.3 s | 1.02 m | 0.183 m | 1.2 mm | - | N/A (no carried object) |
| S2_sway_020mm | shared | yes | 8.3 s | 0.97 m | 0.183 m | 1.0 mm | - | N/A (no carried object) |
| S3_sway_060mm | assisted | yes | 8.3 s | 1.51 m | 0.145 m | 1.9 mm | - | N/A (no carried object) |
| S3_sway_060mm | direct | yes | 8.3 s | 1.52 m | 0.145 m | 2.0 mm | - | N/A (no carried object) |
| S3_sway_060mm | shared | yes | 8.3 s | 1.46 m | 0.145 m | 1.8 mm | - | N/A (no carried object) |
| S4_sway_100mm | assisted | yes | 10.7 s | 2.01 m | 0.117 m *(below floor)* | 2.1 mm | - | N/A (no carried object) |
| S4_sway_100mm | direct | yes | 10.7 s | 2.03 m | 0.117 m *(below floor)* | 2.3 mm | - | N/A (no carried object) |
| S4_sway_100mm | shared | yes | 10.7 s | 1.97 m | 0.117 m *(below floor)* | 2.3 mm | - | N/A (no carried object) |

---

## Regenerate

```bash
ros2 launch srl_moveit_config demo.launch.py     # sim, arms AT HOME
python3 scripts/record_verification.py --all     # TF clips, plots, bags
python3 scripts/record_rviz.py --all             # RViz screen captures
python3 scripts/make_clip_index.py               # this file
```

`record_rviz.py` starts its own Xvfb on :99 and its own RViz. It has to:
**x11grab on WSLg's :0 records BLACK** -- a full-screen grab with RViz plainly
visible measures mean pixel value 0.0, because XWayland window pixels are
composited by Wayland and never reach the X root window that x11grab reads.
Xvfb has no compositor, and the same grab there gives mean 126.8.

