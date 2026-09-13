# THE SRDF CLEARANCE GAP — which numbers it touched, and which stand

Written 2026-08-15. **Read this before citing any workspace, layout or
clearance figure produced by this project.** Nothing here is a new
measurement; it is the ledger that says which of the old ones survived the
2026-08-15 wearer-clearance measurement and which did not.

If you only read one paragraph: **every workspace figure in this project up to
2026-08-15 was computed against a collision model that could not see the
wearer where a shoulder-mounted arm actually threatens them.** The figures are
not fabricated and most are not even wrong — they are IK reachability, and
they were quoted as usable space. Those are different quantities and in four
of the six directions the smaller one is the clearance one.

---

## 1. WHAT THE GAP WAS

`src/srl_moveit_config/config/srl_dual.srdf` permanently excludes

    torso, harness, backpack           vs  {left,right}_base_link
    human_left_upper_arm                   {left,right}_shoulder_link
    human_right_upper_arm                  {left,right}_half_arm_1_link

— 44 pairs. Every workspace and layout number this project has ever quoted
came from `/compute_ik` with `avoid_collisions`, so MoveIt returned `valid`
for poses with the arm's own tube inside the person, and nothing else measured
the distance.

**The exclusions are not the defect and have not been removed.** They are
permanent adjacencies of a shoulder mount: `base_link` is moved by no joint,
so the interference is set by the mount POSITION, and left enabled it made
every state invalid — `/compute_ik` returned NO_IK_SOLUTION for 48 of 48
sampled poses on both arms. The SRDF records that, with the mechanical action
it implies, at line 406.

The defect was that `avoid_collisions` was treated as the wearer check and it
never was one. HARD CONSTRAINT 11 already said the words — *an SRDF exclusion
silences an alarm; it does not move the metal* — and the exclusion had
silenced this one for the whole life of the project.

**What is left enabled is everything distal of `half_arm_1`, and that is real
protection.** This ledger is about the proximal links, which are the ones that
sweep across the wearer's chest.

## 2. WHAT CLOSED IT

Not an SRDF edit. A separate, geometric measurement against the mount guard's
own capsule model, which does not consult the planning scene at all:

| | |
| --- | --- |
| `scripts/measure_clearance_region.py` | every surveyed cell, whole pick path, worst arm-to-wearer distance per waypoint |
| `scripts/measure_what_binds.py` | the four-rung ladder that names what stops the arm per direction, with the wearer split out from self-collision |
| `scripts/verify_t1_paths.py` | T1's and t1s2's own waypoints, N=10, counting waypoints below the floor |
| `scripts/find_presentation_pose.py` | already did this, for one pose, and said so — it is where the gap was first visible |

The floor is **0.150 m** (`CLEAR_FLOOR` in `measure_what_binds.py`). Each
script carries its own controls and prints nothing if one fails; a pose inside
the torso must read negative, home must clear, an outboard cell must clear
wide.

`recordings/baselines/work_surface_region.json` now carries **`clear_cells`**
beside `cells`. Three consumers read it and **refuse a region file that has
none** — `scripts/clip_scene.py`, `src/srl_experiments/experiments/abc/
msc_clip_tasks.py` and `src/srl_autonomy/srl_autonomy/named_places.py`. The
last of those was missed on 2026-08-15 and fixed on the same day; see §5.

---

## 3. THE LEDGER — AFFECTED

**Do not cite any of these as usable workspace.** Where a figure is still
correct as an IK result, that is said; the point is that it was quoted as
something it is not.

| figure | where it came from | what is wrong with it | cite instead |
| --- | --- | --- | --- |
| the surveyed region: **left 181 cells, x 0.25–0.70; right 228 cells** | `work_surface_region_PRE20260815.json`, 2026-08-13, 14751 IK calls | IK only. Understated the region by 300 mm per side (the survey box stopped at 0.70) while overstating it by 175 mm inboard, where it is not safe | `work_surface_region.json` → `clear_cells`: **left 193, x 0.425–1.000; right 246, x −1.000…−0.400** |
| the **workspace marking** drawn in every clip before 2026-08-15 | the file above | it is the region a participant is told to work in, and it contained **72 left / 68 right cells inside the floor, worst −2.7 mm** | the marking now drawn from `clear_cells` |
| **T1 on the right arm**: left 2/4 cubes, right 4/4 | `t1_layout_options.json`, 2026-08-12 | wrong twice. `survey_work_surface.py` applied the LEFT arm's wrist-to-pad offset to both arms, so every right-arm cell was tested 48.3 mm from where the right hand closes; and the layout it chose sat inside the floor | T1 stage 1 is the **LEFT** arm. `docs/TASK_SPEC.md` §2 |
| **T1 right-arm layout: N=10, 0 IK failures** | `verify_msc_tasks.py`, pre-2026-08-15 | true as stated and useless as reassurance: **70 of its 143 waypoints were inside the floor**, worst 58.7 mm, at zero IK failures | the current left-arm layout: 150 waypoints, **0 IK failures, 0 below the floor** |
| **stage 2 verified clean** | same | **21 of 71 waypoints** inside the floor on seed 0, left arm | current stage 2: 5 seeds, N=10, 0 below the floor |
| the **idle arm's park pose** | never measured | sat at **30.6 mm** for whole clips, in shot, while the working arm was being checked | the presentation pose, 0.1610 m |
| the stage-2 **sampling pool** | `randomisation_region.json` | IK only; it could draw a cube position inside the floor | `clear_cells` |
| **T0's band** | `task0_band_survey.json` | IK only | nothing yet — see §6 |
| envelope figures: reach per direction | `workspace_20260806.json`, `workspace_n10_20260806.json`, `workspace_n10_retry_20260806.json`, `workspace_audit.json` | correct as **IK reach**. Never quote them as usable or safe space | `what_binds.json` for the safe boundary |
| **innermost workable column: x = 0.250 (left) / 0.075 (right)** | `centre_reach.json`, and it says so in its own `method`: *grasp pose only — an optimistic bound* | IK only, and grasp-pose only, so optimistic twice over | the clearance-safe row of the same table: **x = 0.425 / 0.400** |
| **forward reach y max 0.425 / 0.450**, and the whole work-plane-height table | `band_vs_work_height.json` | IK boundary. At those cells the arm is already at **0.132 m (L) / 0.116 m (R)** against a 0.150 m floor — the clearance floor is reached about 25 mm before IK stops solving | `what_binds.json`, and read its last column with its third |
| the **mount tilt price, ~0.15 m forward** | `forward_reach.json`, option A | IK only, and option A is implemented as the inverse transform on the TARGET — exact for kinematics, wrong for wearer collision, because the wearer does not tilt with the mount | it is an upper bound, not an offer. `docs/TASK_SPEC.md` §2A |

## 4. THE LEDGER — STANDS

| figure | why it survives |
| --- | --- |
| the **presentation pose**: tool axis +6.5° (L) / +7.5° (R), clearance **0.1610 m** | measured geometrically from the day it was built. `find_presentation_pose.py` explicitly does not treat `/check_state_validity` as the authority, for this reason |
| `clearance_region*.json`, `what_binds.json`, `work_surface_region.json` (2026-08-15) | these ARE the fix |
| the current **T1 stage 1 and stage 2** layouts | re-derived and verified N=10 over the full path: 0 IK failures, **0 waypoints below the floor** |
| **the front centre is not reachable, and the shortfall is 450 mm** | 0 cells at \|x\| ≤ 0.10 in the IK set AND in the clear set, so the conclusion does not depend on the gap. The 450 mm is measured from the clearance-safe column (0.425) plus the project's 20 mm margin, not from the IK column |
| **the two arms' reachable sets are disjoint** | 0 cells in both, both ways |
| **raising or lowering the table buys nothing**, band 0.00–0.05 at every height 0.90–1.30 | the mechanism is the table top, not the wearer. `band_vs_table_height.json` |
| **T1-1 is BLOCKED** — the highest slab that costs nothing is where the table already is | a furniture question. `objects_on_table.json`, and its two controls |
| **the pinned wrist binds forward and outboard** | an orientation result from the ladder, whose first rung is the collision-aware one; the wearer rung is separate and named separately |
| everything not about the workspace | the accuracy table, grasp reality and penetration, detection and depth, the capability ladder, channel health, capture and render rate, mode paths, the GUI audit. None of them ask where the arm may go |

## 5. THE CONSUMER THAT WAS MISSED, AND WHAT IT MEANS

`clip_scene.py` and `msc_clip_tasks.py` were switched to `clear_cells` on
2026-08-15. `srl_autonomy/named_places.py` was not, and it is the resolver
that turns a spoken **"move to the left side"** into a pose for
`autonomy_executive` — mode 06's whole input path.

It was reading `cells`. So the fix was complete for the clips and incomplete
for the one mode where a person says a place name and the system chooses the
coordinate. Measured: the IK-set centroid it would have returned, (0.625,
0.175), happens to clear the floor — **it was safe by luck of where the
centroid fell**, and a re-survey that moved the centroid inboard would have
returned an unsafe pose with nothing disagreeing. The `n_cells` it reported
also overstated the usable region by exactly the 72 and 68 unsafe cells.

Fixed the same day: it reads `clear_cells`, and it **refuses** a region file
that has none rather than falling back — demonstrated against the 2026-08-13
survey still on disk. Resolved poses moved outboard by 100 mm (left) and
75 mm (right); reported counts dropped 265 → 193 and 314 → 246.

The general lesson is the one this project keeps re-learning: a fix applied to
the files you were looking at is not a fix applied to the consumers. Three
consumers read that survey and only two were changed.

## 6. WHAT IS STILL NOT MEASURED

Stated here rather than left as an absence that reads like a clean bill.

* **T0, T2 and T3 waypoints have never been checked against the wearer
  geometrically.** `verify_t1_paths.py` is the only task-path clearance check
  and it covers T1 and t1s2 only. T0's spheres in particular sit at chest
  height in front of the person by design, and `task0_band_survey.json` is an
  IK survey.
* **The dance routines** (D1–D3) likewise. `verify_dance_paths.py` uses
  `avoid_collisions`.
* **Clearance is sampled, not bounded.** `/compute_ik` returns one of many
  solutions for a 7-DOF arm and TRAC-IK restarts randomly, so a cell's
  clearance is the clearance of the solution the solver happened to return.
  It is the right quantity — it is the pose the follower would be commanded to
  — but it is a sample. `measure_clearance_region.py` says so at the top.
* **The wearer is a mannequin of rigid cylinders**, not a person. The capsule
  model is the mount guard's, and a real chest is not a capsule.
* **None of this has run against a real arm.** Every number in this ledger,
  on both sides of it, is simulation.

---

## SEE ALSO

`docs/system/findings.md` — the measurement session itself, the four
instrument defects found on the way, and the per-direction table.
`docs/TASK_SPEC.md` §2A — the workspace as it now stands.
`docs/ENGINEERING_LOG.md` HARD CONSTRAINT 11 — why the floor is not lowered.
