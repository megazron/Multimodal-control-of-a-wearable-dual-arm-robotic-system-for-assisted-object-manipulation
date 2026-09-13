# Measure the environment, then plan against what was measured

Written 2026-08-23. The operator's words: *"there is so much space to
calibrate, why can't you create an algorithm to properly figure out the
environment"* — and, about teleoperation, *"due to the limitations at the
moment there is no proper planning of the robot"*.

## THE PROBLEM, STATED PLAINLY

Almost everything in this repository that touches an object was **told where
it is**. `run_abc` builds T1's path from `T1_CUBES`, four coordinates in a
source file; the table is at 1.250 because `work_surface` says so. All of it
is verified N=10 over densified paths, and **none of it survives a different
table, a different object, or the same object moved 50 mm**. The number in the
file becomes a lie the arm acts on confidently.

Three pieces that could have sensed instead existed and were never composed:

* `table_scene.analyse` finds a plane and the things on it — from ONE view,
  and was used to *print a description*;
* `joint_planner.VoxelWorld` lets the planner avoid things that are not the
  wearer — and was **constructed nowhere outside its own self-test**, so no
  path this project has ever planned knew the table was there;
* `work_surface.set_measured()` exists so the surface height can come from
  depth, and docs/ENGINEERING_LOG.md records that nothing calls it.

## IS THIS A ONE-TIME THING?

Two things are calibrated here and they have different lifetimes. Bundling
them into one command hid that, so it is worth stating plainly.

| | what it is a fact about | when to redo it |
| --- | --- | --- |
| `reachable_cells.json` | the **ROBOT**: which camera poses the arm can reach | **once**, and it re-probes itself |
| `world_map.json` | the **SCENE**: the surface and what is standing on it | **whenever the scene changes** |

**Reachability is one time.** It is keyed on every bound, layer, facing,
elevation and step, so moving the mount, changing home, editing the URDF or
altering any sweep setting invalidates it automatically. `--reprobe` forces a
fresh probe. Nothing about the objects on the table affects it.

**The map is not.** Somebody moves a cube and it is wrong immediately. Today
that means the full sweep again — 13 minutes with the cache warm — because
there is **no incremental re-map**: `build()` fuses every view from scratch
every time. That is fine for "new cell, new table" and much too slow for "I
just put a different object down".

**The surface sits in between.** If the table has not moved, the plane is
stable across object changes, so a cheap re-look at one region would be enough
to update the objects. That is the obvious next piece and it is not built.

Until it is, `pick_from_map` reports the map's age on every run and **REFUSES
TO MOVE off a map older than 30 minutes** (`--accept-stale-map` overrides).
Planning off an old map is harmless and only reported; driving the arm at
coordinates measured in a previous session is the failure that costs
something, and a stale file looks exactly as authoritative as a fresh one.

## WHAT RUNS NOW

```
bash scripts/start_gui.sh          # RUN tab, top panel: MAP THE ENVIRONMENT
./.venv_vision/bin/python scripts/calibrate_environment.py --arm left
./.venv_vision/bin/python scripts/pick_from_map.py --arm left --object 1 \
      --to 0.29 0.50 --execute
./.venv_vision/bin/python scripts/record_calibration.py --arm left
```

| stage | file | what it does |
| --- | --- | --- |
| the sweep | `srl_perception/calibration_sweep.py` | straight serpentine rows, every step one cell in one axis. 1.140 m of travel against 1.702 m and no long returns for the obvious alternative, which is measured and not asserted |
| the eyes | `srl_perception/segment_lift.py` | FastSAM cuts every region out of each frame; each mask is lifted through THAT frame's depth; masks are split by height layer and nested masks suppressed |
| the map | `srl_perception/world_model.py` | many views → one map. Surface MEASURED, objects fused by overlap, disconnected lumps split apart, fragments rejoined |
| the seam | `joint_planner.world_from_map()` | the map becomes a `VoxelWorld`, so the planner knows the table exists |
| the pick | `scripts/pick_from_map.py` | pregrasp → grasp → lift → carry → place → retreat, every coordinate from the map |
| the voice | `srl_experiments/narration.py` | 15 phases, one vocabulary, published on `/robot_say` and shown in the window |

**It is not task-specific and must not become so.** The stage imports no task
module and its self-test checks that by AST, not by grepping — the first
version grepped its own source and found the words it was searching for.

## THE WRIST HOLDS STILL, AND THE SWEEP IS A VOLUME (2026-08-23, later)

The operator's words: *"the gripper and the whole joints move everywhere
without the gripper being stable"*, and *"imagine how a 3D printer works. it
has only 1 plane to calibrate but our robot is in 3 space, it should map all
the sides with all the lengths"*.

Both were right, and the first was the important one.

**The wrist was re-aimed at every cell.** `look_at_quat` pointed the tool axis
AT each grid point, so the orientation changed between neighbours 90 mm apart:
the arm swung continuously, and no two views were taken from the same
attitude — which means a residual wrist error was a *different* error in every
frame and could never cancel or be measured.

A printer's probe does not re-aim. It holds one attitude and **translates**.
So does this: one wrist direction per PASS, and the camera grid is the work
grid rigidly translated back along that fixed ray, so consecutive cells differ
by a pure translation. Several passes at different yaws give the sides.

**And it is not a plane.** A printer probes a bed, where the only unknown is
z. This arm works in a volume where objects have sides, and a side is
invisible from a single elevation. The sweep is 2 layers × 3 yaws × 24 cells
× 2 arms = **144 cells, 13.4 m of travel**.

**Stationary is not the same as arrived.** `at()` asks whether the joints are
near the commanded pose; a joint can sit 0.02 rad away and still be moving
through it, and a frame captured then is smeared across two poses while every
statistic downstream reports fine. `is_still` watches for 400 ms and requires
no joint to move more than **0.0015 rad** — a tenth of the arrival tolerance.
Capture is refused otherwise. **0 refusals** on the full run.

**Both arms, one map.** The two arms cannot cross the centreline, so a one-arm
map is missing exactly the half its own arm can never reach — and that half is
not empty, it is *unmeasured*. All six objects are now seen by **both** arms.

### Coverage, honestly split

| | |
| --- | --- |
| cells planned | 144 |
| reachable at their pass's attitude | **70** |
| outside the arm's envelope | 74 |
| photographed | **68 of 70 — 97%** |

The 74 are a fact about the ROBOT, not a scan failure, and they used to be
discovered by *driving there*: a seeded IK search with six restarts, then a
six-second wait for an arrival that never came, 74 times. That is most of the
twenty minutes the first full run took, and the report then read "48%
coverage" as though the scan had failed. IK is solved before anything moves
now and the two numbers are separate.

### Both arms, against the renderer's own geometry

| truth | measured | width |
| --- | --- | --- |
| surface 1.2500 | **1.2476** (−2.4 mm, tilt 0.23°) | |
| cube_0 (0.420, 0.450) | **(0.420, 0.450)** | 41 mm / 40 |
| cube_1 (0.480, 0.450) | **(0.480, 0.450)** | 41 mm / 40 |
| cube_2 (−0.420, 0.450) | **(−0.420, 0.450)** | 41 mm / 40 |
| cube_3 (−0.480, 0.450) | **(−0.480, 0.449)** | 41 mm / 40 |

All four exact in x and y. The residual is z, where only the top and near
faces are seen, and the pads, whose masks catch some table.

### An object needs evidence

The first two-arm run returned **14 objects where 6 exist**: the six carrying
110 000–500 000 points from 46–66 viewpoints, the other eight carrying 25–77
points from one or two. Four orders of magnitude apart, and `pick_from_map`
would have planned a grasp on either. Under 200 points or fewer than 2 views
is filed as `weak` — kept and named in the map, never deleted, because a scene
where *everything* is weak is worth seeing.

## MEASURED (first working version, one arm)

Scored against the geometry `mock_rgbd_camera` was itself given, left arm,
objects on its own side of the centreline:

| | mean | worst | surface |
| --- | --- | --- | --- |
| first working run (clustering) | 100.7 mm | 828 mm | +8.0 mm |
| segmentation, all fixes | **12.1 mm** | **15.7 mm** | **+7.9 mm** |

Both cubes come back at their true (x, y) to within a millimetre, 39 and
41 mm wide against 40 mm; the pad reads 139 mm against 140. The worst-case
figure is dominated by z, where only the top and near faces are seen.

The pick planned from that map solved all six waypoints and ran end to end,
with **561 occupied voxels handed to the planner** — the first time
`world_from_map` has ever had a caller.

## THE SURFACE WAS NEVER 5 mm LOW. I WAS ASKING ABOUT THE WRONG PLACE.

The map reported the support surface at 1.2448 m against a table top of
1.2500 — **5.2 mm low** — and 78% of all points below the true top. I wrote a
plane-refinement to correct that bias before checking whether it was one.

It was not. `Map.surface_z` returned `offset / n_z`: the plane's **z-intercept
at x = y = 0**. A fitted support plane comes out slightly tilted, so its
intercept is not its height anywhere a table exists — and on this rig the
origin is *inside the wearer*, half a metre from the nearest tabletop.

Measured on constructed geometry that reproduces the live case (a top at
0.9000 with its 35 mm front edge face in view, which is what the sweep sees at
38.9°):

| tolerance | tilt | z at origin | z at the table | top median height |
| --- | --- | --- | --- | --- |
| 0.008 | 0.66° | 0.8934 | **0.8994** | +0.0004 |
| 0.004 | 0.24° | 0.8977 | **0.8998** | +0.0002 |

**The plane was right to under a millimetre the whole time.** `surface_z` now
evaluates it at the centroid of the points lying on it, and `plane_z_at(x, y)`
exists because a plane is not a number. Live: **1.2502 m against 1.2500 —
+0.2 mm.**

The refinement I had written was deleted. A fix for a bias that does not exist
later reads as evidence the bias existed.

### The tilt, which is a different thing and did matter

A support plane tipped to catch the slab's edge face sits *below* the tabletop
over part of the table. The height gate is 4 mm, so the table clears its own
surface there — and the map duly reported two slabs of tabletop, 231 × 392 mm
and 79 × 314 mm, as objects. The plane tolerance is 4 mm now (0.66° → 0.24°),
and those are gone.

### The map, against the renderer's own geometry

| measured | truth |
| --- | --- |
| surface 1.2502, tilt 0.09° | 1.2500 |
| pad (0.290, 0.500) 141 × 162 × 9 | (0.290, 0.500) 140 × 160 × 10 |
| pad (−0.290, 0.500) 141 × 163 × 10 | (−0.290, 0.500) 140 × 160 × 10 |
| cube (0.420, 0.450) 41 × 42 × 39 | (0.420, 0.450) 40³ |
| cube (0.480, 0.449) 41 × 42 × 39 | (0.480, 0.450) 40³ |
| cube (−0.480, 0.450) 41 × 42 × 39 | (−0.480, 0.450) 40³ |
| cube (−0.420, 0.450) 41 × 42 × 40 | (−0.420, 0.450) 40³ |

Six objects, and the scene has six. Every one exact in x and y to a
millimetre, every dimension within 3 mm.

## THE SWEEP REMEMBERS WHAT IT CAN REACH

The sweep already solved IK at every cell and threw the answer away. It writes
it now — `recordings/baselines/reachable_cells.json`, keyed on every bound,
layer, facing, elevation and step, because reachability is a fact about the
arm AT A GEOMETRY and a cache that did not key on those would silently skip
cells that became reachable when the volume moved.

Second run: **101 of 210 cells dropped as outside the envelope before
anything moved**, 13m15s against 20+, and the same map. A run planned *from*
the cache does not re-save it — it has not tested the cells the cache
excluded, and saving its answer would shrink the set a little further every
time. `--reprobe` forces a full probe.

## THE SWEPT VOLUME IS A MEASUREMENT TOO (2026-08-23, later still)

The box swept up to this point — x 0.30–0.58, y 0.30–0.52 — was **written
down**, and 74 of the 144 cells planned inside it came back "outside the
arm's envelope". The obvious reading is that the arms are small. The real
reading is that a guessed box sat badly inside a much larger one.

`scripts/measure_reachable_volume.py` solves IK at the shipped attitude over a
deliberately over-wide grid — 924 poses per arm, nothing commanded:

| | left | right |
| --- | --- | --- |
| reachable at SOME facing+layer | x 0.10–0.75, y 0.15–0.65, **146 of 154** | x −0.75–−0.10, y 0.15–0.65, **146 of 154** |
| reachable at EVERY facing+layer | 4 cells | 4 cells |

So each arm can observe **almost the whole probed area** from *some*
attitude, and almost none of it from *all* of them. The sweep is bound to the
first, and the per-cell evidence at the edges is thinner — which is why the
map carries `seen_by` per object and an evidence rule.

**THE INBOARD BOUND IS NOT TAKEN FROM THIS MEASUREMENT.** IK calls x = 0.10
reachable, and IK is not the wearer check — hard constraint 11: the SRDF
excludes the 44 proximal pairs a shoulder mount actually threatens, so a pose
MoveIt calls valid can have the tube inside the person. The inboard bounds are
the columns this project has measured geometrically, **0.325 left and 0.450
right**, and the asymmetry is real. Widened outboard and forward, where the
wearer is not; left alone inboard, where they are.

Swept volume now: **x 0.325–0.750 (left), −0.750–−0.450 (right), y
0.15–0.65**, two layers, three facings. On the recorded run: 324 cells
planned, **117 reachable, 115 photographed — 98%**, 0 stillness refusals.

## THE VIEWING ANGLE IS A MEASUREMENT, NOT A GUESS

`scripts/measure_view_geometry.py`, on the live stack, walking every cell:

| elevation | cells reached | object pixels |
| --- | --- | --- |
| 25.3° | 9 / 12 | 6.9 % |
| 39.0° | 8 / 12 | 8.2 % |
| 47.9° | 7 / 12 | 9.3 % |
| 56.1° | 6 / 12 | 9.4 % |
| 64.4° | 3 / 12 | 11.8 % |
| 72.1° | **0 / 12** | — |
| 78.5° | **0 / 12** | — |

Steeper sees more object and less room, and the arm reaches less of it. Above
72° nothing solves — the same wall `search_centre_on_surface` hit from the
other side (*top-down on a surface: 0 of 840 cells*). **39° is the default**,
because coverage wins: a cell the arm cannot reach contributes no view at all.

The first version stood the camera 400 mm out and 200 mm up — **25°**, at
which the table is a thin wedge and 72% of the frame is background. That was
found by saving a frame and looking at it, after three rounds of blaming the
segmenter.

## WHAT IS STILL NOT TRUE

* **No camera has ever been attached to this host.** Every number above came
  from depth rendered by `mock_rgbd_camera`, so this measures the pipeline and
  not the room. The map records that in its own `camera.caveat`.
* **`camera_link` has never been checked against the physical module.** It is
  forward kinematics, exact given the URDF, and a 10 mm URDF error lands
  directly in every point. It is the cheapest high-value calibration left.
* **The mock's scene is degenerate for a learned segmenter**: flat-shaded
  boxes, a cube the same colour as the pad it stands on, no texture. FastSAM
  returns between 2 and 12 regions for the same picture. The height split
  exists because of this and would matter less on a real photograph.
* **A few fragments survive**: small pieces at the feet of real cubes, from
  masks that covered a pad and the cube on it. They are extra objects, not
  wrong ones, and `pick_from_map` refuses to choose an object for you.
* **Nothing is wired into the modes or the clip sweep yet.** The GUI panel,
  the two scripts and the recorder are the whole surface.
* **The reachability cache is per-settings and IK is stochastic.** TRAC-IK
  restarts randomly, so a cell that solved on the probing run may not solve on
  the next — 35 of 109 planned cells on the cached run. The map came out the
  same; the number is reported rather than hidden.
* **THE SCENE CAMERA CANNOT CONTRIBUTE TO THE MAP AT ALL**, and this is not a
  wiring gap. `scene_camera_node` publishes `image_raw` and `camera_info` and
  **no depth** — it is a monocular colour camera. A single colour image gives
  a direction to an object, never a distance, so it cannot place a point in
  metres. It could label objects the wrist cameras already measured, by
  projecting them into its frame; it cannot measure geometry. And no camera
  has ever been attached to this host regardless.

---

# WHAT THE CALIBRATION IS WORTH, MEASURED

Two questions the operator asked: how accurate is the pick and place after
calibration, and how much does shared-autonomy teleoperation improve.

## PICK AND PLACE: 11.3 mm mean, 4 of 4 inside the gate

`accuracy_table.py` reports 100% grasp and 0.000 mm for every mode. Both are
sound and neither answers this, because the arm is scored against **the
coordinate it was given**. Hand it a coordinate 50 mm off the cube and it hits
that to 0.000 mm and the table still says 100%.

`measure_pick_accuracy.py` scores against where the object **really is** and
follows the whole chain — true centre → belief → grasp pose → IK → **FK to the
finger pads**, walking the Robotiq mimic chain at the opening the grasp is
made at.

| | belief err | **pad miss** | inside the 30 mm gate |
| --- | --- | --- | --- |
| MEASURED (calibrated) | 11.3 mm mean | **11.3 mm mean, 14.3 worst** | **4 of 4** |
| DECLARED | 0.0 mm | 0.0 mm | 4 of 4 |

**DECLARED reading 0.0 is not a result about the declared pipeline.** In
simulation the declared coordinate *is* the truth — the renderer builds the
scene from the same file. On a real table it is wrong by however far the
object differs from the file, which is unbounded.

Pad miss equals belief error on every row, which is the expected result: the
grasp is built as `centre − axis·pad` and FK on an exact IK solution puts the
pads back at `centre`. It says the grasp construction adds nothing of its own.
`--inject-pad-error-mm 15` moves DECLARED 0.0 → 15.0 and MEASURED 11.3 → 26.0,
so it can see the class of defect that cost this project 13.47 mm once.

**On "all modes":** `run_abc` builds the waypoints with no mode argument, so
this planning figure is one number for all five. What differs per mode is
execution, which the clips measure.

## SHARED AUTONOMY: the confidently-WRONG rate goes to zero

`IntentEstimator` is a Bayesian posterior over which object the operator is
reaching for. Its inputs are the object positions — that is the entire
connection to calibration.

In simulation the declared coordinates *are* the truth, so a declared and a
calibrated estimator get identical inputs. So the declared error is **swept**:
the estimator is given positions displaced by D mm from where the objects
really are, and the operator reaches for a real one. The calibrated arm
carries its **measured** 11.3 mm, not a hoped-for zero. 400 trials per cell.

| table is off by | DECLARED correct | DECLARED **WRONG** | CALIBRATED correct | CALIBRATED **WRONG** |
| --- | --- | --- | --- | --- |
| 0 mm | 71.5% | 0.0% | 74.2% | 0.0% |
| 20 mm | 72.8% | 0.5% | 73.5% | 0.0% |
| 40 mm | 57.0% | **17.5%** | 72.5% | **0.0%** |
| 60 mm | 53.2% | **35.0%** | 77.2% | **0.0%** |
| 80 mm | 53.5% | **39.8%** | 72.8% | **0.0%** |
| 120 mm | 50.2% | **47.5%** | 76.2% | **0.0%** |

**WRONG is the number that matters.** Ambiguous means assistance declines to
help and the operator drives; wrong means assistance confidently pulls them
toward the wrong cube. Uncalibrated, that rises to **47.5%** as the table
drifts from the file — at 60 mm, one cube pitch, the file is pointing at the
neighbour. Calibrated it is **0.0% at every displacement**, because the
positions track the objects regardless of what the file says.

At 0 mm the two are the same question and the 2.7 point difference is sampling
noise at 400 trials. **Today's simulation cannot show this gain**, which is
why the sweep exists.

The ~25% ambiguous floor is geometry, not calibration: reaching for the inner
right cube puts the outer one **directly behind it** along the approach, so the
direction points at both. Measured per target with exact positions —
20/20, 20/20, 1/20, 15/20. An object with another behind it is hard, and the
estimator is right to stay unsure rather than guess.


---

# WILL IT ADAPT TO A TABLE ANYWHERE, OR ONLY TO THE ONE IN FRONT?

The operator asked, and the answer was **only the one in front**. Worth
separating what did adapt from what did not.

**Adapted already:** the table's HEIGHT (measured, 1.2502 against a true
1.2500), its SIZE, and every object's position, size and count.

**Did not adapt:** the table's POSITION. `BOUNDS` was a constant — a box in
front of the wearer, x 0.325..0.750, y 0.15..0.65 — the height guess was the
constant 1.25, and the camera's base heading pointed forward. Put the table to
one side and the sweep would quarter the empty air where the table used to be
and report a confident map of nothing.

## FIRST, WHERE CAN A TABLE PHYSICALLY BE?

There is no point promising adaptation to positions the arm cannot reach. The
earlier envelope measurement only ever asked about y 0.15..0.65, so the answer
it gave was bounded by the question. Re-run wide (x 0.05..1.00, y −0.25..0.95):

| | measured | note |
| --- | --- | --- |
| outboard | x to **0.85** | real: the probe asked to 1.00 and the arm stopped short |
| forward | y to **0.65** | real: asked to 0.95 |
| rearward | at least y = **−0.25** | **not established** — only asked to −0.25 |

The arm can place its camera *behind the frontal plane*. That is IK-reachable
and **not thereby safe**: hard constraint 11, `avoid_collisions` is not the
wearer check. The auto bounds never go inboard of the columns measured
geometrically (0.325 left, 0.450 right).

## THE TABLE IS FOUND NOW, NOT ASSUMED

`--auto-bounds` runs a coarse probe over the arms' whole measured envelope,
fits the support plane, takes the extent of the points lying on it, and sets
the sweep bounds from that — clipped to the envelope and to the wearer-safe
columns. It **refuses** rather than falling back to the constant: a sweep of
the wrong volume is worse than no sweep, because it produces a map.

Measured, with the whole rendered scene shifted 250 mm outboard and 150 mm
back while the task files stayed put:

```
table found: surface z = 1.2501 m, tilt 0.09 deg,
             spanning x -0.596..1.062  y 0.219..0.650
left arm will sweep x 0.325..0.850  y 0.219..0.650
6 objects, all at their shifted positions
```

Both the fixed-box and the found-bounds versions located the moved objects at
that displacement — the cameras see well beyond the swept box at 0.5 m
standoff — but only the second **knew** where the table was.

## AND A TABLE OUT OF REACH IS REFUSED

Shifted 450 mm forward, past the measured forward limit:

```
stopping -- a surface was found at z = 1.2490 spanning y 0.819..1.327, and the
arms can only reach y -0.25..0.65 (measured). The table is beyond the arms'
reach -- move it, or move the wearer. Nothing was swept and no map was written.
```

**That case found a real bug.** Clipping the found extent to the envelope can
empty the interval, and I guarded the x span and not the y: the table's true
extent was y 0.819..1.327, clipping gave lo = 0.819 and hi = 0.65, and the
sweep was cheerfully told to cover **"y 0.819..0.650"** — an inverted range.
The "a check that cannot fail" rule, with one axis left out.

## WHAT IS STILL NOT ADAPTIVE

* **The rearward limit is unmeasured.** The probe only asked back to y = −0.25.
* **A table BESIDE the wearer is untested.** The coarse probe spans the
  envelope, so it should find one; nothing has demonstrated it.
* **The camera heading is still a fixed forward-ish base direction** yawed by
  the facings. A table at a very different bearing would be viewed obliquely.
* **`SRL_SCENE_SHIFT_XY` is how the mock was made to disagree with the task
  files** — without it, simulation cannot produce the case where declared
  coordinates are wrong, which is the only case where any of this matters.


---

# A TABLE BESIDE THE WEARER, AND THE LAST OF THE SPURIOUS OBJECTS

## Beside, and 300 mm lower

The untested case, tested. The whole rendered scene shifted (+0.35, −0.35,
**−0.30**) with the task files untouched, so the table is at the wearer's
SIDE, close in, at a different height, and every declared coordinate is wrong.
`SRL_SCENE_SHIFT_XY` grew a third component for this — "any height" cannot be
tested by moving a table sideways — and the coarse probe looks from two camera
heights so `--surface-z` is no longer load bearing.

```
table found: surface z = 0.9503 m, tilt 0.03 deg,
             spanning x -0.758..0.995  y 0.019..0.540
left  arm will sweep x  0.325..0.850  y 0.019..0.540
right arm will sweep x -0.758..-0.450 y 0.019..0.540
```

| truth (after the shift) | measured |
| --- | --- |
| surface 0.9500 | **0.9503** |
| pad (0.640, 0.150) | **(0.640, 0.150)** |
| pad (0.060, 0.150) | **(0.060, 0.150)** |
| cubes at 0.770 / 0.830 / −0.070 / −0.130, y 0.100 | **all four exact** |

19 of 19 reachable cells, **six objects and the scene has six**.

## The spurious detections are gone, and it took two rules

The first run of that scene carried three extras at the pads' edges. Both
rules are physical, not tuned:

**Points per view.** A total count does not separate a real object from a long
thin sliver, because a sliver seen by eighteen viewpoints accumulates a
respectable total out of nothing. Measured:

| | points per view |
| --- | --- |
| the six real objects | **754 – 18 083** |
| the three fragments | **13, 16, 28** |

Seven times clear at the worst. A viewpoint that genuinely sees an object
returns hundreds of points from it; one that catches its edge returns a
handful.

**A size floor.** That rule left one survivor, **1 mm across** — which is not
a small object, it is a line of points where two surfaces meet. At the sweep's
working range one camera pixel subtends about 2.1 mm, so an extent under two
pixels is below what the sensor can resolve. The floor is 5 mm and a genuinely
thin object clears it.

## How far behind the wearer the arms reach

Asked out to y = −0.70 and got −0.70: the left arm can place a camera **0.7 m
behind the frontal plane**, and the limit is *still* the question rather than
the arm. It does not matter operationally, because going there is IK-reachable
and not thereby safe — hard constraint 11 — and the swept bounds never go
inboard of the columns measured geometrically. Recorded so nobody re-derives
it: `recordings/baselines/reachable_rear.json`.

---

# ANY SHAPE, ANY COLOUR? TESTED.

The operator asked. Both were reasoned about first — the plane fit is
geometry-only and the segmenter cuts on edges rather than on particular
colours — and reasoning is not measuring, so both were run.

Two hooks were added to the mock for it: `SRL_SCENE_TABLE_RGB` recolours the
table, `SRL_SCENE_EXTRA_BOX` adds solids so it need not be a rectangle.

## Colour: no effect

The table rendered **dark grey (0.30, 0.30, 0.34)** — close to the background
the scene is drawn on, which is the hardest case, not a token change.

| | measured | truth |
| --- | --- | --- |
| surface | **1.2503** | 1.2500 |
| pad | (0.291, 0.499) | (0.290, 0.500) |
| pad | (−0.289, 0.500) | (−0.290, 0.500) |
| cube | (0.480, 0.449) 39 mm | (0.480, 0.450) 40 mm |
| cube | (0.420, 0.450) 40 mm | (0.420, 0.450) 40 mm |

Identical to the white table. Nothing in the mapping path reads colour to
decide anything: `mean_bgr` is *recorded* per object so a caller can ask for
"the green one", and that is all.

## Shape: an L-shaped table works, and the missing corner refuses

The usual bar plus a wing at x 0.30–0.90, y 0.13–0.43, with an orange cube
standing **on the wing**.

```
table found: surface z = 1.2502, tilt 0.05 deg, y 0.069..0.650
left arm will sweep x 0.325..0.850  y 0.069..0.650
object 2 at (0.600, 0.280, 1.282), 41 mm across the jaws, graspable
```

The bounds **stretched forward to cover the wing**, and the cube on it came
back at its exact position. Five objects, which is what is on that side.

And the L's concave corner did the right thing:

> *no support plane in this view … There is no support surface in this view —
> refusing rather than grasping against a guess.*

A viewpoint aimed where the table is not **refused rather than inventing a
surface**. That is the shape showing up as an honest skip, not as bad data.

## What the bounding box costs

`find_table` takes the axis-aligned bounding box of the surface points, so a
non-rectangular table has cells planned over its missing parts. Measured on
this L:

| | |
| --- | --- |
| swept box | 0.3050 m² |
| table inside it | 0.2730 m² |
| **actually table** | **90%** |
| a round table inscribed in its box, for reference | 79% |

The waste is bounded and it is not silent: a cell over nothing refuses by
name. Planning from the measured surface CELLS rather than their bounding box
would recover it, and is not done — the same improvement the reachable set
already got.

**Not tested:** a table with two surfaces at different heights (the plane fit
takes the biggest and says nothing about the other), and a genuinely round or
curved edge — the mock renders boxes.
