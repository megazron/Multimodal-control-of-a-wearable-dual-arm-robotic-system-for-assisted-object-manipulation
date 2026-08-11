# The randomisation region — MEASURED, before anything was designed around it

**Run it:** `python3 scripts/measure_randomisation_region.py --repeats 3`
→ `recordings/baselines/randomisation_region.json`

An ALOHA-style protocol randomises the object's start position inside a marked
region. That region is not a design choice here — it is whatever the arms can
actually pick from — so it was measured first. Designing the task and
discovering the region afterwards is how the previous task set acquired five
waypoints no arm could reach.

Object positions are swept, not end-effector poses: `ee_for()` derives the
wrist from the object, because doing it the other way round is what once put
the wrist 95 mm inside the bench.

---

## Two orientations, and they are not the same region

| | what it is | which modes |
| --- | --- | --- |
| **ANCHOR** | the pinned wrist `orientation_mode: fixed` gives. Tool axis measured at (−0.153, +0.846, +0.511) — 120.8° from straight down, **30.7° above horizontal** — so the hand comes in from the near side and *below* the object | 1–4, and it is the **only** thing they can command: nothing in the master measures the wrist |
| **TOP-DOWN** | approach along −z, built with `grasp_library._quat_from_z_and_x` so there is one definition of top-down in the repo | 5–6, which emit a full 6-DOF pose |

## Results

40 mm block, bench in the planning scene, k = 3, both arms, x mirrored for the
right arm. "Supported" means the object is actually **resting on the bench**
(near face at or behind `BENCH_NEAR_Y` = 0.245, so `y ≥ 0.265`).

### Per arm

| region | cells | largest rectangle |
| --- | --- | --- |
| anchor, left, reachable | 31 / 121 | \|x\| 0.30–0.39, y 0.13–0.23 → **90 × 100 mm** |
| anchor, left, **supported** | **0** | none |
| anchor, right, reachable | 7 / 121 | \|x\| 0.54 only, y 0.13–0.21 → **0 × 80 mm** |
| anchor, right, **supported** | **0** | none |
| top-down, left, reachable | 71 / 121 | \|x\| 0.30–0.42, y 0.13–0.29 → 120 × 160 mm |
| top-down, left, **supported** | 14 | \|x\| 0.24–0.42, y 0.27–0.29 → **180 × 20 mm** |
| top-down, right, reachable | 51 / 121 | \|x\| 0.33–0.54, y 0.13–0.21 → 210 × 80 mm |
| top-down, right, **supported** | 5 | \|x\| 0.51–0.54, y 0.27–0.29 → **30 × 20 mm** |

### Shared between the arms

| region | cells | rectangle |
| --- | --- | --- |
| both arms, anchor | **0 / 121** | none |
| both arms, top-down | 39 / 121 | \|x\| 0.33–0.54, y 0.13–0.19 → 210 × 60 mm (**unsupported**) |
| both arms, both orientations (cross-mode comparable) | **0 / 121** | none |
| both arms **and** supported, either way | **0 / 121** | none |

## The three findings

**1. Under teleoperation the two arms share no pickable position at all.**
The left arm's anchor region is |x| 0.30–0.39; the right arm's is |x| 0.54.
They do not overlap in mirrored x by a single cell. This is the disjoint-
workspace result at grasp resolution — sharper than the coarse "0 of 63
frontal cells", and from the same cause: the arms are parked asymmetrically
(`|v_R − M·v_L| = 1.3837 m`, provably independent of the mount).

Note also how *small* the right arm's anchor region is — 7 cells, one column
wide. Any task that randomises a right-arm pick under teleoperation is
randomising over essentially one x value.

**2. Nothing the teleoperated modes can grasp is actually on the bench.**
`anchor_*_supported` is **0 for both arms**. Every position the pinned wrist
can pick lies at y ≤ 0.23, while an object is only supported from y ≥ 0.265 —
a **35 mm gap**. The graspable strip is in front of the bench edge, in mid-air.

This was invisible until now because task objects are markers, and *markers do
not fall*. Measured directly on the shipped geometry: **Task A's 40 mm block
sits at y = 0.185, spanning 0.165–0.205 against a bench that starts at 0.245 —
zero overlap, 40 mm clear of the edge, entirely unsupported.** Under the
architecture the physics gate recommends, where objects have dynamics, it hits
the floor before the trial starts.

It is not a coordinate typo: `on_bench()` applies `OVERHANG = 0.08`
deliberately, because the anchor approach comes from *below* and needs the
object clear of the bench top. The overhang is doing its job; it is simply
larger than the object.

**3. There is no cross-mode comparable region with real objects.**
Anchor ∩ top-down ∩ supported is empty for both arms and for each arm alone
(because the anchor half is already empty). So a colour-matched placement task
that is **both** comparable across teleoperated and autonomous modes **and**
uses physically supported objects **cannot be built on this bench as it
stands.**

## What to do about it — and what not to

**Do not lower the tolerance or fudge the support test.** The support test is
`(y − h/2) ≥ BENCH_NEAR_Y`, which is arithmetic, and the 35 mm gap is real.

**The lever is the bench, not the software.** The graspable strip is fixed by
the arm and the pinned wrist; the *supported* strip is fixed by where the
bench edge is. Moving `BENCH_NEAR_Y` forward by ~55 mm (0.245 → ~0.19) would
bring the supported boundary to y ≥ 0.21, inside the anchor region both arms
reach in y.

> **This is a prediction, not a result.** It follows from the two boundaries
> above but has not been measured. Re-run this script with the moved edge
> before designing anything around it — moving the bench also moves every clip
> task's object, which is exactly the coupling that produced a sweep whose
> score was invariant to the parameter being swept (`OVERHANG`'s own comment
> records that).

**A shared region still will not appear.** Moving the bench changes y; the
left/right non-overlap is in **x** and is the parking asymmetry. So even after
the move, colour-matched placement must be **per arm**, with its own region
and its own bins on each side — the same conclusion Task 0 reached, for the
same reason.

**Until the bench moves, the honest options are:**

| option | what it costs |
| --- | --- |
| run colour-matched placement in **autonomy modes only**, per arm, in the measured supported strips (left 180 × 20 mm, right 30 × 20 mm) | no mode comparison — the same defect that removed the old Task 2 from the programme |
| run it in **all modes with unsupported objects** (a fixture, a peg, a shallow cradle at the strip) | the object no longer *falls*, so the placement dynamics the physics decision was made for are gone |
| **move the bench edge forward and re-measure** | one geometry change, then re-verify every clip task against it |

The third is the only one that keeps both the mode comparison and the physics,
and it is a bench change, not a code change.

## A fourth finding, arrived at by accident and worth keeping

The first version of this script left the bench in the planning scene when it
finished. The scene lives in `move_group` and outlives the process, so the
next run of `verify_abc_scenarios.py` inherited it — and **refused its own
instrument control**: "a declared Task A target → UNREACHABLE".

That is not a fluke of the leak. Task A declares a target at **z = 1.05**,
and the bench slab occupies **z = 1.06 – 1.10**. The target is under the
bench. It verifies today only because `verify_abc_scenarios` checks the study
tasks in **free space** and applies the furniture solely for the clip tasks —
so no study task's coordinates have ever been checked against the bench they
are performed over.

Task 0's band was re-derived against the bench for exactly this reason
(`experiments/abc/task0.py`). **Task A's has not been**, and should be before
it is run with furniture present. The leak is fixed — the script now removes
the furniture it adds — but the finding it exposed is real and independent of
it.

## Instrument notes

* **The first run of this sweep reported an empty region for every
  orientation, and it was the instrument.** It swept y from `BENCH_NEAR_Y`
  (0.245) *backwards*, while the graspable strip is at y ≈ 0.13–0.23 —
  entirely in front of where it looked. The y range must straddle the edge.
* The run is bracketed by controls: Task A's own pick must come back
  reachable and a pose 1.6 m out must not. It **refuses to report** if either
  is wrong, because an empty region and a broken solver look identical — and
  three of the numbers above are zeros.
* The deliverable is the largest axis-aligned **rectangle**, not the cell
  count. A randomisation region has to be something an experimenter can mark
  with tape and drop an object anywhere inside; a scattered feasible set is
  not that, and a raw count would flatter it.

---

# T1 UPDATE (2026-08-11) — the cube, and whether the layout fits

Measured by `scripts/measure_t1_cubes_t2_transfer.py`, bench in scene.

## Grasping a 40 mm CUBE, per arm, per orientation policy

A cube admits only **four** grasp yaws (its 4-fold symmetry about the
vertical); a cylinder admits all of them. That distinction is the whole of the
cube-versus-cylinder question, so it is measured on identical poses:

| arm | fixed (pinned anchor) | cube, best of 4 yaws | cylinder, best of 8 | top-down, 4 yaws |
| --- | --- | --- | --- | --- |
| left | **12/12 (100 %)** | 12/12 | 12/12 | 12/12 |
| right | **5/12 (42 %)** | **11/12 (92 %)** | **11/12 (92 %)** | 12/12 (100 %) |

**THE CUBE COSTS NOTHING AGAINST A CYLINDER: 11/12 either way.** Gate 2's
5/12-versus-11/12 gap was *fixed versus yaw-free*, not *cube versus cylinder* —
four yaws already recover everything eight do. The object shape is not the
binding factor; **wrist freedom is**. Switching to cylinders would have bought
nothing and would have changed the task for no measured gain.

Read per mode:

| mode | wrist | left | right |
| --- | --- | --- | --- |
| MASTER_TELEOP | pinned, no yaw freedom | 100 % | **42 %** |
| VR_TELEOP | 6-DOF, operator chooses | 100 % | 92–100 % |
| MASTER_SHARED / VR_SHARED / FULL_AUTONOMY | autonomy supplies it | 100 % | 100 % |

**The hole in the matrix is MASTER_TELEOP on the RIGHT arm, and only there.**

## The randomisation region, and the bench edge

"Supported" = the cube's near face is at or behind the bench edge, i.e. it is
resting on something rather than floating.

| bench edge | left/fixed | left/top-down | right/fixed | right/top-down |
| --- | --- | --- | --- | --- |
| **0.245 (current)** | **NONE** | 180 × 20 mm | **NONE** | 330 × 20 mm |
| 0.210 | 120 × 0 mm | 180 × 60 mm | NONE | 330 × 60 mm |
| 0.190 | 120 × 20 mm | 210 × 80 mm | 30 × 0 mm | 330 × 80 mm |
| 0.170 | 120 × 40 mm | 240 × 100 mm | 30 × 20 mm | 330 × 100 mm |
| 0.150 | 150 × 60 mm | 240 × 120 mm | 30 × 40 mm | 330 × 120 mm |

The ~55 mm forward move predicted earlier is confirmed and now quantified: it
is the difference between **no supported teleop region at all** and
120 × 20 mm.

## Does the layout fit? Arithmetic, so it can be checked

Four cubes at the 60 mm pitch T0 measured, plus two 110 × 80 mm placement
targets:

    4 cubes  0.0144 m²   +   2 planes  0.0176 m²   =   0.0320 m²

| region | area | 4 cubes + 2 planes | 4 cubes alone |
| --- | --- | --- | --- |
| edge 0.245, left/top-down | 0.0036 m² | no | no |
| edge 0.190, left/top-down | 0.0168 m² | **no** | **fits** |
| edge 0.170, left/top-down | 0.0240 m² | **no** | fits |
| edge 0.150, left/top-down | 0.0288 m² | **no** | fits |

**Four cubes and two planes do not fit in any measured region at any tested
bench edge**, and no region reaches the 0.0320 m² required. Four cubes alone
fit once the edge moves to 0.190 or forward.

### What has to move — three options, none of them chosen here

1. **Bench edge forward to ~0.190 and the planes go outside the cube region**
   (further back on the bench, still within reach for the release). Keeps four
   cubes and all five conditions. Needs the plane positions verified
   separately — a placement target must be *reachable*, which is the same
   constraint as a pick.
2. **Two cubes instead of four** (one blue, one green). Fits at edge 0.190
   with the planes. Halves T1's 40 pick-place actions to 20, which is a real
   loss of statistical power and should be costed before it is chosen.
3. **T1 becomes left-arm only.** The right arm is the whole of the
   MASTER_TELEOP problem (42 % against 100 %), and T1 is already specified as
   one arm at a time. This preserves all five conditions at 100 % grasp
   feasibility and costs the arm comparison within T1 — which T0 measures
   anyway, on both arms, in every mode.

Option 3 costs nothing the study needs and removes the matrix hole; option 1
keeps the arm contrast at the price of a bench move. **Not decided here.**

---

# THE GENERAL CONSTRAINT — why support must come from BEHIND (2026-08-11)

**Record this once, because it explains the T1 layout result and it will
explain the next surprise too.**

The pinned wrist's tool axis is measured at **(−0.153, +0.846, +0.511)** in
world: 120.8° from straight down, and **30.7° ABOVE horizontal**. So under
`orientation_mode: fixed` — which is every teleoperated mode, because nothing
in the master measures the wrist — the hand enters **from the near side and
from BELOW**, and the fingers close **underneath the object and lift into it**.

Three consequences follow, and none of them is negotiable by software:

1. **The object must OVERHANG its support.** The space directly beneath an
   object is where the fingers go. An object sitting flush on a surface has no
   room for the approach.
2. **Support must therefore come from BEHIND** — from a surface whose *edge*
   is behind the object's centre. That is the only direction left once the
   front, the underside and the approach cone are excluded.
3. **A shelf cannot help.** The instinctive fix for "the object is floating"
   is to put something under it. That is precisely the volume the fingers
   need, so a shelf converts an unsupported object into an unreachable one.
   Only a *cantilevered* support — one with free space beneath its own
   overhang — can do both jobs at once.

This is why `on_bench()` carries `OVERHANG = 0.08` and why the bench EDGE, not
the bench, is the load-bearing parameter. It is also why the T1 layout fails:
the overhang is doing exactly what it should, and it is simply larger than a
40 mm cube, so the cube ends up entirely in front of the edge with nothing
under it.

**The test to apply to any future support geometry:** is there free space in
the cone 30.7° above horizontal, entering from the near side, beneath the
object? If not, the pinned modes cannot grasp it, however reachable the pose
looks.
