# 13 — WHY THE OBJECTS ARE FIXTURED, AND WHAT THAT COSTS

**This is a result about the platform, not a workaround.** Nothing in the task
layout can be rearranged to fix it, and the next person to look at a clip and
think "the cubes are floating, put a shelf under them" should read this first
and then re-measure rather than re-derive.

Follows directly from `12_randomisation_region.md`, which states the geometry
this document measures the consequence of.

---

## THE CONSTRAINT IT FOLLOWS FROM

The wrist is PINNED: `orientation_mode` is `fixed`, so the tool axis is
whatever the arm's anchor orientation makes it, and it is the same for every
pose in a clip. Measured from TF, that axis is

    (-0.153, +0.846, +0.511)

which is **120.8 deg from straight down and 30.7 deg ABOVE horizontal**. The
hand therefore enters from the NEAR side and from BELOW, and the fingers close
UNDERNEATH the object and lift into it.

Three consequences were already recorded:

  * the object must OVERHANG its support — hence `OVERHANG = 0.08` in
    `clip_tasks.on_bench()`, and hence the fact that anything shallower than
    160 mm ends up entirely in front of the bench edge;
  * support must come from BEHIND, because the front, the underside and the
    approach cone are all excluded;
  * the instinctive fix — put something under it — fills exactly the volume
    the fingers need, converting an unsupported object into an unreachable
    one.

The third was demonstrated once, for a continuous rail. What follows is the
general case.

---

## THE MEASUREMENT

`scripts/sweep_t1_supports.py`. T1's six pick paths (four cubes, two planes),
each densified at 20 mm over standoff → descend → lift, N=5 repeats per
waypoint against a live `/compute_ik` with T1's own scene loaded. Both
controls correct: the shipped bench-only layout scores 0, and a slab
deliberately laid across the approach scores 61.

| support geometry | waypoint failures |
| --- | --- |
| **bench only, no support** (the shipped, verified layout) | **0** |
| cantilevered lip, 75% of the object's depth supported | 22 |
| cantilevered lip, 50% | 20 |
| cantilevered lip, 25% | 16 |
| the same lips under the PLANES, 75% / 50% / 25% | 52 / 46 / 32 |
| cubes and planes together, 75% / 50% / 25% | 52 / 46 / 32 |
| a pad under the object's OWN FOOTPRINT only | 26 |
| side ledges on outboard posts, dx = 0.06 / 0.09 | 65 / 65 |

Full JSON: `recordings/baselines/t1_support_sweep.json`.

### Reading it

**The overhang is not the lever.** Sliding the lip back from 75% to 25% of the
object's depth — that is, supporting less and less of it — buys 22 → 16. The
trend is shallow and it does not approach zero. There is no overhang at which
a lip is free.

**The last two rows settle it.** A footprint-only pad does not reach back to
the bench at all, so whatever it obstructs is DIRECTLY BENEATH THE OBJECT,
which is where the fingers close. And moving the support outboard to the sides
is *worse*, because the posts then stand in the gripper's own corridor.

Front, underside, sides. There is no direction left for a support to come
from. The earlier rail sweep found the same wall from the other side: FIXED
cells 0 at every continuous-rail position from 0.190 to 0.245.

---

## SO THE OBJECTS ARE FIXTURED, AND HERE IS THE COST

An object that is held rigidly in place **cannot fall**. Therefore:

> **`drops` is not a measurable outcome in any task whose objects are
> fixtured.** It must not appear in a results table for T1, and a zero in that
> column would be an artefact of the fixturing, not a finding about the
> operator or the mode.

What survives, and is unaffected:

| measure | survives? | why |
| --- | --- | --- |
| grasp success | YES | the fingers still have to reach the object and close on it |
| placement success | YES | the object still has to end up on the right plane |
| **wrong-colour placement** | YES | this is T1's headline outcome and it is untouched |
| completion time | YES | |
| trajectory length | YES | |
| **drops** | **NO** | a cube that cannot fall cannot be dropped |

T2 is a separate case and is NOT covered by this document: its tray is held at
two points by two grippers from the first frame of the clip, so "rests on a
surface" does not apply to it at all. Its ball is drawn on the tray and is not
simulated — the clip shows the geometry, not the physics.

---

## WHAT WOULD ACTUALLY FIX IT

Raise the objects onto stands well clear of the bench top, so the approach
cone lies in free air ABOVE the bench instead of inside the 60 mm strip in
front of its edge. That moves `T1_Z`, which means re-deriving the whole layout
against `scripts/verify_t1_layout.py` — a task-position change with its own
verification pass, not a scene edit, and it invalidates the current verified
cube and plane coordinates.

`SUPPORTS_ENABLED` in `scripts/clip_scene.py` re-enables the lip machinery so
any candidate geometry can be re-measured with the sweep above rather than
argued about.

---

## THE HONEST SUMMARY FOR A WRITE-UP

> Objects in the pick-and-place task are fixtured rather than resting on the
> work surface. This is forced by the platform: with a pinned wrist the tool
> axis lies 30.7 deg above horizontal, so the hand approaches from below and
> in front, and every support geometry tested — cantilevered lips at three
> overhangs, footprint-only pads, and side ledges on outboard posts — made
> some of the verified pick path infeasible, against zero failures with no
> support at all. The cost is that `drops` is not a measurable outcome for
> that task; grasp success, placement success and wrong-colour placement are
> unaffected.
