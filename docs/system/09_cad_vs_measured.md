# The CAD against the measured arm

## THE HEADLINE: a convention-free cross-check, agreeing to 1%

**Bend-to-bend spacing does not depend on where a roll joint is said to sit
along its own axis, so it is the one comparison that carries no convention:**

| | measured | CAD-inferred | delta |
| --- | --- | --- | --- |
| **J2 → J4** | 80.0 mm | **79.0 mm** | −1.0 |
| **J4 → J6** | 79.0 mm | **79.0 mm** | **0.0** |

A tape measure on the built arm and a solid model, independently, agreeing to
about 1%. **This is the strongest independent confirmation the forward
kinematics has ever had** — every workspace figure and every task coordinate
rests on those lengths, and until now nothing outside the measurement itself
had ever corroborated them.

## AND THE RIG IS NOT A CONSISTENT SCALE MODEL

| | |
| --- | --- |
| mannequin | 305 mm (12 in) |
| URDF wearer | 1750 mm |
| **torso scale** | **1 : 5.74** |
| master arm, kinematic | 272 mm measured / 260 mm CAD |
| a human arm, shoulder to grip | ~600 mm |
| **arm scale** | **~1 : 1.8** |

**The master arms are more than three times too long for the doll they are
bolted to.** That is the right choice for an input device sized to the
operator's hand travel rather than to the mannequin — but it means **the
mannequin's proportions say nothing about the robot's mount geometry**, and
any figure derived by scaling the master rig up to wearer size is a
coincidence rather than a corroboration.


**The working URDF derives from MEASURED link lengths. Nothing in this document
changes that, and nobody should later treat the CAD as the kinematic source.**
The CAD was designed for 3D printing: no articulation, no joint frames, one
solid per printed part. Every frame below is *inferred from geometry*, and each
carries the confidence that inference earns.

## 0. `/mnt/c`

Healthy. The I/O errors recorded in `docs/ENGINEERING_LOG.md` are gone — a WSL restart at
some point fixed the dead 9p mount. Both files read directly, no remount
needed.

## 1. What is in the files

Reader: **`cadquery-ocp`** (pip wheels, no root, no conda) — the full
OpenCASCADE kernel via OCP bindings. Bounding boxes, volumes and surface
classification all need the kernel, so a STEP *parser* is not enough.
`pythonocc-core` is conda-first with unreliable pip wheels; headless FreeCAD
needs apt (root) and about a gigabyte for the same kernel.

| | `PotArm.step` | `backpack.step` |
| --- | --- | --- |
| STEP unit | **MM** | **MM** |
| solids | 75 | 418 |
| bounding box | **106.5 × 94.3 × 336.7 mm** | **75.0 × 60.0 × 65.6 mm** |
| volume | 127.7 cm³ | 61.0 cm³ |
| centre of mass | (−8.0, −0.5, 204.9) | (4.4, 0.0, 28.8) |
| mass @ PLA 1.24 g/cm³ | ~158 g | ~76 g |

**No material properties are in either file** — the masses are estimates at
PLA density and are labelled as such wherever they appear. The file is
`PotArm.step`, not `leftpotarm`; it is the part printed twice.

## 2. The inferred kinematics

Three groups of solids appear **exactly seven times each** — one per joint:

| dimension signature | count | what it is |
| --- | --- | --- |
| 9.2 × 19.8 × 26.5 mm | **7** | the pot body |
| 11.0 × 16.9 × 19.0 mm | **7** | its bracket |
| 14.5 × 14.5 × 16.0 mm | **7** | the shaft boss |

### The alternation is confirmed, not assumed

The pot bodies are flat, so the short dimension lies **along the rotation
axis**. Reading it off each of the seven:

| joint | pot body dims (x, y, z) | thin axis | ⇒ |
| --- | --- | --- | --- |
| J1 | 19.8, 26.5, **9.2** | z | **roll** |
| J2 | 19.8, **9.2**, 26.5 | y | **bend** |
| J3 | 19.8, 26.5, **9.2** | z | **roll** |
| J4 | 19.8, **9.2**, 26.5 | y | **bend** |
| J5 | 19.8, 26.5, **9.2** | z | **roll** |
| J6 | 19.8, **9.2**, 26.5 | y | **bend** |
| J7 | 19.8, 26.5, **9.2** | z | **roll** |

**roll / bend / roll / bend / roll / bend / roll**, recovered from the
geometry alone. This is an independent confirmation of the chain convention
the software has always assumed. **Confidence: HIGH** — it is a direct
consequence of seven measured bounding boxes, not a fit.

### Joint positions

| | value | confidence |
| --- | --- | --- |
| bend joints J2/J4/J6, z | 60.7, 139.7, 218.7 mm | **HIGH** — a bend axis is a line at a definite height, and two independent solid groups agree to 0.2 mm |
| roll joints J1/J3/J5/J7, z | 23.0, 103.0, 180.0, 260.0 mm | **MEDIUM** — a roll axis is the z line itself, so position *along* it is a convention, not a measurement. Taken from the shaft-boss centroid |
| all axes, x | 0.000 mm | **HIGH** — every group centres on x = 0 to 0.08 mm |
| J1 base offset | ambiguous | **LOW** — where the kinematic chain "starts" in a printed base plate is a choice |

## 3. CAD against measured — the comparison

### Link lengths

| link | measured (URDF) | CAD-inferred | Δ |
| --- | --- | --- | --- |
| L1 base→J1 | 43 mm | 28.0 | **−15.0** |
| L2 J1→J2 | 37 mm | 37.7 | +0.7 |
| L3 J2→J3 | 43 mm | 42.3 | −0.7 |
| L4 J3→J4 | 37 mm | 36.7 | −0.3 |
| L5 J4→J5 | 43 mm | 40.3 | −2.7 |
| L6 J5→J6 | 36 mm | 38.7 | +2.7 |
| L7 J6→J7 | 33 mm | 41.3 | **+8.3** |
| **total** | **272 mm** | **265.0** | −7.0 (2.6%) |

**The strongest agreement is the one that needs no convention.** Bend-to-bend
spacing is measured directly and is independent of where a roll joint is said
to sit along its own axis:

| | measured | CAD | Δ |
| --- | --- | --- | --- |
| J2→J4 | L3+L4 = 80 mm | **79.0** | −1.0 |
| J4→J6 | L5+L6 = 79 mm | **79.0** | 0.0 |

**Within 1 mm on both.** That is a real cross-check: two independent sources,
one a tape measure on the built arm and one a solid model, agreeing to about
1%.

### Which do I trust, and where they disagree

**The measured values, for the shipped URDF — and the disagreement is not
evenly spread.**

- **L2–L4 agree to within 0.7 mm.** Both sources are right here.
- **L5, L6 differ by 2.7 mm each, in opposite directions.** That is the
  signature of a boundary being drawn one joint out of step, not of a wrong
  arm. The pair sums to 79 in both.
- **L1 (−15 mm) and L7 (+8.3 mm) are the ends of the chain, and they are
  exactly where "the link length" stops being a physical distance and starts
  being a convention.** L1 is base-plate to first pot: the CAD's base plate
  extends 5 mm below z=0 and the printed boss adds more, so where the chain
  begins is a choice. L7 is the last pot to the tip: the CAD part continues
  to z=331.7 with a handle the kinematic chain does not model, so the CAD's
  41.3 mm is the pot-to-pot distance while the measured 33 mm is pot-to-grip.

**So this is not a discrepancy that invalidates the FK.** The interior of the
chain — the part where both sources measure the same physical thing — agrees
to 1 mm. The ends differ because they are defined differently, and the
measured definition is the one the FK, the workspace and every task coordinate
were built on.

**What would change my mind:** if L5/L6 disagreed while their sum did not
match either, or if bend-to-bend were out by more than a few mm. Neither is
the case.

### The backpack mount — and a distinction that has to be made first

**The CAD backpack is the MASTER-ARM mount on a 12-inch mannequin. The URDF's
mount rpy is the ROBOT-ARM mount on a 1.75 m wearer. They are different
brackets on different bodies, and comparing their rotations directly would be
a category error.** The brief expected this comparison to be a first check on
the analytically-solved mount rotation; it cannot be, because the physical
bracket in this CAD is not the bracket that rotation describes.

What the CAD *does* say about the master rig:

| | |
| --- | --- |
| arm seat faces | the ±y faces, normals (0, ∓1, 0), area 2308 mm² each, centroids (5.8, ∓30, 30) |
| seat separation | **60 mm** |
| through-bore joining them | r = 5.0 mm, axis along **y**, at (12.6, −0.9, 51.5) |
| implied arm orientation | both arms extend **laterally**, ±y, with **zero forward tilt and zero splay from the bracket** |

So the master arms come straight out of the sides of the pack. Any forward
tilt in the physical rig comes from how the pack sits on the mannequin, not
from the bracket.

### The scale factor, reconciled

| | |
| --- | --- |
| mannequin | 305 mm (12 in) |
| URDF wearer | 1750 mm |
| **torso scale** | **1 : 5.74** |
| master arm, CAD | 336.7 mm overall, 272 mm kinematic |
| a human arm, shoulder to grip | ~600 mm |
| **arm scale** | **~1 : 1.8** |

**The rig is not a consistent scale model, and that is the finding.** The
torso is 1:5.74 and the arms are 1:1.8 — the arms are more than three times
too long for the mannequin they are bolted to. That is the correct choice for
a *master input device*, where the linkage is sized to the operator's hand
travel rather than to the doll, but it means the mannequin's proportions carry
no information about the robot's mount geometry. Scaling the 60 mm seat
separation up by 5.74 gives 344 mm against the URDF's 400 mm mount separation
— a 16% difference which, given the arms are at a different scale entirely, is
a coincidence rather than a corroboration and should not be read as one.

## Limits — what is inferred, and what is not

| item | basis | confidence |
| --- | --- | --- |
| roll/bend alternation | seven measured bounding boxes | **HIGH** |
| bend joint heights | two solid groups agreeing to 0.2 mm | **HIGH** |
| all axes at x = 0 | measured to 0.08 mm | **HIGH** |
| bend-to-bend spacing | direct, convention-free | **HIGH** |
| roll joint position along its own axis | shaft-boss centroid; a convention | **MEDIUM** |
| L1 and L7 | depend on where the chain is said to begin and end | **LOW** |
| the pot *shaft* bores | not separated from screw holes — the r = 1.9–2.2 mm clusters are dominated by fasteners | **NOT FOUND** |
| backpack → URDF mount rpy | **not comparable** — different bracket, different body | **N/A** |

**The working URDF derives from measured lengths and should continue to.** The
CAD corroborates the interior of the chain to about 1 mm and confirms the
joint convention independently; it does not supersede the measurements, and
the two places they differ are both places where the CAD is answering a
slightly different question.
