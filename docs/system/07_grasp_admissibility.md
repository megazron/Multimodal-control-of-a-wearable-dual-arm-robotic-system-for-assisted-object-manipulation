# The admissible grasp set with the wrist pinned

**The system does not choose an approach. It has one.**

That is the finding, and everything below follows from it. `orientation_mode`
is `fixed`, so the commanded wrist quaternion is the anchor — the arm's own
orientation at its home pose. Nothing about an object's shape, pose, principal
axis or affordance enters the calculation anywhere. There is no approach
selection to improve; there is an approach that was inherited and never
examined.

## What the inherited approach actually is — measured

From live TF at the anchor, rotating the tool frame into world:

| arm | approach axis (tool +z) | from straight down | jaw closing axis | jaw yaw |
| --- | --- | --- | --- | --- |
| left | (−0.153, +0.846, +0.511) | **120.8°** | (−0.984, −0.082, −0.159) | **4.7°** |
| right | (+0.259, +0.890, +0.376) | **112.1°** | (−0.962, +0.202, +0.184) | **168.1°** |

0° would be a top-down grasp, 90° horizontal. Both arms sit past horizontal:
the hand points **forward and upward**, 30.7° (left) and 22.1° (right) above
the horizontal plane. It comes in from the near side, below the object, and
reaches up into it.

Two consequences, both load-bearing:

- **The objects must overhang a bench edge.** The wrist trails the finger pads
  by 95 mm horizontally and 57 mm vertically, so grasping anything resting on
  a surface puts the wrist *below the surface top*. That is free space only in
  front of an edge. This is why every graspable object in the clip set sits at
  the bench edge with 80 mm of overhang, and why a target aimed at an object's
  **centre** fails while the same target aimed at its **near edge** succeeds.
- **A top-down grasp is unavailable to every mode.** It would need the wrist
  rotated by 120.8°/112.1°, and `orientation_mode: fixed` commands no rotation
  at all.

## The admissible set

Whether an object can be grasped at a fixed jaw yaw depends only on its
rotational symmetry about the approach axis.

| object class | admissible? | why |
| --- | --- | --- |
| **sphere** | **yes, any orientation** | every diameter is a valid jaw line; no yaw exists to get wrong |
| **vertical cylinder** | **yes, any yaw** | circular cross-section, so the jaws close on a diameter whatever the yaw |
| **cube** | **only at the right yaw** | the jaws must face a parallel pair of faces. A cube repeats every 90°, so it is admissible when its yaw is within the jaw tolerance of 4.7° (left) / 168.1° (right) modulo 90° |
| **rectangular box** | **only at the right yaw** | repeats every 180°, so half as many admissible orientations as a cube |
| **horizontal cylinder** | **only at the right yaw** | must be gripped across its axis; admissible when its axis is perpendicular to the jaw axis |
| **irregular / thin plate** | **no** | needs an approach chosen from the object's own geometry, which nothing here does |

The clip set's own objects, against this:

| object | shape | admissible with the pinned wrist? |
| --- | --- | --- |
| task A block, 40 mm | cube | yes — placed at the jaw yaw by construction |
| task B part, 45×45×50 mm | box | yes — placed at the jaw yaw by construction |
| task C multimeter, 50×90×130 mm | box | yes — gripped across its 50 mm dimension |

They are admissible because the scene **places them** at an orientation the
gripper happens to hold. That is not the same as being able to grasp an object
found in the world, and no participant object should be assumed graspable
without checking its yaw against the table above.

## Which modes can command an approach

| # | mode | wrist commanded by | can set the approach? |
| --- | --- | --- | --- |
| 1 | DIRECT_MANNEQUIN | master arm | **no** — `orientation_mode: fixed`, and the master could not express it anyway: right j5/j7 dead, left j6 clamped, so wrist roll is unobservable |
| 2 | DIRECT_VR | Quest controller | **no** — the controller has 6-DOF and the mapper carries orientation, but the follower is pinned to the anchor |
| 3 | ORIENTATION_ASSIST | vision | **STUB** — this `/compute_ik` ignores `OrientationConstraint` entirely (measured: identical IK success with and without it) |
| 4 | SHARED_AUTONOMY | arbiter | **yes in principle** — the arbiter servos wrist orientation, but only between the anchor and a grasp orientation the grasp library supplies |
| 6 | FULL_AUTONOMY | executive | **yes in principle** — same path as 4 |

So modes 1 and 2 cannot vary the approach at all, mode 3 cannot despite its
name, and modes 4 and 6 can only choose among orientations that something
upstream proposes. **Nothing in any mode derives an approach from the object.**

## What would have to change

1. `orientation_mode` would need a per-object setting, which needs the
   `OrientationConstraint` path to work — it does not on this `/compute_ik`
   plugin, so it needs a constraint-aware plugin or explicit yaw sampling in
   the follower.
2. The grasp library would need to emit an approach from the object's minor
   axis rather than a fixed top-down pose.
3. For modes 1 and 2 the wrist channels would have to be repaired first
   (right j5/j7, left j6); until then the operator cannot express a wrist
   rotation regardless of what the follower would accept.

None of that is done. The honest description of the current system is: **a
fixed side approach at 120.8°/112.1°, with objects arranged to suit it.**
