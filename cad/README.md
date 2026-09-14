# CAD

Design files of the hardware built for the project.

| file | what it is | format |
| --- | --- | --- |
| `master_arm/master_arm_complete.f3z` | the complete redesigned master mannequin (stage 3): both arms, every potentiometer housing, the central column and stand, as one Fusion 360 archive with the full design history | Autodesk Fusion 360 archive |
| `master_arm/master_arm.step` | the same master arm exported as solid geometry for manufacture or import into any CAD package | STEP (ISO 10303) |
| `backpack/backpack.step` | the backpack frame that carries the two Kinova Gen3 arms | STEP |

The printable meshes derived from these, and the collision meshes the robot
description uses, are under `src/srl_description/meshes/`. The STEP files are
solid geometry only; the kinematic constants the software uses come from the
measured arm (`src/srl_teleop/srl_teleop/master_calibration.py`), not from
the CAD.
