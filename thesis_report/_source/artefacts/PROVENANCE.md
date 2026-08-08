# ARTEFACT PROVENANCE

*Copied 2026-08-08 from the Windows side into `thesis_report/_source/artefacts/`.
**Originals were not modified.** Every row records the original path, the file's
own date, and what the artefact evidences.*

At Gate 1 I reported that no master-arm design artefacts existed. That was true
of the git repository and **false of the machine**. The design record was never
committed; it lives on the Desktop. This file exists so that the omission is
recorded rather than quietly repaired.

## Source roots

| Key | Original path |
| --- | --- |
| `S` | `/mnt/c/Users/Gausms/Desktop/MSc_Project/` |
| `D` | `/mnt/c/Users/Gausms/Downloads/` |

## Ingested artefacts

| Local path | Original | File date | Evidences |
| --- | --- | --- | --- |
| `basic_control/kinematics.cpp`, `.h` | `S/basic_control/` | — | **The verified forward kinematics.** `master_pose_node.py:5` cites this file by name as the source the Python was ported from. Primary evidence for the FK section of Chapter 3 |
| `Trajectories/*.csv` (12) | `S/Trajectories/` | **27 Jul 2026** | **The only real hardware data in the project.** Per-direction master recordings; see the analysis below |
| `CAD/PotArm.step` | `S/CAD/` | — | The master pot-arm assembly. Source for the kinematic-chain figure |
| `3dprint/J1..J7.stl` | `S/3dprint/` | — | The printed miniature link set — the physical kinematic chain |
| `3dprint/J7_left.stl`, `J7_right.stl`, `J7hold.stl`, `J7 LID.stl` | `S/3dprint/` | — | Wrist mounting and pot housing |
| `3dprint/ARMSTAND.stl`, `base.stl` | `S/3dprint/` | — | Master frame |
| `3dprint/backpackfinal.stl`, `backpacklid.stl` | `S/3dprint/` | — | Wearable frame |
| `3dprint/*.gcode.3mf` | `S/3dprint/` | — | Sliced print jobs; fabrication record with author name embedded |
| `BOM/Gaus_Sayyad_dococ{1,1.1,1.2,1.3,2,3,4,5}.pdf` (8) | `S/BOM/` | — | Bill of materials and procurement. Source for the components table |
| `mujoco/MUJOCO_LOG.TXT` | `S/` | **7 Jun 2026** | **Proves MuJoCo executed**, dating the simulation work and evidencing the Isaac Lab → MuJoCo divergence |
| `mujoco/two_arms.xml` | `S/` | — | Dual-arm simulation scene |
| `downloads/WH148 PH1 Single-Joint Potentiometer.f3d` | `D/` | — | **Identifies the sensing component**: WH148 PH1 single-turn potentiometer |

## Located but NOT ingested (reporting paths only, as instructed)

| Path | Why it matters |
| --- | --- |
| **`/mnt/c/Users/Gausms/Downloads/teensy_final.ino`** | **The k1j1 firmware.** Contains `k1j`/`K1IMU`/`fsr1` tokens |
| **`/mnt/c/Users/Gausms/Downloads/teensy_merged.ino`** | Second firmware revision, same protocol |
| `/mnt/c/Users/Gausms/Downloads/Dr_Octopus.STEP`, `Dr_Octopus (1).STEP` | Platform CAD |
| `/mnt/c/Users/Gausms/Downloads/Force Sensor.SLDPRT` | Force-sensor part |
| `/mnt/c/Users/Gausms/Desktop/MSc_Project/mujoco_menagerie/kinova_gen3/` | Kinova MuJoCo model |
| `/mnt/c/Users/Gausms/Desktop/MSc_Project/{two_arms,singlekinova,srl_teleop,recordval,serialcommand}.py` | Precursor host scripts |

**Gate 1's conclusion that the firmware was lost was wrong.** It is not in the
Arduino sketchbook (`Documents/Arduino/libraries/`, whose six sketches belong to
an unrelated project and contain none of the protocol tokens) but in
`Downloads/`. Ingesting it would supply the pin map, the ADC resolution and the
sampling regime — the three items Chapter 3 currently cannot state.
