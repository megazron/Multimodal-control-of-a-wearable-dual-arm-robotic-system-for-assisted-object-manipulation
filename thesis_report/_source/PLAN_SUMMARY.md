# PLANNING REPORT — STRUCTURED SUMMARY

*Source: `thesis_report/_source/planning_report.pdf`, 12 pages, 3838 words,
dated Thursday 23 April 2026. Author Gaus Sayyad (06053822); supervisors
Dr Dandan Zhang and Dr Etienne Burdet.*

Everything below is transcribed from the planning report. Where the report
contains an internal inconsistency it is flagged rather than silently
corrected.

---

## 1. Title and framing

**Title as submitted:** *Multimodal control of a wearable dual-arm robotic
system for assisted object manipulation.*

**Platform:** the MUlti-limb Virtual Environment (MUVE), known as the
Dr. Octopus platform at Imperial College London, mounting up to two
lightweight Kinova 7-DoF arms on a backpack.

**Core concept:** a small physical mannequin acts as the *master* device,
commanded directly by the fingers of one or two hands, and drives the
full-scale Kinova arms (the *slaves*). The report states that using a physical
mannequin makes the teleoperation system "intrinsically safe through direct
finger interaction".

**Declared focus:** "This project specifically focuses on the master mannequin
design and control mapping for the master-slave manipulator." Vision-based
object detection and gesture-based intent interfaces, completed previously by
the team, were to be integrated "where relevant".

---

## 2. The three-stage structure

| Stage | Content as planned |
| --- | --- |
| **Stage 1** | Physical prototyping with a figurine and *real springs* to determine optimal spring placement for intuitive force feedback; simulation setup of Kinova arm dynamics **in Isaac Lab** to validate virtual spring behaviour. |
| **Stage 2** | Complete simulation in Isaac Lab; design of the physical mannequin master hardware with real miniature arms; design of the master-to-slave control system for the dual Kinova arms. Stage 2 work to run **in parallel**. |
| **Stage 3** | Implementation of representative tasks (pick-and-place, synchronised multi-arm motion such as "dancing sequences") and their demonstration on the real Dr. Octopus platform. |

**Milestones as stated:** virtual spring parameters validated and basic master
mannequin plus control mapping complete by **end of May 2026**; operational
master--slave synchronisation on two arms by **end of June**; full dual-arm
teleoperation and task demonstrations by **end of July**; complete evaluation
by **end of August**.

**Fallback positions stated:** a simplified 2-DoF prototype per arm if
miniature arm fabrication is delayed; conservative scaling factors validated
on the physical figurine if real-time slave synchronisation proves unstable.

**Status at the time of writing:** Stage 1 declared complete; Stage 2
"now underway".

---

## 3. Aims and objectives, as written

**Aim.** To design, prototype and implement a master--slave teleoperation
system using a physical figurine mannequin with miniature arms controlled by
the fingers of one or two hands, enabling safe and efficient intuitive control
of up to two Kinova supernumerary robotic limbs on the Dr. Octopus platform.

**Objectives.**

1. Conduct physical prototyping with a figurine and real springs to determine
   optimal spring placement for intuitive force feedback.
2. Perform simulation of the Kinova arm dynamics **in Isaac Lab or MuJoCo** to
   validate virtual spring behaviour.
3. Design the physical mannequin master hardware **with real miniature
   motorised arms**.
4. Design the control mapping that commands the dual Kinova 7-DoF slave arms
   while ensuring stable master--slave synchronisation and haptic
   transparency.
5. Implement representative tasks — coordinated pick-and-place and
   synchronised multi-arm motions — and demonstrate them on the real
   Dr. Octopus platform.

Note objective 3 specifies **motorised** miniature arms, which implies
actuation at the master, not only sensing.

---

## 4. Ethical analysis (§2), for reuse in the thesis appendix

- Laboratory-based; robotic hardware development and testing only.
- No human subjects beyond voluntary supervised researcher demonstrations
  **and a small user study of 8--10 laboratory volunteers**, all giving
  written informed consent with the right to withdraw.
- No biological specimens, no personal data.
- Risks confined to minor mechanical hazards, mitigated by Kinova joint
  limits, emergency stops, impedance-based compliance and supervised
  operation.
- Hardware: components selected to eliminate sharp edges and pinch points.
- Electronics: low-voltage operation, hardware current limiting against motor
  overcurrent, secured enclosures.
- Software: Kortex API operated within manufacturer joint torque and velocity
  limits; impedance parameters validated in simulation before hardware;
  redundant software e-stop alongside physical buttons; **automatic velocity
  zeroing on communication loss between master and slave**.
- Environmental: Kinova hardware reused from laboratory inventory; miniature
  parts desktop 3D-printed with recyclable filament; electronics from existing
  stock.

A distinctive argument is made that the mannequin is itself a safety tool: by
handling the figurine the operator becomes aware of geometrically possible
contacts between the arms and the body, so "the prototyping process itself
[is] a safety validation tool".

---

## 5. Literature review — the sixteen references

All sixteen are real and are to be migrated verbatim into the bibliography.

| # | Reference |
| --- | --- |
| 1 | Tong Y, Liu J, et al. (2021) *Review of Research and Development of Supernumerary Robotic Limbs.* IEEE/CAA Journal of Automatica Sinica. |
| 2 | Imperial College London. *MUlti-limb Virtual Environment (aka Dr. Octopus).* https://www.imperial.ac.uk/a-z-research/human-robotics/dr-octopus-/ (accessed April 2026). |
| 3 | Kinova Robotics (2022) *Gen3 Ultra Lightweight Robot User Guide.* |
| 4 | Prattichizzo D, et al. (2021) *Human augmentation by wearable supernumerary robotic limbs: review and perspectives.* Prog Biomed Eng. |
| 5 | Yang B, Huang J, Chen X, Xiong C, Hasegawa Y. (2021) *Supernumerary Robotic Limbs: A Review and Future Outlook.* IEEE Trans. Medical Robotics and Bionics, 3(3):623--639. |
| 6 | Zheng Y, et al. (2026) *Supernumerary Robotic Limbs: A Quantitative Analysis and Review of Design, Embodiment, and Future Challenges.* Journal of Field Robotics. |
| 7 | Parietti F, Asada HH. (2016) *Supernumerary Robotic Limbs for Human Body Support.* IEEE Trans. Robotics, 32(2):301--311. |
| 8 | Zhang K, et al. (2023) *Review of Supernumerary Robotic Limbs.* Journal of Physics: Conference Series. |
| 9 | Toubar H, Goldsmith G, Tafrishi SA, Innala M, Bergstromer V. (2022) *Design, Modeling, and Control of a Series Elastic Actuator with Discretely Adjustable Stiffness.* Mechanism and Machine Theory, 170:104720. |
| 10 | de Wolde S, Kober J, Babuska R. (2024) *Current-Based Impedance Control for Interacting with Mobile Manipulators.* arXiv:2403.13079. |
| 11 | NVIDIA (2026) *Isaac Lab Documentation: Impedance Control and Motion Generators.* https://isaac-sim.github.io/IsaacLab (accessed April 2026). |
| 12 | Guggenheim JW, Asada HH. (2021) *Inherent Haptic Feedback From Supernumerary Robotic Limbs.* IEEE Trans. Haptics, 14(1):123--131. |
| 13 | Lee C, Kwak S, Kwak J. (2019) *Development, Analysis, and Control of Series Elastic Actuator for a Wearable Robotic Leg.* Frontiers in Neurorobotics, 13:17. |
| 14 | Pinardi M, Longo MR, Formica D, Strbac M, Mehring C, Burdet E, Di Pino G. (2023) *Impact of supplementary sensory feedback on the control and embodiment in human movement augmentation.* Communications Engineering, 2:64. |
| 15 | Zhou H, Gao A, Ducos P, Eden J, Burdet E. (2026) *Spatial Guidelines for Supernumerary Robotic Limbs in Near-Body Interactions.* CHI Conference on Human Factors in Computing Systems. ACM. |
| 16 | Moon C, Kim J. (2024) *Assessing the Physical Impact of Supernumerary Limbs on a Human Subject: A Simulation Study.* Proc. 46th EMBC, pp. 1--4. IEEE. |

**Note.** Reference 16 is listed but is **never cited in the body text** of the
planning report. Reference 15 is the CHI paper on near-body SRL spatial
guidelines, which is the same work this project's own engineering log refers
to as "SRL Proxemics" — the thesis should cite it consistently as [15].

### Gap statement, as argued in the report

"Few studies combine a custom miniature dual-arm master mannequin (with
physical springs) and direct teleoperation of a wearable dual-arm SRL platform
for coordinated manipulation tasks." Existing multi-arm SRL work either uses
gestural or VR interfaces with higher cognitive load, or single-arm
master--slave configurations that do not scale to dual-limb coordination.
The claimed distinguishing feature is **dual-layer compliance**: SEA-style
physical compliance at the miniature master together with virtual spring
impedance at the full-scale slave.

---

## 6. Table 1 — control strategies comparison

Eight approaches, to be reproduced with `booktabs` and extended.

| Approach | Control mechanism | Strengths | Limitations | Ref |
| --- | --- | --- | --- | --- |
| Direct master--slave mapping | Scaled miniature master physically commands full-scale slave joints via position mapping | Highly intuitive; preserves natural hand motion; low end-to-end latency (**~120 ms**); no cognitive abstraction layer | Geometric scaling must be precisely calibrated; kinematic dissimilarity introduces mapping errors | [12] |
| Gesture and VR-based teleoperation | Vision or inertial sensors capture hand/body gestures, mapped to slave joint commands | Hands-free; no physical master; adaptable across morphologies | Higher cognitive load without haptics; drift and occlusion; poor transparency during contact | [5] |
| Virtual springs via impedance control | Kortex API impedance mode emulates spring--damper dynamics at slave joint level | Hardware-native; no extra actuators; stiffness and damping tunable in software | Careful gain tuning needed to avoid instability; cannot fully replicate physical spring feel; latency-sensitive | [3] |
| Physical springs on miniature master | Coil springs at master joint locations give direct mechanical force reflection to fingertips | Passive, inherently safe, zero-latency; no electronics at master; encodes postural strain directly | Fabrication and calibration labour-intensive; stiffness fixed at manufacture; reflects gross posture, not fine contact | [9] |
| Simulation-in-the-loop tuning | Mapping and impedance parameters validated in Isaac Lab before hardware | Safe offline optimisation; reduces hardware damage risk; rapid gain iteration | Sim-to-real gap; tuned parameters need further adjustment on hardware | [11] |
| EMG-based intent recognition | Surface EMG decoded to infer intended arm motion | Intuitive biological signal; hands free; low learning curve when trained | Electrode placement variability, muscle fatigue, skin impedance; needs user-specific calibration | [4] |
| Shared autonomy with vision | Robot assists with low-level positioning; operator retains high-level intent | Reduces workload on repetitive or precise sub-tasks; improves success rate | Needs reliable detection; autonomous layer may conflict with operator intent | [5] |
| Spring-based SEA compliance at slave | SEA principles applied at Kinova joint level via software-defined virtual stiffness | Shock tolerance and passive safety; precise torque control; well-studied theory | Compliance reduces positioning bandwidth; stiffness--bandwidth trade-off non-trivial for multi-arm | [9] |

*Caption as written:* "Control strategies in supernumerary and teleoperated
robotic systems. Master-slave with miniature models and hybrid
physical/virtual springs aligns with SEA principles for the Dr. Octopus
platform [9, 13]."

---

## 7. Risk register R1--R10

| ID | Risk | Likelihood | Impact | Mitigation as planned |
| --- | --- | --- | --- | --- |
| R1 | Mismatch between miniature master kinematics and full-scale slave | Medium | High | Direct geometric scaling from figurine tests; simulation for validation |
| R2 | Latency or instability in master--slave mapping | Low | High | Conservative gains and Isaac Lab/MuJoCo tuning; Kinova safety limits |
| R3 | Hardware access delays for Dr. Octopus | Medium | Medium | Parallel simulation work; fallback to single-arm testing |
| R4 | Integration with team's vision/gesture modules | Low | High | Modular API; weekly team sync |
| R5 | Safety during dual-arm teleoperation demo | Low | High | Redundant e-stops; supervised testing only |
| R6 | Miniature arm motor or spring fabrication delays | Medium | Medium | Off-the-shelf servos and 3D-printed parts; early prototyping |
| R7 | Poor haptic transparency (force reflection mismatch) | Medium | High | Calibrate physical springs on figurine, transfer parameters to virtual springs |
| R8 | Real-time communication lag between master and slave | Low | High | High-speed Ethernet/ROS 2; fallback to wired direct connection |
| R9 | Overloading Kinova arms during multi-arm coordination | Low | High | Limit payload and velocity in software; monitor torque feedback |
| R10 | User finger fatigue or discomfort during long demos | Medium | Medium | Ergonomic handle design; short sessions with breaks |

---

## 8. Quantitative evaluation targets (§6)

These are the numbers the thesis must mark **met / not met / not measured**.

| # | Target | Threshold |
| --- | --- | --- |
| E1 | Master--slave tracking, positional error | **< 10 cm** |
| E2 | Master--slave tracking, angular error | **< 15°** |
| E3 | Force reflection error (physical springs + Kinova torque feedback) | **< 0.8 N** |
| E4 | Coordinated pick-and-place success rate | **> 80 %** |
| E5 | Average completion time per object | **< 45 s** |
| E6 | Synchronised motion phase lag across arms | **< 200 ms** |
| E7 | Closed-loop overshoot | **< 8 %** |
| E8 | End-to-end latency | **< 200 ms** |
| E9 | User study | **8--10 laboratory volunteers**, qualitative haptic intuitiveness and embodiment |

Method as planned: standardised household objects in a controlled laboratory
arena, comparing baseline position control against the full master--slave
system with haptic feedback, using simple statistics (mean and standard
deviation).

---

## 9. Spring configuration study (Stage 1 result)

For seven joints there are $\binom{7}{2} = 21$ unique pairwise connections.
Three configurations were evaluated on the figurine.

**Configuration A — full connectivity.** Retains a representative subset of
all geometrically feasible inter-joint connections, including non-adjacent
joints across the chain. Maximises force information richness, but introduces
**mechanical coupling ambiguity**: the contribution of individual joint
displacements to spring deformation cannot be cleanly isolated, so signal
attribution is unreliable.

**Configuration B — informative subset.** Retains a connection only if its
deformation is **dominated by the motion of a single joint** rather than
spread across several. This eliminates cross-coupled springs whose force
signals would be ambiguous at the fingertips, leaving each spring as a clean
proprioceptive channel for one dominant degree of freedom. The figurine also
carries a support spring separating the upper from the lower arm.

**Configuration C — optimal minimal set, and the one selected.** The minimal
set satisfying both **postural observability** and **signal separability**.
Each spring responds predominantly to one degree of freedom:

| Connection | What it encodes |
| --- | --- |
| **J5--J6** | elbow flexion state |
| **J3--J2** | wrist proximity to the base |
| **J4--J1** | overall arm extension relative to the base |

Together these three form "a minimal observability basis for the seven-DoF arm
posture", giving the operator directional feedback about which part of the arm
is under strain without requiring them to disentangle overlapping signals. The
report adds that Configuration C "also represents the virtual springs present
in Kinova Gen3 Arm", which is why it was preferred for the physical figurine.

**Selection criteria, in the report's own terms:** single-joint deflection
dominance; signal separability; postural observability.

---

## 10. Figure captions, and the numbering defect

**THE PLANNING REPORT'S FIGURE NUMBERING IS BROKEN. Do not inherit it.**

| As printed | Content | Defect |
| --- | --- | --- |
| "Figure 1" (p. 9) | Spring attachment configurations on the Kinova Gen3 7-DoF arm figurine (J1--J7): (A) full connectivity, (B) informative subset, (C) optimal configuration | — |
| "Figure 1" (p. 10) | Physical figurine model with springs, Configuration B | **Duplicate label.** Second figure also numbered 1 |
| "Figure 2" | — | **Referenced in the body ("In the Figure 2. The figurine also has a support spring") but never defined.** The sentence appears to mean the Configuration B photograph |
| "Figure 3" (p. 10) | Physical figurine model with springs, Configuration C | Numbering jumps; no Figure 2 exists |
| "Figure 4" (p. 11) | CAD model of the miniature master mannequin arm assembly from three perspectives: (A) back, (B) front, (C) isometric. A scaled-down kinematic replica of the Kinova Gen3 7-DoF arm on a symmetric cross-frame base, with coiled spring elements along the links | — |

**Proposed renumbering for the thesis:** the schematic of the three spring
configurations; the Configuration B photograph; the Configuration C
photograph; the CAD assembly (three views). Each needs its own `\label` and at
least one `\cref` from the body.

---

## 11. Preliminary results as claimed (§7)

- Stage 1 complete: optimal spring placement points identified for balanced
  compliance.
- Initial virtual spring models "are setup" in Isaac Lab "to show stable
  impedance behaviour for light payloads" — note the hedged phrasing; no
  numbers are given.
- Basic CAD model for the figurine with frame set up for slave position
  control.
- Stage 2 (simulation and design of the mannequin master and control mapping)
  underway.
- "Early tests with physical dummy model indicate promising force reflection
  for intuitive finger-based multi-arm coordination" — qualitative only, no
  measurement reported.

**No quantitative preliminary result appears anywhere in the planning
report.** Every number in §6 is a target, not an outcome.
