# Literature Review

**Project:** dual Kinova Gen3 supernumerary arms on a wearable backpack, worn by one
person (the **WEARER**) and teleoperated by a **DIFFERENT** person (the **OPERATOR**)
from an instrumented mannequin master arm (potentiometers + IMU, Teensy 4.1, ROS 2
Jazzy).

> **REFRAMED 2026-08-08 — TWO PEOPLE, NOT ONE.** Sections 1-6 below were written for a
> single person who both wore and drove the arms. That framing is superseded. Where a
> section says "the operator" and means the wearer, read it as describing the
> *single-person* literature, which remains the correct background for the control and
> intent-inference work but is **not** the architecture this project instantiates. The
> two-person literature, the corrected gap statement and the revised novelty claim are
> in **Section 7**, which takes precedence over Section 6 wherever they disagree.

**Scope and method.** Citations below were located by web search and, where possible,
verified against the publisher record or the paper text itself. Claims I could not
verify are marked **[UNVERIFIED]**. Where this project's own measurements are quoted
they are taken from `CLAUDE.md` in this repository and are labelled as such; they are
engineering measurements from a single rig, not published results.

A caveat on the search itself: this was a web search, not an exhaustive query of
Scopus/Web of Science/IEEE Xplore, and it was English-language only. Section 6's
novelty claim should be read with that limitation in mind.

---

## 1. Shared autonomy in teleoperation

### 1.1 The predict-then-blend formalism

The dominant formalism is **policy blending**. Dragan and Srinivasa (2012, 2013)
proposed that essentially every assistive teleoperation method can be written as an
arbitration between two policies: the user's input policy `U` and a robot policy `P`
derived from a prediction of the user's goal. The contribution is unifying — virtual
fixtures, potential fields, autonomous grasp completion and goal-directed planning all
become special cases differing only in how `P` is computed and how the arbitration
function weights the two.

Prediction in this line of work is grounded in inverse optimal control: the user is
modelled as a noisily-rational agent minimising a cost, and the posterior over goals
is obtained by inverting that model (Dragan and Srinivasa, 2013; Ziebart et al., 2008).

### 1.2 Beyond predict-then-act

Javdani et al. (2015, 2018) identified a structural weakness in predict-then-act: if
the system waits until it is confident about a single goal, it often cannot become
confident until the user has nearly reached the goal, so assistance arrives too late to
help. Their alternative formulates shared autonomy as a POMDP with the goal as hidden
state and assists by minimising expected cost-to-go **under goal uncertainty**, solved
approximately by hindsight optimization. Their abstract reports that, compared with
predict-then-act methods, the approach "achieves goals faster, requires less user input,
decreases user idling time, and results in fewer user-robot collisions" (Javdani et al.,
2018).

### 1.3 Customisation and human-in-the-loop optimisation

Gopinath, Jain and Argall (2017) argued that the arbitration function should not be
fixed by the designer at all. They formalised customisation of shared autonomy as a
non-linear optimisation whose cost function is left indeterminate and is optimised *by
the end-user*, interactively. Their pilot study ran 17 subjects (4 with spinal cord
injury, 13 without). This matters for section 6 below: it is direct evidence that the
"right" level of assistance is a per-person quantity rather than a system constant.

Jain et al. (2015) coupled shared autonomy to a body-machine interface, mapping residual
body motion of users with motor impairment into a low-dimensional control signal and
letting autonomy make up the difference — an early instance of the "autonomy supplies
what the interface cannot" pattern that is central to this project.

### 1.4 Degraded input channels: BCI

Muelling et al. (2017) address the case where the input device itself is the bottleneck.
Their "autonomy-infused teleoperation" framework targets brain-computer-interface control
of an assistive arm, where commands are low-dimensional, noisy and erratic; the system
combines computer vision, intent inference, and arbitration with adjustable assistance
levels. This is the closest classical analogue to the present project's situation: the
input device cannot express the full task space, and autonomy fills in.

### 1.5 Taxonomy: traded vs shared, blending vs handover

- **Levels of automation.** Sheridan and Verplank (1978) laid out a ten-point scale from
  fully manual to fully autonomous, with intermediate levels where the computer narrows
  options, suggests, or acts subject to veto. Beer, Fisk and Rogers (2014) revisited this
  for HRI, proposing a ten-level LORA taxonomy and — importantly for experimental design —
  arguing that autonomy level must be reported alongside HRI variables such as acceptance,
  situation awareness and reliability, rather than treated as a scalar knob.
- **Traded control** hands the whole task back and forth between operator and autonomy
  (a discrete handover). **Shared control** runs both simultaneously and combines their
  outputs continuously. Policy blending (Dragan and Srinivasa, 2013) is the general form
  of the latter; the arbitration function's shape is what distinguishes "timid"
  (assistance grows with confidence but plateaus, never taking over) from "aggressive"
  (assistance eagerly takes charge).
- **Mode switching** is a third, often-ignored cost. Herlant, Holladay and Srinivasa
  (2016) found that on a Kinova JACO arm driven by a low-DOF interface, mode switching
  consumed about **17.4% of execution time even for able-bodied users**, and that
  automating the switch significantly improved user satisfaction. Any comparison of
  "direct teleoperation vs shared autonomy" that ignores mode-switch overhead is
  measuring the wrong baseline.

### 1.6 When blending helps and when it hurts — the empirical picture

This is the part of the literature most often misquoted, so the verified findings are
worth stating precisely.

Dragan and Srinivasa (2012) ran a within-subjects study, 8 participants (4 male,
4 female), balanced Latin square over task order and mode order, crossing three factors:
arbitration aggressiveness (timid vs aggressive), prediction correctness (right vs wrong),
and task difficulty (easy vs hard). Dependent measures were completion time and a 7-point
forced-choice preference between the two modes. Verified findings from the paper text:

- Aggressive assistance performs better on **hard** tasks **when the robot is right**;
  timid assistance performs better on **easy** tasks **when the robot is wrong**.
- "Our data indicates that the aggressive mode is overall more efficient if the robot is
  wrong in less than 16% of the cases." Above that error rate, aggressive blending is a
  net loss.
- "Being aggressive and wrong results in large penalties in time and user preference."
- Preference correlated with timing (Pearson's r(30) = .66, p < .001) — but imperfectly.
  "Some users prefer the timid mode despite it being slightly less efficient",
  "motivating that they felt more in control of the robot."
- Some users followed the robot's motion while it was aggressive **without realising they
  were not in control**, because the predicted policy happened to match their intent.
- The paper explicitly reconciles two contradictory prior results: You and Hauser (2011)
  found users preferred a fully autonomous point-and-click mode over more reactive
  assistance for a hard motion-planning problem; Kim et al. (2012) found users preferred
  a **manual** mode over an autonomous one for object grasping. Dragan and Srinivasa's
  account is that these differ in task difficulty — assistance wins when autonomy is
  substantially more efficient, opinions split when it is not.

Two further verified results sharpen this:

- You and Hauser (2011), 22 novice subjects across five strategy subgroups: obstacle
  avoidance strategies gave "major safety improvements, although subjects felt noticeably
  less in control of the robot than those using the baseline methods." Safety and
  perceived control moved in opposite directions.
- Kim et al. (2012), spinal-cord-injured subjects over three weeks: at study end,
  autonomous and manual modes had comparable completion times, user effort in autonomous
  mode was significantly lower, "however, the autonomous mode failed to commensurately
  raise the user's level of satisfaction." Manual-mode users showed a pronounced learning
  effect; auto-mode users mainly reduced variability.
- Collier, Narayan and Admoni (2025) measured sense of agency directly and found that
  "higher robot autonomy during assistance leads to improved task performance but a
  decreased sense of agency" — an explicit performance/agency trade-off. They also note
  that when users were free to customise the arbitration function they chose functions
  that were **not** performance-optimal, even after having experienced the optimal one.
  (Participant count for their own study is reported in the paper; I could only verify a
  24-participant figure indirectly and so do not quote it. **[UNVERIFIED: n]**)

**Summary for this project.** Blending helps when (a) the task is genuinely hard for the
operator through the given interface, and (b) prediction is right well over ~84% of the
time. It hurts when prediction is wrong, and it can cost subjective control even when it
wins objectively. Any autonomy this project adds must therefore be evaluated on *both*
performance and agency, and its prediction accuracy must be characterised, not assumed.

---

## 2. Supernumerary robotic limbs (SRL)

### 2.1 Origins: bracing and body support

The SRL concept in its modern form comes from Asada's d'Arbeloff Laboratory at MIT.
Parietti and Asada (2014) built shoulder-mounted limbs for aircraft fuselage assembly
whose function was **bracing** — stabilising the wearer's body and guiding the hands by
placing a drill jig, rather than manipulating objects independently. Parietti and Asada
(2016) extended this to whole-body support. Bright and Asada (2017) applied SRLs to
overhead assembly.

The design logic is worth noting because it recurs throughout the field: the earliest
successful SRLs did *not* require the wearer to teleoperate them continuously. They either
acted autonomously against a physical constraint (bracing) or were commanded episodically.
Continuous, dexterous, operator-driven control of a supernumerary limb is the harder and
less-solved problem.

### 2.2 Supernumerary fingers

Prattichizzo, Malvezzi, Hussain and Salvietti (2014) introduced the "Sixth-Finger", a
modular extra finger mounted on the wrist opposite the palm, enlarging the hand's
workspace and grasp capability. Its control is an **object-based mapping**: the whole
hand's motion during a grasp is interpreted and mapped to the extra finger, i.e. the
device is commanded implicitly by the natural limb rather than by a dedicated input
channel. The same group later applied it as a compensatory tool for paretic hands.

### 2.3 How SRLs are commanded

The review by Prattichizzo et al. (2021) surveys the field. Verified quotes:

- "From the control point of view, it is fundamental to find the right trade-off between
  the degrees of freedom that are under the direct control of the user and the level of
  robot autonomy."
- On early use: "Pilot tests ... showed that controlling the device demands high
  coordination and cognitive load at the beginning."
- On hardware: "achieving a trade-off between weight and dexterity is still an open
  challenge", particularly for supernumerary arms and legs.

The first of these is, essentially, the research question of this project stated by the
field's own review as an open problem.

Control modalities reported across the literature:

| modality | example | what it gives you |
|---|---|---|
| sEMG | EMG interface for a supernumerary finger's motion and compliance (see Prattichizzo et al., 2021) | low-DOF, placeable anywhere, intuitive for fingers |
| BCI / EEG | Penaloza and Nishio (2018) | very low DOF, hands-free, high error rate |
| Feet | MetaLimbs (Sasaki et al., 2017); MetaArms (Saraiji et al., 2018) | high DOF, but consumes the legs |
| Gaze | gaze-signal control of SRLs (see the surveys) | pointing only; no depth, no orientation |
| Mouth / tongue | Jing et al. (2025) | 14 discrete operations mapped onto 6 DOF |
| Body redundancy | Lisini Baldi et al. (2025) | uses motor redundancy of the natural limbs, so the natural task is not blocked |
| Natural-limb mapping | Sixth-Finger (Prattichizzo et al., 2014) | zero extra input channel, but no independent control |
| Task-level autonomy | Parietti and Asada (2014, 2016) | no operator input at all during the motion |

Note what is missing from that table: **no entry provides a full 6-DOF, wearable,
hands-free pose measurement.** Feet and body-redundancy interfaces come closest on DOF
count but consume a body resource. This is the structural constraint argued in
`02_baseline_and_hypotheses.md`.

Two design-side reviews cover the hardware space: Li et al. (2023) systematically review
wearable extra robotic limbs, and Vatsal and Hoffman (2018) present a wearable robotic
forearm whose kinematic analysis claims a 246% increase in reachable workspace over
natural human reach, for scenarios including "fetching objects while the human's hands are
occupied" — an explicitly divided-attention use case, though the paper is a design and
kinematics contribution rather than a divided-attention study.

### 2.4 Body schema and embodiment of extra limbs

The question of whether a limb that was never part of the body can be incorporated into
the body schema has a real experimental literature.

- Guterstam, Petkova and Ehrsson (2011) showed a **supernumerary** hand illusion: a rubber
  right hand placed beside the participant's real right hand, stroked synchronously, is
  experienced as a third limb belonging to the body. Supported by questionnaire data plus
  skin-conductance responses to threat. Critically, this is *addition*, not the classic
  rubber-hand *substitution*.
- Kieliba, Clode, Maimon-Mor and Makin (2021) trained able-bodied participants on the
  "Third Thumb" over 5 days of lab-based and unstructured daily use. Motor augmentation
  was readily achieved, with "reduced cognitive reliance, and increased sense of
  embodiment" — but also measurable changes to the neural representation of the biological
  hand. Augmentation is not free.
- Dominijanni et al. (2021) frame the general problem as **neural resource allocation**:
  how to channel motor commands and sensory information to and from an augmentative device
  "without hindering the motor control of biological limbs". This is the theoretical
  statement of the divided-attention problem in section 3 of the experimental design.
- Zhou et al. (2025) ran an exploratory VR study (N = 14) on control strategies for
  supernumerary multi-arms with varying autonomy, using a Wizard-of-Oz to simulate
  semi-autonomous limbs. Participants adapted their strategy to task complexity and
  autonomy level, and **experienced shifts in body ownership depending on autonomy level**.
  Their design guidelines are commands, demonstration, delegation and labelling.

**What is known, then:** extra limbs can be embodied; embodiment interacts with autonomy
level; and control of an extra limb competes with control of the biological body for
neural resources. What is *not* established is a quantitative model of how much
independent control capacity a wearer has to spare.

---

## 3. Intent inference from operator motion

### 3.1 Goal prediction from partial trajectories

The standard machinery is maximum-entropy inverse optimal control. Ziebart et al. (2008)
introduced MaxEnt IRL, giving a globally normalised distribution over decision sequences;
Ziebart et al. (2009) applied it to predicting pedestrian goals from partial trajectories,
with the property that the learned cost function generalises to new environments. Javdani
et al. (2018) use exactly this as the observation model over user goals.

The Bayesian-cognitive counterpart is Baker, Saxe and Tenenbaum (2009): action
understanding as **inverse planning**, inferring mental states by inverting a model of
approximately rational planning. Same mathematics, different framing — and it supplies the
justification for treating the operator as an optimiser whose objective can be recovered.

A useful non-IRL alternative: Pérez-D'Arpino and Shah (2015) predicted the target of a
human reaching motion in real time by time-series classification over multiple
demonstrations, rather than by inverse planning. For a project with a small, fixed set of
candidate targets and plenty of recorded trajectories, this is often the cheaper route.

### 3.2 Gaze and pointing as intent cues

Gaze is a genuinely anticipatory signal. Johansson, Westling, Bäckström and Flanagan
(2001) showed in object manipulation that gaze "supports hand movement planning by marking
key positions to which the fingertips or grasped object are subsequently directed" — that
is, the eye reaches the landmark before the hand does.

Admoni and Srinivasa (2016) added eye gaze as an observation to Javdani's POMDP
formulation, updating the goal posterior from both gaze and joystick commands.

The most directly relevant result — and it is a **negative** one for anyone hoping gaze or
pointing alone can drive intent inference — is Aronson and Admoni (2022). Verified
findings: when gaze provides a prediction earlier in the task, assistance improves;
**however, "gaze on its own is unreliable and assistance using only gaze performs
poorly"**; control input and natural gaze "serve different and complementary roles in goal
prediction, and using them together leads to improved assistance." They also note that
modal (mode-switched) control reduces the efficiency of assistance under their model.

The HARMONIC dataset (Newman et al., 2022) is the resource for anyone wanting to
re-analyse this: 24 people performing an assistive-eating task with a 6-DOF arm, with
both eyes on video, egocentric video, joystick commands, forearm EMG, third-person stereo
video, and robot joint positions.

### 3.3 What is actually known about pointing direction vs end-effector velocity

Stated plainly, because it is easy to overclaim here:

- **End-effector velocity / control input is the well-validated cue.** Every major shared
  autonomy system (Dragan and Srinivasa, 2013; Javdani et al., 2018; Muelling et al.,
  2017) predicts the goal from the *history of user commands*, i.e. from the direction the
  operator is driving the end-effector. The MaxEnt IOC likelihood is defined over that
  command sequence.
- **Gaze is earlier but noisier.** Johansson et al. (2001) establishes the temporal
  advantage physiologically; Aronson and Admoni (2022) establishes empirically that the
  advantage does not survive being used alone, and that the two cues are complementary.
- **I found no study that isolates "pointing direction" — as distinct from gaze, and as
  distinct from end-effector velocity — and compares it head-to-head against
  end-effector velocity as an intent signal for shared autonomy.** If one exists it did
  not surface in these searches. Treat any claim that pointing beats velocity as
  unestablished. **[UNVERIFIED — absence of evidence, not evidence of absence]**

This matters to the present project because its master arm measures a *shoulder
direction*, not an end-effector command, and because its lateral (azimuth) channel is
known-bad (see `CLAUDE.md`: azimuth is derived from j1 alone; Kabsch fit of measured to
intended sweep directions leaves 54.8° RMS residual on the left arm and 54.7° on the
right; intended left-right and forward-back directions are 90° apart but measured 137.7°
apart, so no rotation can reconcile them). A pointing-direction intent cue on this rig
would inherit that error. An end-effector-velocity cue computed on the *commanded* pose
would inherit it too, but only through the axes that are actually good — up/down and
fore/aft track correctly on both arms.

---

## 4. Fitts' law applied to teleoperation

### 4.1 The model

Fitts (1954) proposed that the time to make an aimed movement is a linear function of an
information-theoretic index of difficulty, with the rate of performance approximately
constant over a wide range of amplitudes and tolerances. MacKenzie (1992) supplied the
now-standard **Shannon formulation**:

```
ID = log2(A/W + 1)        [bits]
MT = a + b * ID           [s]
TP = ID / MT              [bits/s]
```

where `A` is movement amplitude and `W` target width. The Shannon form gives better
correlations than Fitts' or Welford's original and cannot produce a negative ID
(MacKenzie, 2018).

### 4.2 The ISO methodology

Soukoreff and MacKenzie (2004) set out the recommended practice that underlies
**ISO 9241-9** (later ISO/TS 9241-411). Two points from it matter operationally:

1. Throughput should be computed from the **effective** index of difficulty, using the
   observed spread of endpoints (`We` from the standard deviation of endpoint positions)
   rather than the nominal target width. This normalises for speed-accuracy trade-off:
   an operator who goes fast and misses does not get credit for the speed.
2. Throughput is reported per participant per condition and then averaged, not computed
   from pooled means.

### 4.3 Reported throughput values, and why cross-study comparison is weak

Verified values:

| interface | throughput | source |
|---|---|---|
| Mouse, ISO 9241-9 studies | **3.7 – 4.9 bits/s** (range over studies) | Soukoreff and MacKenzie (2004), Table 5, as reported in MacKenzie (2018) |
| XR head-mounted displays | **median 3.25 bits/s** (median MT 1.25 s, median error rate 11.45%) | Amini et al. (2025), meta-analysis of 122 studies |
| Stereo displays | **median 3.20 bits/s** (median MT 1.32 s, error rate 3.20%) | Amini et al. (2025) |

For teleoperation specifically I could not find a comparable normative range. What I did
find:

- Pan, Eden, Oetomo and Johal (2025) propose Fitts' law explicitly as a benchmark for
  *assisted* human-robot performance, with 24 participants doing a target-reaching task
  on a Franka Emika Research 3. Their reported contribution is a model of performance as
  a function of task difficulty and autonomy level. **The abstract does not report
  throughput in bits/s** — they use the adapted Fitts model to estimate cognitive load
  indirectly. So even the most on-point teleoperation Fitts paper does not give a
  throughput number to compare against. **[UNVERIFIED: whether bits/s appears in the full
  paper]**
- Reported studies of bimanual robotic-arm control find that "robotic arm movements
  observe Fitts' law for reaching in depth but deviate for lateral and concentric
  movements" — i.e. the model's fit is *anisotropic* for arm-like devices.
  **[UNVERIFIED: I could not identify the specific study behind this summary and do not
  cite it below.]**

**The honest position on cross-study comparison.** Amini et al. (2025) reviewed 119
publications / 122 studies applying Fitts' law in 3D XR and found that "over half of these
studies referenced Fitts' law without thoroughly investigating throughput, movement time,
or error rate", that reporting of the index of difficulty is inconsistent (71% report an
ID range, 49% report exact IDs), and that 26.1% of studies presenting a Fitts regression
omit the regression coefficient. Their participant pools average 18 people, mean age 28,
and are about two-thirds male. If that is the state of reporting in a comparatively
well-standardised sub-field, then comparing a bespoke wearable teleoperation rig against
published mouse or XR numbers is at best an order-of-magnitude sanity check, never a
statistical comparison. Throughput's value here is as a **within-study, within-subject**
normaliser across conditions, plus a single externally-legible number that lets a reader
locate this interface on a scale they already understand.

---

## 5. VLA-based manipulation, and why this project does not use one

### 5.1 The lineage

- **RT-2** (Brohan et al., 2023) co-fine-tunes a large vision-language model on both
  robot trajectory data and internet vision-language tasks, expressing robot actions as
  text tokens. It demonstrated that web-scale semantic knowledge transfers into
  manipulation policies.
- **OpenVLA** (Kim et al., 2024) is the open 7B-parameter counterpart: a Llama-2 backbone
  with a fused DINOv2 + SigLIP visual encoder, trained on **970k real-world robot
  demonstrations** from the Open X-Embodiment dataset. It outperforms RT-2-X (55B) by
  16.5% absolute success across 29 tasks with 7x fewer parameters.
- **DexVLA** (Wen et al., 2025) adds a ~1B-parameter diffusion action expert and an
  embodiment curriculum (cross-embodiment pre-training → embodiment alignment → task
  post-training), reporting adaptation to single-arm, bimanual and dexterous-hand
  embodiments and long-horizon tasks such as laundry folding from language prompts.

### 5.2 What they demonstrate, and what they do not

Verified from the OpenVLA paper itself:

- **Reliability.** "it does not yet offer very high reliability on the tested tasks,
  typically achieving <90% success rate."
- **Latency.** OpenVLA "runs at approximately 6Hz on one NVIDIA RTX 4090 GPU (without
  compilation, speculative decoding, or other inference speed-up tricks)", and the authors
  state that improving throughput "is critical to enable VLA control for high-frequency
  control setups such as ALOHA, which runs at 50Hz."
- **Data.** 970k demonstrations, aggregated across many labs and embodiments.

Follow-up work (OpenVLA-OFT, Kim et al., 2025) targets exactly this, reportedly reaching
77.9 Hz with optimisation. **[UNVERIFIED: the 77.9 Hz figure came from a secondary
summary, not from the paper text.]**

### 5.3 Why this project deliberately does not use a VLA

The reasons are concrete and specific to this rig, not ideological:

1. **The research question is not "can a policy do the task".** It is "how much of a
   DOF-deficient operator input can autonomy restore, and does that survive divided
   attention". A VLA replaces the operator; it does not measure what the operator could
   not supply. Substituting an end-to-end policy would delete the independent variable.
2. **Data.** 970k demonstrations is the scale at which VLAs work. This project has
   recordings numbered in tens of thousands of *frames* on a single rig with known dead
   sensor channels — three orders of magnitude short, on non-stationary hardware.
3. **Latency budget.** The real-arm command path here is already latency-bound: the
   Kortex cyclic path is unusable over WSL, and the high-level `SendJointSpeedsCommand`
   bridge achieves **18.4–18.7 Hz with ~26 ms average send latency** (`CLAUDE.md`).
   Inserting a ~6 Hz / ~160 ms policy in front of that produces a control loop slower
   than the operator's own corrective bandwidth.
4. **Reliability floor.** <90% task success is acceptable for a benchmark and
   unacceptable for a 7-DOF arm mounted 0.126 m from the wearer's head — the tightest
   measured clearance in this system (`CLAUDE.md`). The autonomy in this project must be
   auditable: a goal posterior and an arbitration weight can be logged, inspected and
   bounded; a 7B token-prediction policy's action cannot.
5. **Reproducibility of the claim.** A VLA result would be a claim about the policy's
   training data. A shared-autonomy result is a claim about the human-machine system,
   which is what the thesis is about.

This is a scoping decision, not a criticism of VLAs. The honest statement is: VLAs
demonstrate broad semantic generalisation at demonstrated success rates below 90% and
control rates around 6–10 Hz for open 7B models, and neither of those is compatible with
the measurement this project needs to make.

---

## 6. The intersection

**The question.** Has anyone studied shared autonomy for *supernumerary* limbs, where the
autonomy specifically supplies the degrees of freedom that the input device physically
cannot measure, evaluated under *divided attention* (the operator's own body concurrently
performing a different task)?

**The answer, honestly: no single piece of work covers all three, but this is a narrow gap
between well-populated neighbours, not an empty field.** Every pair of the three is
covered. Below is what I found, and precisely what each does and does not do.

### 6.1 Nearest neighbours

**(a) Jing et al. (2025) — SRL + shared control + genuinely low-DOF input.**
A 6-DOF wearable robotic arm (9.1 kg for the SRL, 9.82 kg total) commanded by a
mouth-and-tongue interface: Hall sensors plus an air-pressure sensor detect tongue stick
manipulation and inhalation/exhalation, yielding 14 discrete operations classified by a
random forest (98.57% test accuracy) and mapped onto 6-DOF motion. Autonomy supplies
automatic hole alignment via HoloLens 2 recognition. Ten subjects, pin-hole assembly,
three conditions (shared / autonomous / voluntary). Results: 100% success for shared and
voluntary, 48% for autonomous-only; autonomous fastest, voluntary slowest; user experience
best for shared (4.5/5).
*Does not cover:* divided attention — "subjects focused exclusively on assembly", no
concurrent primary task. Also, the interface is *slow*, not *dimensionally deficient* —
it can express all 6 DOF, one discrete operation at a time. The autonomy compensates for
bandwidth, not for missing dimensions.

**(b) Zhou et al. (2025), CHI — SRL + autonomy level + attention management.**
N = 14, virtual supernumerary multi-arms in VR, Wizard-of-Oz semi-autonomy at varying
levels. Participants adapted control strategies to task complexity and autonomy; task
delegation, coordination and body ownership all shifted with autonomy level; the paper
explicitly calls for "adaptive control mechanisms that help users balance attention,
streamline task-switching".
*Does not cover:* a real robot, a real input device, or DOF deficiency. The autonomy is
simulated by an experimenter, so it has no prediction accuracy to characterise. This is
the closest work on the *attention* axis and the furthest from the *DOF* axis.

**(c) Penaloza and Nishio (2018), Science Robotics — SRL + true divided attention +
extremely DOF-deficient input.**
15 healthy volunteers; a non-invasive EEG BMI commands a humanoid robotic arm to grasp a
bottle **while the participant's own two hands balance a ball on a board**. Good
performers multitasked ~85% of the time, poor performers ~52%.
*Does not cover:* shared autonomy in the arbitration sense. The BMI signal is essentially
a binary intent trigger and the arm's grasp is a fixed pre-programmed motion — the
autonomy is *all* of the DOFs, not a blend, and there is no goal posterior, no arbitration
function, and no measurement of how much control the operator recovered. It is the
strongest existing demonstration that divided-attention SRL control is possible, and it
sets the ceiling condition (near-zero operator DOF) rather than the interesting middle.

**(d) Losey et al. (2020) / You and Hauser (2011) / Herlant et al. (2016) / Jain et al.
(2015) — autonomy supplying the DOFs a low-DOF interface cannot.**
This is the closest cluster on the *DOF* axis. Losey et al. explicitly frame the problem
as "teleoperating a 7-DoF robot arm with a 2-DoF joystick" and embed high-dimensional
robot actions into a learned low-dimensional latent space. You and Hauser drive a robot
arm from 2D input. Herlant et al. automate the mode switch that a low-DOF interface forces.
Jain et al. map residual body motion into a low-dimensional signal and let shared autonomy
complete the task.
*Does not cover:* supernumerary/wearable limbs, or divided attention. All four are
desk-mounted or wheelchair-mounted assistive arms operated as the user's sole task, by
users for whom the arm *replaces* rather than *supplements* function.

**(e) Muelling et al. (2017) — degraded input + arbitration + adjustable assistance.**
The BCI-driven assistive arm case; autonomy compensates for low-dimensional, noisy,
erratic commands, with adjustable assistance to balance capability against perceived
control authority. Structurally the closest formalism to what this project needs.
*Does not cover:* wearable/supernumerary mounting, divided attention.

**(f) Song and Asada (2023) — SRL + principled control allocation.**
Assigns predictable phases of a bimanual eating task to robot reactive control and
unpredictable ones to human voluntary control, using Lipschitz-quotient predictability.
*Does not cover:* DOF deficiency (not addressed in the paper) or divided attention (the
application is a bimanual eating task with no stated secondary cognitive demand).

**(g) Pan et al. (2024, 2025) — autonomy level + cognitive load + Fitts.**
RA-L 2024: N = 24, teleoperated trajectory tracking; autonomy level influences perceived
cognitive load and trust, with no clear interaction between them. HRI 2025: Fitts' law as
a benchmark for assisted human-robot performance, N = 24, Franka Emika Research 3.
*Does not cover:* SRL, wearable input, or DOF deficiency. This is the methodological
nearest neighbour for E1 and E2 — it is the template for how to run the autonomy-level
experiment, not a result about supernumerary limbs.

**(h) Prattichizzo et al. (2021), the field's own review.**
States the DOF/autonomy trade-off as an open control question for SRLs, in exactly those
words. This is the strongest evidence that the gap is real: the review that surveys the
whole field names the question and does not cite an answer.

**(i) Zhou et al. (2026), SRL Proxemics; Mete et al. (2026), SRL reconfiguration.**
Recent SRL work that touches autonomy: SRL Proxemics finds that "perceived safety hinges
on spatially calibrated, legible behaviors, not higher autonomy" (Wizard-of-Oz, near-body
interaction); the reconfiguration paper treats autonomy selection as part of a design
space alongside placement and morphology. Neither addresses DOF-deficient input or
divided attention as an experimental variable.

### 6.2 What is left

Combining the above, the specific claims that remain unmade in the literature I could find:

1. **A quantitative measurement of DOF recovery.** No study I found measures "the input
   device cannot observe DOF *k*; here is the task success / accuracy with and without
   autonomy supplying *k*, on the same operator, same hardware, same trial structure."
   The nearest is Jing et al. (2025), which compares shared vs voluntary vs autonomous but
   where the interface is not dimensionally deficient. Losey et al. (2020) address the
   dimensional deficit but on a non-wearable arm and without an ablation isolating which
   DOFs the latent action restored.
2. **DOF recovery *under divided attention*.** Penaloza and Nishio (2018) prove
   divided-attention SRL control is achievable at the extreme low-DOF end; Zhou et al.
   (2025) show autonomy level changes attention allocation in VR. Nobody, as far as I
   found, crosses "how much input DOF the operator has" with "whether their body is busy"
   as a factorial design on real hardware.
3. **Fitts throughput for a wearable supernumerary master.** I found no bits/s figure for
   a body-mounted SRL input device. Amini et al. (2025) gives XR medians and Soukoreff and
   MacKenzie (2004) gives mouse ranges; the SRL literature reports task success and
   completion time, not throughput.

### 6.3 What is *not* novel, and should not be claimed

To be explicit, so that the thesis does not overreach:

- **Shared autonomy for low-DOF interfaces is not novel.** It is a mature line running from
  Dragan and Srinivasa (2012) through Javdani et al. (2018) to Losey et al. (2020).
- **Applying shared autonomy to an SRL is not novel.** Jing et al. (2025) and Song and
  Asada (2023) both do it.
- **Studying SRL control under a concurrent task is not novel.** Penaloza and Nishio
  (2018) did it in Science Robotics.
- **Using Fitts' law to benchmark teleoperation with assistance is not novel.** Pan et al.
  (2025) did precisely that.
- **The finding that autonomy trades performance against felt control is not novel** and
  should be cited, not rediscovered: Dragan and Srinivasa (2012); You and Hauser (2011);
  Kim et al. (2012); Collier et al. (2025).

What is defensible is the *conjunction*, and the specific mechanism: autonomy supplying
the dimensions a **wearable, hands-free, body-mounted** master physically cannot measure,
measured factorially against divided attention, with throughput reported so the result is
comparable outside this hardware.

---

## What this project can honestly claim

1. **A characterisation, in Fitts throughput, of a wearable supernumerary master arm.**
   No such number appears in the SRL literature reviewed here. It is a small, checkable
   contribution and it is the only figure in this work that a reader without the hardware
   can compare to anything.

2. **A factorial measurement of DOF recovery by autonomy, crossed with divided attention.**
   Each factor is individually precedented; the crossing is not, in the work found. The
   claim must be stated at that level — "we cross two known factors that have not been
   crossed" — not as "nobody has studied shared autonomy for supernumerary limbs", which
   is false (Jing et al., 2025; Song and Asada, 2023).

3. **A negative or bounded result about azimuth observability on a body-mounted master.**
   This project has already measured that a shoulder-roll-derived azimuth cannot produce
   an orthogonal direction triad: Kabsch residual 54.8° (left) / 54.7° (right), and
   intended-90°-apart directions measured 137.7° apart, which no rotation can reconcile
   (`CLAUDE.md`). That is a *structural* statement about single-angle azimuth on a
   wearable master and is publishable as such, independent of the autonomy question.

4. **An engineering account of what a wearable master actually costs**, with numbers:
   which channels survive (LEFT j1–j4; RIGHT j1, j2, j6), which fail and how (RIGHT j3/j5/j7
   dead, j4 intermittent at 12.9% dropout), gyro drift of −5.0 °/min (left) versus
   +44.0 °/min (right), and IK feasibility collapsing from 92–100% to 8–75% when
   orientation is taken from an unreliable wrist rather than held at an anchor
   (`CLAUDE.md`). Papers that propose wearable masters rarely report this.

5. **What it cannot claim:** that shared autonomy for SRLs is new; that pointing beats
   end-effector velocity as an intent cue (unestablished — see 3.3); that blending is
   generally beneficial (it is conditional on prediction accuracy above roughly 84%, per
   Dragan and Srinivasa, 2012); or any generalisation from a single-rig, single-operator
   dataset to wearable robotics at large.

---

## References

Admoni, H. and Srinivasa, S. S. (2016). Predicting user intent through eye gaze for shared
autonomy. *AAAI Fall Symposium Series: Shared Autonomy in Research and Practice*.
https://aaai.org/papers/14137-14137-predicting-user-intent-through-eye-gaze-for-shared-autonomy/

Amini, M., Stuerzlinger, W., Teather, R. J. and Batmaz, A. U. (2025). A systematic review
of Fitts' law in 3D extended reality. *Proceedings of the 2025 CHI Conference on Human
Factors in Computing Systems (CHI '25)*. DOI: 10.1145/3706598.3713623

Aronson, R. M. and Admoni, H. (2022). Gaze complements control input for goal prediction
during assisted teleoperation. *Robotics: Science and Systems XVIII*.
DOI: 10.15607/RSS.2022.XVIII.025

Baker, C. L., Saxe, R. and Tenenbaum, J. B. (2009). Action understanding as inverse
planning. *Cognition*, 113(3), 329–349. DOI: 10.1016/j.cognition.2009.07.005

Beer, J. M., Fisk, A. D. and Rogers, W. A. (2014). Toward a framework for levels of robot
autonomy in human-robot interaction. *Journal of Human-Robot Interaction*, 3(2), 74–.
DOI: 10.5898/JHRI.3.2.Beer  *(end page not verified)*

Bright, Z. and Asada, H. H. (2017). Supernumerary robotic limbs for human augmentation in
overhead assembly tasks. *Robotics: Science and Systems XIII*.
https://www.roboticsproceedings.org/rss13/p62.pdf

Brohan, A., Brown, N., Carbajal, J., Chebotar, Y., et al. (2023). RT-2: Vision-language-
action models transfer web knowledge to robotic control. arXiv:2307.15818.

Collier, M. A., Narayan, R. and Admoni, H. (2025). The sense of agency in assistive
robotics using shared autonomy. *Proceedings of the 2025 ACM/IEEE International Conference
on Human-Robot Interaction (HRI '25)*. arXiv:2501.07462.

Dominijanni, G., Shokur, S., Salvietti, G., Buehler, S., Palmerini, E., Rossi, S.,
De Vignemont, F., d'Avella, A., Makin, T. R., Prattichizzo, D. and Micera, S. (2021). The
neural resource allocation problem when enhancing human bodies with extra robotic limbs.
*Nature Machine Intelligence*, 3(10), 850–860. DOI: 10.1038/s42256-021-00398-9

Dragan, A. D. and Srinivasa, S. S. (2012). Formalizing assistive teleoperation. *Robotics:
Science and Systems VIII*. (Best paper award finalist.)
https://www.roboticsproceedings.org/rss08/p10.pdf

Dragan, A. D. and Srinivasa, S. S. (2013). A policy-blending formalism for shared control.
*The International Journal of Robotics Research*, 32(7), 790–805.
DOI: 10.1177/0278364913490324

Fitts, P. M. (1954). The information capacity of the human motor system in controlling the
amplitude of movement. *Journal of Experimental Psychology*, 47(6), 381–391.

Gopinath, D., Jain, S. and Argall, B. D. (2017). Human-in-the-loop optimization of shared
autonomy in assistive robotics. *IEEE Robotics and Automation Letters*, 2(1), 247–254.

Guterstam, A., Petkova, V. I. and Ehrsson, H. H. (2011). The illusion of owning a third
arm. *PLoS ONE*, 6(2), e17208. DOI: 10.1371/journal.pone.0017208

Herlant, L. V., Holladay, R. M. and Srinivasa, S. S. (2016). Assistive teleoperation of
robot arms via automatic time-optimal mode switching. *Proceedings of the 11th ACM/IEEE
International Conference on Human-Robot Interaction (HRI '16)*.
https://www.ri.cmu.edu/pub_files/2016/3/mode_switching.pdf

Hincapié-Ramos, J. D., Guo, X., Moghadasian, P. and Irani, P. (2014). Consumed endurance:
a metric to quantify arm fatigue of mid-air interactions. *Proceedings of the SIGCHI
Conference on Human Factors in Computing Systems (CHI '14)*.

Jain, S., Farshchiansadegh, A., Broad, A., Abdollahi, F., Mussa-Ivaldi, F. and Argall, B.
(2015). Assistive robotic manipulation through shared autonomy and a body-machine
interface. *2015 IEEE International Conference on Rehabilitation Robotics (ICORR)*,
526–531. DOI: 10.1109/ICORR.2015.7281253

Javdani, S., Srinivasa, S. S. and Bagnell, J. A. (2015). Shared autonomy via hindsight
optimization. *Robotics: Science and Systems XI*. arXiv:1503.07619.

Javdani, S., Admoni, H., Pellegrinelli, S., Srinivasa, S. S. and Bagnell, J. A. (2018).
Shared autonomy via hindsight optimization for teleoperation and teaming. *The
International Journal of Robotics Research*, 37(7), 717–742.
DOI: 10.1177/0278364918776060

Jing, H., Zhao, S., Zheng, T., Li, L., Zhang, Q., Sun, K., Zhao, J. and Zhu, Y. (2025).
Shared control of supernumerary robotic limbs using mixed reality and mouth-and-tongue
interfaces. *Biosensors*, 15(2), 70. DOI: 10.3390/bios15020070

Johansson, R. S., Westling, G., Bäckström, A. and Flanagan, J. R. (2001). Eye-hand
coordination in object manipulation. *The Journal of Neuroscience*, 21(17), 6917–6932.

Kieliba, P., Clode, D., Maimon-Mor, R. O. and Makin, T. R. (2021). Robotic hand
augmentation drives changes in neural body representation. *Science Robotics*.
DOI: 10.1126/scirobotics.abd7935

Kim, D.-J., Hazlett-Knudsen, R., Culver-Godfrey, H., Rucks, G., Cunningham, T., Portée, D.,
Bricout, J., Wang, Z. and Behal, A. (2012). How autonomy impacts performance and
satisfaction: results from a study with spinal cord injured subjects using an assistive
robot. *IEEE Transactions on Systems, Man, and Cybernetics — Part A: Systems and Humans*,
42(1), 2–14. DOI: 10.1109/TSMCA.2011.2159589

Kim, M. J., Pertsch, K., Karamcheti, S., Xiao, T., Balakrishna, A., Nair, S., Rafailov, R.,
Foster, E., Lam, G., Sanketi, P., Vuong, Q., Kollar, T., Burchfiel, B., Tedrake, R.,
Sadigh, D., Levine, S., Liang, P. and Finn, C. (2024). OpenVLA: an open-source
vision-language-action model. arXiv:2406.09246. (Also *CoRL 2024*.)

Kim, M. J., et al. (2025). Fine-tuning vision-language-action models: optimizing speed and
success. arXiv:2502.19645. *(cited only for the existence of the optimisation line; the
77.9 Hz figure is [UNVERIFIED] here)*

Li, H.-B., Li, Z., He, L., et al. (2023). Wearable extra robotic limbs: a systematic review
of current progress and future prospects. *Journal of Intelligent & Robotic Systems*.
DOI: 10.1007/s10846-023-01940-0

Lisini Baldi, T., D'Aurizio, N., Gaudeni, C., Gurgone, S., Borzelli, D., d'Avella, A. and
Prattichizzo, D. (2025). Exploiting body redundancy to control supernumerary robotic limbs
in human augmentation. *The International Journal of Robotics Research*.
DOI: 10.1177/02783649241265451

Losey, D. P., Srinivasan, K., Mandlekar, A., Garg, A. and Sadigh, D. (2020). Controlling
assistive robots with learned latent actions. *2020 IEEE International Conference on
Robotics and Automation (ICRA)*, 378–384.
http://collab.me.vt.edu/pdfs/losey_icra2020.pdf

MacKenzie, I. S. (1992). Fitts' law as a research and design tool in human-computer
interaction. *Human-Computer Interaction*, 7(1), 91–139.
DOI: 10.1207/s15327051hci0701_3

MacKenzie, I. S. (2018). Fitts' law. Chapter in the *Handbook of Human-Computer
Interaction* (Wiley). Author's copy: https://www.yorku.ca/mack/hhci2018.html
*(chapter/page numbers [UNVERIFIED])*

Mete, M., Bolotnikova, A., Schuessler, A. and Paik, J. (2026). Reconfiguration of
supernumerary robotic limbs for human augmentation. arXiv:2603.29808.

Muelling, K., Venkatraman, A., Valois, J.-S., Downey, J. E., Weiss, J., et al. (2017).
Autonomy infused teleoperation with application to brain computer interface controlled
manipulation. *Autonomous Robots*, 41, 1401–1422. DOI: 10.1007/s10514-017-9622-4.
Preprint: arXiv:1503.05451.

Newman, B. A., Aronson, R. M., Srinivasa, S. S., Kitani, K. and Admoni, H. (2022).
HARMONIC: a multimodal dataset of assistive human-robot collaboration. *The International
Journal of Robotics Research*. DOI: 10.1177/02783649211050677

Pan, J., Eden, J., Oetomo, D. and Johal, W. (2024). Effects of shared control on cognitive
load and trust in teleoperated trajectory tracking. *IEEE Robotics and Automation Letters*,
9(6), 5863–5870. arXiv:2402.02758.

Pan, J., Eden, J., Oetomo, D. and Johal, W. (2025). Using Fitts' law to benchmark assisted
human-robot performance. *Proceedings of the 2025 ACM/IEEE International Conference on
Human-Robot Interaction (HRI '25)*. arXiv:2412.05412.

Parietti, F. and Asada, H. H. (2014). Supernumerary robotic limbs for aircraft fuselage
assembly: body stabilization and guidance by bracing. *2014 IEEE International Conference
on Robotics and Automation (ICRA)*, 1176–1183.

Parietti, F. and Asada, H. H. (2016). Supernumerary robotic limbs for human body support.
*IEEE Transactions on Robotics*, 32(2), 301–311.

Penaloza, C. I. and Nishio, S. (2018). BMI control of a third arm for multitasking.
*Science Robotics*, 3(20), eaat1228. DOI: 10.1126/scirobotics.aat1228

Pérez-D'Arpino, C. and Shah, J. A. (2015). Fast target prediction of human reaching motion
for cooperative human-robot manipulation tasks using time series classification. *2015 IEEE
International Conference on Robotics and Automation (ICRA)*.

Prattichizzo, D., Malvezzi, M., Hussain, I. and Salvietti, G. (2014). The Sixth-Finger: a
modular extra-finger to enhance human hand capabilities. *23rd IEEE International Symposium
on Robot and Human Interactive Communication (RO-MAN)*, 993–998.

Prattichizzo, D., Pozzi, M., Lisini Baldi, T., Malvezzi, M., Hussain, I., Rossi, S. and
Salvietti, G. (2021). Human augmentation by wearable supernumerary robotic limbs: review
and perspectives. *Progress in Biomedical Engineering*, 3(4), 042005.
DOI: 10.1088/2516-1091/ac2294

Saraiji, M. Y., Sasaki, T., Kunze, K., Minamizawa, K. and Inami, M. (2018). MetaArms: body
remapping using feet-controlled artificial arms. *Proceedings of the 31st Annual ACM
Symposium on User Interface Software and Technology (UIST '18)*.

Sasaki, T., Saraiji, M. Y., Fernando, C. L., Minamizawa, K. and Inami, M. (2017).
MetaLimbs: multiple arms interaction metamorphism. *ACM SIGGRAPH 2017 Emerging
Technologies*. DOI: 10.1145/3084822.3084837

Sheridan, T. B. and Verplank, W. L. (1978). *Human and Computer Control of Undersea
Teleoperators*. Technical report, Man-Machine Systems Laboratory, MIT.

Song, H. and Asada, H. H. (2023). Shared control based on extended Lipschitz analysis with
application to human-superlimb collaboration. arXiv:2309.00685.

Soukoreff, R. W. and MacKenzie, I. S. (2004). Towards a standard for pointing device
evaluation: perspectives on 27 years of Fitts' law research in HCI. *International Journal
of Human-Computer Studies*, 61(6), 751–789. https://www.yorku.ca/mack/ijhcs2004.pdf

Vatsal, V. and Hoffman, G. (2018). Design and analysis of a wearable robotic forearm. *2018
IEEE International Conference on Robotics and Automation (ICRA)*.
https://hrc2.io/assets/pdfs/papers/VatsalHoffmanICRA18.pdf

Wen, J., Zhu, Y., et al. (2025). DexVLA: vision-language model with plug-in diffusion expert
for general robot control. arXiv:2502.05855. *(full author list not verified beyond the
first authors)*

Wickens, C. D. (2008). Multiple resources and mental workload. *Human Factors*, 50(3),
449–455. DOI: 10.1518/001872008X288394

You, E. and Hauser, K. (2011). Assisted teleoperation strategies for aggressively
controlling a robot arm with 2D input. *Robotics: Science and Systems VII*.
https://www.roboticsproceedings.org/rss07/p45.pdf

Zhou, H., Kip, T., Dong, Y., Bianchi, A., Sarsenbayeva, Z. and Withana, A. (2025). Juggling
extra limbs: identifying control strategies for supernumerary multi-arms in virtual
reality. *Proceedings of the 2025 CHI Conference on Human Factors in Computing Systems
(CHI '25)*. DOI: 10.1145/3706598.3713647

Zhou, H., Fan, C.-A., Dong, Y., Takashita, S., Inami, M., Sarsenbayeva, Z. and Withana, A.
(2026). SRL proxemics: spatial guidelines for supernumerary robotic limbs in near-body
interactions. arXiv:2602.00494.

Ziebart, B. D., Maas, A. L., Bagnell, J. A. and Dey, A. K. (2008). Maximum entropy inverse
reinforcement learning. *Proceedings of the 23rd AAAI Conference on Artificial
Intelligence*, 1433–1438.

Ziebart, B. D., Ratliff, N., Gallagher, G., Mertz, C., Peterson, K., Bagnell, J. A.,
Hebert, M., Dey, A. K. and Srinivasa, S. (2009). Planning-based prediction for pedestrians.
*2009 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*,
3931–3936.

**Standards**

ISO 9241-9:2000. *Ergonomic requirements for office work with visual display terminals
(VDTs) — Part 9: Requirements for non-keyboard input devices.* Superseded by
ISO/TS 9241-411:2012 (later ISO 9241-411:2012). *(revision history summarised from
secondary sources; [UNVERIFIED] in detail)*

**Project-internal source (not a publication)**

`kortex_ws/CLAUDE.md` — engineering log of measured channel health, azimuth/Kabsch
residuals, gyro drift, IK reachability, clearance and real-arm latency for this rig.


---

# 7. THE TWO-PERSON ARCHITECTURE — operator, wearer, and the gap

*Added 2026-08-08 when the project was reframed. This section supersedes Section 6.*

## 7.1 Fusion (Saraiji, Sasaki, Matsumura, Minamizawa, Inami — SIGGRAPH 2018 E-Tech)

**The direct predecessor.** Fusion is a backpack worn by a *surrogate* carrying a
**3-axis robotic head** with stereo vision and binaural audio, and **two anthropomorphic
6-DOF arms with removable hands**. A remote **operator** wears an Oculus CV1 and sees
through the surrogate's shoulders, controlling the arms with handheld controllers. The
architecture is exactly ours: *one person bears the machine, another drives it.*

**The three "levels of bodily driven communication" are levels of PHYSICAL COUPLING,
not levels of machine autonomy.** This distinction is the whole of the gap and is easy
to misread, because "Direct / Enforced / Induced" reads like an autonomy ladder:

| Fusion level | what it means physically | who decides the motion |
| --- | --- | --- |
| **Direct** | robot arms operate INDEPENDENTLY of the surrogate's own arms — a collaborative third-and-fourth arm | 100% operator |
| **Induced** | the hands guide or instruct the surrogate's arms, cueing a motion the surrogate then makes | 100% operator |
| **Enforced** | the removable hands are LINKED to the surrogate's wrists, so the operator drives the surrogate's own limbs directly | 100% operator |

In all three the robot decides **nothing**. There is no goal inference, no arbitration,
no assistance, no autonomous segment. Fusion moved along the axis of *how much of the
wearer's body the operator commands* and never along the axis of *how much the machine
decides.*

**What it did NOT do, stated precisely:**

- **No autonomy of any kind**, therefore no shared autonomy.
- **No user study and no quantitative evaluation.** It is a 2-page Emerging Technologies
  demonstration (Article 7, pp. 1-2). Task performance, workload, trust, embodiment and
  safety are all unmeasured. Fusion **demonstrated** the architecture; it did not
  **validate** it.
- **No measurement of the wearer as a participant.** The surrogate's experience —
  perceived safety, agency, comfort, willingness — is discussed as motivation and never
  instrumented.
- **No treatment of the wearer's motion as a disturbance** to the arms it carries.
- **No task in which the two people must coordinate to succeed.** The scenarios are
  demonstrations of coupling, not tasks with a failure mode.

## 7.2 SRL Proxemics (CHI 2026) — the nearest and most awkward neighbour

This is the paper that most constrains what we may claim, and it did not exist when
this project's earlier gap statement was written.

18 participants wore MetaLimbs (two 7-DOF back-mounted arms) in a **Wizard-of-Oz**
study. Crucially, **a concealed separate operator drove the limbs via mechanical
linkage — not the wearer.** Measures: think-aloud, semi-structured interview, **skin
conductance response**, and questionnaires on trust, safety and embodiment.

**Its central finding is a direct challenge to the naive hypothesis of this project:**

> Higher autonomy did **not** enhance perceived safety. Approach-phase arousal was
> significantly *higher* in the High-Autonomy condition (SCR Mdn = 0.85) than under
> participant-defined rules (Mdn = 0.25, p = .002), and trust was *lower*
> (Mdn = 4.18 vs 5.42, p < .05).

It also produced a body-zone sensitivity map that we should simply adopt rather than
rediscover: head/face rejected by 17/18 at roughly arm's length; torso entry tolerated
by 13/18 only with evident purpose and a palm-width (~10 cm) buffer; hands/forearms
near-contact accepted by 15/18 during task-relevant handovers; the **elbow** named by
14/18 as a reflex boundary. Preferred motion was **pause-move-pause**, **arc-shaped
rather than frontal**, whole-limb rather than isolated-hand, with an audible motor cue
for limbs behind the body.

**What it did NOT do:** the autonomy was *simulated* — a hidden human, not a planner —
so it measures the wearer's **perception of autonomy**, not the behaviour or
performance of an actual shared-autonomy system. The wearer had **no task**. There was
**no task-performance measure at all**, so the question "does autonomy trade wearer
comfort against operator performance" cannot be asked of it. And the operator was an
experimental instrument, not a participant: their workload, awareness and trust are
unmeasured.

## 7.3 Teleoperation from a moving, non-inertial base

The wearer's body is a floating base, and this is a mature control literature. Base
disturbance is compensated in mobile manipulation via extended
uncertainty-and-disturbance estimation, via virtual-spring master-slave schemes with
local force feedback, and on legged platforms via whole-body and residual-learning
controllers that counteract locomotion-induced inertial effects.

**Within SRLs specifically the problem is named and solved as a control problem.**
Zhang et al. (2024, *Advanced Intelligent Systems*) model supernumerary arms on an
omnidirectional floating base and give three feedback schemes, reporting tracking
errors of **1.18 ± 0.56 mm** (point) and **1.42 ± 0.43 mm** (circular) with the
manipulator as floating base, and **1.37 ± 0.58 mm** with the **human shoulder** as
floating base. A related framework (arXiv:2310.10029) reconstructs the SRA Jacobian
against IMU-derived whole-body skeleton feedback.

**THE CRITICAL LIMITATION FOR US, AND IT IS NOT A SMALL ONE:** in every one of these,
**the wearer IS the operator.** The disturbance is *self-generated*. A person who is
about to sway knows they are about to sway, and their own motor commands are available
as feed-forward. Splitting the roles removes that: the disturbance now originates in a
**different nervous system**, is unavailable to the operator as efference copy, and is
visible to them only through a camera that is itself mounted on the moving base.

## 7.4 Trust and agency when you bear the risk without the control

The wearer's position has no clean precedent in robotics, and the closest validated
literature is the **autonomous-vehicle passenger**: a person who has ceded control,
remains physically exposed to the consequences, and whose risk perception is driven by
the violation of self-set **safety margins** rather than by objective performance.
That literature contributes two things we should take: validated trust instruments,
and the repeated finding that **physiological measures (electrodermal activity) and
subjective report dissociate** — which is precisely why SRL Proxemics' SCR result is
more persuasive than a questionnaire alone would have been.

The disanalogy matters too, and is in our favour as a research contribution: an AV
passenger has chosen a destination and can usually stop the vehicle, whereas our
wearer has **no goal of their own and no control input whatsoever**. That is a more
extreme position than the passenger literature has had to describe.

## 7.5 THE GAP, stated precisely

**The original reading was: "Fusion established the operator/wearer architecture but
did not study shared autonomy within it." That is correct, and it is not the whole
story. Three corrections, one of which weakens the claim and two of which sharpen it.**

**Correction 1 — Fusion is weaker evidence than "established" implies.** It is a
2-page demonstration with no study and no numbers. It established the architecture as
*buildable and compelling*, not as *characterised*. Anything we cite it for must be
existence, never performance.

**Correction 2 — the gap is NOT untouched, and claiming so would be wrong.**
SRL Proxemics (CHI 2026) already studied a separate operator, varied autonomy, and
measured wearer safety and trust — and found **higher autonomy made things worse**.
Any claim of the form "nobody has looked at autonomy in a two-person wearable system"
is false as of 2026. What remains open is narrower and better: their autonomy was a
concealed human, their wearer had no task, and they measured **no task performance**.

**Correction 3 — the same applies to base motion, which was briefed as untested.**
The T9 hypothesis was put to me as "the strongest hypothesis available and no one has
tested it." The *control* problem is tested — Zhang et al. report millimetre tracking
under human-induced disturbance with the human shoulder as the floating base. What is
untested is the **two-person** case, where the disturbance is generated by someone the
operator cannot feel, cannot predict, and did not command.

**So the gap that survives contact with the literature:**

> No study has measured, in a system where one person wears the arms and a **different**
> person drives them, whether **machine autonomy** changes **task performance and the
> wearer's experience at the same time** — and in particular whether autonomy earns its
> cost when the base is moving under a disturbance the operator cannot anticipate
> because it originates in another person's body.

Three things make that a real gap rather than a gap by construction:

1. **The two measures are in tension and both must be reported.** SRL Proxemics says
   autonomy *lowers* wearer trust; the shared-autonomy literature (Section 1) says it
   *raises* operator performance. Nobody has measured both ends of that trade in one
   system, so nobody knows the exchange rate.
2. **The disturbance is exogenous to the operator.** This is a genuinely different
   control-and-attention problem from the self-generated case, and it is the one an
   operator/wearer architecture necessarily creates.
3. **Coordination becomes a task variable.** With two people there are tasks neither
   can complete alone (T8), and that class does not exist in the single-person
   literature at all.

## 7.6 What must NOT be claimed

- **Not** "the first wearable robot driven by a remote operator" — Fusion, 2018.
- **Not** "the first study of autonomy and wearer safety in such a system" — SRL
  Proxemics, 2026.
- **Not** "the first compensation of wearer-induced base disturbance" — Zhang 2024.
- **Not** any claim of superiority over Fusion on performance. Fusion reported no
  performance numbers, so there is nothing to be superior to.
- **Not** an embodiment claim for the wearer. Our arms are not linked to their limbs;
  Fusion's Enforced mode is the mode that would license embodiment language, and we do
  not implement it.

## 7.7 Sources for this section

- Saraiji, Sasaki, Matsumura, Minamizawa, Inami. *Fusion: full body surrogacy for
  collaborative communication.* ACM SIGGRAPH 2018 Emerging Technologies, Art. 7, 1-2.
  https://dl.acm.org/doi/10.1145/3214907.3214912 —
  https://history.siggraph.org/experience/fusion-full-body-surrogacy-for-collaborative-communication-by-saraiji-sasaki-matsumura-minamizawa-and-inami/
- IEEE Spectrum, *Fusion: A Collaborative Robotic Telepresence Parasite That Lives on
  Your Back* (system details; Direct / Induced / Enforced).
  https://spectrum.ieee.org/fusion-a-collaborative-robotic-telepresence-parasite-that-lives-on-your-back
- *SRL Proxemics: Spatial Guidelines for Supernumerary Robotic Limbs in Near-Body
  Interactions.* CHI 2026. https://arxiv.org/html/2602.00494v1 —
  https://doi.org/10.1145/3772318.3790532
- Zhang et al. *Motion-Compensation Control of Supernumerary Robotic Arms Subject to
  Human-Induced Disturbances.* Advanced Intelligent Systems, 2024.
  https://doi.org/10.1002/aisy.202300448
- *A Human Motion Compensation Framework for a Supernumerary Robotic Arm.*
  arXiv:2310.10029. http://arxiv.org/abs/2310.10029
- Lisini Baldi et al. *Exploiting body redundancy to control supernumerary robotic
  limbs in human augmentation.* IJRR 2025. https://doi.org/10.1177/02783649241265451
- *Shared Control of Supernumerary Robotic Limbs Using Mixed Reality and
  Mouth-and-Tongue Interfaces.* Biosensors 2025.
  https://pmc.ncbi.nlm.nih.gov/articles/PMC11853150/
- *I Need a Third Arm! Eliciting Body-based Interactions with a Wearable Robotic Arm.*
  CHI 2023. https://dl.acm.org/doi/10.1145/3544548.3581184 (adjustable autonomy and
  intent transparency as elicited user requirements; WoZ remote operator condition)
- *Trust, risk perception, and intention to use autonomous vehicles.*
  https://pmc.ncbi.nlm.nih.gov/articles/PMC11968569/
- *Risk Assessment by a Passenger of an Autonomous Vehicle Among Pedestrians:
  Relationship Between Subjective and Physiological Measures.*
  https://pmc.ncbi.nlm.nih.gov/articles/PMC10790836/

**Search caveat, unchanged from the header:** web search only, English only, not an
exhaustive Scopus/IEEE query. The novelty claim in 7.5 should be read against that.
