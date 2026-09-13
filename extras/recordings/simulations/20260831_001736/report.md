# Simulation campaign 20260831_001736

Started 2026-08-31T00:17:36, finished 2026-08-31T00:19:44.

| stage | group | tier | status | s |
| --- | --- | --- | --- | --- |
| `cv_arithmetic` | computer vision | standalone | **ok** | 11.6 |
| `cv_replay` | computer vision | standalone | **ok** | 87.7 |
| `cv_orientation` | computer vision | standalone | **ok** | 0.3 |
| `teleop_motion` | teleoperation | standalone | **ok** | 4.6 |
| `teleop_smoothing` | teleoperation | standalone | **ok** | 0.4 |
| `teleop_budget` | teleoperation | standalone | **ok** | 0.8 |
| `tasks_accuracy` | robot tasks | standalone | **ok** | 0.1 |
| `tasks_status` | robot tasks | standalone | **ok** | 0.9 |
| `autonomy_instructions` | autonomy | standalone | **ok** | 0.3 |
| `autonomy_arbitration` | autonomy | standalone | **ok** | 3.7 |
| `study_power` | study | standalone | **ok** | 16.9 |
| `sim_to_real` | transfer | recorded | **ok** | 0.6 |
| `teleop_orientation_cost` | teleoperation | stack | **skipped** | - |
| `tasks_t1` | robot tasks | stack | **skipped** | - |
| `tasks_msc` | robot tasks | stack | **skipped** | - |
| `tasks_dance` | robot tasks | stack | **skipped** | - |

## `cv_arithmetic` -- Vision arithmetic against a constructed scene

*every geometric stage run against a depth frame whose plane, object heights and object widths were CHOSEN, so the answer is known before the stage runs*

```
  self_test      11/33 fastsam                            REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      12/33 area_filter                        REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      13/33 segment_match                      REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      14/33 yoloworld                          REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      15/33 raw_depth                          ok          0.0 s
  self_test      16/33 alignment                          ok          0.0 s
  self_test      17/33 deprojection                       ok          0.2 s
  self_test      18/33 ransac                             ok          0.0 s
  self_test      19/33 pca_refine                         ok          0.1 s
  self_test      20/33 height_map                         ok          0.0 s
  self_test      21/33 mode_surface                       ok          0.1 s
  self_test      22/33 on_surface                         ok          2.5 s
  self_test      23/33 cube_measurement                   ok          0.1 s
  self_test      24/33 layers                             REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      25/33 min_width                          REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      26/33 nested                             REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      27/33 grasp                              REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      28/33 apriltag                           REFUSED     0.0 s
      -> no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly wh
  self_test      29/33 table                              ok          0.1 s
  self_test      30/33 boxes3d                            REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      31/33 distances                          REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      32/33 people                             REFUSED     0.0 s
      -> --fast was given, so the segmentation, open-vocabulary and per-instance stages were skipped. They are the slow ones; drop --fast for the full set.
  self_test      33/33 self_view                          ok          0.0 s
  every stage produced a figure without crashing                   ok

self-test PASSED  (21 checks, 0 failed)

written to recordings/simulations/20260831_001736/cv_selftest/self_test
```

## `cv_replay` -- The full pipeline replayed on the recorded cameras

*the frames are real; the RUN is offline, so every constant can be changed and the whole set redrawn without a camera*

```
  scene_rs       09/33 minarearect                        REFUSED     0.0 s
      -> no contour over the 400 px floor in this frame, so there is no rotated box to fit
  scene_rs       10/33 size_at_range                      REFUSED     0.0 s
      -> no blob over 400 px to test
  scene_rs       11/33 fastsam                            ok          1.2 s
  scene_rs       12/33 area_filter                        ok          0.1 s
  scene_rs       13/33 segment_match                      ok          0.1 s
  scene_rs       14/33 yoloworld                          REFUSED     3.5 s
      -> YOLO-World ran in 3.5 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration
  scene_rs       15/33 raw_depth                          ok          0.0 s
  scene_rs       16/33 alignment                          ok          0.0 s
  scene_rs       17/33 deprojection                       ok          0.1 s
  scene_rs       18/33 ransac                             ok          2.9 s
  scene_rs       19/33 pca_refine                         ok          0.0 s
  scene_rs       20/33 height_map                         ok          0.0 s
  scene_rs       21/33 mode_surface                       ok          0.0 s
  scene_rs       22/33 on_surface                         ok          1.6 s
  scene_rs       23/33 cube_measurement                   REFUSED     0.0 s
      -> only 20 points are both green-coloured AND more than 5 mm above the plane (the pick needs 60 and returns None below that). 0 points are coloured but O
  scene_rs       24/33 layers                             ok          0.1 s
  scene_rs       25/33 min_width                          ok          2.3 s
  scene_rs       26/33 nested                             ok          0.1 s
  scene_rs       27/33 grasp                              REFUSED     0.1 s
      -> the grasp planner refused, which is an ANSWER: object is 1387 mm across its narrowest axis; the Robotiq 85 opens 75 mm usable. It cannot be grasped as
  scene_rs       28/33 apriltag                           REFUSED     0.0 s
      -> no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly wh
  scene_rs       29/33 table                              ok          0.0 s
  scene_rs       30/33 boxes3d                            ok          1.2 s
  scene_rs       31/33 distances                          ok          1.0 s
  scene_rs       32/33 people                             ok          1.2 s
  scene_rs       33/33 self_view                          ok          0.0 s

96 of 132 stages produced a result; 0 crashed

written to recordings/simulations/20260831_001736/cv_replay
  measurements.csv  723 measurements, every one with a unit
  data/          per stage: the same numbers as JSON, plus a CSV per object table
  report.md      every stage with its formula and its numbers
  sheet_*.png    one page per camera
  raw/           the frames, so figures can be redrawn with --replay recordings/simulations/20260831_001736/cv_replay
```

## `cv_orientation` -- Which way up the scene camera is

*12 known-answer checks on the rotation, its intrinsic map and the instrument that decides it*

```
  ok    cx' = W-1-cx
  ok    cy' = H-1-cy
  ok    focal lengths are untouched
  ok    rotating K twice is the identity
  ok    the stripe moved to the bottom
  ok    the depth moved with it
  ok    K moved with the image
  ok    apply=False is the identity
  ok    an upright frame reads UPRIGHT (+1.00 vs +0.57)
  ok    a flipped frame reads UPSIDE_DOWN (+0.57 vs +1.00)
  ok    unrelated pictures read UNDECIDED (+0.01 vs -0.01)
  ok    the stored orientation is upright

12 checks, 0 failed
```

## `teleop_motion` -- Motion generation: three generators compared

*off-path excursion, same-cycle excursion, peak commanded velocity against the declared limit, and arrival synchrony*

```
instrument self-test passed (FK control, four known answers)

LEFT arm, 50 Hz, slew [0.5, -0.3, 0.1, 0.25, -0.05, 0.15, 0.02]
  generator             off-path  same-cyc    peak v    arrive  sync
                          max mm    max mm  of limit    cycles
  clamp_towards             51.3     135.3    12.53x       1,2  NO
  ruckig                     0.0      69.8     1.00x        36  YES
  synchronised-clamp         0.0      22.7     1.00x        18  YES

RIGHT arm, 50 Hz, slew [0.5, -0.3, 0.1, 0.25, -0.05, 0.15, 0.02]
  generator             off-path  same-cyc    peak v    arrive  sync
                          max mm    max mm  of limit    cycles
  clamp_towards             39.2      75.0    12.53x       1,2  NO
  ruckig                     0.0      54.8     1.00x        36  YES
  synchronised-clamp         0.0      18.1     1.00x        18  YES

the per-joint clamp against its own step size (left arm):
  max_step 0.35 rad ->   2 cycles,   51.3 mm off the path
  max_step 0.20 rad ->   3 cycles,   34.4 mm off the path
  max_step 0.10 rad ->   5 cycles,   68.0 mm off the path
  max_step 0.05 rad ->  10 cycles,   68.0 mm off the path
  max_step 0.02 rad ->  25 cycles,   68.0 mm off the path
  max_step 0.01 rad ->  50 cycles,   68.0 mm off the path

  51.3 mm at the shipped 0.35 rad, 68.0 mm at 0.01 rad. The excursion does NOT
  tend to zero as the step shrinks -- it settles -- because the joints are
  at different fractions of their own travel however small the step is.
  Each row is a genuinely different path, densified at 0.02 rad, not one
  path sampled at different rates. It is the shape of the algorithm.

wrote /home/gausms/kortex_ws/recordings/baselines/teleop_motion.json
```

## `teleop_smoothing` -- Master smoothing: 1-Euro against the fixed EMA

*stillness and lag measured together, which is the pair a fixed alpha cannot win*

```
Master-arm smoothing, measured through the shipped MasterPoseNode.smooth_tip (50 Hz)
  still: a stationary master + 1.5 mm-rms tremor -> output RMS
  lag:   a 0.40 m/s reach -> how far the tip trails the master

law                        still_mm     lag_mm
none (raw)                     2.57       0.02
ema(0.3)  [was]                1.10      18.67
one_euro  [now]                0.83       1.59

  the node actually applies the filter                 PASS   0.83 vs 2.57 mm raw
  one_euro is steadier than the ema(0.3) it replaced   PASS   0.83 vs 1.10 mm
  one_euro does not pay for it in lag                  PASS   1.59 vs 18.67 mm

PASS
```

## `teleop_budget` -- The positioning error budget

*each term measured separately, combined in quadrature and added, against the 30 mm capture gate*

```
  a quick reach or a fidget    562 ms   843-1405 mm   EXCEEDS IT
  a startle or flinch           62 ms   155-248 mm   EXCEEDS IT
  a startle or flinch          562 ms  1405-2248 mm   EXCEEDS IT

  5 of 6 cases move further than the whole floor before the guard has
  heard about it. The floor is a DISTANCE, not a reaction time, and nothing
  in this system reacts to a wearer who moves quickly.

==========================================================================
3  WHAT THE GUARD GETS WRONG WHEN TRACKING DROPS
   the assumed posture is 'down'; the person may not be
==========================================================================
  left  at HOME, clearance to the wearer by ARM POSTURE:
      out      0.0933 m   (-224 mm vs 'down')  BREACHES THE 0.15 m FLOOR
      behind   0.2779 m   (-39 mm vs 'down')
      down     0.3170 m   (+0 mm vs 'down')
      folded   0.3211 m   (+4 mm vs 'down')
      none     0.3211 m   (+4 mm vs 'down')
  right at HOME, clearance to the wearer by ARM POSTURE:
      out      0.0933 m   (-224 mm vs 'down')  BREACHES THE 0.15 m FLOOR
      behind   0.2779 m   (-39 mm vs 'down')
      down     0.3170 m   (+0 mm vs 'down')
      folded   0.3211 m   (+4 mm vs 'down')
      none     0.3211 m   (+4 mm vs 'down')

==========================================================================
4  HOW FREE THE ARMS ARE
==========================================================================
  at_the_work_point (12 walks):
      pinned   0.065 m mean reach, 0 walk(s) wearer-bound
      cone15   0.304 m mean reach, 0 walk(s) wearer-bound
      cone45   0.325 m mean reach, 0 walk(s) wearer-bound
      free     0.362 m mean reach, 0 walk(s) wearer-bound
  at_home (28 walks):
      pinned   0.460 m mean reach, 6 walk(s) wearer-bound
      cone15   0.492 m mean reach, 6 walk(s) wearer-bound
      cone45   0.530 m mean reach, 6 walk(s) wearer-bound
      free     0.549 m mean reach, 6 walk(s) wearer-bound

wrote recordings/baselines/control_budget.json
```

## `tasks_accuracy` -- Accuracy table over the recorded clip set

*grasp success, positioning error at closure and placement error, per task and per mode, from committed clip data*

```
instrument self-test passed (7 known answers, including two that must FAIL on bad input)

recordings/verification
mode                   cells  grasp %  pos err mm    place mm     var mm   min obj
----------------------------------------------------------------------------------
01_master_teleop           1   100.0%       0.000          --         --     30 mm
02_vr_teleop               1   100.0%       0.000          --         --     30 mm
03_shared_autonomy         1   100.0%       0.000          --         --     30 mm
04_vr_shared               1   100.0%       0.000          --         --     30 mm
06_full_autonomy           3   100.0%       0.000      56.258      0.591     30 mm
   NOTE: placement is scored against the CURRENT targets. Clips recorded before the
   planes moved are scored against coordinates they never ran in, so treat the
   placement and variance columns as INVALID for those. Grasp rate and positioning
   error are independent of target position and are sound.
-> recordings/verification/accuracy_table.json
```

## `tasks_status` -- Which cells of the task/mode matrix exist

*planned cells against recorded cells, with by-design absences separated from real gaps*

```
BLOCKED ON THE LAB -- not on code, and not fixable from here
==============================================================================
  * two real Kortex sessions
      all dual-arm work is against mocks; the arm permits exactly ONE session and a leaked one refuses the next
  * detection rate at working distance
      MEASURED 0-4% on rendered primitives vs 0.89-0.91 on a real photograph -- the renderer is out of distribution, so the 95% gate is NEITHER passed nor failed
  * the real camera intrinsics, distortion and extrinsic
      the mock's values are nominal; only the driver is a source for the real ones
  * whether the vision module streams at all
      open, given that UDP cyclic is dead on this host and only SHM works
  * the master arm
      7 of 14 channels INCOHERENT as of the 2026-08-06 baseline; this is a soldering problem, not a software one
  * l_j2 and l_j4 specifically
      regression names them: the left arm has NO reach observable without them (R2 0.133, residual 93% of spread)
  * the right arm's home joint values
      recorded as NEVER READ from hardware; P_HOME for that arm depends on them
  * the arms sharing a workspace
      0 of 63 frontal cells reachable by both, 0 of 319 surveyed cells at |x| <= 0.10. T4 handover is BLOCKED by geometry and needs the right arm RE-PARKED in hardware
  * the mount's proximal interference
      real and mechanical; the SRDF exclusion silences the alarm, it does not move the metal
  * Kortex close-then-reconnect
      the riskiest thing in the recovery layer and the part a real run hits first
  * adb / Android Platform-Tools
      installed on neither side; the one hard prerequisite before any headset test
  * this machine is off the lab network
      eth0 DOWN, 100% loss to 192.168.1.10 -- nothing needing a real arm can run here at all

==============================================================================
BLOCKED ON ETHICS -- approval, not equipment
==============================================================================
  * any run with a real participant
      every figure in this repo is scripted or synthetic; no human data has been collected
  * the two-person dyad protocol
      operator and wearer are DIFFERENT people, so both are participants and both consent separately
  * worn operation
      >17 kg with harness and NO gravity compensation, on a person who did not choose the motion; recommended against until the proximal interference is fixed mechanically
  * physiological measures (EDA)
      trust/arousal measures on the wearer need their own approval
  * recording of participants
      the clip pipeline records the SIM only; any camera on a person is a separate matter
```

## `autonomy_instructions` -- Full autonomy: 75 phrasings through the grammar

*correct / asked / refused / MISUNDERSTOOD, against the committed earlier grammar on the same cases*

```
  lift the second cube                                       CORRECT        1 cube, picked up and held (number 2)

-- MISSPELLING
  put the bleu ones on the bleu pad                          CORRECT        2 cubes, on the blue pad (read 'bleu' as 'blue', 'bleu' as
  put the blue cubbes on the blue pad                        CORRECT        2 cubes, on the blue pad (read 'cubbes' as 'cubes')
  put the gren ones on the gren pad                          ASKED          I could not read 'gren' -- did you mean green or grey?
        reply 'green'                                        CORRECT  2 cubes, on the green pad
  put the gerne ones on the gerne pad                        ASKED          Which cube? I can see 2 blue, 2 green, and 'put the gerne 
  pt the blue ones on the blue pad                           REFUSED        no known verb in 'pt blue ones on blue pad'
  put the blue ones on the blue padd                         ASKED          I could not read 'padd' -- did you mean pad or pads?
  sort teh cubes by colour                                   CORRECT        4 cubes, each on the pad of the colour the camera saw
  put the blue ones on the blue pad wen you are ready        CORRECT        2 cubes, on the blue pad
  put the lefmost blue cube on the blue pad                  CORRECT        1 cube, on the blue pad (the left one) (read 'lefmost' as 

==============================================================================
TOTALS over 75 phrasings
==============================================================================
                 at b782a9b      now
  CORRECT                21       34   (45.3%)
  ASKED                  14       14   (18.7%)
  REFUSED                37       27   (36.0%)
  MISUNDERSTOOD           3        0   (0.0%)

  'at b782a9b' is the SAME cases run against the grammar and the grounding
  layer as committed, checked out of git into a temporary tree and run in
  its own interpreter. The two columns differ by the code and by nothing
  else.

  OF THE 10 THAT WERE ANSWERED, IN 12 REPLIES:
     CORRECT        10
  These are NOT counted as CORRECT above. A grammar that asks about
  everything must not be able to look like one that understands everything.

  ASKED and REFUSED are SAFE -- the arm did not move and the operator was told.

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
NO MISUNDERSTANDINGS.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  -> recordings/simulations/20260831_001736/instructions.json
```

## `autonomy_arbitration` -- Shared autonomy: the arbiter over its whole input range

*what fraction of the confidence range assists, what the position path does in each state, and the tie case*

```
shared-autonomy intent inference, 300 trials per condition
4 cubes on a 60 mm pitch; the calibrated arm carries its MEASURED 11.3 mm error

table is off positions    correct    WRONG  ambiguous    frames
--------------------------------------------------------------
0 mm         DECLARED       70.7%     0.0%      29.3%      14.3
             CALIBRATED     75.7%     0.0%      24.3%      15.5
20 mm        DECLARED       72.7%     0.7%      26.7%      16.3
             CALIBRATED     74.7%     0.0%      25.3%      15.3
40 mm        DECLARED       58.7%    16.7%      24.7%      17.2
             CALIBRATED     72.0%     0.0%      28.0%      15.4
60 mm        DECLARED       53.3%    35.7%      11.0%      15.6
             CALIBRATED     74.0%     0.0%      26.0%      15.1
80 mm        DECLARED       52.7%    41.7%       5.7%      13.9
             CALIBRATED     71.7%     0.0%      28.3%      16.0
120 mm       DECLARED       47.7%    50.0%       2.3%      10.9
             CALIBRATED     74.7%     0.0%      25.3%      15.7

DECLARED positions are the task file's. CALIBRATED are what the
sweep measured. At 0 mm they are the same question, which is why
today's simulation cannot show the difference.
-> recordings/simulations/20260831_001736/arbitration.json
```

## `study_power` -- The pre-registered analysis on constructed data

*power and false-positive rate of the procedure that will be applied to the participant data*

```
      "dyads": 18,
      "uncorrected": 0.8289,
      "corrected": 0.7239333333333333,
      "family_wise": 0.9871666666666666
    },
    {
      "dyads": 20,
      "uncorrected": 0.8762666666666666,
      "corrected": 0.8028666666666666,
      "family_wise": 0.9956666666666667
    },
    {
      "dyads": 24,
      "uncorrected": 0.9286333333333333,
      "corrected": 0.8995666666666666,
      "family_wise": 0.9996666666666667
    },
    {
      "dyads": 28,
      "uncorrected": 0.9581333333333333,
      "corrected": 0.9467666666666666,
      "family_wise": 1.0
    },
    {
      "dyads": 32,
      "uncorrected": 0.9788666666666667,
      "corrected": 0.9748,
      "family_wise": 1.0
    }
  ],
  "design": {
    "dyads": 16,
    "contrasts": 5,
    "alpha": 0.05,
    "effect_d": 0.73,
    "correction": "Holm-Bonferroni, pre-registered"
  }
}

written to /home/gausms/kortex_ws/recordings/baselines/analysis_pipeline_validation.json
```

## `sim_to_real` -- The simulation-to-hardware gap

*ANALYSIS OF RECORDED HARDWARE DATA, not a simulation. Tagged separately so its numbers are never read as one*

```
weak: by DIRECTION leaks through the +/- pair, and by AXIS is the honest one:
  arm      identity        fit    HO by dir    HO by AXIS
  left      7.240 mm    0.976 mm     1.937 mm       93.5 mm
  right     7.215 mm    1.134 mm     2.265 mm       93.6 mm

TIERS. Held-out RMS (mm) per tier per hold-out scheme. The AXIS column is the one that decides whether a tier may be used for a direction nobody measured:
  arm    tier                     by DIR     by AXIS
  left   identity                               7.24  <- identity 7.24
  left   scalar                     3.25        3.52  GENERAL
  left   diagonal                   2.74        7.24  measured directions only
  left   full3x3                    1.94       93.50  measured directions only
  right  identity                               7.21  <- identity 7.21
  right  scalar                     3.40        3.76  GENERAL
  right  diagonal                   2.69        7.21  measured directions only
  right  full3x3                    2.27       93.58  measured directions only

CROSS-ARM (fit on one, predict the other, never seen):
  left_to_right     7.215 mm ->    4.349 mm
  right_to_left     7.240 mm ->    4.311 mm

THE JOINT-TERMINAL MODEL. One parameter per arm: every joint parks EPS short,
on the side it came from. Predicted cartesian error = J . (-EPS * sign).
  arm       EPS deg    identity      fitted HELD OUT axis signs scrambled
  left       0.3052     7.24 mm     1.83 mm       1.88 mm       7.19 mm
  right      0.3059     7.21 mm     1.68 mm       1.72 mm       7.16 mm
  the 3x3 scores 93.5 mm on the same axis hold-out. This model is ONE parameter and
  it generalises, because it is about the joints and not about a direction basis.

wrote recordings/baselines/sim_to_real_gap.json

THE HEADLINE. The error is a per-joint TERMINAL OFFSET of 0.3052 deg (left), 0.3059 deg (right), not a Cartesian gain.
  Removed, HELD OUT on an axis the fit never saw: left 74%, right 76%.
  Compensation is to overshoot each joint by EPS in the direction of travel. That needs
  no direction basis and no step length, so unlike the 3x3 it applies to ANY move.

WHAT STILL NEEDS THE ARMS. The recording does not store the commanded joint
  trajectory, so the SIGN of each joint's travel is taken from the sign of its
  recorded error. The magnitude and the generalisation are measured; 'the joint
  parks on the side it came from' is the mechanism they are consistent with, and
  one hardware run confirms or kills it.
```

## `teleop_orientation_cost` -- What the pinned wrist costs

*mean reach per direction under each orientation policy*

> needs a live /compute_ik; re-run with --with-stack


## `tasks_t1` -- T1, both stages, over the whole densified path

*N=10 repeats per pose, densified to 20 mm, both arms confirmed at home before any sample*

> needs a live /compute_ik; re-run with --with-stack


## `tasks_msc` -- The MSc task set, every coordinate

*IK success and wearer clearance for every waypoint of T0, T1, T2 and T3*

> needs a live /compute_ik; re-run with --with-stack


## `tasks_dance` -- The demonstration routines

*every waypoint of the three routines*

> needs a live /compute_ik; re-run with --with-stack

