# Simulation campaign 20260831_002055

Started 2026-08-31T00:20:55, finished 2026-08-31T00:28:36.

| stage | group | tier | status | s |
| --- | --- | --- | --- | --- |
| `teleop_orientation_cost` | teleoperation | stack | **failed** | 196.5 |
| `tasks_t1` | robot tasks | stack | **ok** | 132.8 |
| `tasks_msc` | robot tasks | stack | **ok** | 47.7 |
| `tasks_dance` | robot tasks | stack | **ok** | 37.7 |

## `teleop_orientation_cost` -- What the pinned wrist costs

*mean reach per direction under each orientation policy*

```
  left  +1-1+1   pinn 0.900(CAP)  spin 0.900(CAP)  cone 0.900(CAP)  cone 0.900(CAP)  free 0.900(CAP)   +spin 0.000  +free 0.000
  left  +1+1-1   pinn 0.175(UNRE)  spin 0.175(UNRE)  cone 0.225(UNRE)  cone 0.275(UNRE)  free 0.275(UNRE)   +spin 0.000  +free 0.100
  left  +1+1+1   pinn 0.300(UNRE)  spin 0.300(UNRE)  cone 0.325(UNRE)  cone 0.350(UNRE)  free 0.350(UNRE)   +spin 0.000  +free 0.050
  right +1+0+0   pinn 0.400(WEAR)  spin 0.400(WEAR)  cone 0.375(WEAR)  cone 0.475(WEAR)  free 0.400(WEAR)   +spin 0.000  +free 0.000
  right -1+0+0   pinn 0.350(UNRE)  spin 0.350(UNRE)  cone 0.400(UNRE)  cone 0.500(UNRE)  free 0.500(UNRE)   +spin 0.000  +free 0.150
  right +0+1+0   pinn 0.200(UNRE)  spin 0.200(UNRE)  cone 0.225(UNRE)  cone 0.250(UNRE)  free 0.250(UNRE)   +spin 0.000  +free 0.050
  right +0-1+0   pinn 0.900(CAP)  spin 0.900(CAP)  cone 0.900(CAP)  cone 0.900(CAP)  free 0.900(CAP)   +spin 0.000  +free 0.000
  right +0+0+1   pinn 0.750(UNRE)  spin 0.750(UNRE)  cone 0.775(UNRE)  cone 0.800(UNRE)  free 0.800(UNRE)   +spin 0.000  +free 0.050
  right +0+0-1   pinn 0.300(UNRE)  spin 0.300(UNRE)  cone 0.375(UNRE)  cone 0.450(UNRE)  free 0.475(UNRE)   +spin 0.000  +free 0.175
  right -1-1-1   pinn 0.550(UNRE)  spin 0.550(UNRE)  cone 0.600(UNRE)  cone 0.725(UNRE)  free 0.850(UNRE)   +spin 0.000  +free 0.300
  right -1-1+1   pinn 0.750(UNRE)  spin 0.875(UNRE)  cone 0.900(CAP)  cone 0.900(CAP)  free 0.900(CAP)   +spin 0.125  +free 0.150
  right -1+1-1   pinn 0.200(UNRE)  spin 0.200(UNRE)  cone 0.225(UNRE)  cone 0.275(UNRE)  free 0.275(UNRE)   +spin 0.000  +free 0.075
  right -1+1+1   pinn 0.300(UNRE)  spin 0.300(UNRE)  cone 0.325(UNRE)  cone 0.350(UNRE)  free 0.350(UNRE)   +spin 0.000  +free 0.050
  right +1-1-1   pinn 0.200(UNRE)  spin 0.225(WEAR)  cone 0.225(WEAR)  cone 0.475(WEAR)  free 0.450(WEAR)   +spin 0.025  +free 0.250
  right +1-1+1   pinn 0.075(WEAR)  spin 0.425(WEAR)  cone 0.475(WEAR)  cone 0.475(WEAR)  free 0.475(WEAR)   +spin 0.350  +free 0.400
  right +1+1-1   pinn 0.275(UNRE)  spin 0.275(UNRE)  cone 0.300(UNRE)  cone 0.325(UNRE)  free 0.325(UNRE)   +spin 0.000  +free 0.050
  right +1+1+1   pinn 0.425(UNRE)  spin 0.425(UNRE)  cone 0.450(UNRE)  cone 0.450(UNRE)  free 0.450(UNRE)   +spin 0.000  +free 0.025

CONTROL 3 FAILED on 6 walk(s): a LOOSER policy reached LESS far. That is arithmetically impossible -- the optimiser drew this boundary, not the arm. Raise --seeds and re-run.
    left  -1+0+0  cone15 0.400 > cone45 0.375
    left  -1-1-1  cone45 0.500 > free 0.475
    left  -1-1+1  cone15 0.500 > cone45 0.450
    right +1+0+0  spin 0.400 > cone15 0.375
    right +1+0+0  cone45 0.475 > free 0.400
    right +1-1-1  cone45 0.475 > free 0.450

MEAN REACH over 28 walks
  pinned  0.408 m   (4 walks bound by the WEARER)
  spin    0.446 m   (6 walks bound by the WEARER)
  cone15  0.479 m   (6 walks bound by the WEARER)
  cone45  0.519 m   (6 walks bound by the WEARER)
  free    0.529 m   (6 walks bound by the WEARER)

  unpinning the ROLL alone is worth +38 mm of mean reach
  a 45 deg cone is worth            +111 mm
  position only is worth            +121 mm

  The floor was 0.15 m under every policy and was never relaxed.

wrote /home/gausms/kortex_ws/recordings/baselines/orientation_cost.json
```

## `tasks_t1` -- T1, both stages, over the whole densified path

*N=10 repeats per pose, densified to 20 mm, both arms confirmed at home before any sample*

```
CONTROLS
   inside_torso_is_negative               -0.1595 at [0.05, 0.02, 1.22]
   far_1.6m_unreachable                   True
   shipped_cube_reachable_and_clear       True / 0.2202
   slab_control_free_then_buried          True -> False (want True -> False)

T1 STAGE 1 (both arms, 4 cubes, 2 pads)
   approach   elevation -10.0 deg, heading -50.0 deg inboard, roll 0.0
   table top  1.250, objects rest at z = 1.270 (float gap 0)
   pads       [[0.29, 0.5], [-0.29, 0.5]]   290 mm and 290 mm off the centreline
   wrist      through the FOLLOWER'S policy: wrist within a 15 deg cone of the commanded approach axis; roll free within it
   waypoints  left 199, right 199, N=10, floor 0.150 m

   left  199 waypoints: 0 IK failures, 0 inside the 150 mm floor
         worst clearance 0.2202 m to torso, at waypoint 0

   right 199 waypoints: 0 IK failures, 0 inside the 150 mm floor
         worst clearance 0.2202 m to torso, at waypoint 0

THE GRASP ITSELF, from FK on the finger tips
   cube_0 left  across the closing axis  56.3 mm (hand opens 85)  pad miss  9.57 mm  lowest finger tip  +21.3 mm above the table  tilt  5.0 deg  
   cube_1 left  across the closing axis  56.3 mm (hand opens 85)  pad miss  9.57 mm  lowest finger tip  +21.3 mm above the table  tilt  5.0 deg  
   cube_2 right across the closing axis  56.3 mm (hand opens 85)  pad miss  9.58 mm  lowest finger tip  +21.3 mm above the table  tilt  5.0 deg  
   cube_3 right across the closing axis  56.3 mm (hand opens 85)  pad miss  9.58 mm  lowest finger tip  +21.3 mm above the table  tilt  5.0 deg  

========================================================================
VERDICT: CLEAN -- every waypoint of both arms solves and keeps the floor
4005 IK calls -> /home/gausms/kortex_ws/recordings/baselines/t1_paths.json
```

## `tasks_msc` -- The MSc task set, every coordinate

*IK success and wearer clearance for every waypoint of T0, T1, T2 and T3*

```
==========================================================================
MSc TASKS T0 / T2 / T3 -- N=10, bench in scene, both arm assignments
==========================================================================

CONTROLS
   1.6 m out            -> unreachable (want unreachable)
   inside the wearer    -> unreachable (want unreachable)
   INSIDE THE BENCH     -> unreachable (want unreachable)
   a known-good pick    -> reachable (want reachable)

T0  sphere pointing -- NO FURNITURE (free space), transits, and 20 sampled trials
   left  0 sphere failures, 0 transit failures
   right 0 sphere failures, 0 transit failures
   sampled: 120 poses, 0 unreachable

T1  pick and place -- NOT VERIFIED HERE.
    Two arms, two stages, a seeded layout and a centreline neither
    arm can cross. scripts/verify_t1.py walks it at N=10 with the
    wearer floor and reads the finger tips out of FK.

T2  coordinated carry -- both grippers, 500 mm span, both assignments (scene: table)
   S1_short_lift     6 waypoints, 0 failures
   S2_full_lift     16 waypoints, 0 failures
   S3_detour        18 waypoints, 0 failures

T3  circuit box and multimeter -- hold poses and the full picks (scene: table)
   circuit_box  right arm, 13 waypoints, 0 failures
   multimeter   left  arm, 13 waypoints, 0 failures

CLIP PATHS  the waypoints record_abc_sweep actually drives
   t0  target reaching              L  25 distinct 0 fail   R  25 distinct 0 fail
   t1  pick and place, colour matched L  46 distinct 0 fail   R  46 distinct 0 fail
   t1s2 pick and place, both arms at once L  45 distinct 0 fail   R  49 distinct 0 fail
   t2  coordinated carry            L  11 distinct 0 fail   R  11 distinct 0 fail
   t3  circuit box and multimeter   L   8 distinct 0 fail   R   8 distinct 0 fail
   274 distinct clip waypoints tested at N=10

==========================================================================
  6024 IK calls   TOTAL FAILURES: 0
  -> /home/gausms/kortex_ws/recordings/baselines/msc_verification.json
```

## `tasks_dance` -- The demonstration routines

*every waypoint of the three routines*

```
wrist: workspace anchor; the home wrist is left 42.9, right 27.3 deg from WORKSPACE_ORIENT
wrist policy: wrist within a 15 deg cone of the commanded approach axis; roll free within it
CONTROLS  1.6 m out unreachable {'left': True, 'right': True} | inside the wearer unreachable {'left': True, 'right': True}

DANCE PATHS, N=3, every 2th waypoint, collision-aware
  d1  dance: flow         0 of  382 waypoints FAIL
  d2  dance: pulse        0 of  356 waypoints FAIL
  d3  dance: play         0 of  560 waypoints FAIL

3898 IK calls, 0 failures
Envelope clear. |x| 0.47..0.76  y 0.22..0.44  z 1.02..1.58
-> /home/gausms/kortex_ws/recordings/baselines/dance_paths.json
```
