# Simulation campaign 20260831_002858

Started 2026-08-31T00:28:58, finished 2026-08-31T00:40:44.

| stage | group | tier | status | s |
| --- | --- | --- | --- | --- |
| `teleop_orientation_cost` | teleoperation | stack | **failed** | 660.5 |

## `teleop_orientation_cost` -- What the pinned wrist costs

*mean reach per direction under each orientation policy*

```
  left  -1-1-1   pinn 0.450(WEAR)  spin 0.500(WEAR)  cone 0.500(WEAR)  cone 0.500(WEAR)  free 0.500(WEAR)   +spin 0.050  +free 0.050
  left  -1-1+1   pinn 0.450(WEAR)  spin 0.500(WEAR)  cone 0.500(WEAR)  cone 0.500(WEAR)  free 0.500(WEAR)   +spin 0.050  +free 0.050
  left  -1+1-1   pinn 0.250(UNRE)  spin 0.250(UNRE)  cone 0.275(UNRE)  cone 0.325(UNRE)  free 0.350(UNRE)   +spin 0.000  +free 0.100
  left  -1+1+1   pinn 0.425(UNRE)  spin 0.425(UNRE)  cone 0.450(UNRE)  cone 0.450(UNRE)  free 0.450(UNRE)   +spin 0.000  +free 0.025
  left  +1-1-1   pinn 0.525(UNRE)  spin 0.525(UNRE)  cone 0.600(UNRE)  cone 0.700(UNRE)  free 0.850(UNRE)   +spin 0.000  +free 0.325
  left  +1-1+1   pinn 0.900(CAP)  spin 0.900(CAP)  cone 0.900(CAP)  cone 0.900(CAP)  free 0.900(CAP)   +spin 0.000  +free 0.000
  left  +1+1-1   pinn 0.175(UNRE)  spin 0.175(UNRE)  cone 0.225(UNRE)  cone 0.275(UNRE)  free 0.275(UNRE)   +spin 0.000  +free 0.100
  left  +1+1+1   pinn 0.300(UNRE)  spin 0.300(UNRE)  cone 0.325(UNRE)  cone 0.350(UNRE)  free 0.350(UNRE)   +spin 0.000  +free 0.050
  right +1+0+0   pinn 0.400(WEAR)  spin 0.400(WEAR)  cone 0.450(WEAR)  cone 0.475(WEAR)  free 0.475(WEAR)   +spin 0.000  +free 0.075
  right -1+0+0   pinn 0.350(UNRE)  spin 0.350(UNRE)  cone 0.400(UNRE)  cone 0.500(UNRE)  free 0.500(UNRE)   +spin 0.000  +free 0.150
  right +0+1+0   pinn 0.200(UNRE)  spin 0.200(UNRE)  cone 0.225(UNRE)  cone 0.250(UNRE)  free 0.250(UNRE)   +spin 0.000  +free 0.050
  right +0-1+0   pinn 0.900(CAP)  spin 0.900(CAP)  cone 0.900(CAP)  cone 0.900(CAP)  free 0.900(CAP)   +spin 0.000  +free 0.000
  right +0+0+1   pinn 0.750(UNRE)  spin 0.750(UNRE)  cone 0.775(UNRE)  cone 0.800(UNRE)  free 0.800(UNRE)   +spin 0.000  +free 0.050
  right +0+0-1   pinn 0.300(UNRE)  spin 0.300(UNRE)  cone 0.375(UNRE)  cone 0.450(UNRE)  free 0.475(UNRE)   +spin 0.000  +free 0.175
  right -1-1-1   pinn 0.550(UNRE)  spin 0.550(UNRE)  cone 0.600(UNRE)  cone 0.725(UNRE)  free 0.850(UNRE)   +spin 0.000  +free 0.300
  right -1-1+1   pinn 0.900(CAP)  spin 0.875(UNRE)  cone 0.900(CAP)  cone 0.900(CAP)  free 0.900(CAP)   +spin -0.025  +free 0.000
  right -1+1-1   pinn 0.200(UNRE)  spin 0.200(UNRE)  cone 0.225(UNRE)  cone 0.275(UNRE)  free 0.275(UNRE)   +spin 0.000  +free 0.075
  right -1+1+1   pinn 0.300(UNRE)  spin 0.300(UNRE)  cone 0.325(UNRE)  cone 0.350(UNRE)  free 0.350(UNRE)   +spin 0.000  +free 0.050
  right +1-1-1   pinn 0.475(WEAR)  spin 0.475(WEAR)  cone 0.475(WEAR)  cone 0.475(WEAR)  free 0.475(WEAR)   +spin 0.000  +free 0.000
  right +1-1+1   pinn 0.475(WEAR)  spin 0.475(WEAR)  cone 0.475(WEAR)  cone 0.475(WEAR)  free 0.475(WEAR)   +spin 0.000  +free 0.000
  right +1+1-1   pinn 0.275(UNRE)  spin 0.275(UNRE)  cone 0.300(UNRE)  cone 0.325(UNRE)  free 0.325(UNRE)   +spin 0.000  +free 0.050
  right +1+1+1   pinn 0.425(UNRE)  spin 0.425(UNRE)  cone 0.450(UNRE)  cone 0.450(UNRE)  free 0.450(UNRE)   +spin 0.000  +free 0.025

CONTROL 3 FAILED on 1 walk(s): a LOOSER policy reached LESS far. That is arithmetically impossible -- the optimiser drew this boundary, not the arm. Raise --seeds and re-run.
    right -1-1+1  pinned 0.900 > spin 0.875

MEAN REACH over 28 walks
  pinned  0.460 m   (6 walks bound by the WEARER)
  spin    0.463 m   (6 walks bound by the WEARER)
  cone15  0.492 m   (6 walks bound by the WEARER)
  cone45  0.525 m   (6 walks bound by the WEARER)
  free    0.542 m   (6 walks bound by the WEARER)

  unpinning the ROLL alone is worth +3 mm of mean reach
  a 45 deg cone is worth            +65 mm
  position only is worth            +82 mm

  The floor was 0.15 m under every policy and was never relaxed.

wrote /home/gausms/kortex_ws/recordings/baselines/orientation_cost.json
```
