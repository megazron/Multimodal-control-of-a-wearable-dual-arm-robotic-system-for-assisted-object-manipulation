# Simulation campaign 20260831_005008

Started 2026-08-31T00:50:08, finished 2026-08-31T00:50:09.

| stage | group | tier | status | s |
| --- | --- | --- | --- | --- |
| `teleop_replay` | teleoperation | recorded | **ok** | 0.9 |

## `teleop_replay` -- The smoother replayed over a recorded session

*the same two quantities as the constructed test, on 20 440 frames of a person actually driving the master. The INPUT is a recording; what is simulated is the rest of the pipeline run offline over it*

```
THE SMOOTHER ON A REAL OPERATOR'S HAND, replayed offline

left arm: 20440 frames over 755 s (27.1 Hz), 60 still segments (15994 frames), 8 moving segments (365 frames)
  commanded speed: median 0.000 m/s, 95th 0.119 m/s
  law                      still_mm     lag_mm
  none (raw)                   3.74       0.00
  ema(0.3)  [was]              3.55      10.78
  one_euro  [now]              3.40       2.13

right arm: 20440 frames over 755 s (27.1 Hz), 39 still segments (15530 frames), 8 moving segments (356 frames)
  commanded speed: median 0.000 m/s, 95th 0.147 m/s
  law                      still_mm     lag_mm
  none (raw)                   2.23       0.00
  ema(0.3)  [was]              1.94      10.27
  one_euro  [now]              1.78       3.53

wrote recordings/baselines/teleop_replay.json
```
