# T6 — compliant coupled transport

**Identical to T3 in every respect except the coupling.** Same arms, same
paths, same conditions, same analysis pipeline (`coupled_metrics.py`). That is
what makes the comparison clean: the only manipulated variable is rigid vs
compliant.

## The scientific point

Rigid coupling transmits any relative error **instantly** — the tray tilts the
moment the two grippers differ in height, and the error is visible and
correctable from the start. Compliant coupling **absorbs** small errors into
the sag and then fails **nonlinearly**: nothing happens, nothing happens, the
ball leaves.

**PREDICTION, pre-registered:** divided attention hurts more under RIGID
coupling, because compliant coupling forgives the small continuous errors that
divided attention produces. The competing prediction — that compliance hurts
more because the failure is unanticipated — is equally publishable and is why
both are stated in advance.

## Object — 350 mm sling, 40 mm ball

```
  ◄──────────────────  350 mm  ──────────────────►
  ┌────┐                                    ┌────┐
  │tab │════════════════════════════════════│tab │      0.5 mm polypropylene
  └────┘            40 mm wide              └────┘      or stiff fabric
   80×25 mm                                  80×25 mm
                      ●  40 mm ball, free-rolling, resting in the sag
```

- length 350 mm between the inner tab edges
- tabs 80 × 25 mm, gripper closes across the 25 mm
- ball 40 mm diameter, not attached

## Verified geometry

```
sag(s) = sqrt((L/2)^2 - (s/2)^2)      ball retained while sag >= 2r

  separation   200    280    310    330    340    349    360 mm
  sag          143.6  105.0   90.1   58.3   41.5   13.2    0.0 mm
  retained     yes    yes    yes    yes    yes     NO     NO
```

Geometric retention limit **s ≤ 340.7 mm**.

**MEASURED against the arms**, 3 repeats per separation at the mid-waypoint
(0.00, 0.35, 1.15):

| | |
| --- | --- |
| reachable separation | **280 – 360 mm** |
| retained AND reachable | **280 – 340 mm** |
| **nominal for the protocol** | **310 mm** |

**Why 350 mm is the right length:** the 340 mm failure threshold sits INSIDE
the reachable band. A longer sling would put failure out of reach and the task
could never fail, which would measure nothing. The arms cannot close below
280 mm, so the operator works close to the threshold — a thin margin, on
purpose.

**Gripper height difference matters too:** the ball rolls to the low side and
escapes there, so `height_difference_mm` is kept SIGNED.

## Metrics (all from TF, no vision)

- gripper separation error from nominal, **continuous** — headline
- gripper height difference, signed, continuous
- ball retained (binary)
- completion time; path efficiency (straight-line / actual)
- time above the 340 mm failure threshold

## Scenarios

The four verified in `scenarios_verified.yaml`, identical to T3:
S1 short straight, S2 long with height change, S3 curved detour,
S4 tight (330 mm nominal, 10 mm from failure).
