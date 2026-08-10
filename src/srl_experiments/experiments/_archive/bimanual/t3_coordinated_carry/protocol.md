# T3 — COORDINATED CARRY

**Both arms grip a tray too wide for one gripper. Lift, transport, place.
A loose ball on the tray makes coordination error visible and physical.**

**HEADLINE METRIC: inter-arm coordination error, logged CONTINUOUSLY.**
This is the measurement nothing single-arm can produce, and it is the reason
the task exists.

> **NEEDS THE LATERAL AXIS, WHICH IS NOT FIXED.** The transport leg is
> predominantly lateral, and lateral is the one axis that has never tracked
> (azimuth from j1 alone, coupled to elevation). Build T3 **after** T2 and T5,
> and do not run it for data until either the lateral axis is resolved or the
> transport leg is re-planned to be fore/aft. See "Lateral dependency" below.

**Orientation:** position-only is sufficient *if* the tray handles are
vertical-grip. Wrist orientation is NOT commanded, so the tray's yaw is
whatever the two grip points impose. This constrains the tray design below.

---

## Object — tray, 300 × 200 mm

```
        ◄──────────── 300 mm ────────────►
     ┌──┬──────────────────────────────┬──┐   ▲
     │██│                              │██│   │
     │██│         flat surface         │██│  200 mm
     │██│          with 8 mm lip       │██│   │
     └──┴──────────────────────────────┴──┘   ▼
      ▲                                 ▲
      └── grip block 40 x 30 x 25 mm ───┘
          vertical faces, one each end
          gripped ACROSS the 30 mm dimension

     tray mass ~250 g,  8 mm lip retains the ball at rest
     grip separation 300 mm — a 2F-85 spans 85 mm, so ONE gripper
     cannot hold it: the task is bimanual by construction
```

**Ball:** 40 mm diameter, ~30 g, free to roll. The 8 mm lip retains it while
level; it escapes past roughly 11°, which is the point.

**Why the grip blocks are vertical-faced.** With `orientation_mode: fixed` the
wrist cannot rotate to match a handle angle. Vertical faces let a fixed wrist
grip correctly at both ends.

---

## Fixed layout — NO APRILTAGS

| item | x | y | z |
| --- | --- | --- | --- |
| tray grip, −x end (RIGHT arm) | −0.155 | 0.35 | 1.10 |
| tray grip, +x end (LEFT arm) | +0.155 | 0.35 | 1.10 |
| lift waypoints | ±0.155 | 0.35 | 1.15 … 1.30 |
| detour waypoints (S3) | ±0.155 | **0.38** | 1.20, 1.25 |

**CORRECTED 2026-08-08.** The previous figures were at table height
(z 0.885) and placed the tray at y 0.16–0.34; audited at N=10, **five of the
six were reachable by no arm**. The transport is a VERTICAL lift in the
z 1.10–1.30 band — carrying toward the wearer (y 0.35 → 0.25) measured 0/20.

The S3 detour is at **y = 0.38, not 0.40**. y = 0.40 measured 0/5 at z 1.15,
1.25 and 1.30 and only 2/5 at 1.20 — it is not a usable detour at a 310 mm
separation, and the earlier re-route moved the top of the detour while
keeping the y that was the actual problem.

> **These coordinates are generated and verified, not typed.** They come
> from `scenarios_verified.yaml`, which is written by
> `scripts/verify_scenarios.py` / `verify_t2_scenarios.py` and re-checked
> by `scripts/audit_scenario_reachability.py` at **N=10 repeats over the
> whole densified path**. If you change a number here, change it there
> and re-run; a figure that only lives in this document has not been
> verified by anything.

Grip points are 300 mm apart in x and share y and z. The **place** station is
180 mm toward the wearer in **y** — see the lateral note.

### Lateral dependency, stated explicitly

As laid out, the transport is **fore/aft (−y)**, which is an axis that tracks
correctly. That is deliberate: it makes T3 runnable before the lateral fix.

An **optional lateral variant** moves the place station to `x ± 0.30` instead,
making transport lateral. **Do not run the lateral variant until the azimuth
problem is resolved** — coordination error would be dominated by a known
input defect and the result would say nothing about coordination.

Record which variant was used in the trial log as `transport_axis`.

---

## THE HEADLINE METRIC — inter-arm coordination error

Computed from TF at 50 Hz, **every sample, for the whole motion**, not
pass/fail at the end:

```
    left EE                          right EE
       ●                                ●
       │╲                              ╱
       │ ╲___________________________ ╱
       │              tray             ╲
       │                                │
       └──── Δz = |z_left − z_right| ───┘

    tilt = atan2(Δz, 0.300)          separation is 300 mm

       Δz = 20 mm  ->   3.8°   visible wobble
       Δz = 60 mm  ->  11.3°   the ball slides off
```

Logged per sample:

| column | meaning |
| --- | --- |
| `dz_mm` | \|z_left − z_right\| |
| `tilt_deg` | `degrees(atan2(dz, 0.300))` |
| `separation_mm` | ‖EE_left − EE_right‖ — should stay 300 ± 10; drift means a grip is slipping |
| `sep_error_mm` | separation − 300 |

Derived per trial: `tilt_max_deg`, `tilt_rms_deg`, `time_above_3_8_deg`,
`time_above_11_3_deg`, `sep_error_max_mm`.

**`tilt_rms_deg` over the transport phase is the primary outcome.** Max alone
rewards a trial that was terrible once and fine otherwise; RMS is what the
ball actually integrates.

---

## Trial structure

1. Scan gate; presence check.
2. Both grippers close on the grip blocks. Record `separation_at_grip`.
3. **Lift** to the waypoint (125 mm).
4. **Transport** to the place station.
5. **Place** and open both grippers.

Timeout 120 s. Phases are logged so tilt can be reported per phase — lift,
transport and place load coordination differently.

**Conditions:**

| | |
| --- | --- |
| **DIRECT** | operator drives both arms, switching attention |
| **ASSISTED** | autonomy holds the LEFT arm's height, operator drives the right; the tray becomes a one-armed task with a stabilised partner |
| **SHARED** | autonomy maintains Δz between the arms and assists both grasps |

---

## Success detection — `/joint_states` and TF only

**Success = all three:**

1. Both grippers still **closed** at the destination — knuckle position below
   the free-air threshold (0.74 rad) at the moment the EEs reach the place
   station.
2. `tilt_max_deg < 10°` through the whole motion.
3. Both EEs within 30 mm of their place points.

### Coarse presence check

- **At grip:** both grippers must stop short of free-air closure
  (< 0.74 rad). The grip blocks are 30 mm across, so a correct grip lands near
  0.58 rad. Full closure = **the tray is absent or was knocked**, logged
  `tray_absent`, and the trial does not start.
- **Separation sanity:** if `separation_mm` leaves 300 ± 40 at any point, one
  gripper has lost the tray. Recorded as `grip_lost` — a mechanical failure,
  distinct from poor coordination, and it must not be scored as high tilt.
- **Ball:** presence is not sensed. Ball retention is scored by the
  experimenter as a binary observation and logged as `ball_retained`; it is a
  secondary outcome, and `tilt_rms_deg` is the primary because it is measured
  rather than judged.

---

## Analysis

`analyse_t3.py`. Per condition: tilt RMS and max, time above each threshold,
separation error, per-phase breakdown, and the DIRECT→ASSISTED difference.
Tilt is plotted as a time series per trial — a coordination failure has a
shape, and a single number hides whether it was a step, a drift or a wobble.
