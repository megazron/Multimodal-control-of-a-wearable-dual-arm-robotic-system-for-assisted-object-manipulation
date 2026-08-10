# T2 — HOLD AND FILL

> **Task 2 has no DIRECT or VR condition.** Teleoperated grasping is not
> achievable with this master: a top-down grasp needs 169.7 deg (left) /
> 164.6 deg (right) from the pinned wrist anchor and no channel commands it.
> Task 2 runs in modes 4 and above only, as a capability demonstration, and
> yields no comparative measure. See `docs/research/09_task2_grasping_finding.md`.


**Left holds an open box steady; right drops graded blocks into it.**

The failure mode IS box movement. A single arm cannot do this task at all: the
box must be held while the other hand works, and the whole measurement is how
much the holding arm drifts while the working arm is busy.

**Feasible with position-only control.** Both roles need position and a
gripper. Neither needs commanded wrist orientation, so this runs on the
current rig with `orientation_mode: fixed`. Build first, with T5.

---

## Objects

### Box — open-top, light

```
            120 mm
      ┌───────────────┐
      │               │  ┌── 15 mm grippable lip, all four sides
      │   ┌───────┐   │  │   (outer wall to inner wall)
      │   │       │   │◄─┘
      │   │ open  │   │        80 mm deep (into page)
      │   │       │   │
      │   └───────┘   │
      └───────────────┘
                          60 mm tall
      ╔═══════════════╗   ─┬─
      ║               ║    │  wall height 60
      ║               ║    │
      ╚═══════════════╝   ─┴─
      └──── grip here ────┘
        lip at 45 mm above table

  outer 120 x 80 x 60 mm     mass ~150 g       lip 15 mm
```

The lip is the only grippable feature. 15 mm against the 2F-85's 85 mm span
means the grip is shallow — a small rotation of the holding wrist lifts the
box off the pad, which is precisely the sensitivity this task probes.

### Blocks — graded, 3 sizes

```
   A: 25 x 25 x 25 mm   ~20 g   easy      (clears the 90 mm opening by 65 mm)
   B: 40 x 40 x 40 mm   ~60 g   moderate
   C: 55 x 55 x 55 mm  ~120 g   hard      (clears by only 35 mm)
```

Three of each per trial, presented A, B, C, A, B, C, A, B, C.

---

## Fixed layout — measured once, stored, NO APRILTAGS

All positions in the `world` frame, in metres. Measure once with a rule from
the marked table origin and store in `layout.yaml`; re-measure only if the
table or the wearer's chair moves.

```
                        +y (front, away from wearer)
                         ▲
        BLOCK FEED       │        BOX STATION
        (right arm)      │        (left arm)
      ┌─────────────┐    │      ┌─────────────┐
      │  A   B   C  │    │      │   ┌─────┐   │
      │  ○   ○   ○  │    │      │   │ box │   │
      └─────────────┘    │      │   └─────┘   │
                         │      └─────────────┘
   ──────────────────────┼──────────────────────► +x (wearer's right)
                         │
                      WEARER
```

**CORRECTED 2026-08-08.** The table below used to place everything at table
height (z 0.87–0.95) on the wearer's left. Audited at N=10, **four of those
five coordinates were reachable by no arm at all** — they predate the move of
this whole task to the chest-height band. The verified layout:

| item | x | y | z | arm | note |
| --- | --- | --- | --- | --- | --- |
| container handle (hold) | −0.15 | 0.35 | 1.10 | RIGHT | held for the whole trial |
| container opening centre | +0.15 | 0.35 | 1.20 | — | 300 mm across, 100 mm above the handle |
| block A pick | +0.30 | 0.35 | 1.15 | LEFT | S1, short transit |
| block B pick | +0.35 | 0.35 | 1.15 | LEFT | S3 |
| block C pick | +0.45 | 0.35 | 1.15 | LEFT | S2, long transit |
| block D pick | +0.35 | 0.35 | 1.25 | LEFT | S4, tight tolerance |
| stand top | | | 1.05 | — | the task is on a STAND, not a table |

The arm assignment is **right holds, left fills** — the opposite of what an
earlier draft said. `left_*` links sit at POSITIVE x in this model, so the
"left" arm works the +x side.

> **These coordinates are generated and verified, not typed.** They come
> from `scenarios_verified.yaml`, which is written by
> `scripts/verify_scenarios.py` / `verify_t2_scenarios.py` and re-checked
> by `scripts/audit_scenario_reachability.py` at **N=10 repeats over the
> whole densified path**. If you change a number here, change it there
> and re-run; a figure that only lives in this document has not been
> verified by anything.

Marked on the table with tape crosses at each pick point and a box footprint
outline. The outline is also the success criterion — see below.

---

## Trial structure

One trial = one box hold plus 9 block transfers.

1. **Scan gate.** `/recovery/ready` must return success. No trial starts on a
   fault, and the coarse presence check below must pass.
2. Left arm approaches the lip grasp point and closes. **Record
   `hold_origin`** — the left EE pose at the moment the gripper closes.
3. For each of the 9 blocks: right arm picks, transports, releases above the
   box opening, returns to a neutral pose.
4. Trial ends after the 9th release or at `timeout_s` (180 s).

**Conditions** — 3, order counterbalanced by Williams square across
participants:

| | left (holding) arm | right (working) arm |
| --- | --- | --- |
| **DIRECT** | operator | operator, switching attention |
| **ASSISTED** | **autonomy holds** the box at `hold_origin` | operator |
| **SHARED** | autonomy holds | autonomy assists the grasp |

ASSISTED is the condition the task exists for: it removes the holding load
entirely, so the difference DIRECT → ASSISTED is the cost of dividing
attention between two arms.

---

## Metrics

**Primary — box disturbance.** Computed from TF continuously at 50 Hz:

- `box_drift_max_mm` — max ‖left_EE − hold_origin‖ during the trial
- `box_drift_rms_mm`
- `hold_joint_excursion_deg` — max |q − q_at_close| over the left arm's 7 joints

**Secondary**

- `blocks_placed` of 9, and per grade (A/B/C)
- `time_per_block_s`, median and IQR
- `drops` — released outside the box footprint
- `regrasps` on the box (holding gripper reopened)

---

## Success detection — `/joint_states` and TF only

A block counts as **placed** when both hold:

1. **Released inside the footprint.** At the moment the right gripper opens
   (knuckle position crosses below `open_thresh` = 0.10 rad after having been
   closed), the right EE lies inside the box footprint in x/y, expanded by
   10 mm, and within 120 mm above the opening centre.
2. **The holding arm did not move.** Over a ±0.5 s window around that release,
   `‖left_EE − hold_origin‖ < 15 mm` **and** every left joint within
   `2 deg` of its value at close.

Criterion 2 is the point of the task. A block that lands in a box that was
dragged 40 mm is not a success.

### Coarse presence check — a missing object must not read as a failed grasp

Before each pick, and after each release:

- **Gripper closure signature.** A gripper that has closed on an object stops
  short of free-air closure. Free air on the 2F-85 reaches ≈0.79 rad;
  a 25 mm block stops near 0.62 rad, a 55 mm block near 0.38 rad.
  `present = closed_position < 0.74 rad`. If the gripper closes fully to
  free-air, **the object was absent or knocked away** — logged as
  `object_absent`, NOT as a grasp failure.
- **Box still there.** If the left gripper closes past 0.74 rad at any point
  during the hold, the box has been dropped or was never grasped; the trial is
  marked INVALID with cause `box_lost`, not scored as poor holding.

These two are the difference between "the operator failed" and "the scene was
wrong", and without them the second is silently recorded as the first.

---

## Analysis

`analyse_t2.py` — see the script alongside. Reports per condition: placed/9,
box drift max and RMS, hold joint excursion, time per block by grade, and the
DIRECT→ASSISTED difference with a paired test across participants.

Invalid trials are excluded exactly as pre-registered
(`srl_experiments/validity.py`); a session where every trial is invalid fails
loudly rather than producing an empty result.
