# T4 — INTER-ARM HANDOVER  ***BLOCKED***

> **BLOCKED pending wrist orientation. Do not build, do not run.**
>
> Under `orientation_mode: fixed` both wrists present the same approach
> direction, so the two grippers approach the same faces of the block and
> collide. This was reached independently from the geometry and from the
> channel data: left j6 and right j3/j5/j7 are dead or incoherent, and left j7
> has only 26 deg of range.
>
> **The stepped-block workaround is NOT to be built.** It removes the
> orientation requirement and with it most of the point, leaving a sequential
> timing task rather than an inter-arm regrasp.
>
> **Stepped trials must NEVER be pooled with oriented ones.**
> `analyse_t4.py` raises `PooledVariants` rather than averaging two different
> experiments.

**Left picks a block low; right grips the same block high; left releases;
right places.** Sequential by construction — the two arms must occupy the same
object at the same time, briefly, without fighting.

**BUILD LAST. This is the hardest task in the set, for two reasons:**

1. **It needs wrist orientation, which we do not have.** During the shared
   grasp both grippers hold one 60 × 60 mm block from different directions.
   With `orientation_mode: fixed` both wrists present the same approach
   direction, so the two grippers approach the same faces and collide. A
   usable handover needs the receiving wrist rotated roughly 90° about the
   block's long axis relative to the giving wrist.
2. The transfer window is a genuine two-arm constraint: release too early and
   the block drops, too late and the arms fight.

**Until the wrist channels are repaired** (left j7 works but has only 26° of
range; left j6 and right j3/j5/j7 do not), T4 can only be run in a **degraded
variant** — see below — and the degraded variant does NOT test what the task
is for.

---

## Object — block, 60 × 60 × 140 mm

```
        ┌────────┐  ▲
        │        │  │
        │        │  │  140 mm tall
        │        │  │
        │  60x60 │  │      LEFT grips the LOWER third across a 60 mm face
        │        │  │      RIGHT grips the UPPER third across the ORTHOGONAL
        │        │  │            60 mm face
        └────────┘  ▼
        ◄── 60 ──►

   mass ~180 g.  Two grip bands marked, 40 mm tall, at 20 mm and 100 mm
   from the base. The 40 mm clear gap between them is the whole margin.
```

## Fixed layout — NO APRILTAGS

| item | x | y | z |
| --- | --- | --- | --- |
| block pick (left) | −0.22 | 0.32 | 0.880 | low, on the table |
| transfer station | −0.05 | 0.28 | 1.060 | where the exchange happens |
| place target (right) | +0.24 | 0.30 | 0.880 |

The transfer station is chosen inside BOTH arms' median reach. Verify before
the session with `scripts/measure_workspace.py --repeats 10`; a transfer point
inside only the max reach will succeed on some trials and not others.

## Trial structure

1. Scan gate; presence check.
2. Left picks the block at the lower band.
3. Left carries to the transfer station and **holds station**.
4. Right approaches and closes on the upper band. **Both now hold.**
5. Left opens. `t_release`.
6. Right carries to the place target and releases.

Timeout 150 s.

**Conditions:** DIRECT / ASSISTED (autonomy holds the giving arm at the
transfer station) / SHARED (autonomy holds and assists the receiving grasp).

## Metrics

- `handover_duration_s` — right-closed to left-open. The coordination window.
- `both_holding_s` — time both grippers are closed on the block.
- `drop` — block released by both
- `transfer_force_proxy_mm` — EE separation change while both hold; if the
  arms fight, they push each other and the separation drifts
- `t_total_s`, `regrasps`

## Success detection — `/joint_states` and TF only

**Success = both:**

1. **Receiving gripper shows an object present** — right knuckle closes and
   stops short of free-air (< 0.74 rad), holding for at least 0.5 s.
2. **Giving gripper opens** — left knuckle rises above 0.10 rad AFTER (1).

Ordering matters and is checked: right-closed-then-left-opened is a handover;
the reverse is a drop that happened to be caught.

### Coarse presence check

- **At pick:** left closure past 0.74 rad = block absent, abort before the
  trial, cause `block_absent`.
- **After left opens:** right must STILL read < 0.74 rad. If the right gripper
  slams shut at the moment the left opens, the block fell through the
  exchange — `dropped_at_transfer`, distinct from a failed grasp.
- **At place:** right opens; the block's presence is not sensed after release,
  so placement is scored by EE position within 30 mm of the target.

## Degraded variant, and what it costs

If wrist orientation is unavailable, use a **stepped block**: 60 × 60 × 140 mm
with the upper band rotated 90° as a physical step, so a fixed wrist can grip
both bands from the same approach direction.

**This removes the orientation requirement and with it most of the point.**
The result then measures the timing of a sequential exchange, not a genuine
inter-arm regrasp. Record `variant: stepped` in the trial log and do not
compare stepped trials against oriented ones.
