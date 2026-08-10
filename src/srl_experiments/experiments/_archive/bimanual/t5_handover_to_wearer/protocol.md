# T5 — HANDOVER TO THE WEARER

**The robot brings a tool to a fixed position at the wearer's waist while the
wearer's own hands are busy. The wearer presses a button on receipt.**

The only single-arm task in the set, and deliberately so: it is the control
against which the bimanual tasks are read, and it is the one task whose
outcome depends on a human judgement rather than on geometry.

**Feasible with position-only control.** One arm, position and gripper.
Orientation is fixed throughout, and the receiving pose is chosen so a fixed
wrist presents the tool usably. Build first, with T2.

---

## Object — tool-shaped, ~200 g

```
        ╔════════════════════════╗
        ║                        ║   handle 130 x 32 x 24 mm
        ╚════════════════════════╝   knurled or taped for grip
        │◄────── 130 mm ───────►│
                                 ╲
                                  ╲  head 55 x 40 x 24 mm
                                   ╲ offset mass toward the head
                                    ╲___
        total length 185 mm     mass ~200 g

  GRIPPED across the handle at 24 mm depth, 40 mm from the butt end,
  leaving the head clear so the wearer takes it head-first.
```

The mass is deliberately offset toward the head: a symmetric object would
hide the wrist-torque problem that a real tool creates.

---

## Fixed layout — measured once, NO APRILTAGS

```
                  +y (front)
                    ▲
     TOOL CRADLE    │
    ┌──────────┐    │
    │   ══╲    │    │
    └──────────┘    │
                    │
   ─────────────────┼─────────────►  +x
                    │
              ╔═════╧═════╗
              ║  WEARER   ║
              ║           ║
              ║   ◄── RECEIVE POINT, at the waist,
              ║       on the wearer's dominant side
              ╚═══════════╝
```

| item | x | y | z | note |
| --- | --- | --- | --- | --- |
| tool cradle grasp | −0.30 | 0.35 | 1.05 | handle centre (S1; S2 is −0.40/1.15) |
| receive point | −0.16 | 0.35 | 1.05 | marked on the harness (S2 is −0.20/1.10) |
| retreat | −0.16 | 0.35 | 1.13 | 80 mm clear, above the receive point |

**CORRECTED 2026-08-08.** The previous figures were on the +x side at table
height and at y 0.18; audited at N=10, **two of the three were reachable by
no arm**. The delivering arm is the RIGHT one, which works the −x side, and
the whole delivery — approach, grasp, lift, transit, present, retreat — is
verified densified to 20 mm, not just its endpoints.

> **These coordinates are generated and verified, not typed.** They come
> from `scenarios_verified.yaml`, which is written by
> `scripts/verify_scenarios.py` / `verify_t2_scenarios.py` and re-checked
> by `scripts/audit_scenario_reachability.py` at **N=10 repeats over the
> whole densified path**. If you change a number here, change it there
> and re-run; a figure that only lives in this document has not been
> verified by anything.

**The receive point is fixed and marked** — a tape cross on the wearer's
harness. It is NOT chosen per trial, so the handover latency measures the
handover, not the search.

Verify the receive point is inside the reachable set before the session:

```bash
python3 scripts/measure_workspace.py --arm right --repeats 10
```

The point must sit inside the **median** reach in its direction; if it only
falls inside the max, the arm will reach it on some trials and not others and
the latency distribution becomes bimodal for a reason that has nothing to do
with the participant.

---

## Wearer's hands are busy — and this must be enforced, not requested

The wearer performs a continuous two-handed task throughout: a peg-transfer
board placed at chest height, moving pegs left to right at a comfortable pace.
The receipt button is a **foot pedal or a button mounted on the peg board**, so
pressing it does not require abandoning the task.

If the wearer can simply wait with empty hands for the tool, the task measures
reaction time and nothing else.

---

## Trial structure

One trial = one delivery.

1. **Scan gate** — `/recovery/ready` success; presence check passes.
2. Wearer begins the peg task. A 10 s settling period is logged but not scored.
3. Robot picks the tool from the cradle.
4. Robot transports to the receive point and **holds station**.
   `t_arrival` = the first sample where the right EE is within 30 mm of the
   receive point and its speed is below 20 mm/s.
5. Wearer takes the tool and presses the button. `t_press`.
6. **Handover latency = `t_press − t_arrival`.**
7. Robot opens the gripper on the press, then retreats.

Trial ends on the press or at `timeout_s` (60 s). A timeout is a **failed
handover**, recorded as such, not discarded.

**Conditions** — 3:

| | |
| --- | --- |
| **DIRECT** | operator teleoperates the whole delivery |
| **ASSISTED** | autonomy holds station at the receive point once the operator brings it within 100 mm |
| **SHARED** | autonomy performs the approach and holds; operator confirms release |

---

## Metrics

- **`handover_latency_s`** — the headline. `t_press − t_arrival`.
- `t_transport_s` — pick to arrival
- `station_drift_mm` — max EE excursion from the receive point between
  arrival and press. A robot that drifts while waiting makes the wearer chase
  it, and that inflates latency for a reason worth separating.
- `peg_rate_before / peg_rate_during` — the wearer's own task rate, from the
  peg board, as an interference measure
- `failed_handovers` — timeouts
- NASA-TLX after each condition block

---

## Success detection — the wearer's button, plus presence

**Success = the button press.** This is the one task where a human judgement
is the criterion, and it is correct here: only the wearer knows whether they
actually have the tool.

### Coarse presence check

- **Before the trial:** the right gripper closes on the cradle. If the closed
  knuckle position exceeds **0.74 rad** (free-air closure), the tool is absent
  from the cradle — abort BEFORE the delivery, cause `tool_absent`. Never
  during.
- **At the press:** the gripper opens. If the knuckle was already past
  0.74 rad before the press, the tool had been dropped in transit — recorded
  as `dropped_in_transit`, distinct from a slow handover.
- **After retreat:** if the gripper never opened, the wearer pressed without
  taking the tool — `press_without_receipt`, flagged for review rather than
  counted as a success.

---

## Analysis

`analyse_t5.py`. Reports per condition: latency median and IQR, transport
time, station drift, failed handovers, and the wearer's peg-rate interference.
Latency is reported as a distribution, never a mean alone — a bimodal latency
means the receive point is at the edge of the reachable set, and a mean hides
exactly that.
