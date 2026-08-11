# Archived task sets — kept, not reachable

Three generations of task set were simultaneously launchable from the GUI and
from `run_experiment.sh`, spanning incompatible geometry:

| set | span | status |
| --- | --- | --- |
| `bimanual/` (t2, t3, t5–t9) | **300 / 310 mm** | superseded |
| `final5/` | 500 mm | superseded by `abc/` |
| `e1`–`e6` | — | superseded protocols; PILOT result CSVs kept |
| `abc/` (**live**) | **500 mm**, verified N=10 over the full path | current |

**Why this matters and is not tidying.** A button labelled "T3 rigid carry"
ran the 300 mm span while the current specification is 500 mm. The tilt
threshold rescales with the span — 60 mm of height difference is 11.3° at
310 mm and 6.8° at 500 mm — so a trial run from the wrong set produces a
plausible number against the wrong criterion.

The data and protocols are kept in full. What is removed is the ability to
launch them by accident: `run_experiment.sh` dispatches `a|b|c` only, and the
GUI offers only those.

To read the archived results, point an analyser at the CSVs directly. To
revive a set, move it back and re-verify its coordinates at N=10 first — the
spans and thresholds in it are not interchangeable with the current spec.

---

## THERE ARE TWO T3s. DO NOT RESTORE THIS ONE.

`bimanual/t3_coordinated_carry/` is the **310 mm** generation. It declares its
grips at **|x| = 0.155**, which is **inside the 0.25 m dead band** between the
two arms' disjoint reachable sets — the region reachable by neither arm. At
that span it failed **10 of 25 waypoints**. Its coordinates are a record of
what was tried, not a specification.

The live T3 is the **500 mm** version, in the `abc/` set under the key **`B`**
(`tasks.TASK_B`, `name="t3_coordinated_carry"`). Each grip sits at |x| = 0.25,
which is simultaneously outside the dead band and the smallest half-span
clearing the 120 mm wearer floor. It is verified N=10 over the full densified
path.

So: **T3's mechanism was reinstated; T3's archived coordinates were not.**
Restoring this directory to "get T3 back" would reinstate a known failure and
would look entirely plausible while doing it — the tilt threshold rescales
with the span (60 mm of height difference is 11.0° at 310 mm and 6.8° at
500 mm), so a trial run from here produces a believable number against the
wrong criterion.

The merged Task B that T3 replaced on 2026-08-11 — rigid **and** compliant —
is at `task_b_rigid_and_compliant/`, with what was lost recorded there. That
one is a supersession; this one is a regression. They are not the same kind of
archive and should not be treated alike.
