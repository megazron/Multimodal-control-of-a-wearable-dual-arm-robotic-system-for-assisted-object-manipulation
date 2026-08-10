# Task B, merged (rigid AND compliant) — SUPERSEDED 2026-08-11

Replaced in the live `abc/` set by **T3 coordinated carry, rigid only**. The
slot keeps the key `B`; only its content changed. `task_b_merged.py` is the
superseded spec, verbatim as it stood at commit `e642506`.

## What it was

One task, two objects, run against the same three vertical paths at the same
500 mm grip separation:

| object | failure mode | threshold |
| --- | --- | --- |
| rigid tray | **TILT** — the ball rolls off the lip | 6.8° (60 mm over 500 mm) |
| 540 mm sling | **SEPARATION** — the sag closes and the ball escapes | 0.5340 m (`sag = 2r`) |

The contrast was the point: rigid coupling transmits coordination error
instantly and proportionally, compliant coupling absorbs it and then fails
abruptly, so the two change the *form* of the failure rather than its size.

## Why it was replaced, and what that costs

**Not because the contrast was wrong.** It was 18 trial slots where T3 alone
is 9, and the session had no room. Task 0 was added to the programme on
2026-08-10 and the timeline was already at 116 min against a 120 min cap, with
the binding constraint being the wearer (>17 kg, 12 min continuous / 48 min
total pack-on), not the clock. Something had to give and this was the half
that duplicated a path rather than added one.

**What is genuinely lost:** the only within-task manipulation of coupling
*compliance* in the programme. Any claim about how coordination error
propagates through a compliant versus a rigid coupling is no longer supported
by data from this study, and must not be made. What survives is the rigid
case, which is the one every other measure is read against.

**What is not lost:** the sling arithmetic. `tasks.ARCHIVED_COMPLIANT` keeps
`L = 0.540`, `r = 0.020`, `s_max = 0.5340 m` in the live module, because the
rigid tilt threshold is stated *against* it and stranding the comparison would
make 6.8° look arbitrary.

## If you bring it back

Restore the sling half into `TASK_B["objects"]`, restore `object_factor`,
`fail_sep_m` and the `sep_err_*` / `min_sag_mm` metrics, and re-run
`scripts/verify_abc_scenarios.py --repeats 10` — it checks the sling
arithmetic against the declared threshold to 0.1 mm and will fail loudly if
the two drift. Then re-run `session_timeline.check()`, which is what refused
the 3-repeat version of Task A and is the thing that will refuse this too if
the session no longer fits.

## Note on the OTHER T3

There are two T3s and only one is usable. `_archive/bimanual/t3_coordinated_carry/`
declares its grips at |x| = 0.155 on a **310 mm** span — inside the 0.25 m dead
band between the two disjoint reachable sets, where it failed 10 of 25
waypoints. Those coordinates are a record, not a spec. What the live task
reinstates is T3's **mechanism** at the current verified **500 mm** geometry.
