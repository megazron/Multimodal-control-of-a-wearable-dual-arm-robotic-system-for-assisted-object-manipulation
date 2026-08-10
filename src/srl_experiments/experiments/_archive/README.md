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
