# Process gallery — how the project was done, from its own records

Regenerate: `python3 extras/thesis/thesis_v3/figures/gallery/process/make_process_gallery.py`.
Each figure is `.pdf` and `.png` (150 dpi). No participant name appears in any figure
(the yield chart uses dates only).

| file | plain title | sources | takeaway | key numbers | verdict |
| --- | --- | --- | --- | --- | --- |
| `instrument_defects` | The 26 times a measurement was wrong and the system was not: what went wrong, and how it was caught | counts as stated in `extras/thesis/thesis_v3/appendix/g_instrument.tex` (mechanism groups 7/6/5/5/3; catch routes 15/5/6) | Care did not catch them; a second, independent number did — 15 of 26 were caught by a known quantity disagreeing with the reading. | 26 cases; 15 caught by an independent quantity, 5 by reading code, 6 by looking at the raw artefact | **MAIN** candidate for Discussion §Validity (the two numbers that section already carries in words, as a picture; costs one figure slot and ~0 words) |
| `commits_and_tests` | Commits per day, and the growing test suite | `git log` on the repository (449 commits, 27 active days, 2026-07-14 → 2026-09-05); first-add dates of `src/*/test/test_*.py`; test functions counted today | The test suite grew with the code rather than after it: 140 test files, 1 376 individual tests. | 449 commits; peak 60/day (9 and 11 Aug); 140 test files; 1 376 tests | APPENDIX B (development record) — evidence of method, but not a result |
| `recording_yield` | Recording sessions started per day, and how many actually hold data | `recordings/sessions/*/` (bag `metadata.yaml` message counts, `trail*.csv` row counts; "holds data" = more than 100 messages or 50 rows) | Recording was unreliable until 2 Sep and near-perfect after: 0/6, 1/10, 4/17, 0/2, 0/9 on the early days, then 17/26, 3/3, 14/15. | 88 session folders; 39 hold data; 2–4 Sep: 34 of 44 | APPENDIX L (the pilot chapter already states 7 of 16 master attempts were empty; this is the whole-project version) |

## Not made

- **Latency distribution** (wearer-tracking / reaction budget): no file with per-sample
  latencies exists in `recordings/`; only the summary numbers in `docs/`. Not drawn.
