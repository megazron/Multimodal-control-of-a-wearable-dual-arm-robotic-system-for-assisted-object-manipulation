# T1S2 clips under four modes the task does not run in

Recorded 2026-08-17. `msc_clip_tasks` declares that **T1 and T1S2 run under
`06_full_autonomy` only** — T1 carries its own approach and is driven from a
typed sentence — and `record_abc_sweep` honours that, printing
`SKIP (t1s2 runs under 06_full_autonomy only)`.

These four directories predate that declaration. Left in the live tree they
made `scripts/status_table.py` report `CD` — a complete, verified cell — for a
combination the recorder will never produce again, so the set looked more
complete than it is and a stale clip stood in for a live one.

Not deleted: ARCHIVE, NEVER DELETE. The whole tree as it stood before the
2026-08-23 accuracy pass is also in
`extras/archive/recordings/verification_20260823_pre_accuracy_pass`.
