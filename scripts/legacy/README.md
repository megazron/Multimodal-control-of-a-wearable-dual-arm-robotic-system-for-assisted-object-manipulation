# Superseded scripts

Kept runnable, not deleted: they hold measurements and diagnoses that the log
in `docs/ENGINEERING_LOG.md` refers to by name, and a reader following a citation to a
deleted script cannot tell whether the tool was wrong or merely gone.

## Scenario verifiers for the ARCHIVED task sets

`verify_final5.py`, `verify_scenarios.py`, `verify_t2_scenarios.py` verify the
T1-T9 / E1-E6 generations, whose protocols and data now live under
`src/srl_experiments/experiments/_archive/`. They ran a 300/310 mm span; the
current specification is 500 mm, so a result they produce would be filed under
geometry that no longer matches.

**The live equivalents are `scripts/verify_abc_scenarios.py` (tasks A/B/C) and
`scripts/audit_scenario_reachability.py` (N=10 over the densified full path).**

## Kept in `scripts/`, deliberately

- **`verify_task_scenes.py`** exports `Solver` and `HOME_TOL_RAD`, which
  `measure_wrist_capability.py` and `verify_t8_t9_scenarios.py` import. It is
  shared infrastructure wearing a verifier's name; moving it would break two
  live tools. Extracting the solver into its own module is the right fix and
  has not been done.
- **`verify_t8_t9_scenarios.py`** verifies T8 and T9, the two-person tasks,
  which are current.
