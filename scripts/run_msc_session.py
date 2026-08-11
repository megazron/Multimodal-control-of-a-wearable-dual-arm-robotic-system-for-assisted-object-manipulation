#!/usr/bin/env python3
"""Run the MSc data-collection sequence END TO END in sim, and report what
was actually captured.

    python3 scripts/run_msc_session.py [--cells 4] [--participant PILOT]

WHY IT REPORTS COLUMNS AND NOT "OK". The question this answers is not "did it
run" -- a runner that logs a header and no rows also runs. It is "does the
pipeline produce a COMPLETE dataset", and the only honest form of that answer
is: for every column the logger declares, how many rows carry a value.

That distinction has bitten this project twice. The E1-E5 pilot passed with
`clutch` and `intent_top` EMPTY in every log, because nothing published
/master_status_<arm> in scripted mode and nothing published objects -- the
whole intent -> grasp -> handover path was never exercised and no check
noticed. A column that is silently blank in every row is the same failure as a
metric nobody computes.

So: every declared column is listed with its fill rate, and a column that is
0% is named as UNPOPULATED with what would have to be running to fill it.
"""
import argparse
import csv
import glob
import json
import os
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(WS, "src/srl_experiments"))
import msc_session as MS                                     # noqa: E402
from srl_experiments.trial_logger import (                   # noqa: E402
    SAMPLE_COLUMNS, TRIAL_COLUMNS)

# What each column needs in order to be non-empty. Only listed for columns
# that CAN be empty in a sim run -- the point is to name the cause, not to
# excuse it.
NEEDS = {
    "clutch": "/master_status_<arm> -- master_pose_node, absent with master:=false",
    "master_x": "a real or virtual Teensy publishing /master_arm_pose_<arm>",
    "pointing_x": "the IMU pointing vector, same source as master_*",
    "intent_top": "intent_node plus published objects to be uncertain between",
    "intent_dist": "as intent_top",
    "grasp_offered": "grasp_generator with a validated candidate",
    "payload_kg": "payload_manager, whose hardware call is a STUB",
    "channels_live": "/master_capability_<arm> from channel_manager",
}
for _k in ("master_y", "master_z", "master_qx", "master_qy", "master_qz",
           "master_qw", "pointing_y", "pointing_z"):
    NEEDS[_k] = NEEDS["master_x"]
for _k in ("intent_top_p", "intent_margin", "intent_ambiguous", "intent_ids"):
    NEEDS[_k] = NEEDS["intent_top"]


def fill_rates(paths):
    """column -> (rows with a value, total rows), across every sample CSV."""
    have = {c: 0 for c in SAMPLE_COLUMNS}
    total = 0
    for p in paths:
        with open(p) as fh:
            for row in csv.DictReader(fh):
                total += 1
                for c in SAMPLE_COLUMNS:
                    v = row.get(c, "")
                    if v not in ("", None):
                        have[c] += 1
    return have, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cells", type=int, default=4,
                    help="how many of the sequence's cells to run")
    ap.add_argument("--participant", default="PILOT")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    out = a.out or os.path.join(WS, "recordings", "sessions",
                                "msc_e2e_%s" % time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    cells = MS.cells()[:a.cells]

    print("=" * 74)
    print("MSc SEQUENCE END TO END -- %d of %d cells" % (len(cells),
                                                         len(MS.cells())))
    print("=" * 74)
    ran, failed = [], []
    for task, mode, trials, secs, worn in cells:
        key = {"T0": "m0", "T1_s1": "m1", "T1_s2": "m1s2",
               "T2": "m2", "T3": "m3"}[task]
        cmd = [sys.executable, "-u",
               os.path.join(WS, "src/srl_experiments/experiments/abc",
                            "run_abc.py"),
               "--taskset", "msc", "--task", key, "--mode", mode,
               "--hold-s", "0.05"]
        t0 = time.time()
        r = subprocess.run(cmd, capture_output=True, text=True,
                           cwd=os.path.join(WS,
                                            "src/srl_experiments/experiments/abc"))
        dt = time.time() - t0
        ok = r.returncode == 0
        travel = [ln.strip() for ln in (r.stdout or "").splitlines()
                  if "tf2 EE path" in ln]
        print("   %-8s %-20s %-5s %5.1f s  %s"
              % (task, mode, "OK" if ok else "FAIL", dt,
                 " | ".join(travel) or (r.stdout or r.stderr or "")[-80:]))
        (ran if ok else failed).append((task, mode))

    # THE HEADLINE, AND IT IS NOT A GREEN TICK.
    #
    # `run_abc.py` is the only entry point the MSc tasks have -- run_experiment.sh
    # --taskset msc dispatches straight to it -- and it contains ZERO references
    # to TrialLogger. TrialLogger is used by runner.py (the legacy E-series),
    # fault_injector.py and an ARCHIVED bimanual runner, none of which know the
    # MSc tasks exist.
    #
    # So the MSc set can be DRIVEN and RECORDED AS CLIPS, and running it
    # produces no trial CSV, no sample rows and no manifest: nothing a study
    # could analyse. That is the answer to "does the pipeline produce a
    # complete dataset before a participant sits down", and the answer is NO.
    samples = sorted(glob.glob(os.path.join(WS, "recordings", "**",
                                            "*sample*.csv"), recursive=True))
    have, total = fill_rates(samples)
    print("\nCAPTURED: %d sample rows across %d files" % (total, len(samples)))
    if not total:
        print("\n   THE DATASET IS EMPTY, AND THIS IS A REAL GAP, not a "
              "misconfiguration of this run.")
        print("   run_abc.py is the ONLY entry point the MSc tasks have and "
              "it never constructs a TrialLogger.")
        print("   TrialLogger is used by runner.py, fault_injector.py and an "
              "ARCHIVED bimanual runner -- none of which")
        print("   know the MSc tasks exist. The set has a CLIP path and no "
              "DATA path.")
        print("   To close it: wire the MSc tasks into runner.py/task_trial.py "
              "so every trial opens a")
        print("   TrialLogger, writes the %d sample columns and %d trial "
              "columns, and emits a manifest."
              % (len(SAMPLE_COLUMNS), len(TRIAL_COLUMNS)))
    if total:
        print("\n%-20s %7s  %s" % ("column", "filled", "if empty, what is missing"))
        empty = []
        for c in SAMPLE_COLUMNS:
            pct = 100.0 * have[c] / total
            mark = "" if pct > 0 else "  <- UNPOPULATED"
            if pct == 0:
                empty.append(c)
            print("%-20s %6.1f%%%s %s" % (c, pct, mark,
                                          NEEDS.get(c, "") if pct == 0 else ""))
        print("\n%d of %d sample columns are UNPOPULATED"
              % (len(empty), len(SAMPLE_COLUMNS)))
    json.dump(dict(ran=ran, failed=failed, sample_rows=total,
                   files=len(samples),
                   fill={c: have[c] for c in SAMPLE_COLUMNS}),
              open(os.path.join(out, "capture_report.json"), "w"), indent=2)
    print("-> %s" % os.path.join(out, "capture_report.json"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
