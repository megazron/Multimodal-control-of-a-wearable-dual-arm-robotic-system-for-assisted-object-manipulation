#!/usr/bin/env python3
"""
run_divided_attention.py — E3, the SRL-specific experiment.

E2 crossed with a concurrent manual primary task.

THE CONTRIBUTION LIVES HERE. A supernumerary limb is not a better
teleoperator's arm; it is an extra limb used WHILE the operator's own two are
busy. If shared autonomy helps anywhere it should help most when attention is
divided, and the INTERACTION term is the result this project is after.

The primary task is continuous manual tracking rather than an n-back, because
it loads the same manual and visual channel the SRL competes for.

Run:
    bash scripts/run_experiment.sh e3 --participant P01
    bash scripts/run_experiment.sh e3 --participant PILOT --scripted
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml

from srl_experiments.conditions import assign_order, block_trials, check_balance
from srl_experiments.runner import ExperimentRunner
from srl_experiments.task_trial import run_trial, SCENARIOS

HERE = Path(__file__).resolve().parent


def base_args(desc):
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--participant", required=True,
                    help="ANONYMISED code only. Never a name.")
    ap.add_argument("--participant-index", type=int, default=0)
    ap.add_argument("--config", default=str(HERE / "config.yaml"))
    ap.add_argument("--results", default="")
    ap.add_argument("--scripted", action="store_true",
                    help="drive the master from ScriptedOperator: no human, "
                         "no hardware. This is how the pipeline is tested.")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--max-trials", type=int, default=0,
                    help="cap the schedule; used by the pilot")
    return ap


def start_scripted(arm):
    """A ScriptedOperator on its own executor inside this process."""
    import threading
    from rclpy.executors import SingleThreadedExecutor
    from srl_experiments.scripted_operator import ScriptedOperator
    op = ScriptedOperator()
    op.set_parameters([rclpy.parameter.Parameter(
        "arm", rclpy.Parameter.Type.STRING, arm)])
    ex = SingleThreadedExecutor()
    ex.add_node(op)
    threading.Thread(target=ex.spin, daemon=True).start()
    return op, ex


def schedule(cfg, pidx):
    order = assign_order(cfg["conditions"], pidx)
    sched = []
    b = 0
    for att in cfg["attention"]:
        for cond in order:
            for s in block_trials(cfg["scenarios"], cfg["repeats"], seed=b):
                sched.append((b, cond, s, att))
            b += 1
    return order, sched


def main():
    a = base_args("E3 divided attention").parse_args()
    cfg = yaml.safe_load(open(a.config))
    order, sched = schedule(cfg, a.participant_index)
    if a.max_trials:
        sched = sched[:a.max_trials]
    if a.dry_run:
        print("E3 order %s, %d trials, %d blocks"
              % (order, len(sched), 1 + max(s[0] for s in sched)))
        return 0

    rclpy.init()
    r = ExperimentRunner("e3_divided_attention", a)
    ex = None
    if a.scripted:
        _, ex = start_scripted(r.arm)
    r.log.write_manifest(experiment="e3_divided_attention", arm=r.arm,
                         scripted=a.scripted, mounting="bench",
                         condition_order=order, config=cfg,
                         primary_task=cfg["primary_task"],
                         note="autonomy level CROSSED with divided attention; "
                              "the interaction term is the result of interest")
    r.spin(2.0)
    for i, (b, cond, scen, att) in enumerate(sched):
        # The dual-task condition is recorded in `condition` so the crossed
        # design survives into the summary file as a single factor label.
        t = run_trial(r, cfg, scen, "%s|%s" % (cond, att), i, block=b,
                      attention=att)
        print("  %3d  block %d  %-10s %-12s %-12s  %6.2f s  success=%s"
              % (i, b, cond, att, scen, t["completion_time_s"], t["grasp_success"]))
    print("E3 done: %s" % r.log.summary_path)
    if ex:
        ex.shutdown()
    r.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
