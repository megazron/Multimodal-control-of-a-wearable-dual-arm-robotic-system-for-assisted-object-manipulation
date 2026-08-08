#!/usr/bin/env python3
"""
run_autonomy_level.py — E2, the core comparison.

Three levels of autonomy, within-subjects, Latin-square counterbalanced:

  direct     teleoperation only. THE BASELINE. srl_autonomy is not consulted.
  shared     the Part 5 pipeline: intent inference, an IK-validated grasp
             offer, and a discrete operator-confirmed handover.
  full_auto  after the operator DESIGNATES a target, the arm completes it.

Within-subjects on purpose: between subjects, variance from operator skill on
a novel wearable device would swamp the effect.

Run:
    bash scripts/run_experiment.sh e2 --participant P01
    bash scripts/run_experiment.sh e2 --participant PILOT --scripted
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
    for b, cond in enumerate(order):
        for s in block_trials(cfg["scenarios"], cfg["repeats"], seed=b):
            sched.append((b, cond, s))
    return order, sched


def main():
    a = base_args("E2 autonomy level").parse_args()
    cfg = yaml.safe_load(open(a.config))
    order, sched = schedule(cfg, a.participant_index)
    if a.max_trials:
        sched = sched[:a.max_trials]
    bal = check_balance(cfg["conditions"], 12)
    if a.dry_run:
        print("E2 order for participant index %d: %s" % (a.participant_index, order))
        print("   %d trials; balance at n=12: %s" % (len(sched), bal))
        return 0

    rclpy.init()
    r = ExperimentRunner("e2_autonomy_level", a)
    ex = None
    if a.scripted:
        _, ex = start_scripted(r.arm)
    r.log.write_manifest(experiment="e2_autonomy_level", arm=r.arm,
                         scripted=a.scripted, mounting="bench",
                         condition_order=order, config=cfg,
                         counterbalance=bal,
                         note="core comparison; within-subjects")
    r.spin(2.0)
    for i, (b, cond, scen) in enumerate(sched):
        t = run_trial(r, cfg, scen, cond, i, block=b)
        print("  %3d  block %d  %-10s %-12s  %6.2f s  success=%s"
              % (i, b, cond, scen, t["completion_time_s"], t["grasp_success"]))
    print("E2 done: %s" % r.log.summary_path)
    if ex:
        ex.shutdown()
    r.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
