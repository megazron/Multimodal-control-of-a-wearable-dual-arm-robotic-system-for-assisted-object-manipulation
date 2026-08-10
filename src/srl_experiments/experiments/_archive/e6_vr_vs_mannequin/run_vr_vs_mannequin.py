#!/usr/bin/env python3
"""
run_vr_vs_mannequin.py — E6, the experiment that isolates the central claim.

2x2 within-subjects: INPUT DEVICE (mannequin, vr) x AUTONOMY (direct, shared).

The claim under test is that the autonomy recovers CAPABILITY the input cannot
express, not merely that autonomy helps. The mannequin cannot measure wrist
orientation at all (left j7 railed and j6 clamped; right j3/j5/j7 dead), so
its autonomy supplies a missing degree of freedom. VR already has 6-DOF, so
its autonomy can only supply PRECISION. If the interaction is real, autonomy
should buy far more on the mannequin than in VR -- and that difference is the
result, not either main effect.

DEVICE IS A BLOCK FACTOR, never alternated trial-by-trial: swapping a headset
for a mannequin takes ~30 s and would dominate every timing measure.

Run:
    bash scripts/run_experiment.sh e6 --participant P01
    bash scripts/run_experiment.sh e6 --participant PILOT --scripted
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
    a = base_args("E6 VR vs mannequin").parse_args()
    cfg = yaml.safe_load(open(a.config))
    order, sched = schedule(cfg, a.participant_index)
    if a.max_trials:
        sched = sched[:a.max_trials]
    bal = check_balance(cfg["conditions"], 12)
    if a.dry_run:
        print("E6 order for participant index %d: %s" % (a.participant_index, order))
        print("   %d trials; balance at n=12: %s" % (len(sched), bal))
        return 0

    rclpy.init()
    r = ExperimentRunner("e6_vr_vs_mannequin", a)
    ex = None
    if a.scripted:
        _, ex = start_scripted(r.arm)
    r.log.write_manifest(experiment="e6_vr_vs_mannequin", arm=r.arm,
                         scripted=a.scripted, mounting="bench",
                         condition_order=order, config=cfg,
                         counterbalance=bal,
                         note="2x2 device x autonomy; device is a BLOCK "
                              "factor because a headset swap takes ~30 s")
    r.spin(2.0)
    for i, (b, cond, scen) in enumerate(sched):
        t = run_trial(r, cfg, scen, cond, i, block=b)
        print("  %3d  block %d  %-10s %-12s  %6.2f s  success=%s"
              % (i, b, cond, scen, t["completion_time_s"], t["grasp_success"]))
    print("E6 done: %s" % r.log.summary_path)
    if ex:
        ex.shutdown()
    r.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
