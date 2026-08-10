#!/usr/bin/env python3
"""
run_dof_recovery.py — E4, DOF recovery.

Objects whose required approach orientation makes them ungraspable by
direct teleoperation and graspable with assist.

THE RESULT IS CATEGORICAL. "37% faster" invites a reader to imagine a
continuum; "impossible becomes possible" is the actual claim.

THE CONTROL THAT ANSWERS THE REVIEWER. The objection is "you added autonomy to
compensate for a broken input device". The deficit under test must therefore
be the STRUCTURAL one - a wearable master cannot carry a grounded 6-DOF
measurement chain, and gravity gives roll and pitch but NEVER yaw - not this
rig's incidental dead pots. So orientation is masked IN SOFTWARE on healthy
channels, and every trial records which channels were live. Trials exceeding
2% dropout on a required channel are excluded, a threshold set from the
measured separation between healthy (0%) and faulty (12.9%) channels.

Run:
    bash scripts/run_experiment.sh e4 --participant P01
    bash scripts/run_experiment.sh e4 --participant PILOT --scripted
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
    a = base_args("E4 DOF recovery").parse_args()
    cfg = yaml.safe_load(open(a.config))
    order, sched = schedule(cfg, a.participant_index)
    if a.max_trials:
        sched = sched[:a.max_trials]
    if a.dry_run:
        print("E4 order %s, %d trials" % (order, len(sched)))
        print("   scenarios and their required approach tilt:")
        for s in cfg["scenarios"]:
            print("     %-14s %3d deg   %s" % (s,
                  SCENARIOS[s].get("approach_tilt_deg", 0),
                  SCENARIOS[s]["difficulty"]))
        return 0

    rclpy.init()
    r = ExperimentRunner("e4_dof_recovery", a)
    ex = None
    if a.scripted:
        _, ex = start_scripted(r.arm)
    r.log.write_manifest(
        experiment="e4_dof_recovery", arm=r.arm, scripted=a.scripted,
        mounting="bench", condition_order=order, config=cfg,
        mask_orientation_in_software=cfg["mask_orientation_in_software"],
        required_channels=cfg["require_healthy_channels"],
        max_dropout_fraction=cfg["max_dropout_fraction"],
        note="CATEGORICAL result. Orientation deficit imposed in SOFTWARE on "
             "healthy channels so the deficit under test is the STRUCTURAL "
             "one, not this rig's incidental broken pots.")
    r.spin(2.0)
    for i, (b, cond, scen) in enumerate(sched):
        t = run_trial(r, cfg, scen, cond, i, block=b)
        tilt = SCENARIOS[scen].get("approach_tilt_deg", 0)
        print("  %3d  block %d  %-8s %-14s tilt %3d  %6.2f s  success=%s  valid=%s"
              % (i, b, cond, scen, tilt, t["completion_time_s"],
                 t["grasp_success"], t["valid"]))
    print("E4 done: %s" % r.log.summary_path)
    if ex:
        ex.shutdown()
    r.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
