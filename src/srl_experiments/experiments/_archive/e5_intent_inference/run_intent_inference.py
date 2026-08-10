#!/usr/bin/env python3
"""
run_intent_inference.py — E5, intent inference characterisation.

Vary object spacing and distractor count; report accuracy against
end-effector-to-target distance, and the cost of a wrong inference.

THIS EXPERIMENT SETS THE PART 5 THRESHOLDS. `p_threshold` and
`distance_threshold_m` in handover_arbiter are currently defaults chosen by
argument; after E5 they are chosen by measurement.

The cost of a wrong inference is measured, not assumed: a wrong ASSIST costs a
cancel and a re-approach, and that time is what makes a low threshold
expensive.

Run:
    bash scripts/run_experiment.sh e5 --participant P01
    bash scripts/run_experiment.sh e5 --participant PILOT --scripted
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


def schedule(cfg):
    cells = [(s, d) for s in cfg["object_spacings_m"]
             for d in cfg["distractor_counts"]]
    return block_trials(cells, cfg["repeats"], seed=5)


def main():
    a = base_args("E5 intent inference characterisation").parse_args()
    cfg = yaml.safe_load(open(a.config))
    sched = schedule(cfg)
    if a.max_trials:
        sched = sched[:a.max_trials]
    if a.dry_run:
        print("E5 %d trials over spacing x distractors" % len(sched))
        print("   thresholds to sweep: p=%s  d=%s"
              % (cfg["p_thresholds"], cfg["distance_thresholds_m"]))
        return 0

    rclpy.init()
    r = ExperimentRunner("e5_intent_inference", a)
    ex = None
    if a.scripted:
        _, ex = start_scripted(r.arm)
    r.log.write_manifest(experiment="e5_intent_inference", arm=r.arm,
                         scripted=a.scripted, mounting="bench", config=cfg,
                         note="system evaluation; SETS the Part 5 thresholds")
    r.spin(2.0)
    for i, (spacing, ndist) in enumerate(sched):
        # Spacing and distractor count are realised as a scenario choice: the
        # near_pair layout with objects pushed to the requested separation.
        scen = "near_pair" if ndist >= 1 else "mid_single"
        if ndist >= 3:
            scen = "cluttered"
        t = run_trial(r, cfg, scen, "spacing%.2f_dist%d" % (spacing, ndist), i)
        print("  %3d  spacing %.2f m  distractors %d  %-12s  %6.2f s  "
              "intent_correct=%s  switches=%s"
              % (i, spacing, ndist, scen, t["completion_time_s"],
                 t["intent_correct_at_handover"], t["intent_switches"]))
    print("E5 done: %s" % r.log.summary_path)
    if ex:
        ex.shutdown()
    r.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
