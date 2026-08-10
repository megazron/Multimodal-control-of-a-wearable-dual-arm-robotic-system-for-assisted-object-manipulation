#!/usr/bin/env python3
"""
run_fitts.py — E1, Fitts characterisation of the degraded master.

Targets varying in distance and width; movement time vs index of difficulty;
throughput in bits/s.

WHY THIS EXPERIMENT EXISTS. Every other result in this project is internal to
our own hardware. Fitts throughput in bits/s is the one figure another group
can compare against without owning this rig — it is the bridge to the
literature, and the honest way to state the size of the problem the autonomy
is meant to address.

NO GRASPING, deliberately: it isolates the cost of POINTING from the cost of
GRASPING, so E2-E4's differences cannot be attributed to reaching.

Run:
    bash scripts/run_experiment.sh e1 --participant P01
    bash scripts/run_experiment.sh e1 --participant PILOT --scripted
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml

from srl_experiments.conditions import block_trials
from srl_experiments.runner import ExperimentRunner

HERE = Path(__file__).resolve().parent


def base_args(desc):
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--participant", required=True,
                    help="ANONYMISED code only. Never a name.")
    ap.add_argument("--participant-index", type=int, default=0)
    ap.add_argument("--config", default=str(HERE / "config.yaml"))
    ap.add_argument("--results", default="")
    ap.add_argument("--scripted", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--max-trials", type=int, default=0)
    return ap


def start_scripted(arm):
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
    pairs = [(d, w) for d in cfg["distances_m"] for w in cfg["widths_m"]]
    return block_trials(pairs, cfg.get("repeats", 3), seed=1)


def target_for(home, d, k):
    """Targets on a circle in the frontal plane, at distance d from home."""
    ang = 2 * math.pi * (k % 8) / 8.0
    u = np.array([math.cos(ang), 0.25, math.sin(ang)])
    return np.asarray(home, float) + d * u / np.linalg.norm(u)


def main():
    a = base_args("E1 Fitts characterisation").parse_args()
    cfg = yaml.safe_load(open(a.config))
    sched = schedule(cfg)
    if a.max_trials:
        sched = sched[:a.max_trials]
    if a.dry_run:
        print("E1 schedule: %d trials" % len(sched))
        for i, (d, w) in enumerate(sched[:6]):
            print("   %2d  D=%.3f W=%.3f  ID=%.2f bits" % (i, d, w, math.log2(d / w + 1)))
        return 0

    rclpy.init()
    r = ExperimentRunner("e1_fitts_characterisation", a)
    ex = None
    if a.scripted:
        _, ex = start_scripted(r.arm)
    home = np.array(cfg["home_xyz"], float)
    r.log.write_manifest(experiment="e1_fitts_characterisation", arm=r.arm,
                         scripted=a.scripted, mounting="bench", config=cfg,
                         n_trials=len(sched),
                         note="system evaluation; no participants required")
    r.spin(2.0)
    for i, (d, w) in enumerate(sched):
        tgt = target_for(home, d, i)
        r.log.start(i, condition="D%.3f_W%.3f" % (d, w), scenario="fitts",
                    target_id="", arm=r.arm)
        r.estop_events = 0
        r.aborts = []
        faults_at_start = set(getattr(r, "recovery_faults", {}) or {})
        r.recovery_seen = {}
        r.send_operator(home)
        r.spin(0.5)
        r.send_operator(tgt, point_at=tgt)
        t0 = time.monotonic()
        dwell = None
        pts = []
        hit = 0
        while time.monotonic() - t0 < cfg["timeout_s"] and rclpy.ok():
            rclpy.spin_once(r, timeout_sec=0.005)
            r.sample_once("reach")
            p = r.master.pose.position if r.master else None
            if p is None:
                continue
            cur = np.array([p.x, p.y, p.z])
            pts.append(cur)
            if np.linalg.norm(cur - tgt) < w / 2.0:
                dwell = dwell or time.monotonic()
                if time.monotonic() - dwell > cfg["dwell_s"]:
                    hit = 1
                    break
            else:
                dwell = None
        mt = time.monotonic() - t0
        good, bad = r.channels_ok()
        if not good:
            r.log.invalidate("channel dropout mid-trial: %s" % ",".join(bad))
        if r.estop_events:
            r.log.invalidate("e-stop during trial")
        # E1 has its own trial loop rather than using task_trial, so the
        # safety-layer invalidation has to be repeated here. It was missing,
        # and the consequence was silent: a fault mid-trial produced a trial
        # marked only "e-stop during trial", losing which rig fault caused it.
        new_faults = set(getattr(r, "recovery_seen", {}) or {}) - faults_at_start
        if r.aborts or new_faults:
            causes = sorted({str(x.get("cause", "?")) for x in r.aborts})
            if not causes:
                seen = getattr(r, "recovery_seen", {})
                causes = [str(seen.get(f, f)) for f in sorted(new_faults)]
            r.log.invalidate("aborted mid-trial: %s" % "; ".join(causes))
        final = pts[-1] if pts else home
        sl = float(np.linalg.norm(tgt - home))
        r.log.finish(completion_time_s=round(mt, 4),
                     positioning_error_m=round(float(np.linalg.norm(final - tgt)), 5),
                     path_length_m=round(r.path_length(pts), 5),
                     straight_line_m=round(sl, 5),
                     path_ratio=round(r.path_length(pts) / max(1e-6, sl), 4),
                     fitts_distance_m=d, fitts_width_m=w,
                     fitts_id_bits=round(r.fitts_id(d, w), 4),
                     fitts_movement_time_s=round(mt, 4),
                     failed_attempts=0 if hit else 1,
                     estop_events=r.estop_events)
        print("  %3d  D=%.3f W=%.3f ID=%.2f  MT=%6.2f s  %s"
              % (i, d, w, r.fitts_id(d, w), mt, "hit" if hit else "TIMEOUT"))
    print("E1 done: %s" % r.log.summary_path)
    if ex:
        ex.shutdown()
    r.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
