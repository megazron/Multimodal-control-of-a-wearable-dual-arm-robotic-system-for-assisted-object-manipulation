#!/usr/bin/env python3
"""WHAT VOLUME CAN EACH ARM ACTUALLY OBSERVE, HOLDING ITS WRIST STILL?

    ./.venv_vision/bin/python scripts/measure_reachable_volume.py

WHY THIS EXISTS
---------------
`calibrate_environment` sweeps a volume whose bounds were WRITTEN DOWN --
x 0.30 to 0.58, y 0.30 to 0.52, two layers -- and the first full two-arm run
found **74 of its 144 cells outside the arms' envelope**. The scan reported
that honestly, but reporting it is not the same as knowing it: the volume was
a guess, and a guess that is half unreachable wastes half the sweep and makes
"coverage" a number about my arithmetic rather than about the robot.

This measures it instead. For the shipped attitude -- one fixed wrist
direction per pass, `elev_deg` below horizontal, yawed by each facing -- it
solves IK at every candidate camera pose over a deliberately over-wide grid,
and reports the box each arm can actually observe.

WHAT IT REPORTS AND WHY IT IS TWO THINGS
----------------------------------------
A cell reachable at ONE facing is observable, but only from that one angle, so
its sides are seen from one direction. A cell reachable at EVERY facing is
seen from all of them and is where the map is strongest. Both are printed,
because a sweep bound to the all-facings box would be small and safe, and one
bound to the any-facing box covers more ground with thinner evidence at the
edges. Choosing between them is the caller's business; measuring them is this
script's.

NOTHING IS COMMANDED. This solves IK and moves no joint.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (HERE, os.path.join(ROOT, "src/srl_perception"),
           os.path.join(ROOT, "src/srl_experiments"),
           os.path.join(ROOT, "src/srl_teleop")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from srl_perception import calibration_sweep as CSW          # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/reachable_volume.json")


def measure(node, arm, xs, ys, layers_m, facings_deg, elev_deg, surface_z,
            tries=2):
    """{(facing, layer): set of reachable (x, y)} plus a per-cell tally."""
    hits = {}
    tally = {}
    total = len(xs) * len(ys) * len(layers_m) * len(facings_deg)
    done = 0
    t0 = time.time()
    for f in facings_deg:
        axis = node.fixed_axis(arm, f, elev_deg)
        quat = node.fixed_quat(arm, f, elev_deg)
        for h in layers_m:
            d = h / max(1e-6, -float(axis[2]))
            ok = set()
            for x in xs:
                for y in ys:
                    cam = [round(x - float(axis[0]) * d, 5),
                           round(y - float(axis[1]) * d, 5),
                           round(surface_z + h, 5)]
                    done += 1
                    if node.solve(arm, cam, quat, tries=tries) is not None:
                        ok.add((round(x, 4), round(y, 4)))
                        tally[(round(x, 4), round(y, 4))] = \
                            tally.get((round(x, 4), round(y, 4)), 0) + 1
                    if done % 60 == 0:
                        print("      %s %d/%d  (%.0f s)"
                              % (arm, done, total, time.time() - t0),
                              flush=True)
            hits[(f, h)] = ok
    return hits, tally, total


def box(cells):
    if not cells:
        return None
    a = np.array(sorted(cells), float)
    return dict(x=[float(a[:, 0].min()), float(a[:, 0].max())],
                y=[float(a[:, 1].min()), float(a[:, 1].max())],
                n=len(cells))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", nargs="+", default=["left", "right"])
    ap.add_argument("--surface-z", type=float, default=1.25)
    ap.add_argument("--elev-deg", type=float, default=38.9)
    ap.add_argument("--layers-m", type=float, nargs="+",
                    default=list(CSW.DEFAULT_LAYERS_M))
    ap.add_argument("--facings-deg", type=float, nargs="+",
                    default=list(CSW.DEFAULT_FACINGS_DEG))
    ap.add_argument("--step-m", type=float, default=0.05)
    ap.add_argument("--x-range", type=float, nargs=2, default=[0.10, 0.76],
                    metavar=("MIN", "MAX"),
                    help="OUTBOARD distance to probe, magnitude. Mirrored for "
                         "the right arm.")
    ap.add_argument("--y-range", type=float, nargs=2, default=[0.15, 0.66],
                    metavar=("MIN", "MAX"),
                    help="FORWARD distance to probe. Negative is behind the "
                         "wearer. The shipped default only ever asked about "
                         "0.15 to 0.65, so the envelope it reported was "
                         "bounded by the question, not by the arm.")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    from env_probe import ProbeNode
    import rclpy
    rclpy.init()
    node = ProbeNode()
    doc = {}
    try:
        for arm in a.arms:
            if not node.wait_ready(arm, 120.0, need_camera=False):
                print("REFUSING: %s not ready: %s"
                      % (arm, ", ".join(node.missing(arm))))
                return 2
        # DELIBERATELY OVER-WIDE, so the answer is bounded by the ARM and not
        # by the range I chose to ask about.
        span = np.arange(a.x_range[0], a.x_range[1] + 1e-9, a.step_m)
        ys = np.arange(a.y_range[0], a.y_range[1] + 1e-9, a.step_m)
        for arm in a.arms:
            xs = span if arm == "left" else -span
            print("=== %s arm: %d x %d cells x %d layer(s) x %d facing(s)"
                  % (arm, len(xs), len(ys), len(a.layers_m),
                     len(a.facings_deg)), flush=True)
            hits, tally, total = measure(node, arm, xs, ys, a.layers_m,
                                         a.facings_deg, a.elev_deg,
                                         a.surface_z)
            n_conf = len(a.layers_m) * len(a.facings_deg)
            any_f = {c for c in tally}
            all_f = {c for c, n in tally.items() if n == n_conf}
            doc[arm] = dict(
                probed=total, configurations=n_conf,
                any_facing=box(any_f), all_facings=box(all_f),
                per_config={"%+.0f/%.2f" % k: len(v) for k, v in hits.items()})
            print("   reachable at ANY facing+layer : %s" % box(any_f))
            print("   reachable at EVERY one        : %s" % box(all_f))
        doc["settings"] = dict(elev_deg=a.elev_deg, layers_m=a.layers_m,
                               facings_deg=a.facings_deg, step_m=a.step_m,
                               surface_z=a.surface_z)
        doc["note"] = ("solved IK only; nothing was commanded. A cell "
                       "reachable at one facing is seen from one direction "
                       "only -- its sides are not.")
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        with open(a.out, "w") as f:
            json.dump(doc, f, indent=2, sort_keys=True)
        print("-> %s" % a.out)
        return 0
    finally:
        try:
            node.destroy_node(); rclpy.shutdown()
        except Exception:                                      # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
