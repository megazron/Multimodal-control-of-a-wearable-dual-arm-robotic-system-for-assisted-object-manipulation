#!/usr/bin/env python3
"""
analyse_fitts.py — E1. Fits MT = a + b·ID and reports throughput in bits/s.

Throughput uses the ISO 9241-9 / MacKenzie EFFECTIVE-WIDTH form where the data
allow it. The plain form overstates throughput whenever participants trade
accuracy for speed, and a wearable master invites exactly that trade; the
effective-width correction folds the observed endpoint spread back in.

The statistical test is the one PRE-REGISTERED in
docs/research/02_baseline_and_hypotheses.md.
"""
import argparse
import csv
import math
import sys

import numpy as np

from srl_experiments.validity import partition, assert_analysable

EXPERIMENT = "E1 Fitts characterisation"


def load(paths):
    rows = []
    for p in paths:
        with open(p) as f:
            rows += list(csv.DictReader(f))
    return rows


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")


def fit(rows):
    keep = []
    for r in rows:
        if r.get("valid", "1") in ("0", ""):
            continue
        mt, d, w, e = (num(r, "fitts_movement_time_s"), num(r, "fitts_distance_m"),
                       num(r, "fitts_width_m"), num(r, "positioning_error_m"))
        if any(x != x for x in (mt, d, w)) or mt <= 0:
            continue
        keep.append((d, w, mt, e))
    if len(keep) < 3:
        return None, len(keep)

    d = np.array([k[0] for k in keep])
    w = np.array([k[1] for k in keep])
    mt = np.array([k[2] for k in keep])
    err = np.array([k[3] for k in keep])

    ids = np.log2(d / w + 1.0)
    # Effective width: We = 4.133 * SD(endpoint error). Applied per (D,W) cell
    # so a cell with tight endpoints is not penalised by another cell's spread.
    ide = ids.copy()
    used_effective = False
    if np.isfinite(err).all() and np.nanstd(err) > 1e-9:
        used_effective = True
        for dd in np.unique(d):
            for ww in np.unique(w):
                m = (d == dd) & (w == ww)
                if m.sum() >= 3:
                    sd = float(np.std(err[m], ddof=1))
                    if sd > 1e-9:
                        ide[m] = np.log2(dd / (4.133 * sd) + 1.0)

    b, a = np.polyfit(ide, mt, 1)
    pred = a + b * ide
    ss_res = float(((mt - pred) ** 2).sum())
    ss_tot = float(((mt - mt.mean()) ** 2).sum())
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    # Mean-of-means throughput, the ISO-recommended form.
    tp = float(np.mean(ide / mt))
    return dict(n=len(keep), a=float(a), b=float(b), r2=float(r2),
                throughput=tp, id_min=float(ide.min()), id_max=float(ide.max()),
                effective_width=used_effective,
                ids=ide, mts=mt), len(keep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("summary", nargs="+")
    ap.add_argument("--plot", default="")
    a = ap.parse_args()
    _all = load(a.summary)
    _good, _bad = partition(_all)
    # Excludes as pre-registered, and REFUSES to analyse a session that is
    # entirely or mostly invalid -- an empty fit reads exactly like a null
    # result, and only one of those is about the hypothesis.
    _good = assert_analysable(EXPERIMENT, _good, _bad)
    rows = _all
    res, n = fit(rows)
    n_invalid = len(_bad)
    if res is None:
        print("E1: only %d usable trials, cannot fit" % n)
        return 1
    print("E1 Fitts characterisation")
    print("  valid trials        %d   (%d excluded as invalid)" % (res["n"], n_invalid))
    print("  effective width     %s" % ("applied" if res["effective_width"]
                                        else "NOT applied (no endpoint spread)"))
    print("  ID range            %.2f .. %.2f bits" % (res["id_min"], res["id_max"]))
    print("  MT = %.4f + %.4f * ID     R^2 = %.4f" % (res["a"], res["b"], res["r2"]))
    print("  THROUGHPUT          %.3f bits/s" % res["throughput"])
    print()
    print("  H1.1 (pre-registered): MT is linear in ID, R^2 > 0.8  ->  %s"
          % ("SUPPORTED" if res["r2"] > 0.8 else "NOT SUPPORTED"))
    if res["r2"] <= 0.8:
        print("       With R^2 <= 0.8 the master is not behaving as a pointing")
        print("       device and the throughput figure should NOT be reported.")
    print("  H1.2 (pre-registered): throughput below the 3.7-4.9 bits/s reported")
    print("       for mouse pointing  ->  %s (%.2f bits/s)"
          % ("SUPPORTED" if res["throughput"] < 3.7 else "NOT SUPPORTED",
             res["throughput"]))
    print()
    print("  Cross-study comparison is weak: interface, task and population all")
    print("  differ. This number is a BRIDGE to the literature, not a verdict.")
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.scatter(res["ids"], res["mts"], s=18, alpha=0.7)
        xs = np.linspace(res["id_min"], res["id_max"], 50)
        ax.plot(xs, res["a"] + res["b"] * xs, "r-",
                label="MT = %.3f + %.3f·ID\nR²=%.3f, TP=%.2f bit/s"
                      % (res["a"], res["b"], res["r2"], res["throughput"]))
        ax.set_xlabel("Index of difficulty (bits)")
        ax.set_ylabel("Movement time (s)")
        ax.set_title("E1 — Fitts fit for the wearable master")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(a.plot, dpi=130)
        print("  plot -> %s" % a.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
