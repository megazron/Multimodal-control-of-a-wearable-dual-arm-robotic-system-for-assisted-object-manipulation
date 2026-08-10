#!/usr/bin/env python3
"""T3 analysis — inter-arm coordination error, the headline metric.

    python3 analyse_t3.py --samples <trial_*.csv> --summary <summary.csv>

Tilt is read from the CONTINUOUS per-sample logs, not from a per-trial
summary field, because a coordination failure has a shape: a step, a drift
and a wobble all reduce to the same maximum.
"""
import argparse, csv, glob, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from srl_experiments.validity import partition, assert_analysable  # noqa: E402
from bimanual_metrics import tilt_deg, summarise_tilt              # noqa: E402

EXPERIMENT = "T3 coordinated carry"
CONDITIONS = ("direct", "assisted", "shared")


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")


def tilt_series(path):
    """Prefer a logged tilt column; else recompute from the two EE heights."""
    out, phase = [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            t = num(r, "tilt_deg")
            if t != t:
                t = tilt_deg(num(r, "left_ee_z"), num(r, "right_ee_z"))
            out.append(t)
            phase.append((r.get("phase") or "").strip())
    return np.array(out, float), phase


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", nargs="+", required=True)
    ap.add_argument("--summary", nargs="*", default=[])
    ap.add_argument("--outdir", default="")
    a = ap.parse_args()

    files = []
    for pat in a.samples:
        files += sorted(glob.glob(pat))
    if not files:
        print("no sample files matched")
        return 2

    by = defaultdict(list)
    for f in files:
        cond = "?"
        for c in CONDITIONS:
            if c in Path(f).name.lower():
                cond = c
        ts, phase = tilt_series(f)
        s = summarise_tilt(ts)
        s["file"] = f
        s["transport"] = summarise_tilt(
            [t for t, p in zip(ts, phase) if p == "transport"])
        by[cond].append(s)

    print("INTER-ARM COORDINATION ERROR — tray tilt")
    print("  20 mm height difference over 300 mm = 3.8 deg (visible wobble)")
    print("  60 mm = 11.3 deg (the ball leaves)\n")
    print("  %-10s %5s %10s %10s %12s %12s"
          % ("cond", "n", "RMS deg", "max deg", ">3.8 deg s", ">11.3 deg s"))
    for c in CONDITIONS:
        rs = by.get(c, [])
        if not rs:
            continue
        rms = np.array([r["tilt_rms_deg"] for r in rs])
        mx = np.array([r["tilt_max_deg"] for r in rs])
        a1 = np.array([r["time_above_3_8_s"] for r in rs])
        a2 = np.array([r["time_above_11_3_s"] for r in rs])
        print("  %-10s %5d %10.2f %10.2f %12.2f %12.2f"
              % (c, len(rs), np.nanmedian(rms), np.nanmedian(mx),
                 np.nanmedian(a1), np.nanmedian(a2)))

    print("\n  TRANSPORT PHASE ONLY (lift and place load coordination "
          "differently)")
    for c in CONDITIONS:
        rs = [r for r in by.get(c, []) if r["transport"]["n"]]
        if rs:
            print("  %-10s RMS %.2f deg   max %.2f deg"
                  % (c, np.nanmedian([r["transport"]["tilt_rms_deg"] for r in rs]),
                     np.nanmedian([r["transport"]["tilt_max_deg"] for r in rs])))

    d = [r["tilt_rms_deg"] for r in by.get("direct", [])]
    s = [r["tilt_rms_deg"] for r in by.get("assisted", [])]
    if d and s:
        print("\n  DIRECT -> ASSISTED  tilt RMS %.2f -> %.2f deg (%.0f%% "
              "reduction)" % (np.nanmedian(d), np.nanmedian(s),
                              100 * (1 - np.nanmedian(s) / max(np.nanmedian(d), 1e-9))))

    outdir = Path(a.outdir or (Path(files[0]).parent / "analysis"))
    outdir.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(10, 4))
    for c in CONDITIONS:
        for r in by.get(c, [])[:4]:
            ts, _ = tilt_series(r["file"])
            ax.plot(np.arange(ts.size) * 0.02, ts, lw=0.8,
                    label="%s" % c if r is by[c][0] else None)
    ax.axhline(3.8, ls="--", lw=0.7, color="orange")
    ax.axhline(11.3, ls="--", lw=0.7, color="red")
    ax.set_xlabel("s")
    ax.set_ylabel("tray tilt (deg)")
    ax.set_title("Inter-arm coordination error over time")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(outdir / "tilt_timeseries.png", dpi=110)
    print("\n  plot -> %s" % (outdir / "tilt_timeseries.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
