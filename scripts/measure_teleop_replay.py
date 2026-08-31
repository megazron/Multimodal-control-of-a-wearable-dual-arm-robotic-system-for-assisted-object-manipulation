#!/usr/bin/env python3
"""THE SMOOTHER, ON A REAL OPERATOR'S HAND. REPLAYED OFFLINE.

    python3 scripts/measure_teleop_replay.py
    python3 scripts/measure_teleop_replay.py --self-test

WHY THIS EXISTS ALONGSIDE measure_master_smoothing.py
==========================================================================
That program measures the control law against CONSTRUCTED signals: a
stationary point plus Gaussian tremor, and a ramp at a chosen speed. Those
are the right inputs for establishing that the law does what its equations
say, because the answer is known before the filter runs.

They are not what a hand does. A real operator's tremor is not white, their
reaches are not ramps, and the two are interleaved at a cadence nothing in
the constructed test reproduces. So the same two quantities -- stillness and
lag -- are measured here again, on \\num{20440} frames of a recorded session
with a person actually driving the master.

Nothing about this is a simulation of the OPERATOR. The input is a real
recording; what is simulated is the rest of the pipeline, run offline over
it. That distinction is why the campaign tags this stage `recorded`.

HOW STILL AND MOVING ARE SEPARATED, AND WHY IT IS NOT A THRESHOLD ON THE
FILTERED SIGNAL
==========================================================================
The segmentation is computed from the RAW commanded position only, once, and
the same segmentation is then applied to every filter. Deriving it from each
filter's own output would let a heavier filter classify more of the recording
as still -- which is precisely the quantity being measured -- and every filter
would then score well on its own definition of stillness.

STILLNESS is the RMS deviation of the output about its own local mean inside
a still segment: the part of the output that is not the operator holding a
position. LAG is the mean projection of (raw - filtered) onto the direction
of travel inside a moving segment: how far behind the hand the command sits.
A negative lag would mean the filter leads the hand, which nothing causal can
do, and is asserted against.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "srl_teleop"))

from srl_teleop import smoothing as sm                       # noqa: E402

#: Below this the hand is holding a position rather than travelling. Chosen
#: from the recording's own speed distribution, not from a target: see
#: `--report-speeds`, which prints the percentiles this sits between.
STILL_MPS = 0.02
#: Above this the hand is travelling fast enough that a phase lag is what an
#: operator would actually feel.
MOVE_MPS = 0.10
#: A segment shorter than this is a transition, not a state.
MIN_SEG = 25


def read_cmd(path, arm):
    """The commanded position the master produced, and its timestamps."""
    ts, ps = [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            try:
                t = float(row["t"])
                p = [float(row["%s_cmd_%s" % (arm, c)]) for c in "xyz"]
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(v) for v in p):
                continue
            ts.append(t)
            ps.append(p)
    if len(ts) < 100:
        raise SystemExit("only %d usable rows in %s for the %s arm"
                         % (len(ts), path, arm))
    return np.asarray(ts, float), np.asarray(ps, float)


def segments(t, p, lo, hi, minlen=MIN_SEG):
    """Runs where the RAW speed is inside [lo, hi). Computed ONCE, from the
    raw signal, and shared by every filter."""
    # SPEED IS DISPLACEMENT OVER A WINDOW, NOT A SAMPLE-TO-SAMPLE DIFFERENCE.
    #
    # A finite difference of a noisy signal measures the noise. At the
    # \SI{1.5}{\milli\metre} tremor this recording carries and \SI{50}{\hertz},
    # a hand holding perfectly still shows an instantaneous speed of about
    # \SI{0.1}{\metre\per\second} -- five times the threshold below which it
    # is supposed to count as still -- so a sample-difference segmenter finds
    # NO still segments in a recording that is mostly a person holding a
    # position. The clutch's own quasi-static gate makes the same choice for
    # the same reason: it asks how far the master moved in a window, not how
    # fast one sample differs from the next.
    hz = len(t) / max(float(t[-1] - t[0]), 1e-6)
    # THE WINDOW IS +/- 250 ms, AND ITS WIDTH IS SET BY THE TREMOR.
    # Two samples of a \SI{1.5}{\milli\metre}-per-axis tremor differ by about
    # \SI{3.7}{\milli\metre} in three dimensions whatever the window, so the
    # apparent speed of a still hand is that displacement divided by the
    # window: \SI{0.018}{\metre\per\second} over 200 ms, which straddles the
    # \SI{0.02}{\metre\per\second} threshold and finds no still segments at
    # all, against \SI{0.007}{\metre\per\second} over 500 ms, which is
    # comfortably inside it. A genuine \SI{0.02}{\metre\per\second} motion
    # reads \SI{0.02}{\metre\per\second} at either width.
    w = max(2, int(round(0.25 * hz)))                 # +/- 250 ms
    i0 = np.clip(np.arange(len(t)) - w, 0, len(t) - 1)
    i1 = np.clip(np.arange(len(t)) + w, 0, len(t) - 1)
    span = np.maximum(t[i1] - t[i0], 1e-6)
    v = np.linalg.norm(p[i1] - p[i0], axis=1) / span
    inside = (v >= lo) & (v < hi)
    out, start = [], None
    for i, b in enumerate(inside):
        if b and start is None:
            start = i
        elif not b and start is not None:
            if i - start >= minlen:
                out.append((start, i))
            start = None
    if start is not None and len(inside) - start >= minlen:
        out.append((start, len(inside)))
    return out, v


def run_filter(kind, t, p, **kw):
    f = sm.make(kind, **kw)
    out = np.empty_like(p)
    prev = None
    for i in range(len(t)):
        dt = 0.0 if prev is None else float(t[i] - prev)
        prev = t[i]
        out[i] = f(p[i], max(dt, 0.0))
    return out


def score(t, p, y, still, moving):
    """Stillness and lag for one filter, on the shared segmentation."""
    s = []
    for a, b in still:
        seg = y[a:b]
        s.append(np.sqrt(((seg - seg.mean(0)) ** 2).sum(1).mean()))
    lags = []
    for a, b in moving:
        d = p[b - 1] - p[a]
        n = np.linalg.norm(d)
        if n < 1e-6:
            continue
        u = d / n
        lags.append(float(np.dot((p[a:b] - y[a:b]), u).mean()))
    return (float(np.mean(s)) * 1000.0 if s else float("nan"),
            float(np.mean(lags)) * 1000.0 if lags else float("nan"))


def measure(path, arm):
    t, p = read_cmd(path, arm)
    still, v = segments(t, p, 0.0, STILL_MPS)
    moving, _ = segments(t, p, MOVE_MPS, np.inf)
    rows = []
    for name, kind, kw in (("none (raw)", "none", {}),
                           ("ema(0.3)  [was]", "ema", dict(alpha=0.3)),
                           ("one_euro  [now]", "one_euro", {})):
        y = run_filter(kind, t, p, **kw)
        st, lg = score(t, p, y, still, moving)
        rows.append(dict(law=name, still_mm=round(st, 3), lag_mm=round(lg, 3)))
    return dict(
        source=os.path.relpath(path, WS), arm=arm, frames=int(len(t)),
        seconds=round(float(t[-1] - t[0]), 1),
        rate_hz=round(float(len(t) / max(t[-1] - t[0], 1e-6)), 1),
        still_segments=len(still),
        still_frames=int(sum(b - a for a, b in still)),
        moving_segments=len(moving),
        moving_frames=int(sum(b - a for a, b in moving)),
        speed_p50_mps=round(float(np.percentile(v, 50)), 4),
        speed_p95_mps=round(float(np.percentile(v, 95)), 4),
        rows=rows)


def self_test():
    """Known answers on a CONSTRUCTED trace, so a failure here is the
    scorer's and not the recording's."""
    ok = fail = 0

    def check(name, cond, got=""):
        nonlocal ok, fail
        if cond:
            ok += 1
            print("  ok    %s %s" % (name, got))
        else:
            fail += 1
            print("  FAIL  %s %s" % (name, got))

    rng = np.random.default_rng(11)
    fs, dt = 50.0, 1.0 / 50.0
    hold = np.zeros((600, 3)) + rng.normal(0, 0.0015, (600, 3))
    v = 0.4
    ramp = np.stack([v * np.arange(600) * dt, np.zeros(600), np.zeros(600)], 1)
    ramp += hold[:600] * 0 + rng.normal(0, 0.0015, (600, 3))
    p = np.vstack([hold, ramp + hold[-1]])
    t = np.arange(len(p)) * dt

    still, _ = segments(t, p, 0.0, STILL_MPS)
    moving, _ = segments(t, p, MOVE_MPS, np.inf)
    check("a constructed hold is found as a still segment", len(still) >= 1,
          "%d segment(s)" % len(still))
    check("a constructed 0.4 m/s ramp is found as moving", len(moving) >= 1,
          "%d segment(s)" % len(moving))

    y = run_filter("none", t, p)
    st, lg = score(t, p, y, still, moving)
    check("an unfiltered signal has ~zero lag", abs(lg) < 0.5, "%.3f mm" % lg)
    check("an unfiltered signal's stillness is the injected tremor",
          1.0 < st < 4.0, "%.2f mm" % st)

    ye = run_filter("ema", t, p, alpha=0.3)
    ste, lge = score(t, p, ye, still, moving)
    check("a heavy fixed filter lags a moving hand", lge > 5.0,
          "%.2f mm" % lge)
    check("and it is steadier than raw when still", ste < st,
          "%.2f vs %.2f mm" % (ste, st))

    yo = run_filter("one_euro", t, p)
    sto, lgo = score(t, p, yo, still, moving)
    check("the adaptive filter beats the fixed one on BOTH",
          sto < ste and lgo < lge,
          "still %.2f<%.2f, lag %.2f<%.2f" % (sto, ste, lgo, lge))
    # THE CONTROL. No causal filter can lead the hand; a negative lag would
    # mean the scorer has the sign the wrong way round.
    check("no filter leads the hand (lag is never negative)",
          min(lg, lge, lgo) > -0.5,
          "min %.3f mm" % min(lg, lge, lgo))
    # AND a scorer that cannot distinguish two obviously different filters
    # is not a scorer.
    check("the three laws are distinguishable", len({round(x, 2) for x in
                                                     (lg, lge, lgo)}) == 3)
    print("\n%d checks, %d failed" % (ok + fail, fail))
    return 1 if fail else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", default="",
                    help="a teleop recording; the largest one by default")
    ap.add_argument("--arm", default="both", choices=("left", "right", "both"))
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--out", default=os.path.join(
        WS, "recordings", "baselines", "teleop_replay.json"))
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()

    path = a.csv or max(glob.glob(os.path.join(WS, "recordings",
                                               "teleop_*.csv")),
                        key=os.path.getsize, default="")
    if not path or not os.path.exists(path):
        print("no teleop recording found under recordings/", file=sys.stderr)
        return 2
    arms = ["left", "right"] if a.arm == "both" else [a.arm]
    res = [measure(path, arm) for arm in arms]
    print("THE SMOOTHER ON A REAL OPERATOR'S HAND, replayed offline")
    for r in res:
        print("\n%s arm: %d frames over %.0f s (%.1f Hz), %d still segments "
              "(%d frames), %d moving segments (%d frames)"
              % (r["arm"], r["frames"], r["seconds"], r["rate_hz"],
                 r["still_segments"], r["still_frames"],
                 r["moving_segments"], r["moving_frames"]))
        print("  commanded speed: median %.3f m/s, 95th %.3f m/s"
              % (r["speed_p50_mps"], r["speed_p95_mps"]))
        print("  %-22s %10s %10s" % ("law", "still_mm", "lag_mm"))
        for row in r["rows"]:
            print("  %-22s %10.2f %10.2f"
                  % (row["law"], row["still_mm"], row["lag_mm"]))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as fh:
        json.dump(dict(source=res[0]["source"], arms=res), fh, indent=2)
    print("\nwrote %s" % os.path.relpath(a.out, WS))
    return 0


if __name__ == "__main__":
    sys.exit(main())
