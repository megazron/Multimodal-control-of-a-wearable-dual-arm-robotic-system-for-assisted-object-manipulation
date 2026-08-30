#!/usr/bin/env python3
"""Measure the master-arm smoothing THROUGH THE NODE'S OWN CODE PATH.

    python3 scripts/measure_master_smoothing.py
    python3 scripts/measure_master_smoothing.py --self-test

The method of measure_vr_smoothing.py, applied to master_pose_node:
`srl_teleop.smoothing` proves the maths on constructed signals, and this
script proves the NODE runs them -- the repeated failure mode here is a
feature that is present and does nothing, so what is driven is the shipped
`MasterPoseNode.tip_filter_sync` / `smooth_tip` / `reset_tip_filter`, the
exact methods `read_serial` calls per frame, with the parameter re-read,
rebuild-on-change and per-arm dt bookkeeping in the measurement.

WHAT IS STUBBED, AND WHY IT IS HONEST TO STUB IT. Everything upstream of the
tip estimate: the serial port, the Teensy line parsing, validation and the
spherical/FK position math. The filter's input is the TIP SIGNAL, and only a
constructed tip signal has arithmetic ground truth -- pushing a known tip
through the real FK would require inverting it to fabricate pot readings,
which would measure the inversion. rclpy's parameter/clock/logger surface is
faked the same way the VR script stubbed the robot pose: it is the harness,
not the path under test. Downstream, `to_robot_frame` is a fixed axis
permutation and the workspace mapping an affine offset, so the mm numbers
here survive both unchanged.

GROUND TRUTH IS CONSTRUCTED, at the master's own 50 Hz frame rate (not the
VR path's 72 -- the fixed EMA's response depends on the tick rate, which is
the defect being replaced, so the rate is part of the condition):
    STILL   a stationary master + 1.5 mm-rms tremor. The right answer is
            "the tip does not move": output RMS, lower is better.
    REACH   a 0.40 m/s constant-speed reach. The right answer is a straight
            line: steady-state lag, lower is better.
Scored together, because either alone can be won by detuning the other way.
"""
import argparse
import sys

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/src/srl_teleop")
from srl_teleop import master_pose_node as mpn  # noqa: E402

RATE = 50.0
DT = 1.0 / RATE
N = 700


class _Param:
    def __init__(self, v):
        self.value = v


class _Log:
    def info(self, *a, **k):
        pass

    warn = info
    error = info


def rig(params):
    """The node's smoothing surface, wired the way __init__ wires it."""
    p = {"smoothing": "one_euro", "ema_alpha": 0.3, "min_cutoff_hz": 0.5,
         "beta": 100.0, "d_cutoff_hz": 0.2, "smooth_orientation": False}
    p.update(params)
    n = mpn.MasterPoseNode.__new__(mpn.MasterPoseNode)
    n.arms = ["left"]
    n.get_parameter = lambda k: _Param(p[k])
    n.get_logger = lambda: _Log()
    n._smooth_key = None
    n._smooth_rejected = None
    n.tip_filt = {}
    n.quat_filt = {}
    n.tip_filter_sync()
    n.filt_t = {a: None for a in n.arms}
    n.last_filt_dt = {a: 0.0 for a in n.arms}
    return n


def run(node, sig):
    t = 100.0
    out = []
    for x in sig:
        t += DT
        out.append(node.smooth_tip("left", x, t))
    return np.asarray(out)


def still_sig(seed=3):
    rng = np.random.default_rng(seed)
    return rng.normal(0.0, 0.0015, (N, 3))


def reach_sig(v=0.40, seed=3):
    rng = np.random.default_rng(seed)
    t = np.arange(N) * DT
    return (np.stack([v * t, np.zeros(N), np.zeros(N)], 1)
            + rng.normal(0.0, 0.0015, (N, 3)))


def score(params):
    out = run(rig(params), still_sig())
    still = float(np.sqrt(((out - out.mean(0)) ** 2).sum(1).mean())) * 1000
    ramp = np.stack([0.40 * np.arange(N) * DT, np.zeros(N), np.zeros(N)], 1)
    out = run(rig(params), reach_sig())
    k = N // 4
    lag = float(np.mean(ramp[-k:, 0] - out[-k:, 0])) * 1000
    return still, lag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true",
                    help="also assert the ordering the change claims")
    a = ap.parse_args()

    print("Master-arm smoothing, measured through the shipped "
          "MasterPoseNode.smooth_tip (%.0f Hz)" % RATE)
    print("  still: a stationary master + 1.5 mm-rms tremor -> output RMS")
    print("  lag:   a 0.40 m/s reach -> how far the tip trails the master")
    print()
    print("%-24s %10s %10s" % ("law", "still_mm", "lag_mm"))
    res = {}
    for label, params in (
            ("none (raw)", {"smoothing": "none"}),
            ("ema(0.3)  [was]", {"smoothing": "ema", "ema_alpha": 0.3}),
            ("one_euro  [now]", {"smoothing": "one_euro"})):
        s, lag = score(params)
        res[label.split()[0]] = (s, lag)
        print("%-24s %10.2f %10.2f" % (label, s, lag))

    print()
    ok = True
    if a.self_test:
        def chk(name, cond, detail=""):
            nonlocal ok
            print("  %-52s %s   %s" % (name, "PASS" if cond else "FAIL",
                                       detail))
            ok = ok and cond
        e_s, e_l = res["ema(0.3)"]
        o_s, o_l = res["one_euro"]
        n_s, _ = res["none"]
        chk("the node actually applies the filter",
            o_s < n_s * 0.8, "%.2f vs %.2f mm raw" % (o_s, n_s))
        chk("one_euro is steadier than the ema(0.3) it replaced",
            o_s < e_s, "%.2f vs %.2f mm" % (o_s, e_s))
        chk("one_euro does not pay for it in lag",
            o_l <= e_l + 0.5, "%.2f vs %.2f mm" % (o_l, e_l))
        print()
        print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
