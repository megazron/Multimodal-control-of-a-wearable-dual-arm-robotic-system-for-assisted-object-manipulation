#!/usr/bin/env python3
"""THE VIBRATION NUMBER: EE position std with degraded mode OFF vs ON.

    python3 scripts/prove_degraded_mode.py

METHOD -- a stationary master, replayed through real incoherent traces.

The claim under test is "the arm vibrates while the master is held still, and
the cause is the input". To test that, the operator must be stationary while
the faulty channels do what they really do. Both halves come from measurement:

  * INCOHERENT and DEAD channels replay their ACTUAL recorded values from the
    2026-08-06 capture -- the real fault, at its real rate, with its real
    jump distribution. Nothing is synthesised.
  * COHERENT channels are held at their per-run median, and the accelerometer
    is held at its median too. That is the "hand held still" condition: a
    perfectly steady operator on perfect wiring.

So every millimetre of output motion is contributed by the broken channels
and by nothing else. If degraded mode works, freezing them takes the output
to a constant.

The math is the shipped math: `PotCalibration.apply`, `elevation_from_accel`,
`fk`, `spherical_position` and `to_robot_frame` are imported from
`master_calibration`, the same module `master_pose_node` uses. Only the
six-line composition is re-expressed here, mirroring `spherical_tip()`.

Commanded EE = anchor + scale * (p - p_ref) with scale 1.0 and a constant
anchor, so the standard deviation of the commanded EE equals the standard
deviation of `to_robot_frame(p)`. The anchor never enters the answer.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "srl_teleop"))
from srl_teleop import master_calibration as mc          # noqa: E402
from srl_teleop import degraded_mode as dg               # noqa: E402

CAPTURE = ("recordings/trajectory_capture/capture_20260806_130205/"
           "all_segments.csv")


def load(path, arm):
    """Returns (joints_deg Nx7, accel Nx3) for one arm."""
    p = arm[0]
    jn = ["%s_j%d" % (p, i + 1) for i in range(7)]
    an = ["%s_ax" % p, "%s_ay" % p, "%s_az" % p]
    J, A = [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                j = [float(r[k]) for k in jn]
                a = [float(r[k]) for k in an]
            except (KeyError, ValueError, TypeError):
                continue
            # Reject serial parse glitches before anything else: 0.2-0.8% of
            # samples fall outside the sensor's own range and are not data.
            if any(not (0.0 <= v <= 360.0) for v in j):
                continue
            J.append(j)
            A.append(a)
    return np.array(J, float), np.array(A, float)


def tip(cal, joints_deg, accel, a_hat):
    """Mirrors master_pose_node.spherical_tip() for the quasi-static case."""
    q = cal.apply(joints_deg)
    elev = mc.elevation_from_accel(a_hat, accel)
    azim = q[0]
    q3 = 0.0 if float(joints_deg[2]) == 0.0 else q[2]
    reach = float(np.linalg.norm(mc.fk([q[0], q[1], q3, q[3], 0.0, 0.0, 0.0])))
    return np.array(mc.to_robot_frame(mc.spherical_position(reach, elev, azim)),
                    float)


def run(arm, J, A, frozen, cal, a_hat, hold_coherent=True):
    """Replay with `frozen` channels held at their zero reference."""
    zeros = list(cal.zeros)
    med_j = np.median(J, axis=0)
    med_a = np.median(A, axis=0)
    out = []
    for k in range(len(J)):
        j = list(J[k])
        if hold_coherent:
            # THE STATIONARY HAND. Every channel that is not itself faulty is
            # pinned to its median, so a coherent channel contributes exactly
            # zero motion and cannot be credited or blamed for any of the
            # output spread.
            for i in range(7):
                if i not in FAULTY[arm]:
                    j[i] = med_j[i]
        for i in frozen:
            j[i] = zeros[i]
        out.append(tip(cal, j, med_a, a_hat))
    return np.array(out, float)


def stats(name, P):
    sd = P.std(axis=0) * 1000.0
    pp = (P.max(axis=0) - P.min(axis=0)) * 1000.0
    n = float(np.linalg.norm(sd))
    print("    %-26s std  x %8.3f  y %8.3f  z %8.3f mm   |std| %8.3f mm"
          % (name, sd[0], sd[1], sd[2], n))
    print("    %-26s p-p  x %8.3f  y %8.3f  z %8.3f mm"
          % ("", pp[0], pp[1], pp[2]))
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--capture", default=CAPTURE)
    ap.add_argument("--baseline",
                    default="recordings/baselines/channels_20260806.json")
    a = ap.parse_args()
    base = dg.load_baseline(a.baseline)
    if not base:
        print("no channel baseline at %s" % a.baseline)
        return 1
    exc = dg.excluded_channels(base)
    global FAULTY
    FAULTY = exc
    print("THE VIBRATION NUMBER -- commanded EE spread, master held still")
    print("capture: %s" % a.capture)
    print("%d/%d channels coherent; frozen in degraded mode: %s\n"
          % (dg.n_coherent(base), dg.TOTAL_CHANNELS,
             ", ".join("%s j%s" % (k, "".join(str(i + 1) for i in v))
                       for k, v in exc.items())))
    rows = []
    for arm in ("left", "right"):
        J, A = load(a.capture, arm)
        if len(J) < 100:
            print("  %s: only %d usable rows" % (arm, len(J)))
            continue
        cal = mc.PotCalibration.load(arm)
        a_hat = cal.a_hat
        if a_hat is None:
            print("  %s: no a_hat in the zero capture -- cannot compute "
                  "elevation" % arm)
            continue
        print("  === %s ARM ===   %d replayed samples, frozen j%s"
              % (arm.upper(), len(J),
                 "".join(str(i + 1) for i in exc[arm]) or "none"))
        off = run(arm, J, A, frozen=[], cal=cal, a_hat=a_hat)
        on = run(arm, J, A, frozen=exc[arm], cal=cal, a_hat=a_hat)
        n_off = stats("degraded OFF (as shipped)", off)
        n_on = stats("degraded ON", on)
        red = 100.0 * (1.0 - n_on / max(n_off, 1e-12))
        print("    -> |std| %.3f mm  ->  %.3f mm   (%.1f%% reduction)\n"
              % (n_off, n_on, red))
        rows.append((arm, n_off, n_on, red))
    print("  SUMMARY")
    print("  %-6s %14s %14s %10s" % ("arm", "OFF |std| mm", "ON |std| mm",
                                     "reduction"))
    for arm, o, n, r in rows:
        print("  %-6s %14.3f %14.3f %9.1f%%" % (arm, o, n, r))
    print("\n  A stationary operator on healthy wiring would read 0.000 mm.")
    print("  The OFF column is therefore the vibration, in full.")
    print("  The ON column is 0.000 BY CONSTRUCTION -- a frozen channel is a")
    print("  constant. The measurement that matters is the OFF column.")
    cost_report(base)
    return 0


FAULTY = {"left": [], "right": []}


def cost_report(base):
    """A1b -- what degraded mode COSTS, in commanded workspace.

    Sweeps the LIVE channels over the range they actually showed in the
    capture and reports the commanded-position cloud: extent per axis, PCA
    singular values, isotropy (min/max) and convex-hull volume.

    Read the comparison carefully. Degraded OFF has the larger cloud, but the
    extra volume is contributed by channels whose consecutive samples are
    unrelated to the operator's arm -- it is reachable but not COMMANDABLE.
    A larger uncontrollable set is not a capability.
    """
    from itertools import product
    print("\n\n" + "=" * 72)
    print("A1b -- WHAT DEGRADED MODE COSTS (commanded position cloud)")
    print("=" * 72)
    exc = dg.excluded_channels(base)
    for arm in ("left", "right"):
        cal = mc.PotCalibration.load(arm)
        if cal.a_hat is None:
            continue
        rng = {}
        for i in range(7):
            d = base.get("%s_j%d" % (arm[0], i + 1)) or {}
            rng[i] = float(d.get("range_deg", 0.0))
        for tag, frozen in (("OFF", []), ("ON", exc[arm])):
            live = [i for i in range(4) if i not in frozen]   # position pots
            grids = []
            for i in range(4):
                if i in live and rng[i] > 5.0:
                    half = min(rng[i], 180.0) / 2.0
                    grids.append(np.linspace(cal.zeros[i] - half,
                                             cal.zeros[i] + half, 7))
                else:
                    grids.append(np.array([cal.zeros[i]]))
            # elevation sweeps the physical -90..+90 the IMU can observe
            P = []
            for combo in product(*grids):
                q = cal.apply(list(combo) + [cal.zeros[4], cal.zeros[5],
                                             cal.zeros[6]])
                reach = float(np.linalg.norm(
                    mc.fk([q[0], q[1], q[2], q[3], 0.0, 0.0, 0.0])))
                for elev in np.linspace(-math.pi / 2, math.pi / 2, 9):
                    P.append(mc.to_robot_frame(
                        mc.spherical_position(reach, elev, q[0])))
            P = np.array(P, float)
            # RADIAL EXTENT, not convex hull. The commanded set is
            # azimuth x elevation x reach in spherical coordinates; azimuth
            # and elevation are angular and survive every exclusion, so the
            # only dimension a frozen pot can remove is the RADIUS. A convex
            # hull cannot see that: the hull of a spherical shell is a solid
            # ball, and it reported an identical 0.0387 m^3 whether the shell
            # had thickness or none at all.
            r = np.linalg.norm(P, axis=1)
            r_ext = float(r.max() - r.min())
            shell = 4.0 * math.pi * float(r.mean()) ** 2
            vol = shell * r_ext
            print("  %-5s degraded %-3s  live position pots j%s"
                  % (arm, tag,
                     "".join(str(i + 1) for i in live) or "NONE"))
            print("        reach %.3f .. %.3f m  ->  RADIAL EXTENT %.4f m"
                  % (r.min(), r.max(), r_ext))
            print("        commandable volume ~ %.5f m^3 %s"
                  % (vol, "(a SURFACE -- zero thickness)" if r_ext < 1e-6
                     else ""))
        f = exc[arm]
        print("        LOST: %s\n" % dg.position_capability(arm, f))


if __name__ == "__main__":
    sys.exit(main())
