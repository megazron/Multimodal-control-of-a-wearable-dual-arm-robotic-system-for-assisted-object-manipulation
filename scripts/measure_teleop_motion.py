#!/usr/bin/env python3
"""What the teleoperation motion generator does to the end effector.

    python3 scripts/measure_teleop_motion.py --self-test    # the INSTRUMENT
    python3 scripts/measure_teleop_motion.py                # the measurement

WHAT IS BEING MEASURED, AND WHY IT IS THE END EFFECTOR AND NOT THE JOINTS.
`ik_follower_node` solves IK for a commanded pose and then has to get the
joints there. The IK solution is a POINT; everything between here and there is
invented by the motion generator, and until 2026-08-23 that generator was
`clamp_towards`, which clamps EACH JOINT INDEPENDENTLY to `max_step_rad` per
cycle. A joint needing 0.10 rad arrives in one cycle and a joint needing 0.50
rad takes two, so the joints are at different fractions of their own travel at
every instant, so the arm is not on the straight line the IK solution implies,
so THE HAND GOES SOMEWHERE NOBODY ASKED FOR. That excursion is what this
script puts a number on, in millimetres, against the 30 mm capture gate a
grasp has to hit.

TWO METRICS, BECAUSE ONE OF THEM MIXES TWO THINGS
-------------------------------------------------
  same-cycle    Where the hand is at cycle k, against where it would be if
                every joint were at fraction k/N. This is the diagnosis as it
                was first written, and it is reproduced here EXACTLY --
                max 135.3 mm, mean 45.1 mm -- because a measurement you cannot
                reproduce is not a baseline.

                It conflates two different faults: leaving the straight line
                (a PATH error) and not being at k/N along it (a TIMING
                difference). A jerk-limited generator is DELIBERATELY not at
                k/N -- that is what a velocity profile is -- so scoring it on
                this metric charges it for the feature. It is reported for
                both, and it is not the headline.

  off-path      The distance from each commanded hand position to the CURVE
                the straight joint-space line traces. Parameterisation-free:
                it does not care how fast anything moved, only whether the
                hand left the path. This is the fault, isolated, and it is the
                number to compare.

THE INSTRUMENT IS CHECKED BEFORE IT IS USED. `--self-test` runs `srl_fk`'s own
control (the compiled FK against the URDF walker), then feeds this script's
metrics three paths whose answers are constructed: the reference path itself
(must read 0.000 mm), a path displaced along the line (must read 0 off-path
and nonzero same-cycle), and a path displaced ACROSS it (must read the
displacement). A metric that cannot report a bad path is not a metric.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (HERE, os.path.join(ROOT, "config"),
          os.path.join(ROOT, "src/srl_teleop")):
    if p not in sys.path:
        sys.path.insert(0, p)

import home_positions                                          # noqa: E402
from srl_fk import FK, CompiledFK, compiled_matches_urdf       # noqa: E402
from srl_teleop.motion_generator import (                      # noqa: E402
    DOF, MotionGenerator, limits_for, plan)

OUT = os.path.join(ROOT, "recordings/baselines/teleop_motion.json")

# THE CASE THE DIAGNOSIS WAS WRITTEN ON. A realistic slew: every joint moves,
# the largest is 25x the smallest, and it starts from the shipped home rather
# than from zeros, so the Jacobian is the one the arm really has there.
SLEW = [0.50, -0.30, 0.10, 0.25, -0.05, 0.15, 0.02]
MAX_STEP = 0.35        # ik_follower_node's shipped max_step_rad
RATE_HZ = 50.0         # ik_follower_node's declared cascade_rate_hz
GRASP_GATE_MM = 30.0   # what a pad miss has to stay under


# ------------------------------------------------------------------- the rig

class Rig:
    """FK for one arm, end effector only. Checked by `srl_fk`'s own control."""

    def __init__(self, arm):
        self.arm = arm
        self.fk = FK()
        self.cf = CompiledFK(self.fk, arm, ["end_effector_link"])

    def ee(self, q):
        return self.cf(np.asarray(q, float))[0][:3, 3]

    def curve(self, q0, qT, n=2001):
        """The hand's path along the straight joint-space line, densely."""
        q0, qT = np.asarray(q0, float), np.asarray(qT, float)
        self._line = (q0, qT, n)
        return np.array([self.ee(q0 + (qT - q0) * s)
                         for s in np.linspace(0.0, 1.0, n)])


def same_cycle_mm(rig, q0, qT, path):
    """EE distance at cycle k from the uniform-time synchronised reference."""
    q0, qT = np.asarray(q0, float), np.asarray(qT, float)
    n = len(path) - 1
    if n < 1:
        return [0.0]
    return [float(np.linalg.norm(rig.ee(path[k]) - rig.ee(q0 + (qT - q0)
                                                          * (k / n))) * 1000.0)
            for k in range(n + 1)]


def densify(path, step_rad=0.02):
    """The path the arm actually TRAVERSES, not just the points commanded.

    The controller interpolates in joint space between one commanded point and
    the next, so a generator that commands three points does not teleport
    between them -- it walks the straight line joining them, and the hand goes
    wherever that leads. A metric that reads only the commanded vertices would
    score a coarse generator as better than a fine one purely because it was
    sampled less often. Same 0.02 rad densifying step `safe_motion.check_path`
    uses, for the same reason.

    ON THIS CASE IT CHANGES NOTHING, and that is worth writing down rather than
    leaving as an implied claim. Every figure in the step sweep is identical
    with and without it, because the clamp's worst excursion happens to fall
    on a commanded vertex. So the 51.3 mm at `max_step` 0.35 and the 68.0 mm
    at 0.01 are not one number undersampled two ways -- they are two genuinely
    different paths, because the step size changes which corners the polyline
    cuts. The finding survives either reading: the excursion does not tend to
    zero as the step shrinks, it settles at 68 mm.
    """
    out = [list(path[0])]
    for a, b in zip(path, path[1:]):
        n = max(1, int(math.ceil(max(abs(x - y) for x, y in zip(a, b))
                                 / step_rad)))
        for k in range(1, n + 1):
            out.append([x + (y - x) * k / n for x, y in zip(a, b)])
    return out


def off_path_mm(rig, curve, path, refine=40):
    """EE distance from each commanded pose to the reference CURVE.

    REFINED, NOT JUST SAMPLED, and the first version was not. A 4001-point
    curve over a 0.4 m path samples it every 0.1 mm, so the metric had a
    ~0.05 mm floor and reported the phase-synchronised generator -- which is
    on the line to 2e-16 rad -- as 0.0509 mm off it. That is the instrument's
    resolution being read as the robot's behaviour, which is the fault this
    whole file exists to avoid, so the nearest sample is refined by bisection
    on the real FK until the answer is the geometry rather than the grid.
    """
    q0, qT, n = rig._line
    q0, qT = np.asarray(q0, float), np.asarray(qT, float)
    out = []
    for q in path:
        p = rig.ee(q)
        i = int(np.argmin(np.linalg.norm(curve - p, axis=1)))
        lo = max(0.0, (i - 1) / (n - 1.0))
        hi = min(1.0, (i + 1) / (n - 1.0))
        for _ in range(refine):
            a = lo + (hi - lo) / 3.0
            b = hi - (hi - lo) / 3.0
            if (np.linalg.norm(rig.ee(q0 + (qT - q0) * a) - p)
                    < np.linalg.norm(rig.ee(q0 + (qT - q0) * b) - p)):
                hi = b
            else:
                lo = a
            if hi - lo < 1e-12:
                break
        s = 0.5 * (lo + hi)
        out.append(float(np.linalg.norm(rig.ee(q0 + (qT - q0) * s) - p)
                         * 1000.0))
    return out


# ----------------------------------------------------------- the generators

def clamp_path(q0, qT, max_step=MAX_STEP):
    """`ik_follower_node.clamp_towards` driven exactly as the node drives it.

    THE REAL FUNCTION, imported from the node. It is kept importable for
    precisely this reason: measuring a reimplementation of the old behaviour
    would measure the reimplementation.
    """
    from srl_teleop.ik_follower_node import clamp_towards, wrap_continuous
    cur = list(q0)
    out = [list(cur)]
    for _ in range(20000):
        cur = clamp_towards(list(qT), wrap_continuous(cur), max_step)
        out.append(list(cur))
        if max(abs(a - b) for a, b in zip(cur, qT)) < 1e-12:
            break
    return out


def generated_path(arm, q0, qT, dt, backend="auto"):
    gen = MotionGenerator(arm, backend=backend)
    gen.resync(q0)
    p = plan(gen, list(qT), dt)
    return p, gen


# ------------------------------------------------------------ the measurement

def measure_arm(arm, dt=1.0 / RATE_HZ, verbose=True):
    rig = Rig(arm)
    lim = limits_for(arm)
    q0 = np.array(home_positions.load_home_radians(arm))
    qT = q0 + np.array(SLEW)
    curve = rig.curve(q0, qT)

    rows = {}

    # ---- BEFORE: the per-joint clamp, as shipped
    cp = clamp_path(q0, qT)
    sc = same_cycle_mm(rig, q0, qT, cp)
    op = off_path_mm(rig, curve, densify(cp))
    arrivals = []
    for j in range(DOF):
        k = next(k for k in range(len(cp)) if abs(cp[k][j] - qT[j]) < 1e-12)
        arrivals.append(k)
    # ITS OWN IMPLIED VELOCITY AND ACCELERATION, by finite difference. The
    # clamp publishes positions only, but a position sequence at a fixed rate
    # IS a velocity and an acceleration, and they are what the arm has to
    # produce. Passing None for the acceleration would have excused it from
    # the continuity check rather than measuring it.
    vel = [[(cp[k + 1][j] - cp[k][j]) / dt for j in range(DOF)]
           for k in range(len(cp) - 1)]
    acc = [[(v2[j] - v1[j]) / dt for j in range(DOF)]
           for v1, v2 in zip([[0.0] * DOF] + vel, vel)]
    rows["clamp_towards"] = _row(sc, op, arrivals, vel, lim, dt, acc,
                                 "per-joint clamp, max_step %.2f rad"
                                 % MAX_STEP)

    # ---- AFTER: the generator
    p, gen = generated_path(arm, q0, qT, dt)
    sc = same_cycle_mm(rig, q0, qT, p.positions)
    op = off_path_mm(rig, curve, densify(p.positions))
    arrivals = p.arrival_cycles(list(qT))
    rows[gen.backend] = _row(sc, op, arrivals, p.velocities, lim, dt,
                             p.accelerations,
                             "%s, %s-synchronised, %d re-plan(s)"
                             % (gen.backend, p.synchronisation, gen.replans))

    # ---- AND THE FALLBACK, measured rather than assumed to be fine
    pf, gf = generated_path(arm, q0, qT, dt, backend="clamp")
    rows[gf.backend] = _row(same_cycle_mm(rig, q0, qT, pf.positions),
                            off_path_mm(rig, curve, densify(pf.positions)),
                            pf.arrival_cycles(list(qT)), pf.velocities, lim,
                            dt, pf.accelerations,
                            "the fallback: one scale factor for the vector")

    if verbose:
        _print_arm(arm, rows, dt)
    return {"arm": arm, "dt_s": dt, "max_step_rad": MAX_STEP,
            "start_rad": [float(v) for v in q0],
            "slew_rad": SLEW,
            "velocity_limits_rad_s": lim.velocity,
            "limit_provenance": lim.provenance,
            "generators": rows}


def _row(sc, op, arrivals, vel, lim, dt, acc, note):
    worst_v = max(abs(v[j]) / lim.velocity[j] for v in vel for j in range(DOF))
    row = {
        "note": note,
        "same_cycle_max_mm": max(sc), "same_cycle_mean_mm": float(np.mean(sc)),
        "off_path_max_mm": max(op), "off_path_mean_mm": float(np.mean(op)),
        "cycles": len(sc) - 1,
        "arrival_cycles": arrivals,
        "synchronised": len(set(arrivals)) == 1,
        "peak_velocity_frac_of_limit": worst_v,
        "velocity_limits_respected": worst_v <= 1.0 + 1e-6,
    }
    if acc is None:
        row["accel_step_frac_of_jerk_dt"] = None
        row["jerk_limited"] = False
    else:
        worst = 0.0
        for k in range(len(acc) - 1):
            for j in range(DOF):
                worst = max(worst, abs(acc[k + 1][j] - acc[k][j])
                            / (lim.jerk[j] * dt))
        row["accel_step_frac_of_jerk_dt"] = worst
        row["jerk_limited"] = worst <= 1.0 + 1e-6
    return row


def _print_arm(arm, rows, dt):
    print("\n%s arm, %.0f Hz, slew %s" % (arm.upper(), 1.0 / dt, SLEW))
    print("  %-20s %9s %9s %9s %9s  %s"
          % ("generator", "off-path", "same-cyc", "peak v", "arrive", "sync"))
    print("  %-20s %9s %9s %9s %9s"
          % ("", "max mm", "max mm", "of limit", "cycles"))
    for name, r in rows.items():
        print("  %-20s %9.1f %9.1f %8.2fx %9s  %s"
              % (name, r["off_path_max_mm"], r["same_cycle_max_mm"],
                 r["peak_velocity_frac_of_limit"],
                 ",".join(str(a) for a in sorted(set(r["arrival_cycles"]))),
                 "YES" if r["synchronised"] else "NO"))


# ------------------------------------------------------- the instrument check

def self_test(verbose=True):
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-4s %s%s" % ("PASS" if cond else "FAIL", name,
                                   ("  -- " + detail) if detail else ""))

    # 1 -- the FK itself, against srl_fk's own control
    check("compiled FK agrees with the URDF walker",
          compiled_matches_urdf(verbose=False))

    rig = Rig("left")
    q0 = np.array(home_positions.load_home_radians("left"))
    qT = q0 + np.array(SLEW)
    curve = rig.curve(q0, qT)

    # 2 -- KNOWN ANSWER: the reference path scores zero on BOTH metrics
    ref = [q0 + (qT - q0) * (k / 20.0) for k in range(21)]
    sc = same_cycle_mm(rig, q0, qT, ref)
    op = off_path_mm(rig, curve, ref)
    check("the reference path reads 0.000 mm same-cycle", max(sc) < 1e-6,
          "max %.2e mm" % max(sc))
    check("the reference path reads 0.000 mm off-path", max(op) < 1e-4,
          "max %.2e mm" % max(op))

    # 3 -- KNOWN ANSWER: a path ALONG the line but wrongly timed is off-path
    #      zero and same-cycle NONZERO. This is the whole reason there are two
    #      metrics, and if it does not separate them they are one metric.
    slow = [q0 + (qT - q0) * ((k / 20.0) ** 2) for k in range(21)]
    check("a mistimed but on-line path is 0 off-path",
          max(off_path_mm(rig, curve, slow)) < 1e-4,
          "max %.2e mm" % max(off_path_mm(rig, curve, slow)))
    check("...and NONZERO same-cycle", max(same_cycle_mm(rig, q0, qT, slow))
          > 10.0, "max %.1f mm" % max(same_cycle_mm(rig, q0, qT, slow)))

    # 4 -- KNOWN ANSWER: a path pushed ACROSS the line reads the displacement.
    #      A metric that cannot fail on a deliberately broken input is not a
    #      metric, so here is the deliberately broken input.
    bent = []
    for k in range(21):
        q = np.array(q0 + (qT - q0) * (k / 20.0))
        q[2] += 0.20 * math.sin(math.pi * k / 20.0)      # bow joint_3 out
        bent.append(q)
    ob = off_path_mm(rig, curve, bent)
    check("a path bowed off the line is CAUGHT off-path", max(ob) > 20.0,
          "max %.1f mm" % max(ob))

    # 5 -- the shipped clamp is caught, and the generator is not
    cp = clamp_path(q0, qT)
    check("clamp_towards reproduces the recorded diagnosis exactly",
          abs(max(same_cycle_mm(rig, q0, qT, cp)) - 135.3) < 0.05
          and abs(float(np.mean(same_cycle_mm(rig, q0, qT, cp))) - 45.1) < 0.05,
          "max %.1f mean %.1f mm" % (max(same_cycle_mm(rig, q0, qT, cp)),
                                     float(np.mean(same_cycle_mm(rig, q0, qT,
                                                                 cp)))))
    p, gen = generated_path("left", q0, qT, 1.0 / RATE_HZ)
    gop = max(off_path_mm(rig, curve, p.positions))
    check("the generator is on the line", gop < 1e-3, "max %.2e mm" % gop)

    if verbose:
        print("measure_teleop_motion self-test %s"
              % ("PASSED" if ok else "FAILED"))
    return ok


def _jsonable(o):
    """numpy scalars come out of the metrics; json will not take them."""
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    raise TypeError("not JSON serialisable: %r" % (o,))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--rate-hz", type=float, default=RATE_HZ)
    ap.add_argument("--no-write", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return 0 if self_test() else 1

    if not self_test(verbose=False):
        print("INSTRUMENT SELF-TEST FAILED -- refusing to report a "
              "measurement. Run --self-test.")
        return 2
    print("instrument self-test passed (FK control, four known answers)")

    dt = 1.0 / a.rate_hz
    out = {"case": "left/right home + %s, %.0f Hz" % (SLEW, a.rate_hz),
           "grasp_gate_mm": GRASP_GATE_MM, "arms": {}}
    for arm in ("left", "right"):
        out["arms"][arm] = measure_arm(arm, dt)

    # THE STEP SIZE DOES NOT RESCUE IT, and that is the finding that makes
    # this a design fault rather than a tuning value.
    print("\nthe per-joint clamp against its own step size (left arm):")
    rig = Rig("left")
    q0 = np.array(home_positions.load_home_radians("left"))
    qT = q0 + np.array(SLEW)
    curve = rig.curve(q0, qT)
    sweep = {}
    for ms in (0.35, 0.20, 0.10, 0.05, 0.02, 0.01):
        cp = clamp_path(q0, qT, ms)
        op = off_path_mm(rig, curve, densify(cp))
        sweep["%.2f" % ms] = {"cycles": len(cp) - 1, "off_path_max_mm": max(op)}
        print("  max_step %.2f rad -> %3d cycles, %6.1f mm off the path"
              % (ms, len(cp) - 1, max(op)))
    out["clamp_step_sweep_left"] = sweep

    fine = sweep["0.01"]["off_path_max_mm"]
    coarse = sweep["0.35"]["off_path_max_mm"]
    print("\n  %.1f mm at the shipped 0.35 rad, %.1f mm at 0.01 rad. The "
          "excursion does NOT\n  tend to zero as the step shrinks -- it "
          "settles -- because the joints are\n  at different fractions of "
          "their own travel however small the step is.\n  Each row is a "
          "genuinely different path, densified at 0.02 rad, not one\n  path "
          "sampled at different rates. It is the shape of the algorithm."
          % (coarse, fine))

    if not a.no_write:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        with open(OUT, "w") as f:
            json.dump(out, f, indent=2, sort_keys=True, default=_jsonable)
        print("\nwrote %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
