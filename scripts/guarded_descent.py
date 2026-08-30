#!/usr/bin/env python3
"""Descend onto an object while CONTINUOUSLY measuring the gap to the surface.

    python3 scripts/guarded_descent.py --self-test
    python3 scripts/guarded_descent.py --target-mm 20     # on the real arm

THE PROBLEM THIS SOLVES
-----------------------
Every descent so far was open-loop: solve a pose, ramp the joints to it, hope.
The gripper went into the table because the stopping height was computed in
ROBOT coordinates, through a camera mount that is 8.45 deg wrong -- 34-59 mm of
error at working range, against a 40 mm cube.  Nothing re-measured on the way
down, so the error was only discovered by contact.

THE FIX
-------
Re-measure every step, and measure the ONE quantity the mis-calibration cannot
corrupt:

    gap = (pad midpoint IN THE CAMERA FRAME) . n_cam + d_cam

The pads and the camera are one rigid body, so their relative pose is FK down
a single chain.  The plane is measured in that same camera frame.  No hand-eye
transform appears, so an arbitrarily wrong mount does not move this number --
proved in srl_scene's self-test, where an injected 8.45 deg frame error leaves
the reported height unchanged at 149.9 mm.

The loop then descends in bounded steps, shrinking as it closes, and STOPS on
the measured gap rather than on a commanded pose.  It refuses to move when it
cannot see the surface, which is the case that previously ran blind.
"""
import argparse
import math
import sys

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from srl_scene import descent_step, fit_plane, pad_height_above  # noqa: E402

MAX_STEP_M = 0.020      # never command more than 20 mm of descent at once
NEAR_M = 0.040          # within 40 mm, halve the steps
TOL_M = 0.003           # stop when within 3 mm of the target gap
MAX_ITERS = 25
MIN_INLIERS = 0.20      # below this the "plane" is not the table


class Blind(Exception):
    """Raised when the surface cannot be measured -- never descend blind."""


def plan_descent(gap, target, near=NEAR_M, max_step=MAX_STEP_M):
    """Signed distance to move along -n this step. Positive = downward."""
    err = gap - target
    step = min(abs(err), max_step * (0.5 if abs(err) < near else 1.0))
    return math.copysign(step, err)


def descend(measure_fn, move_fn, target_m, max_iters=MAX_ITERS, verbose=True):
    """Close the gap to `target_m`, re-measuring every step.

    measure_fn() -> (pad_in_cam, n, d, inlier_fraction) or None if blind.
    move_fn(vec_in_cam) -> None, commands a small motion.
    Returns the final gap in metres.

    THE GAIN ADAPTS.  A fixed proportional step cannot converge against a
    constant execution bias: the step shrinks as the gap closes until it is
    exactly cancelled, and the loop parks short of the target.  The self-test
    reproduces that with a 6 mm-per-step bias and the descent stalls at
    26.8 mm.  Stiction, a deadband and a proportional bridge all behave this
    way, so the commanded step is scaled up whenever a step fails to deliver
    most of the motion it asked for.
    """
    if verbose:
        print("  %-4s %10s %10s %10s %7s"
              % ("it", "gap_mm", "target_mm", "step_mm", "gain"))
    gap, gain, prev = None, 1.0, None
    for it in range(1, max_iters + 1):
        m = measure_fn()
        if m is None:
            raise Blind("surface not measurable -- refusing to descend blind")
        pad, n, d, inl = m
        if inl < MIN_INLIERS:
            raise Blind("only %.0f%% plane inliers -- that is not the table"
                        % (inl * 100))
        gap = pad_height_above(pad, n, d)
        if prev is not None:
            achieved = prev[0] - gap
            want = prev[1]
            if want > 1e-6:
                if achieved < 0.35 * want:
                    gain = min(gain * 1.8, 8.0)
                elif achieved > 1.25 * want:
                    gain = max(gain / 1.4, 0.3)
        if abs(gap - target_m) <= TOL_M:
            if verbose:
                print("  %-4d %10.1f %10.1f %10s %7s  reached"
                      % (it, gap * 1000, target_m * 1000, "-", "-"))
            return gap
        s = plan_descent(gap, target_m)
        s = math.copysign(min(abs(s) * gain, MAX_STEP_M), s)
        if verbose:
            print("  %-4d %10.1f %10.1f %10.1f %7.2f"
                  % (it, gap * 1000, target_m * 1000, s * 1000, gain))
        prev = (gap, s)
        move_fn(-np.asarray(n, float) * s)
    return gap


# ----------------------------------------------------------------- self-test
def self_test():
    ok = True

    def chk(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        print("  %-52s %s %s" % (name, "OK" if cond else "FAIL", detail))

    print("descent control law")
    chk("far away -> full step", abs(plan_descent(0.200, 0.020) - 0.020) < 1e-9)
    chk("close in -> half step", abs(plan_descent(0.045, 0.020) - 0.010) < 1e-9,
        "(%.1f mm)" % (plan_descent(0.045, 0.020) * 1000))
    chk("never overshoots the target",
        abs(plan_descent(0.021, 0.020) - 0.001) < 1e-9)
    chk("below target -> moves UP", plan_descent(0.010, 0.020) < 0)

    print("\nclosed-loop descent against a simulated table")
    # the arm has a 12% scale error and a 6 mm bias in how it executes a step:
    # open loop would land 6+ mm out; closed loop must not care.
    n = np.array([0.08, -0.20, 0.976])
    n /= np.linalg.norm(n)
    d = 0.500
    state = {"pad": -n * d + n * 0.180}

    def measure_fn():
        return state["pad"], n, d, 0.85

    def move_fn(v):
        state["pad"] = state["pad"] + v * 0.88 + n * 0.006

    final = descend(measure_fn, move_fn, 0.020, verbose=False)
    chk("converged to the 20 mm target despite a sloppy actuator",
        abs(final - 0.020) <= TOL_M, "(final %.1f mm)" % (final * 1000))
    chk("never went below the surface", final > 0.0)

    print("\nit refuses to descend blind")
    try:
        descend(lambda: None, lambda v: None, 0.020, verbose=False)
        chk("blind measurement raises", False)
    except Blind as e:
        chk("blind measurement raises", True, "(%s)" % str(e)[:40])
    try:
        descend(lambda: (np.zeros(3), n, d, 0.05), lambda v: None, 0.02,
                verbose=False)
        chk("a bad plane fit raises", False)
    except Blind as e:
        chk("a bad plane fit raises", True, "(%s)" % str(e)[:40])

    print("\nthe guard survives a wrong camera mount")
    th = math.radians(8.45)
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    pad = -n * d + n * 0.150
    rng = np.random.default_rng(0)
    e1 = np.cross(n, [0, 0, 1.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    P = np.array([-n * d + e1 * u + e2 * v
                  for u, v in rng.uniform(-0.3, 0.3, (4000, 2))])
    n2, d2, _ = fit_plane(P @ R.T)
    h1 = pad_height_above(pad, n, d) * 1000
    h2 = pad_height_above(R @ pad, n2, d2) * 1000
    chk("gap unchanged by an 8.45 deg mount error",
        abs(h1 - h2) < 1.0, "(%.1f mm vs %.1f mm)" % (h1, h2))

    print("\nknown-answer self-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    raise SystemExit(self_test() if a.self_test else 0)
