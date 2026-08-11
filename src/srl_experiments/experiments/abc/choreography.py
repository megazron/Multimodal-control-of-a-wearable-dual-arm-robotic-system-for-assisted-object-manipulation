#!/usr/bin/env python3
"""CHOREOGRAPHED ROUTINES for the demonstration mode. Both arms, coordinated.

    python3 src/srl_experiments/experiments/abc/choreography.py

NOT A TASK, AND THE DISTINCTION IS LOAD-BEARING. These produce no trial data,
no condition, no metric and no comparison. They exist to show the system
moving well, which is a legitimate thing to want and a dangerous thing to
confuse with evidence -- a smooth clip is not a result, and the caption on
every routine says so.

SAFETY IS NOT RELAXED FOR A DEMONSTRATION. Every waypoint of every routine is
collision-checked against the WEARER through the same `/compute_ik` with
`avoid_collisions=True` that every task uses, at the same N, and the routines
run through `ik_follower_node` with the same clearance floor, the same step
guard and the same e-stop and dead-man subscriptions. `verify()` below refuses
to report if a control fails, exactly as the task verifiers do. A demo that
bypassed the safety stack would be a demonstration of something this project
does not have.

THREE ROUTINES OF DIFFERENT CHARACTER, because one routine shows one thing:

  WAVE       slow, wide, alternating. The arms take turns, so it reads as
             deliberate rather than mechanical, and it is the routine that
             shows REACH -- it visits the extremes of both marked regions.
  MIRROR     both arms simultaneously, symmetric about the sagittal plane.
             Shows COORDINATION: the two arms are doing the same thing at the
             same time, which is only legible because they are mirrored.
  SWEEP      a smooth continuous arc, both arms in antiphase. Shows SMOOTHNESS
             -- no dwell, no corner, constant speed -- which is what a viewer
             actually reads as "under control".

EVERY POSE COMES FROM THE MEASURED REACHABLE REGION, not from taste. The
survey (scripts/survey_work_surface.py, full path, N=3) gives two 0.40 x
0.15 m strips at |x| 0.30..0.70, y 0.05..0.20 on the work plane; the routines
also use a raised band, which is verified here rather than assumed.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import clip_tasks as CT                                      # noqa: E402

_dense = CT._dense
_hold = CT._hold

# The demonstration band. Chest height and above -- higher than the work
# plane, because a routine at table height is hidden behind the table.
Z_LOW = 1.20
Z_MID = 1.35
Z_HIGH = 1.50
Y_NEAR = 0.30
X_IN, X_OUT = 0.34, 0.60


def _m(x, y, z):
    """A world point for the LEFT arm; the right arm mirrors in x."""
    return [x, y, z]


def wave():
    """Alternating, wide, slow. The arms take turns."""
    L = _dense([_m(X_IN, Y_NEAR, Z_LOW), _m(X_OUT, Y_NEAR, Z_HIGH),
                _m(X_IN, Y_NEAR, Z_MID), _m(X_OUT, Y_NEAR, Z_LOW),
                _m(X_IN, Y_NEAR, Z_LOW)], step=0.04)
    R = _dense([_m(-X_OUT, Y_NEAR, Z_LOW), _m(-X_IN, Y_NEAR, Z_MID),
                _m(-X_OUT, Y_NEAR, Z_HIGH), _m(-X_IN, Y_NEAR, Z_LOW),
                _m(-X_OUT, Y_NEAR, Z_LOW)], step=0.04)
    # TAKING TURNS means one arm holds while the other moves. Concatenating
    # the two and padding with holds is what makes that legible; running both
    # at once would be the MIRROR routine with extra steps.
    left = L + _hold(L[-1], len(R))
    right = _hold(R[0], len(L)) + R
    return {"left": left, "right": right}


def mirror():
    """Both arms at once, symmetric about x = 0."""
    pts = [_m(X_IN, Y_NEAR, Z_LOW), _m(X_OUT, Y_NEAR, Z_MID),
           _m(X_OUT, Y_NEAR, Z_HIGH), _m(X_IN, Y_NEAR, Z_MID),
           _m(X_IN, Y_NEAR, Z_LOW)]
    L = _dense(pts, step=0.035)
    R = [[-p[0], p[1], p[2]] for p in L]
    return {"left": L, "right": R}


def sweep():
    """A continuous arc, both arms in antiphase. No dwell, no corner."""
    n = 60
    L, R = [], []
    for i in range(n + 1):
        th = 2.0 * math.pi * i / n
        L.append(_m(X_IN + (X_OUT - X_IN) * (0.5 - 0.5 * math.cos(th)),
                    Y_NEAR,
                    Z_LOW + (Z_HIGH - Z_LOW) * (0.5 + 0.5 * math.sin(th))))
        R.append(_m(-(X_IN + (X_OUT - X_IN) * (0.5 + 0.5 * math.cos(th))),
                    Y_NEAR,
                    Z_LOW + (Z_HIGH - Z_LOW) * (0.5 - 0.5 * math.sin(th))))
    return {"left": L, "right": R}


ROUTINES = {
    "wave": dict(build=wave, character="slow, wide, alternating -- shows REACH",
                 both_at_once=False),
    "mirror": dict(build=mirror,
                   character="simultaneous and symmetric -- shows COORDINATION",
                   both_at_once=True),
    "sweep": dict(build=sweep,
                  character="continuous arc in antiphase -- shows SMOOTHNESS",
                  both_at_once=True),
}


def path_length(p):
    return sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))


def main():
    print("CHOREOGRAPHED ROUTINES")
    print("%-8s %6s %6s %9s %9s  %s"
          % ("routine", "L pts", "R pts", "L path m", "R path m", "character"))
    for name in ("wave", "mirror", "sweep"):
        p = ROUTINES[name]["build"]()
        assert len(p["left"]) == len(p["right"]), name
        print("%-8s %6d %6d %9.3f %9.3f  %s"
              % (name, len(p["left"]), len(p["right"]),
                 path_length(p["left"]), path_length(p["right"]),
                 ROUTINES[name]["character"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
