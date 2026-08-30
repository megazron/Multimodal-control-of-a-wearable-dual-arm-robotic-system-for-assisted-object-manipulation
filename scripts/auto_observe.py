#!/usr/bin/env python3
"""Find a pose from which the gripper camera can SEE the cube -- automatically.

WHY THIS EXISTS
---------------
Every attempt so far started from a pose an operator had hand-guided the arm
into.  That does not survive contact with reality:

  * A saved pose stops seeing the cube the moment the cube, the table or the
    rig moves.  One saved pose was re-commanded exactly (joints matched to
    under 0.1 deg) and the camera was looking at the floor past the table
    edge, because the scene had moved and the joints had not.
  * The camera is on the WRIST, so "where it looks" is a property of the whole
    arm configuration, not something a human can judge from outside.

So the viewpoint is SEARCHED for instead of remembered.  The search is a small
grid of perturbations about the current pose, ordered nearest-first, and every
candidate is rejected unless the whole hand clears the table -- the search can
never propose a pose that puts the gripper through the surface.

WHAT "SEEING IT" MEANS
----------------------
Not "some green pixels".  A candidate counts only if the cube segments into a
solid cluster of depth points standing 15-70 mm proud of the fitted table
plane, with plausible cube dimensions.  A green reflection on the table floor
has no height and is rejected.
"""
import argparse
import itertools
import math
import sys

import numpy as np
import rclpy

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from execute_pick_left import ang_wrap  # noqa: E402
from safe_goto_left import TableCheck  # noqa: E402
from servo_pick_left import Eye, NoFrames, measure  # noqa: E402
from srl_fk import FK  # noqa: E402

# Joints that swing the wrist camera over the table without carrying the elbow
# across the body.  Joint 1 is deliberately given the SMALLEST range: rotating
# it is what sweeps the whole arm inboard.
SCAN_AXES = [
    (0, [0.0, -8.0, 8.0]),        # base yaw, kept small on purpose
    (1, [0.0, 10.0, -10.0]),      # shoulder: raises/lowers the look point
    (3, [0.0, 12.0, -12.0]),      # elbow: pushes the view out and back
    (5, [0.0, 15.0, -15.0, 30.0, -30.0]),   # wrist pitch: where it points
]
TABLE_MARGIN_M = 0.045
MIN_CUBE_PTS = 120
CUBE_MM_RANGE = (20.0, 75.0)


def candidates(q0):
    """Perturbations of q0, nearest first."""
    axes = [[(i, d) for d in ds] for i, ds in SCAN_AXES]
    out = []
    for combo in itertools.product(*axes):
        q = np.array(q0, float)
        cost = 0.0
        for i, d in combo:
            q[i] += math.radians(d)
            cost += abs(d)
        out.append((cost, q))
    out.sort(key=lambda x: x[0])
    return out


def plausible(m):
    if m is None:
        return False
    if m["n_pts"] < MIN_CUBE_PTS:
        return False
    return CUBE_MM_RANGE[0] <= m["height_mm"] <= CUBE_MM_RANGE[1]


def live_table(node, fk):
    """Fit the table from the CURRENT view and express it in world.

    The table guard used to come from the last saved cube fix, which goes
    stale the moment the cube or the table moves -- and a stale plane rejected
    EVERY scan candidate, so the search tried nothing at all and reported that
    the cube could not be found. Measuring it now costs one frame.
    """
    from srl_scene import fit_plane
    import numpy as _np
    if not node.frames():
        return None
    d, dk = node.img["d"], node.img["dk"]
    dep = _np.frombuffer(d.data, _np.uint16).reshape(
        d.height, d.width).astype(_np.float32) * 0.001
    fx, fy, cx, cy = dk.k[0], dk.k[4], dk.k[2], dk.k[5]
    v, u = _np.nonzero((dep > 0.08) & (dep < 2.0))
    if len(v) < 500:
        return None
    Z = dep[v, u]
    P = _np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], 1)
    n, dd, msk = fit_plane(P)
    if float(msk.mean()) < 0.06:
        return None
    q = node.fresh_q()
    T, = fk.poses(node.arm_name, q, ["camera_depth_frame"])
    n_w = T[:3, :3] @ n
    on_plane = T[:3, :3] @ (-n * dd) + T[:3, 3]
    if n_w[2] < 0:
        n_w = -n_w
    tab = TableCheck.__new__(TableCheck)
    tab.n = n_w / _np.linalg.norm(n_w)
    tab.d = -float(tab.n @ on_plane)
    return tab


def find_cube(node, fk, tab, max_tries=14, verbose=True):
    """Look from here; if the cube is not seen, search. Returns (measurement, q)."""
    fresh = live_table(node, fk)
    if fresh is not None:
        tab = fresh
        if verbose:
            print("  table re-fitted from the live view "
                  "(hand is %.0f mm above it)"
                  % (tab.min_hand_height(fk, node.fresh_q()) * 1000))
    else:
        # A STALE PLANE IS WORSE THAN NO PLANE.  The saved fix rejected all 134
        # candidates and the search moved nothing, reporting "cube not found"
        # for a cube that was simply out of frame.  Drop the guard rather than
        # gate on a surface that may not be where the file says.  The scan's
        # own moves are small (<=30 deg per joint) and the force guard is armed.
        if verbose:
            print("  no live table fit -- dropping the stale table guard for "
                  "the scan (moves stay small, force guard armed)")
        tab = None
    m = measure(node)   # raises NoFrames if the camera is dead
    if plausible(m):
        if verbose:
            print("  cube visible from the current pose "
                  "(%d pts, %.1f mm tall)" % (m["n_pts"], m["height_mm"]))
        return m, node.fresh_q()

    q0 = node.fresh_q().copy()
    if verbose:
        print("  cube NOT visible from here -- scanning")
    tried = rejected = 0
    for cost, q in candidates(q0):
        if cost == 0.0:
            continue
        if tried >= max_tries:
            break
        # never propose a pose or a path that puts the hand through the table
        if tab is not None:
            if tab.min_hand_height(fk, q) < TABLE_MARGIN_M:
                rejected += 1
                continue
            steps = 30
            low = min(tab.min_hand_height(fk, q0 + (q - q0) * (k / steps))
                      for k in range(steps + 1))
            if low < TABLE_MARGIN_M:
                rejected += 1
                continue
        tried += 1
        dq = math.degrees(np.abs(ang_wrap(q - node.fresh_q())).max())
        if verbose:
            print("  try %2d: move %.1f deg ..." % (tried, dq), end=" ", flush=True)
        if node.goto(q, "scan-%d" % tried) is None:
            if verbose:
                print("guard fired, stopping the scan")
            return None, node.fresh_q()
        try:
            m = measure(node)
        except NoFrames as e:
            print("STOPPING: %s -- the scan was blind, not empty" % e)
            raise
        if plausible(m):
            if verbose:
                print("FOUND (%d pts, %.1f mm tall)" % (m["n_pts"], m["height_mm"]))
            return m, node.fresh_q()
        if verbose:
            print("no")
    if verbose and tried == 0:
        print("  every candidate viewpoint was rejected by the table guard "
              "(%d of them). The hand is probably already close to the "
              "surface, or the table fix is wrong." % rejected)
    return None, node.fresh_q()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cube", default="/home/gausms/kortex_ws/recordings/"
                                      "baselines/cube_located_v2.json")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    rclpy.init()
    node = Eye()
    node.fk = FK()
    if not node.preflight():
        raise SystemExit(2)
    node.set_deadband(0.10)
    node._hold = node.fresh_q().copy()
    try:
        tab = TableCheck(args.cube)
    except Exception:
        tab = None
        print("  (no previous table fix; scanning without a table guard)")
    if args.dry_run:
        m = measure(node)
        print("visible now:", plausible(m),
              "" if m is None else "(%d pts, %.1f mm)" % (m["n_pts"], m["height_mm"]))
        return
    m, q = find_cube(node, node.fk, tab)
    if m is None:
        raise SystemExit("could not find the cube from any scanned viewpoint")
    print("observation pose (deg):", [round(math.degrees(x), 2) for x in q])


if __name__ == "__main__":
    main()
