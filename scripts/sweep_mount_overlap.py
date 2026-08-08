#!/usr/bin/env python3
"""CAN A MOUNT CHANGE GIVE THE TWO ARMS A SHARED WORKSPACE? Quantified.

    python3 scripts/sweep_mount_overlap.py           # the sweep
    python3 scripts/sweep_mount_overlap.py --why     # is 0/63 IK or collision?

REPORT ONLY. Nothing is applied: a mount change invalidates P_HOME, the
workspace anchor and every clearance figure in the project.

METHOD -- why this does not need a URDF edit per candidate
----------------------------------------------------------
Moving an arm's base by a rigid transform T is exactly equivalent, for
REACHABILITY, to leaving the base alone and moving the target by T^-1:

    p reachable by base T*B  <=>  T^-1 p reachable by base B

So one running move_group answers every candidate. The same trick was used
for the earlier mount-pitch sweep.

WHAT THIS TRICK DOES **NOT** PRESERVE, and it matters
-----------------------------------------------------
The WEARER does not move with the mount. Under the transform, the arm stays
put and the target moves, which corresponds to the wearer having moved too.
So collision-with-wearer results are NOT valid under the trick, and this
sweep therefore runs with `avoid_collisions=False` and reports a PURELY
KINEMATIC answer. That is the right question to ask first: if the arms
cannot reach a shared point even ignoring the wearer, no mount rotation
fixes it and clearance is moot. Candidates that pass are flagged as needing a
real URDF check before anyone trusts their clearance.
"""
import argparse
import itertools
import json
import math
import os
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_task_scenes import Solver                    # noqa: E402

# Current mount origins, from srl_dual.urdf.xacro (backpack-relative
# (+-0.20, -0.20, 0.18) -> world (+-0.20, -0.32, 1.25)).
MOUNT = {"left": np.array([+0.20, -0.32, 1.25]),
         "right": np.array([-0.20, -0.32, 1.25])}

# The 63-point frontal grid the disjointness was measured on.
GRID = [(x, y, z)
        for z in (0.90, 1.10, 1.30)
        for y in (0.0, 0.2, 0.4)
        for x in (-0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6)]

# T3 needs BOTH grippers on ONE tray: two grip points 300 mm apart, at the
# same height. That is a harder test than "some shared point exists".
TRAY_SPAN = 0.30


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def inverse_map(arm, dx, dy, dz, tilt, splay):
    """Return f(p_world) -> p_to_test, for a mount moved by these deltas.

    `dx` widens the mounts OUTBOARD symmetrically (+ moves each away from the
    centreline). `tilt` pitches the plate further forward (about world x).
    `splay` rotates each mount outward about world z, mirrored.
    """
    side = +1.0 if arm == "left" else -1.0
    o = MOUNT[arm]
    t = np.array([side * dx, dy, dz], float)
    R = rot_z(side * splay) @ rot_x(tilt)

    def f(p):
        # p' = R^-1 (p - o - t) + o
        return R.T @ (np.asarray(p, float) - o - t) + o
    return f


def both_reachable(n, quats, p, maps, tries=3, avoid=False):
    for arm in ("left", "right"):
        if not n.solve(arm, maps[arm](p), quats[arm], avoid=avoid,
                       tries=tries):
            return False
    return True


def tray_ok(n, quats, maps, z_levels=(0.95, 1.05, 1.15), xs=(-0.10, 0.0, 0.10),
            y=0.34):
    """Two grip points TRAY_SPAN apart, same height, left grips the -x end."""
    for z in z_levels:
        for xc in xs:
            pl = np.array([xc - TRAY_SPAN / 2, y, z])
            pr = np.array([xc + TRAY_SPAN / 2, y, z])
            okL = n.solve("left", maps["left"](pl), quats["left"],
                          avoid=False, tries=3)
            okR = n.solve("right", maps["right"](pr), quats["right"],
                          avoid=False, tries=3)
            if okL and okR:
                return True, (float(xc), float(y), float(z))
            # the arms may be crossed: try the other assignment
            okL2 = n.solve("left", maps["left"](pr), quats["left"],
                           avoid=False, tries=3)
            okR2 = n.solve("right", maps["right"](pl), quats["right"],
                           avoid=False, tries=3)
            if okL2 and okR2:
                return True, (float(xc), float(y), float(z))
    return False, None


def front_reach(n, quats, maps, arm, step=0.02, limit=0.60):
    """How far forward (+y) from the arm's home EE, kinematics only."""
    p0 = n.ee_pos(arm)
    d = 0.0
    while d + step <= limit:
        p = p0 + np.array([0.0, d + step, 0.0])
        if not n.solve(arm, maps[arm](p), quats[arm], avoid=False, tries=3):
            break
        d += step
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--why", action="store_true",
                    help="is the current 0/63 kinematic or collision?")
    ap.add_argument("--json", default="recordings/baselines/"
                                      "mount_overlap_sweep.json")
    a = ap.parse_args()
    rclpy.init()
    n = Solver()
    n.spin(3.0)
    if not n.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik -- start the sim")
        return 2

    # give Solver an ee position helper
    def ee_pos(arm):
        import rclpy.time
        t = n.buf.lookup_transform("world", "%s_end_effector_link" % arm,
                                   rclpy.time.Time())
        v = t.transform.translation
        return np.array([v.x, v.y, v.z])
    n.ee_pos = ee_pos
    quats = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    ident = {arm: inverse_map(arm, 0, 0, 0, 0, 0) for arm in ("left", "right")}

    if a.why:
        print("IS THE CURRENT DISJOINTNESS KINEMATIC, OR THE WEARER?\n")
        for avoid in (True, False):
            both = sum(1 for p in GRID
                       if both_reachable(n, quats, p, ident, avoid=avoid))
            L = sum(1 for p in GRID
                    if n.solve("left", p, quats["left"], avoid=avoid, tries=3))
            R = sum(1 for p in GRID
                    if n.solve("right", p, quats["right"], avoid=avoid,
                               tries=3))
            print("  collisions %-3s : both %2d/63   left %2d/63   right %2d/63"
                  % ("ON" if avoid else "OFF", both, L, R))
        print("\n  If 'both' is 0 with collisions OFF, the wearer is not the")
        print("  binding constraint and no mount ROTATION can fix it -- the")
        print("  two reachable sets simply do not meet.")
        n.destroy_node()
        rclpy.shutdown()
        return 0

    print("MOUNT SWEEP -- both-arm overlap, purely kinematic (collisions OFF)")
    print("REPORT ONLY. Nothing is applied.\n")
    print("  %6s %6s %6s %6s %6s | %8s %8s %8s | %s"
          % ("dx", "dy", "dz", "tilt", "splay", "both/63", "frontL",
             "frontR", "T3 tray"))
    rows = []
    grid = list(itertools.product(
        (0.0, -0.10, -0.20),                 # dx: mounts INBOARD (narrower)
        (0.0, 0.15, 0.30),                   # dy: mounts FORWARD
        (0.0, -0.15),                        # dz: mounts LOWER
        (0.0, math.radians(-30), math.radians(-60)),   # extra forward tilt
        (0.0, math.radians(-25))))           # splay INWARD
    for dx, dy, dz, tilt, splay in grid:
        maps = {arm: inverse_map(arm, dx, dy, dz, tilt, splay)
                for arm in ("left", "right")}
        both = sum(1 for p in GRID
                   if both_reachable(n, quats, p, maps))
        fl = front_reach(n, quats, maps, "left")
        fr = front_reach(n, quats, maps, "right")
        t3, at = tray_ok(n, quats, maps) if both else (False, None)
        rows.append(dict(dx=dx, dy=dy, dz=dz, tilt_deg=math.degrees(tilt),
                         splay_deg=math.degrees(splay), both=both,
                         front_left=fl, front_right=fr, tray=bool(t3),
                         tray_at=at))
        print("  %+6.2f %+6.2f %+6.2f %+6.0f %+6.0f | %8s %8.2f %8.2f | %s"
              % (dx, dy, dz, math.degrees(tilt), math.degrees(splay),
                 "%d/63" % both, fl, fr,
                 ("YES at %s" % (at,)) if t3 else "no"))

    best = max(rows, key=lambda r: (r["both"], r["tray"]))
    print("\n  BEST OVERLAP: %d/63  at dx=%+.2f dy=%+.2f dz=%+.2f tilt=%+.0f "
          "splay=%+.0f  (T3 tray: %s)"
          % (best["both"], best["dx"], best["dy"], best["dz"],
             best["tilt_deg"], best["splay_deg"],
             "achievable" if best["tray"] else "NOT achievable"))
    if best["both"] == 0:
        print("\n  NO mount configuration in this sweep gives ANY shared")
        print("  point. The binding constraint is the arms' own kinematics,")
        print("  not the mount and not the wearer. This is a HARDWARE")
        print("  conclusion: it needs longer arms, a different mounting")
        print("  concept, or tasks that do not require a shared workspace.")
    try:
        os.makedirs(os.path.dirname(a.json), exist_ok=True)
        json.dump(rows, open(a.json, "w"), indent=1)
        print("\n  -> %s" % a.json)
    except OSError:
        pass
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
