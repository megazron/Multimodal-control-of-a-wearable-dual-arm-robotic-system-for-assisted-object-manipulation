#!/usr/bin/env python3
"""CAN THESE ARMS GRASP FROM ABOVE? MEASURED, ON THE MOUNT THAT EXISTS.

    ./.venv_vision/bin/python scripts/measure_grasp_approach_angles.py

WHY THIS IS A MEASUREMENT AND NOT A LOOKUP
------------------------------------------
CLAUDE.md records "top-down on a surface: **0 of 840 cells**, any table height
0.70-1.10, either arm". That is a real result and it may no longer be true:
it was measured on the OLD MOUNT. The mounts moved **150 mm outboard and 15
degrees of yaw** on 2026-08-18, and the same file records that every earlier
"0 of 3360 cells" from that era was invalidated by exactly that change and had
to be re-measured -- after which the objects-on-table check went from
impossible to satisfied from the shipped scene.

So the honest thing is to ask the arm again rather than quote a number taken
before the geometry moved.

WHAT IS SWEPT
-------------
For every object the calibration measured, the approach direction is swept
from **straight down (+90 deg)** through horizontal to the shipped anchor
(about -31 deg, which reaches in from BELOW and is why a grasp today looks
like the hand coming up out of the table). At each elevation several yaws are
tried, and both the grasp pose and its pregrasp must solve -- a grasp you
cannot approach is not a grasp.

The wrist-to-pad offset is read at the object's own measured width, because
the Robotiq's fingers swing on a four-bar and `pad_mid_ee_for` is the measured
table.

WHAT IS NOT ANSWERED HERE
-------------------------
Whether the pose is SAFE. `avoid_collisions` is on, and CLAUDE.md hard
constraint 11 says that is not the wearer check -- the SRDF excludes the 44
proximal pairs a shoulder mount actually threatens. This reports reachability;
clearance is a separate gate and is named as one in the output.
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
for _p in (HERE, os.path.join(ROOT, "src/srl_perception"),
           os.path.join(ROOT, "src/srl_experiments"),
           os.path.join(ROOT, "src/srl_teleop"),
           os.path.join(ROOT, "src/srl_experiments/experiments/abc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

MAP = os.path.join(ROOT, "recordings/baselines/world_map.json")
OUT = os.path.join(ROOT, "recordings/baselines/grasp_approach_angles.json")
STANDOFF_M = 0.12


def axis_for(elev_deg, yaw_deg, arm):
    """Unit approach direction: +90 is straight DOWN onto the object."""
    el = math.radians(float(elev_deg))
    # Base horizontal heading: inboard-ish, mirrored per arm, then yawed.
    sx = -1.0 if arm == "left" else 1.0
    h = np.array([sx * math.cos(el), 0.0, -math.sin(el)])
    th = math.radians(float(yaw_deg))
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    v = R @ h
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else np.array([0.0, 0.0, -1.0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=MAP)
    ap.add_argument("--elevations", type=float, nargs="+",
                    default=[90, 80, 70, 60, 50, 40, 30, 20, 10, 0, -15, -30.76])
    ap.add_argument("--yaws", type=float, nargs="+",
                    default=[-40, -20, 0, 20, 40])
    ap.add_argument("--standoff-m", type=float, default=STANDOFF_M)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    with open(a.map) as f:
        doc = json.load(f)
    objs = [o for o in doc["objects"] if o.get("graspable")]
    if not objs:
        print("no graspable object in %s" % a.map)
        return 2
    import grasp_frames as GF
    from env_probe import ProbeNode
    import rclpy
    from geometry_msgs.msg import Quaternion

    rclpy.init()
    node = ProbeNode()
    try:
        for arm in ("left", "right"):
            if not node.wait_ready(arm, 120.0, need_camera=False):
                print("REFUSING: %s not ready: %s"
                      % (arm, ", ".join(node.missing(arm, need_camera=False))))
                return 2

        print("%d graspable object(s) from %s\n" % (len(objs), a.map))
        print("%-26s %-5s %s" % ("object", "arm", "steepest approach that solves"))
        rows = []
        for o in objs:
            c = np.asarray(o["centre"], float)
            arm = "left" if c[0] > 0 else "right"
            pad = float(GF.pad_mid_ee_for(o["width_m"] * 1000.0, arm)[2])
            best = None
            per_elev = {}
            for elev in a.elevations:
                n_ok = 0
                for yaw in a.yaws:
                    ax = axis_for(elev, yaw, arm)
                    q = GF.q_from_axis(ax)
                    m = Quaternion()
                    m.x, m.y, m.z, m.w = (float(v) for v in q)
                    grasp = c - ax * pad
                    pre = grasp - ax * a.standoff_m
                    if node.solve(arm, list(grasp), m, tries=2) is None:
                        continue
                    if node.solve(arm, list(pre), m, tries=2) is None:
                        continue
                    n_ok += 1
                per_elev["%+.0f" % elev] = n_ok
                if n_ok and best is None:
                    best = elev
            rows.append(dict(centre=[float(v) for v in c], arm=arm,
                             width_mm=round(o["width_m"] * 1000, 1),
                             steepest_deg=best, per_elevation=per_elev))
            print("%-26s %-5s %s"
                  % ("[%6.3f %6.3f %6.3f]" % tuple(c), arm,
                     ("%+.0f deg" % best) if best is not None
                     else "NONE of the swept angles solve"))
        print()
        print("solved yaws per elevation (of %d), by object:" % len(a.yaws))
        hdr = "  %-24s " % "object" + " ".join("%5.0f" % e for e in a.elevations)
        print(hdr)
        for r in rows:
            print("  %-24s " % ("[%6.3f %6.3f]" % (r["centre"][0], r["centre"][1]))
                  + " ".join("%5d" % r["per_elevation"]["%+.0f" % e]
                             for e in a.elevations))
        print("\n+90 is straight DOWN onto the object; -31 is the shipped "
              "anchor, which reaches in from below.")
        print("REACHABILITY ONLY. avoid_collisions is not the wearer check -- "
              "hard constraint 11.")
        doc_out = dict(objects=rows, elevations=a.elevations, yaws=a.yaws,
                       standoff_m=a.standoff_m, map=a.map,
                       caveat=("reachability only; the wearer floor is a "
                               "separate gate and is not applied here"))
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        with open(a.out, "w") as f:
            json.dump(doc_out, f, indent=2, sort_keys=True)
        print("-> %s" % a.out)
        return 0
    finally:
        try:
            node.destroy_node(); rclpy.shutdown()
        except Exception:                                      # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
