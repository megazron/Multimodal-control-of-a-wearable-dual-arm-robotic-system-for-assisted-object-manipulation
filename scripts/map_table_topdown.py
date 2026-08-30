#!/usr/bin/env python3
"""FIND THE REAL TABLE WITH THE REAL ARM, LOOKING STRAIGHT DOWN.

    .venv_vision/bin/python scripts/map_table_topdown.py --arm left
    .venv_vision/bin/python scripts/map_table_topdown.py --arm left --execute

WHY THIS EXISTS, and it is not a duplicate of calibrate_environment.

The stored observe pose is a JOINT VECTOR solved on 2026-08-18 against the
SIMULATED T1 layout -- six points at z 1.26-1.27, x 0.23-0.54, y 0.45. Run on
the real rig it points the wrist camera AT THE CEILING, and the detector
correctly reports zero cubes because there are none up there. Measured
2026-08-25: the frame from `observe_pose_left.json` contains ceiling truss, a
lighting rig and cable trays. Nothing downstream of that can work, and nothing
downstream of it SAID anything -- "0 cubes" reads as a detection failure.

So this does not ask the arm to reproduce a remembered pose. It asks, in
CARTESIAN space against the real arm's own IK, for the camera to be at a grid
of points ABOVE the workspace LOOKING STRAIGHT DOWN, and reports what is
actually under each one.

EVERY COMMAND IS CHECKED BY AN INDEPENDENT WITNESS.

Joint states come from the same graph that accepted the command, so "the arm
reports it arrived" and "the arm moved" are not the same claim -- this project
has been caught by that difference repeatedly (the frozen /real/joint_states
that read as "holding"). The SCENE camera watches the rig from across the
room and is on a completely separate path: a different device, a different
driver, no shared timestamp. If the scene frame does not CHANGE between two
poses that are far apart, the arm did not go where it was told, whatever the
joint stream says.

That check is the point of this script. `--execute` is required before
anything moves; without it every pose is solved and reported and no
trajectory is published.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ("scripts", "src/srl_experiments/experiments/abc", "config",
           "src/srl_perception"):
    p = os.path.join(WS, _p)
    if p not in sys.path:
        sys.path.insert(0, p)

OUT = os.path.join(WS, "recordings", "baselines", "table_topdown")


def straight_down(node, arm):
    """Quaternion with the tool axis pointing at the floor.

    The wrist camera looks ALONG the tool axis, so this is what "top view"
    means for this hardware -- not a wrist angle picked by eye.
    """
    return node.solve_axis_quat(np.array([0.0, 0.0, -1.0]))


def scene_changed(a, b, thresh=0.02):
    """Fraction of pixels that changed, and whether it clears `thresh`.

    Deliberately crude. The question is not "how did the picture change" but
    "did anything move at all", and a mean absolute difference answers that
    without needing the scene camera to be calibrated -- which it is not.
    """
    if a is None or b is None:
        return None, False
    x = a.astype(np.int16)
    y = b.astype(np.int16)
    if x.shape != y.shape:
        return None, False
    frac = float((np.abs(x - y).max(axis=2) > 18).mean())
    return frac, frac > thresh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=("left", "right"))
    ap.add_argument("--execute", action="store_true",
                    help="actually command the arm. Without it, poses are "
                         "solved and reported and nothing moves.")
    ap.add_argument("--z", type=float, default=1.45,
                    help="camera height for the top-down grid, metres")
    ap.add_argument("--settle-s", type=float, default=2.5)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    import cv2
    import rclpy
    from env_probe import ProbeNode

    rclpy.init()
    n = ProbeNode()
    if not n.wait_ready(a.arm, 60.0):
        print("REFUSING: %s" % ", ".join(n.missing(a.arm)))
        return 2

    quat = straight_down(n, a.arm)

    # THE GRID IS THE ARM'S OWN REACHABLE COLUMN, not a guess about the table.
    # Left arm works positive x, right arm negative -- neither crosses the
    # centreline (0 of 10 IK solutions at every cross-side target).
    sign = 1.0 if a.arm == "left" else -1.0
    xs = [sign * v for v in (0.30, 0.40, 0.50, 0.60)]
    ys = (0.20, 0.32, 0.44, 0.56)

    print("top-down grid for the %s arm, camera z = %.2f m" % (a.arm, a.z))
    print("%-22s %-8s %-22s %s"
          % ("camera target", "IK", "scene witness", "what is under it"))

    prev_scene = None
    rows = []
    for x in xs:
        for y in ys:
            tgt = (x, y, a.z)
            q = n.solve(a.arm, tgt, quat)
            if q is None:
                print("%-22s %-8s %-22s %s"
                      % ("(%.2f, %.2f, %.2f)" % tgt, "no IK", "-", "-"))
                rows.append(dict(target=tgt, ik=False))
                continue
            if not a.execute:
                print("%-22s %-8s %-22s %s"
                      % ("(%.2f, %.2f, %.2f)" % tgt, "ok", "(dry run)", "-"))
                rows.append(dict(target=tgt, ik=True))
                continue

            n.send(a.arm, q, 6.0)
            end = time.time() + 25.0
            while time.time() < end and not n.at(a.arm, q):
                n.spin(0.2)
            arrived = n.at(a.arm, q)
            n.spin(a.settle_s)

            scene = n.scene_frame()
            frac, moved = scene_changed(prev_scene, scene)
            prev_scene = scene if scene is not None else prev_scene
            witness = ("no scene cam" if scene is None else
                       "%.1f%% changed %s" % (100.0 * (frac or 0.0),
                                              "OK" if moved else "STILL"))

            depth = n.fresh_depth(a.arm, max_age_s=3.0)
            colour = n.color.get(a.arm)
            under = "-"
            if depth is not None:
                d = np.asarray(depth, float)
                fin = d[np.isfinite(d) & (d > 0.05) & (d < 3.0)]
                if fin.size:
                    under = "median %.2f m, %d%% in 0.2-1.0 m" % (
                        float(np.median(fin)),
                        int(100 * ((fin > 0.2) & (fin < 1.0)).mean()))
            tag = "%s_x%+0.2f_y%+0.2f" % (a.arm, x, y)
            if colour is not None:
                img = np.frombuffer(colour.data, np.uint8).reshape(
                    colour.height, colour.width, -1)
                cv2.imwrite(os.path.join(a.out, "wrist_%s.png" % tag),
                            cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            if scene is not None:
                cv2.imwrite(os.path.join(a.out, "scene_%s.png" % tag), scene)

            print("%-22s %-8s %-22s %s"
                  % ("(%.2f, %.2f, %.2f)" % tgt,
                     "ok" if arrived else "NOT THERE", witness, under))
            rows.append(dict(target=tgt, ik=True, arrived=bool(arrived),
                             scene_change=frac, under=under))

    with open(os.path.join(a.out, "grid_%s.json" % a.arm), "w") as f:
        json.dump(rows, f, indent=1)
    print("\nframes and grid written to %s" % a.out)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
