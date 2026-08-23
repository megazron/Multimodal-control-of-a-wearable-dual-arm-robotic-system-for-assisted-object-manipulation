#!/usr/bin/env python3
"""WHICH VIEWING GEOMETRY ACTUALLY SEES THE TABLE, measured on a live stack.

The first geometry that produced a map put the camera 200 mm above the surface
and 400 mm outboard -- a 25 degree grazing angle. The picture is a thin white
wedge with 72% background, which is why the segmenter returned five regions for
a scene with six objects in it.

Scored on two things a viewing pose has to do at once: be REACHABLE, and put
the surface in the frame. Neither alone is enough -- the steepest pose in the
list is unreachable and the most reachable one sees nothing.
"""
import sys, os, math, time
import numpy as np
sys.path[:0] = ["src/srl_perception", "scripts", "src/srl_experiments"]
import rclpy
from env_probe import ProbeNode
from srl_perception import calibration_sweep as CSW

ARM = "left"
SURF = 1.25
BOUNDS = dict(x=(0.30, 0.58), y=(0.30, 0.52))
GEOMS = [
    # (out, in, up)  -> elevation above the surface plane
    (0.40, 0.14, 0.20),   # what ran: 25 deg
    (0.35, 0.12, 0.30),
    (0.30, 0.10, 0.35),
    (0.25, 0.10, 0.40),
    (0.20, 0.08, 0.45),
    (0.15, 0.06, 0.50),
    (0.10, 0.05, 0.55),
]

rclpy.init()
n = ProbeNode()
if not n.wait_ready(ARM, 120.0):
    print("stack not ready"); sys.exit(2)
plan = CSW.plan(BOUNDS["x"][0], BOUNDS["x"][1], BOUNDS["y"][0], BOUNDS["y"][1],
                SURF, step_m=0.11, hover_m=0.12, order="serpentine")
cells = plan["cells"]
print("%d cells\n" % len(cells))
print("%-22s %5s %8s %8s %8s %8s" % ("out/in/up", "elev", "reach", "valid%",
                                     "table%", "objpx%"))
for out_m, in_m, up_m in GEOMS:
    elev = math.degrees(math.atan2(up_m, math.hypot(out_m, in_m)))
    reached, valid, tablef, objf = 0, [], [], []
    for c in cells:
        o = math.copysign(out_m, c[0])
        cam = [round(c[0] + o, 5), round(c[1] - in_m, 5), round(SURF + up_m, 5)]
        q = n.look_at_quat(cam, [c[0], c[1], SURF])
        j = n.solve(ARM, cam, q)
        if j is None:
            continue
        n.send(ARM, j, 1.6)
        t0 = time.time()
        while time.time() - t0 < 5.0 and not n.at(ARM, j, 0.02):
            n.spin(0.1)
        if not n.at(ARM, j, 0.02):
            continue
        n.spin(1.0)
        reached += 1
        d, bgr, K, st = n.fresh_frame(ARM, 1.5)
        if d is None:
            continue
        v = (d > 0.08) & (d < 1.5)
        valid.append(100.0 * v.mean())
        b, g, r = bgr[:, :, 0].astype(int), bgr[:, :, 1].astype(int), bgr[:, :, 2].astype(int)
        lo = np.minimum(np.minimum(b, g), r); hi = np.maximum(np.maximum(b, g), r)
        tablef.append(100.0 * ((lo > 150) & (hi - lo < 25)).mean())
        objf.append(100.0 * ((hi - lo > 60) & (hi > 100)).mean())
    f = lambda a: (sum(a) / len(a)) if a else 0.0
    print("%-22s %5.1f %4d/%-3d %8.1f %8.1f %8.1f"
          % ("%.2f/%.2f/%.2f" % (out_m, in_m, up_m), elev, reached, len(cells),
             f(valid), f(tablef), f(objf)))
n.destroy_node(); rclpy.shutdown()
