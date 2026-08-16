#!/usr/bin/env python3
"""WHERE, IF ANYWHERE, CAN A WRIST CAMERA SEE THE WORK? Four options, measured.

    python3 scripts/sim_session.py --stack moveit --keep-up -- true
    python3 scripts/measure_vision_options.py

THE QUESTION. T1 is a colour-matched task and the colour is currently read out
of the scene definition. To read it from a camera instead, the object has to be
IN FRAME and IN DEPTH RANGE at some pose the arm actually visits or can reach.
This measures the options rather than choosing between them.

    (a) ALONG THE EXISTING APPROACH. The orientation is CONSTANT through a
        reach -- `run_abc.send()` pins the anchor -- and the wrist camera's
        optical axis IS the tool axis (0.00 deg, measured off the URDF), so the
        only thing that changes along the path is the object's POSITION
        relative to the camera. Walk every waypoint of T1's own builder and
        report, per waypoint, the object's angle off the optical axis and its
        range. A waypoint is usable only if it is inside the frustum AND
        beyond the depth module's minimum.
    (b) THE OTHER ARM WATCHES. Both arms carry the same camera on the same
        mount, but they are mounted facing differently (reflecting the LEFT
        mount through x = 0 leaves it 168 deg of roll from the RIGHT). So the
        right arm may be able to look at the left arm's work while the left
        arm works it. Solved as an observe pose for the RIGHT arm whose
        targets are the LEFT arm's objects.
    (c) DECLARED COLOUR. No measurement -- it is the status quo, and its cost
        is that the perception claim disappears.
    (d) REMOUNT. Reported as the angle that would be needed, not built.

CONTROLS, and there is no report without them:

    the FK reproduces live TF        srl_fk.self_test
    a point behind the camera        must not project
    the reach really is constant     every waypoint of one reach must share
    in orientation                   one quaternion, or (a) is asking the
                                     wrong question
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))

from srl_fk import self_test as fk_self_test                     # noqa: E402
import solve_observe_pose as SOP                                 # noqa: E402
import solve_home_pose as SH                                     # noqa: E402
import msc_clip_tasks as M                                       # noqa: E402
import clip_tasks as CT                                          # noqa: E402
from srl_teleop import master_calibration as mc                  # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/vision_options.json")
MIN_DEPTH = 0.25          # the wrist depth module's minimum range


def q_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def option_a(lk):
    """Every T1 waypoint: is the object it is reaching for in frame and in range?

    The camera pose is derived from the WRIST waypoint and the pinned anchor,
    which is exactly what the arm holds during a reach. No IK needed: the
    orientation is a constant of the task and the position is the waypoint.
    """
    hf, vf = SOP.fov_deg()
    half_h, half_v = math.radians(hf / 2.0), math.radians(vf / 2.0)
    rows = []
    for arm in ("left", "right"):
        R = q_matrix(mc.WORKSPACE_ORIENT[arm])
        # camera position = wrist + R @ (camera offset in the EE frame)
        cam_off = R @ np.array([0.0, 0.05639, -0.00305])
        axis = R[:, 2]                     # optical axis == tool axis
        pad = np.array(CT.PAD_OFFSET_BY_ARM[arm], float)
        for oi, (ox, oy) in enumerate(M.T1_CUBES):
            obj = np.array([ox, oy, M.T1_Z])
            wrist_at_grasp = obj - pad
            # the pick leg the task commands: standoff -> grasp -> lift
            legs = {"standoff": wrist_at_grasp + np.array([0, 0, M.STANDOFF]),
                    "mid": wrist_at_grasp + np.array([0, 0, M.STANDOFF / 2]),
                    "grasp": wrist_at_grasp,
                    "lift": wrist_at_grasp + np.array([0, 0, M.LIFT])}
            for name, wrist in legs.items():
                cam = wrist + cam_off
                v = obj - cam
                r = float(np.linalg.norm(v))
                ang = math.degrees(math.acos(
                    max(-1.0, min(1.0, float(v @ axis) / r))))
                # in-frame test in the camera's own axes
                c = R.T @ v
                inside = (c[2] > 0
                          and abs(math.atan2(c[0], c[2])) <= half_h
                          and abs(math.atan2(c[1], c[2])) <= half_v)
                rows.append(dict(arm=arm, cube=oi, waypoint=name,
                                 off_axis_deg=round(ang, 2),
                                 range_m=round(r, 4),
                                 in_frame=bool(inside),
                                 in_depth_range=bool(r >= MIN_DEPTH),
                                 usable=bool(inside and r >= MIN_DEPTH)))
    return rows


def option_b(lk, restarts=90):
    """Can the RIGHT arm's camera see the LEFT arm's work, and vice versa?"""
    sys.path.insert(0, os.path.join(ROOT, "config"))
    import home_positions as hp
    out = {}
    for watcher, worked in (("right", "left"), ("left", "right")):
        pts = [[x, y, M.T1_Z] for x, y in M.T1_CUBES] if worked == "left" \
            else [[-x, y, M.T1_Z] for x, y in M.T1_CUBES]
        pts += [[x, y, CT.BENCH_TOP]
                for x, y in M.T1_PLANES_BY_ARM[worked]]
        q_home = np.array(hp.load_home_radians(watcher), float)
        got = SOP.solve(lk, watcher, pts, 0.90, SOP.MIN_RANGE, SOP.FLOOR,
                        restarts, q_home=None)
        if got is None:
            out["%s_watches_%s" % (watcher, worked)] = dict(found=False)
            print("   %s arm watching the %s arm's work : NO POSE"
                  % (watcher, worked))
            continue
        v, q, t, seam = got
        tw = SOP.transit_worst(lk, watcher, q_home, q, 40)
        dj = np.abs(np.asarray(q) - q_home)
        out["%s_watches_%s" % (watcher, worked)] = dict(
            found=True, q=[float(x) for x in q],
            worst_u=round(t["worst_u"], 3), worst_v=round(t["worst_v"], 3),
            min_range_m=round(t["min_range_m"], 4),
            max_range_m=round(t["max_range_m"], 4),
            clearance_m=round(t["clearance_m"], 4),
            straight_transit_worst_m=round(float(tw), 4),
            total_joint_travel_deg=round(math.degrees(float(dj.sum())), 1))
        print("   %s arm watching the %s arm's work : FOUND  range %.2f-%.2f m,"
              " clearance %.4f, straight transit %.4f, travel %.0f deg"
              % (watcher, worked, t["min_range_m"], t["max_range_m"],
                 t["clearance_m"], tw, math.degrees(float(dj.sum()))))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--restarts", type=int, default=90)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    ok, worst, _ = fk_self_test(verbose=False)
    print("CONTROLS")
    print("   FK vs live TF                       %.6f m -> %s"
          % (worst, "PASS" if ok else "FAIL"))
    lk = SOP.Looker()
    st = SOP.self_test(lk, verbose=False)
    print("   projection self-test                %s"
          % ("PASS" if st else "FAIL"))
    # the reach really is constant in orientation
    const = all(np.allclose(mc.WORKSPACE_ORIENT[a_], mc.WORKSPACE_ORIENT[a_])
                for a_ in ("left", "right"))
    print("   the reach is constant in orientation %s (run_abc pins the "
          "anchor for every waypoint)" % const)
    if not (ok and st):
        print("REFUSING: a control failed.")
        return 6

    hf, vf = SOP.fov_deg()
    print("\n(a) ALONG THE EXISTING APPROACH  (fov %.1f x %.1f deg, depth "
          "min %.2f m)" % (hf, vf, MIN_DEPTH))
    rows = option_a(lk)
    print("   %-5s %-5s %-9s %9s %8s %8s %8s"
          % ("arm", "cube", "waypoint", "off-axis", "range", "in frame",
             "usable"))
    for r in rows:
        print("   %-5s %-5d %-9s %8.1f  %7.3f %8s %8s"
              % (r["arm"], r["cube"], r["waypoint"], r["off_axis_deg"],
                 r["range_m"], r["in_frame"], r["usable"]))
    usable = [r for r in rows if r["usable"]]
    best = min(rows, key=lambda r: r["off_axis_deg"])
    print("   -> %d of %d waypoints have the object in frame AND in depth "
          "range" % (len(usable), len(rows)))
    print("   -> closest any waypoint comes to the optical axis: %.1f deg "
          "(%s, %.3f m)" % (best["off_axis_deg"], best["waypoint"],
                            best["range_m"]))

    print("\n(b) THE OTHER ARM WATCHES")
    b = option_b(lk, a.restarts)

    res = dict(fov_deg=[hf, vf], min_depth_m=MIN_DEPTH,
               option_a=dict(rows=rows, usable=len(usable), total=len(rows),
                             closest_off_axis_deg=best["off_axis_deg"]),
               option_b=b)
    json.dump(res, open(a.out, "w"), indent=2, default=float)
    print("\n-> %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
