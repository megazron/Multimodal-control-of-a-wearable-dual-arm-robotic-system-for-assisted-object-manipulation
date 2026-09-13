#!/usr/bin/env python3
"""WHERE DO THE GRIPPER PADS ACTUALLY END UP? MEASURED AGAINST GROUND TRUTH.

    ./.venv_vision/bin/python scripts/measure_pick_accuracy.py

WHAT QUESTION THIS ANSWERS, AND WHY THE EXISTING TABLE DOES NOT
---------------------------------------------------------------
`accuracy_table.py` reports 100% grasp and 0.000 mm positioning error for
every mode. Both numbers are sound and neither answers "is the gripper on the
object", because the arm is scored against THE COORDINATE IT WAS GIVEN. Give
it a coordinate 50 mm from the cube and it will hit that coordinate to 0.000
mm and the table will say 100%.

This scores against WHERE THE OBJECT REALLY IS -- the geometry
`mock_rgbd_camera` was itself built from -- and follows the whole chain:

    object's true centre
      -> the pipeline's belief about it   (measured map, or a declared file)
      -> a grasp pose                     (searched approach, measured pad offset)
      -> IK                               (the arm's own solver)
      -> FK to the FINGER PADS            (mimic chain walked)
      -> where the pads are, in metres

The last step is the one that matters and the one nothing else here does. A
plan can be perfect and the pads still miss, because the pad offset is a curve
and the IK solution is not exact.

TWO PIPELINES, SAME OBJECTS
---------------------------
  MEASURED   the centre the calibration produced
  DECLARED   the coordinate the task file states, which is what every mode
             commands today

The difference between them is what the calibration is worth. Both are scored
against the same truth, so the comparison is not between two beliefs.

ABOUT "ALL MODES"
-----------------
docs/ENGINEERING_LOG.md records, and `run_abc` shows, that THE SAME WAYPOINTS ARE COMMANDED
UNDER EVERY MODE -- the builder takes no mode argument. So the PLANNING error
measured here is one number that applies to all of them. What differs per mode
is EXECUTION: how faithfully the arm follows the waypoints once commanded, and
that is what the recorded clips measure. Reporting a separate planning figure
per mode would be reporting the same number five times with five labels.
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
OUT = os.path.join(ROOT, "recordings/baselines/pick_accuracy.json")
# The capture gate this project scores grasps against everywhere else.
CAPTURE_GATE_MM = 30.0


def truth_objects():
    """Where the objects REALLY are: the renderer's own geometry."""
    import t1_task as T
    out = []
    for i, (x, y) in enumerate(T.T1_CUBES):
        out.append(dict(name="cube_%d" % i, centre=[float(x), float(y),
                                                    float(T.T1_Z)],
                        width_m=float(T.CUBE_M)))
    return out


def pad_midpoint(fk, arm, q, width_m):
    """Where the two finger pads meet, in the world, for this joint solution.

    THE ONLY PART OF THE ROBOT THAT TOUCHES ANYTHING. `srl_fk.poses` walks the
    Robotiq's mimic chain, so the tips move with the commanded opening rather
    than sitting at one arbitrary one -- which matters because the fingers
    swing on a four-bar and the wrist-to-pad distance is a CURVE, 0.09833 m
    wide open against 0.10976 m closed on a 40 mm cube.

    The knuckle angle comes from `gripper_state.grip_for`, the same function
    the runner closes the hand with, so the pads are placed at the opening the
    grasp will actually be made at.
    """
    from srl_teleop import gripper_state as GS
    kn = float(GS.grip_for(width_m * 1000.0)) if width_m > 0 else 0.0
    P = fk.poses(arm, list(q),
                 ["robotiq_85_left_finger_tip_link",
                  "robotiq_85_right_finger_tip_link"], gripper=kn)
    a = np.asarray(P[0][:3, 3], float)
    b = np.asarray(P[1][:3, 3], float)
    return (a + b) / 2.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=MAP)
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--inject-pad-error-mm", type=float, default=0.0,
                    help="THE CONTROL. Displace the wrist-to-pad offset by "
                         "this much and the pad miss must grow by it. A "
                         "measurement that cannot show a pad error is not "
                         "measuring the pads.")
    a = ap.parse_args()

    from srl_fk import FK as OfflineFK, compiled_matches_urdf
    import pick_from_map as PFM
    from env_probe import ProbeNode
    import rclpy

    # THE INSTRUMENT FIRST. `srl_fk` has its own control against the URDF, and
    # every number below is an FK answer -- so a disagreement there is a
    # disagreement with all of them. This is the standing rule and it has been
    # the fault nineteen times.
    if not compiled_matches_urdf(verbose=False):
        print("REFUSING: srl_fk disagrees with the URDF. Every figure here is "
              "an FK answer, so none of them would mean anything.")
        return 3

    truth = truth_objects()
    with open(a.map) as f:
        doc = json.load(f)
    measured = [o for o in doc["objects"] if o.get("graspable")]

    rclpy.init()
    node = ProbeNode()
    fk = OfflineFK()
    rows = []
    try:
        for arm in ("left", "right"):
            if not node.wait_ready(arm, 120.0, need_camera=False):
                print("REFUSING: %s not ready: %s"
                      % (arm, ", ".join(node.missing(arm, need_camera=False))))
                return 2

        # DOES THE MAP DESCRIBE THE SCENE THE CAMERAS ARE SHOWING?
        #
        # It did not, and I nearly reported the result. The map on disk was
        # from a re-look after the table had been REARRANGED, and scoring it
        # against the original layout matched one cube to a different cube
        # 62 mm away and called that a 62 mm accuracy figure. A scorer that
        # reads a fixed path scores whatever is at that path -- the same trap
        # as the stale map, one level up.
        unmatched = [t["name"] for t in truth
                     if _nearest(measured, np.asarray(t["centre"], float),
                                 max_m=0.06) is None]
        if unmatched:
            print("REFUSING: %d of %d objects in the ground truth have no "
                  "counterpart within 60 mm in this map: %s.\n"
                  "This map does not describe the scene being scored -- it is "
                  "probably from a different layout. Re-measure first."
                  % (len(unmatched), len(truth), ", ".join(unmatched)))
            return 4

        print("scoring %d object(s) against the renderer's own geometry\n"
              % len(truth))
        print("%-8s %-9s %10s %10s %10s"
              % ("object", "belief", "belief err", "pad miss", "verdict"))
        print("-" * 54)
        for t in truth:
            c_true = np.asarray(t["centre"], float)
            arm = "left" if c_true[0] > 0 else "right"
            for label, belief_centre, belief_w in (
                    ("MEASURED", _nearest(measured, c_true), None),
                    ("DECLARED", list(c_true), t["width_m"])):
                if belief_centre is None:
                    print("%-8s %-9s %10s %10s %10s"
                          % (t["name"], label, "--", "--", "NOT IN MAP"))
                    continue
                bc = np.asarray(belief_centre[0] if isinstance(
                    belief_centre, tuple) else belief_centre, float)
                bw = belief_w if belief_w is not None else belief_centre[1]
                obj = dict(centre=[float(v) for v in bc], width_m=float(bw),
                           _i=0)
                miss, elev, why = _pad_miss(node, fk, arm, obj, c_true,
                                            a.inject_pad_error_mm)
                belief_err = float(np.linalg.norm(bc - c_true)) * 1000
                ok = miss is not None and miss <= CAPTURE_GATE_MM
                print("%-8s %-9s %10.1f %10s %10s"
                      % (t["name"], label, belief_err,
                         "%.1f" % miss if miss is not None else "--",
                         ("PASS" if ok else "MISS") if miss is not None
                         else why[:10]))
                rows.append(dict(object=t["name"], pipeline=label, arm=arm,
                                 truth=[float(v) for v in c_true],
                                 belief=[float(v) for v in bc],
                                 belief_err_mm=round(belief_err, 2),
                                 pad_miss_mm=None if miss is None
                                 else round(miss, 2),
                                 approach_elev_deg=elev, why=why))
        _summary(rows)
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        with open(a.out, "w") as f:
            json.dump(dict(rows=rows, capture_gate_mm=CAPTURE_GATE_MM,
                           map=a.map,
                           note=("pad miss is the distance from the finger-pad "
                                 "midpoint, by FK on the IK solution, to the "
                                 "object's TRUE centre. The same waypoints are "
                                 "commanded under every mode, so this planning "
                                 "figure is not per-mode; execution is, and "
                                 "the clips measure that.")),
                      f, indent=2, sort_keys=True)
        print("\n-> %s" % a.out)
        return 0
    finally:
        try:
            node.destroy_node(); rclpy.shutdown()
        except Exception:                                      # noqa: BLE001
            pass


def _nearest(objs, c_true, max_m=0.15):
    best, bd = None, 1e9
    for o in objs:
        d = float(np.linalg.norm(np.asarray(o["centre"], float) - c_true))
        if d < bd:
            best, bd = o, d
    if best is None or bd > max_m:
        return None
    return (best["centre"], best["width_m"])


def _pad_miss(node, fk, arm, obj, c_true, inject_mm=0.0):
    """(pad_miss_mm, elevation_deg, why). Plans, solves, and does FK.

    WHAT THIS DOES AND DOES NOT SEPARATE. The grasp pose is built as
    `centre - axis * pad`, and FK on an exact IK solution puts the pads back
    at `centre` -- so with a correct pad offset the pad miss EQUALS the belief
    error, and the perception term dominates. That is the expected result and
    it is worth having: it says the grasp construction adds nothing. If the
    pad offset were wrong -- as it was here by 13.47 mm until it was measured
    from FK -- the pad miss would exceed the belief error by exactly that, and
    `--inject-pad-error-mm` proves this can see it.
    """
    import pick_from_map as PFM
    for e in PFM.APPROACH_ELEVATIONS_DEG:
        for y in (0.0, -20.0, 20.0, -40.0, 40.0):
            ax = PFM.axis_at(e, y, arm)
            steps = dict(PFM.poses_for(obj, arm, axis=ax))
            if inject_mm:
                steps["grasp"] = steps["grasp"] - ax * (inject_mm / 1000.0)
            q = node.solve_axis_quat(ax)
            sol = node.solve(arm, list(steps["grasp"]), q, tries=3)
            if sol is None:
                continue
            pads = pad_midpoint(fk, arm, sol, obj["width_m"])
            return (float(np.linalg.norm(pads - c_true)) * 1000.0,
                    float(e), "")
    return None, None, "no approach solved"


def _summary(rows):
    print()
    for label in ("MEASURED", "DECLARED"):
        got = [r for r in rows if r["pipeline"] == label
               and r["pad_miss_mm"] is not None]
        if not got:
            print("%-9s no object scored" % label)
            continue
        m = [r["pad_miss_mm"] for r in got]
        b = [r["belief_err_mm"] for r in got]
        inside = sum(1 for v in m if v <= CAPTURE_GATE_MM)
        print("%-9s belief %5.1f mm mean | PAD MISS mean %5.1f, worst %5.1f "
              "| %d of %d inside the %.0f mm gate"
              % (label, sum(b) / len(b), sum(m) / len(m), max(m),
                 inside, len(got), CAPTURE_GATE_MM))


if __name__ == "__main__":
    sys.exit(main())
