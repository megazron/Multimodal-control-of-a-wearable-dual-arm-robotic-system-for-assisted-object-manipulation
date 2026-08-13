#!/usr/bin/env python3
"""CAN T1'S CUBES REST ON THE TABLE? Measured, because the spec requires it.

    python3 scripts/measure_objects_on_the_table.py [--repeats 5]

TASK_SPEC.md T1-1 says the cubes must rest ON the table and not float. Today
they float 150 mm above it, and the repository's answer is that support is
impossible: pedestals score 0 of 4 cubes reachable, cantilever lips 16-22
waypoint failures, footprint pads 26, side posts 65, no support 0. Those are
real measurements and they are not in doubt.

BUT THEY ALL ASKED THE SAME QUESTION: can a support be added UNDER objects at
z = 1.12 with the table top left at 0.95. There is a different question nobody
has put to the solver, which is the one the spec actually asks: RAISE THE
TABLE ITSELF so its top IS the surface the cubes stand on. The objects do not
move. Only the slab does.

There is a note in `clip_scene.furniture_boxes()` that reads like an answer --
"at 20 mm above the table top the band is 0.025 m, i.e. nothing on the table
is reachable at all" -- but it was measured on the LEFT arm, before T1 moved
to the right, and the two arms are not mirror images: at the layout stage the
same six positions scored 2 of 4 cubes on the left and 4 of 4 on the right. A
result from the arm that could not do the task is not a result about the task.

So this sweeps the SLAB TOP from well below the objects up to their base and
scores T1's own six pick paths, densified, at the arm the task runs on.

CONTROLS, and the sweep refuses to report without them:
  * NO SLAB must score 0 -- otherwise a table of failures is just a broken
    loop and the ladder means nothing.
  * A SLAB THROUGH THE OBJECTS' OWN CENTRES must score more than 0 --
    otherwise the scene is not being applied at all and every row is a
    measurement of nothing.
"""

import argparse
import json
import math
import os
import sys
import time as _t

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
from verify_task_scenes import Solver, HOME_TOL_RAD            # noqa: E402
from audit_scenario_reachability import densify                # noqa: E402
import clip_tasks as CT                                        # noqa: E402
import msc_clip_tasks as MCT                                   # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/objects_on_table.json")
STANDOFF, LIFT = 0.10, 0.08
CUBE_M = 0.040


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=5)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the stack up?")
        return 2
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s is %.4f rad from home" % (arm, w))
            return 3
    quat = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    arm = MCT.T1_ARM

    import clip_scene as CS
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service(timeout_sec=15.0)

    def apply_slab(top_z):
        """The table, with its top at top_z. None removes it entirely."""
        CS.remove_furniture(n)
        if top_z is None:
            return
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "table"
        pr = SolidPrimitive()
        pr.type = SolidPrimitive.BOX
        pr.dimensions = [2 * CS.TABLE_HALF_X,
                         CS.TABLE_FAR_Y - CS.TABLE_NEAR_Y,
                         CS.TABLE_THICK]
        co.primitives.append(pr)
        p = Pose()
        p.position.x = 0.0
        p.position.y = (CS.TABLE_NEAR_Y + CS.TABLE_FAR_Y) / 2.0
        p.position.z = top_z - CS.TABLE_THICK / 2.0
        p.orientation.w = 1.0
        co.primitive_poses.append(p)
        co.operation = CollisionObject.ADD
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = [co]
        fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
        end = _t.time() + 20.0
        while _t.time() < end and not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)

    calls = {"n": 0}

    # THE TOP-DOWN WRIST, for the comparison that turns a dead end into a
    # finding. Every teleop mode PINS the wrist at the arm's anchor
    # orientation -- 30.7 deg above horizontal -- so the hand arrives from
    # the near side and from below, through exactly the volume a table top
    # would occupy. A top-down wrist arrives from above, where there is
    # nothing. If the ladder differs between the two, the obstruction is the
    # WRIST and not the table, and that is a platform result rather than a
    # layout failure.
    # A Quaternion MESSAGE, not a tuple. solve() hands whatever it is given
    # straight to rosidl, which aborts the interpreter on a type mismatch
    # rather than raising -- so a tuple here kills the run after the ladder
    # has already printed, which looks like a crash in the sweep.
    from geometry_msgs.msg import Quaternion
    TOPDOWN = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)   # tool +z at the floor

    def ok(p, q=None):
        # N REPEATS OVER THE WHOLE PATH. TRAC-IK restarts randomly, so one
        # call is one flip; a waypoint counts only if it solves every time.
        for _ in range(a.repeats):
            calls["n"] += 1
            if not n.solve(arm, list(p), q or quat[arm], tries=6):
                return False
        return True

    # THE WRIST OFFSET DEPENDS ON THE WRIST ORIENTATION, and the first
    # version of this comparison forgot that. `ee_for()` subtracts
    # PAD_OFFSET, which is the pad vector for the PINNED axis, so reusing it
    # with a top-down quaternion asks for the pinned positions held at a
    # different angle -- which drives the gripper body into the slab and
    # scores 54 of 54 for a reason that has nothing to do with the question.
    # For a top-down hand the pads are straight below the wrist, so the
    # wrist goes straight above the object by the same pad DISTANCE.
    PAD_LEN = math.dist([0, 0, 0], CT.PAD_OFFSET)

    def ee_for_q(obj, q):
        if q is None:
            return CT.ee_for(obj)
        return [round(obj[0], 4), round(obj[1], 4),
                round(obj[2] + PAD_LEN, 4)]

    def score(q=None):
        """Waypoint failures over T1's six pick paths, densified to 30 mm."""
        bad, tested = 0, 0
        for x, y in MCT.T1_CUBES + MCT.T1_PLANES:
            ee = ee_for_q([x, y, MCT.T1_Z], q)
            pre = [ee[0], ee[1], ee[2] + STANDOFF]
            up = [ee[0], ee[1], ee[2] + LIFT]
            for w in densify([pre, ee], 0.03) + densify([ee, up], 0.03):
                tested += 1
                bad += not ok(w, q)
        return bad, tested

    base_z = MCT.T1_Z - CUBE_M / 2.0        # where a resting cube's base sits
    print("OBJECTS ON THE TABLE -- T1's six pick paths, %s arm, N=%d"
          % (arm, a.repeats))
    print("  objects at z = %.3f, so a cube RESTING has the top at %.3f"
          % (MCT.T1_Z, base_z))

    # ---- CONTROLS FIRST. A ladder with no rungs proves nothing. ----------
    apply_slab(None)
    none_bad, tested = score()
    apply_slab(MCT.T1_Z + 0.02)             # a slab THROUGH the objects
    through_bad, _ = score()
    print("  CONTROL no slab           %3d of %d failures  (must be 0)"
          % (none_bad, tested))
    print("  CONTROL slab through them %3d of %d failures  (must be > 0)"
          % (through_bad, tested))
    if none_bad != 0 or through_bad == 0:
        apply_slab(CS.TABLE_TOP)
        print("\nCONTROLS FAILED -- no result. A ladder of numbers from a "
              "loop that is not applying the scene is worse than no ladder.")
        return 1

    rows = {}
    heights = [0.950, 1.000, 1.020, 1.040, 1.050, 1.060, 1.070, 1.080,
               base_z]
    for z in heights:
        apply_slab(z)
        bad, _ = score()
        rows["%.3f" % z] = bad
        mark = "  <- cubes RESTING on it" if abs(z - base_z) < 1e-9 else ""
        print("  slab top %.3f m   %3d of %d failures%s"
              % (z, bad, tested, mark))

    # ---- IS IT THE TABLE, OR IS IT THE WRIST? ------------------------
    # Same slab, same waypoints, same repeats. Only the commanded wrist
    # orientation changes.
    apply_slab(base_z)
    td_bad, _ = score(TOPDOWN)
    apply_slab(None)
    td_free, _ = score(TOPDOWN)
    print("\n  same slab at %.3f, TOP-DOWN wrist   %3d of %d failures"
          % (base_z, td_bad, tested))
    print("  no slab,               TOP-DOWN wrist   %3d of %d failures "
          "(control: the orientation itself must be reachable)"
          % (td_free, tested))

    apply_slab(CS.TABLE_TOP)                # leave the scene as we found it
    CS.remove_furniture(n)

    resting = rows["%.3f" % base_z]
    print("\n%d IK calls" % calls["n"])
    if resting == 0:
        print("RESULT: the cubes CAN rest on the table. Raise TABLE_TOP to "
              "%.3f and T1-1 is satisfied without any support geometry -- "
              "the objects do not move, only the slab does." % base_z)
    else:
        highest = None
        for z in heights:
            if rows["%.3f" % z] == 0:
                highest = z
        print("RESULT: the cubes CANNOT rest on the table. At a top of "
              "%.3f the six pick paths lose %d of %d waypoints. The highest "
              "slab that costs nothing is %s."
              % (base_z, resting, tested,
                 "%.3f m, which is %.0f mm below the cubes' base"
                 % (highest, (base_z - highest) * 1000)
                 if highest is not None else "none of those tried"))
    json.dump(dict(arm=arm, repeats=a.repeats, tested_waypoints=tested,
                   object_z=MCT.T1_Z, resting_base_z=base_z,
                   controls=dict(no_slab=none_bad, slab_through=through_bad,
                                 topdown_no_slab=td_free),
                   topdown_failures_at_resting=td_bad,
                   failures_by_slab_top=rows, ik_calls=calls["n"]),
              open(OUT, "w"), indent=2)
    # THE CONCLUSION IS READ OFF THE COMPARISON, not off the control. The
    # first version tested only `td_free < tested` -- that the top-down
    # orientation is reachable at all -- and then printed "the pinned wrist
    # is what blocks it" while its own numbers said top-down was WORSE. A
    # report that contradicts the table above it is worse than no report.
    if td_free != 0:
        print("NO COMPARISON: top-down loses %d of %d waypoints with NO SLAB "
              "AT ALL, so it is out of reach here for its own reasons and "
              "says nothing about the table." % (td_free, tested))
    elif td_bad < resting:
        print("WHAT BLOCKS IT IS THE PINNED WRIST. At the same slab height "
              "top-down loses %d of %d against the pinned wrist's %d. The "
              "anchor axis sits 30.7 deg above horizontal, so the hand "
              "arrives from the near side and from BELOW, through the volume "
              "the table top occupies." % (td_bad, tested, resting))
    else:
        print("IT IS NOT THE WRIST ORIENTATION. Both wrists reach every "
              "waypoint with no slab (0 of %d each) and both lose most of "
              "them once the slab is at the objects' base: pinned %d, "
              "top-down %d. The obstruction is the SLAB occupying the "
              "approach volume under and around the object, and no "
              "orientation available here approaches from anywhere else."
              % (tested, resting, td_bad))
    print("-> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
