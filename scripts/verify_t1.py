#!/usr/bin/env python3
"""WALK THE T1 PATH THE TASK ACTUALLY BUILDS, WAYPOINT BY WAYPOINT.

    python3 scripts/verify_t1.py                       # the declared layout
    python3 scripts/verify_t1.py --repeats 10
    python3 scripts/verify_t1.py --self-test           # controls only

WHY THIS IS NOT `solve_t1_layout.py`. That script asks whether each OBJECT can
be reached over its own pick or place path, one object at a time, and it is
what chose the coordinates. This one takes `t1_task.build()` -- the exact list
of poses `run_abc` will publish, both arms, in order, including the transits
between objects and the returns to park -- and solves every one of them.

The difference is where this project has been caught before: a layout whose
six objects all verify can still fail on the CARRY between them, and the carry
is most of the waypoints. T1's own history has an example in the other
direction as well: a cube whose grasp point was fine cost 10 waypoints on its
APPROACH COLUMN, so a check that only tested grasp points passed it.

WHAT IS MEASURED, per waypoint, per arm:

    IK           /compute_ik with avoid_collisions, at the task's own
                 approach orientation, N repeats -- TRAC-IK restarts randomly
                 and one call is one flip
    clearance    the mount guard's geometric wearer model, on EVERY solution
                 rather than the last, because N repeats of one pose are N
                 different postures
    the floor    150 mm, and a breach is reported with the waypoint index and
                 the body part it is close to, not as a single worst number

CONTROLS, imported from `search_t1_centre` so there is one implementation:
a pose inside the torso must read negative, 1.6 m out must not solve, the
shipped floating cube must be reachable and clear, and a cube buried in the
slab must fail where the same cube in free space succeeds.
"""
import argparse
import json
import os
import sys

import rclpy
from geometry_msgs.msg import Quaternion              # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402
from measure_grasp_approach import FK, pad_mid_in_ee, as_msg  # noqa: E402
from srl_fk import FK as OfflineFK                           # noqa: E402
from srl_teleop import gripper_state as _GS                  # noqa: E402
from search_t1_centre import controls                        # noqa: E402
import t1_task as T1                                         # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_paths.json")

# What a grasp has to hit. `clip_scene` scores a grasp against this and the
# whole error budget in `scripts/measure_control_budget.py` is quoted against
# it, so it is the same number here rather than a second one.
CAPTURE_GATE_M = 0.030


def _solve_as_follower(rig, arm, xyz, qm, policy, cone_deg, tries=6,
                       want_tilt=False):
    """One waypoint, through the SAME candidate list `ik_follower_node` uses.

    The commanded orientation is candidate 0 and a tilt is only reached after
    it has failed, so this can never change a waypoint that already solved --
    it can only rescue one that did not. `orientation_policy` owns the order
    and the tilt sizes; this is a caller, not a second implementation.
    """
    if policy == "exact":
        j = rig.n.solve_arm_joints(arm, xyz, qm, avoid=True, tries=tries)
        return (j, 0.0) if want_tilt else j
    import srl_teleop.orientation_policy as _op
    q = [qm.x, qm.y, qm.z, qm.w]
    for cand, tilt, _lab in _op.candidates(q, policy, cone_deg):
        m = Quaternion()
        m.x, m.y, m.z, m.w = (float(v) for v in cand)
        j = rig.n.solve_arm_joints(arm, xyz, m, avoid=True, tries=tries)
        if j is not None:
            return (j, float(tilt)) if want_tilt else j
    return (None, None) if want_tilt else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--mode", default="independent",
                    choices=["independent", "sequential"],
                    help="how each waypoint is seeded; see the "
                         "note in the loop below")
    ap.add_argument("--stage2", type=int, default=None, metavar="SEED",
                    help="walk STAGE 2 at this seed instead of stage 1. Same "
                         "table, same pads, same builder -- the SIDE of each "
                         "cube is drawn, so the split and the columns change "
                         "and the paths have to be walked per seed.")
    # THE POLICY THE FOLLOWER RUNS, NOT A STRICTER ONE.
    #
    # `ik_follower_node` has defaulted to `orientation_policy:=cone`,
    # `orientation_cone_deg:=15.0` in EVERY MODE since 2026-08-22, and this
    # file has always asked /compute_ik for the EXACT commanded orientation.
    # So a waypoint the robot solves by tilting the tool axis two degrees --
    # after every redundancy seed has failed, which is the follower's own
    # order -- was reported here as unreachable.
    #
    # `docs/NEXT_SESSION_2026_08_23.md` asked for exactly this: "run
    # --as-follower ... so the number quoted is the discrete candidate search
    # the robot runs rather than a continuous bound".
    #
    # `exact` is kept and reproduces every earlier number in this file. It is
    # the right question for a GRASP -- at the instant of a grasp the tool
    # axis IS the approach and the roll sets the jaw line -- and the wrong one
    # for transit, which is most of the path. BOTH are reported.
    ap.add_argument("--orientation-policy", default="cone",
                    choices=("cone", "exact", "free"),
                    help="solve through srl_teleop.orientation_policy, as the "
                         "follower does. `exact` reproduces every figure "
                         "recorded before 2026-08-23.")
    ap.add_argument("--orientation-cone-deg", type=float, default=15.0)
    ap.add_argument("--self-test", action="store_true")
    # ONE FILE PER STAGE. `--stage2 N` wrote into the STAGE 1 baseline, so a
    # stage-2 run silently replaced the record `test_t1_layout_is_verified`
    # reads -- and did, on 2026-08-23, turning three green tests red with a
    # result from a different stage. The default now carries the stage.
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    if a.out is None:
        a.out = (OUT if a.stage2 is None
                 else OUT.replace(".json", "_stage2_seed%d.json" % a.stage2))
    pol = a.orientation_policy
    # `exact` with a tolerance is refused by orientation_policy, by design, so
    # the flag combination has to be resolved here rather than passed on.
    cone = 0.0 if pol == "exact" else a.orientation_cone_deg
    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, j = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)." % (arm, w, j))
            return 3
    # THE T1 SCENE, INCLUDING THE TABLE, exactly as clip_scene builds it.
    rig = Rig(n, "t1", 1)
    fk = FK(n)
    pad_mid = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_mid[arm], _ = pad_mid_in_ee(fk, arm, home)

    # OFFLINE FK for the finger tips, so the gripper opening is stated rather
    # than inherited from the simulation. See the note at its use below.
    _off_fk = OfflineFK()
    ctl, good = controls(rig, pad_mid, a.floor)
    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-38s %s" % (k, v))
    if not good:
        print("\nREFUSING: a control failed.")
        return 6
    rig.set_furniture(True)
    if a.self_test:
        print("\nself-test only.")
        return 0

    if a.stage2 is None:
        wp = T1.build()
        layout = [(cx, cy, T1.T1_PAIR[i])
                  for i, (cx, cy) in enumerate(T1.T1_CUBES)]
        print("\nT1 STAGE 1 %s" % ("(both arms, %d cubes, %d pads)"
                                    % (len(T1.T1_CUBES), len(T1.T1_PLANES))))
    else:
        layout = T1.stage2_layout(a.stage2)
        wp = T1.build_stage2(a.stage2)
        sp = T1.stage2_split(a.stage2)
        print("\nT1 STAGE 2, seed %d -- %d cubes left, %d right"
              % (a.stage2, sp["left"], sp["right"]))
        print("   cubes      %s"
              % ", ".join("%+.3f (%s)" % (c[0], T1.PLANE_COLOURS[c[2]])
                          for c in layout))
    print("   approach   elevation %+.1f deg, heading %+.1f deg inboard, "
          "roll %.1f" % (T1.APPROACH_ELEV_DEG, T1.APPROACH_HEAD_DEG,
                         T1.APPROACH_ROLL_DEG))
    print("   table top  %.3f, objects rest at z = %.3f (float gap 0)"
          % (T1.TABLE_TOP, T1.T1_Z))
    oc = T1.offcentre_mm()
    print("   pads       %s   %.0f mm and %.0f mm off the centreline"
          % (T1.T1_PLANES, oc["left"], oc["right"]))
    import srl_teleop.orientation_policy as _opm
    print("   wrist      through the FOLLOWER'S policy: %s"
          % _opm.describe(_opm.normalise(pol, cone)[0], cone))
    print("   waypoints  left %d, right %d, N=%d, floor %.3f m"
          % (len(wp["left"]), len(wp["right"]), a.repeats, a.floor))

    res = {"controls": ctl, "arms": {}, "mode": a.mode}
    bad_total = 0
    for arm in ("left", "right"):
        qm = as_msg(T1.APPROACH[arm])
        fails, breaches, worst, worst_at, who = [], [], 1e9, None, None
        # SEQUENTIAL SEEDING IS WHAT THE FOLLOWER DOES, AND IT IS A DIFFERENT
        # QUESTION FROM THE ONE THIS SCRIPT USED TO ASK.
        #
        # `Solver.solve_joints` seeds every call from the LIVE joint state,
        # which for a stationary sim is HOME -- so each waypoint is solved as
        # if the arm teleported there from home, and a 7-DOF arm has a
        # continuum of elbow configurations for one hand pose. Measured on
        # T1's left arm: the same path came back CLEAN, then 1 waypoint at
        # 0.1489, then 3 at 0.1436, then 7 at 0.0868. The pose is fixed; what
        # moves is the branch.
        #
        # `ik_follower_node` does not work that way. It solves from where the
        # arm IS, one waypoint after the next, so it stays in one branch and
        # actively REJECTS a solution that jumps (its "redundancy flip"
        # check). Modelling that means seeding each solve with the previous
        # solution, which is what `--mode sequential` does by writing the last
        # answer back into the node's joint-state cache -- the same place
        # `solve_joints` reads its seed from.
        #
        # BOTH ARE REPORTED, because they answer different questions and the
        # project needs both. `independent` is the hazard if the solver ever
        # lands in another branch; `sequential` is what the arm would actually
        # track. A layout is only shippable if the sequential run is clean AND
        # the independent worst case is stated.
        seed = None
        for i, w in enumerate(wp[arm]):
            solved = True
            reps = 1 if a.mode == "sequential" else a.repeats
            for _ in range(reps):
                if a.mode == "sequential" and seed is not None:
                    rig.n.js.update(dict(zip(rig.n.names(arm), seed)))
                j = _solve_as_follower(rig, arm, list(w), qm, pol, cone)
                rig.calls += 1
                if j is None:
                    solved = False
                    break
                if a.mode == "sequential":
                    seed = list(j)
                c, k = rig.clearance(arm, j)
                if c is None:
                    solved = False
                    break
                if c < worst:
                    worst, worst_at, who = c, i, k
                if c < a.floor:
                    breaches.append(dict(i=i, xyz=[round(v, 4) for v in w],
                                         clearance=round(c, 4), to=k))
            if not solved:
                fails.append(dict(i=i, xyz=[round(v, 4) for v in w]))
        bad_total += len(fails) + len(breaches)
        res["arms"][arm] = dict(
            waypoints=len(wp[arm]), ik_failures=len(fails),
            floor_breaches=len({b["i"] for b in breaches}),
            worst_clearance=None if worst > 1e8 else round(worst, 4),
            worst_at=worst_at, worst_to=who,
            failures=fails[:40], breaches=breaches[:40])
        print("\n   %-5s %d waypoints: %d IK failures, %d inside the %.0f mm "
              "floor" % (arm, len(wp[arm]), len(fails),
                         len({b["i"] for b in breaches}), a.floor * 1000))
        print("         worst clearance %s m to %s, at waypoint %s"
              % ("----" if worst > 1e8 else "%.4f" % worst, who, worst_at))
        for f in fails[:6]:
            print("         IK FAILED at %d %s" % (f["i"], f["xyz"]))
        for b in breaches[:6]:
            print("         FLOOR  at %d %s  %.4f m to %s"
                  % (b["i"], b["xyz"], b["clearance"], b["to"]))

    # ---------------------------------------------------------- T1-5, in FK
    # "PADS ON THE CUBE, NOT THE WRIST, NOT INSIDE THE TABLE" IS A CLAIM ABOUT
    # THE FINGERS, and until now nothing in this repository has ever checked
    # it. TASK_SPEC section 2 records the consequence: T3's circuit box
    # presents 195 mm to an 85 mm hand and has verified clean for as long as
    # it has existed, because every check asked /compute_ik and none asked
    # what the hand was being asked to close on.
    #
    # Three numbers per cube, all read from FK on the two finger-tip links at
    # the grasp waypoint:
    #
    #   across      the cube's extent along the CLOSING axis. 40 mm is a face,
    #               56.6 mm is a diagonal, and anything over 85 mm cannot be
    #               closed at all
    #   pad miss    how far the pad midpoint is from the cube centre. This is
    #               the quantity the arrival gate compares, and it must be 0
    #               by construction -- if it is not, `ee_for` and the path
    #               disagree
    #   lowest tip  the lower finger tip's height above the TABLE TOP. A
    #               level approach puts the hand at the cube's own height, so
    #               this is the number that says whether the fingers are
    #               reaching through the surface the cube is resting on
    print("\nTHE GRASP ITSELF, from FK on the finger tips")
    from measure_grasp_approach import link_names
    grasps = []
    for i, (cx, cy, _pad) in enumerate(layout):
        arm = T1.arm_for_pad(_pad)
        obj = [cx, cy, T1.T1_Z]
        ee = T1.ee_for(obj, arm)
        # THROUGH THE FOLLOWER'S POLICY, AND THE TILT IS MEASURED.
        #
        # This asked for the EXACT approach, and on 2026-08-23 all four cubes
        # came back GRASP POSE DOES NOT SOLVE while the whole path around them
        # was clean -- because `ik_follower_node` has run a 15 deg cone in
        # every mode since 2026-08-22 and this file had not been told.
        #
        # A tilt at a GRASP is not free, and that is exactly why it is
        # measured rather than assumed away: the FK block below reads the two
        # finger tips at whatever pose came back, so `across_mm`,
        # `pad_miss_mm` and the tip height are the tilted hand's, not the
        # commanded one's. If a tilt moved the pads off the cube, these
        # numbers say so.
        j, tilt = _solve_as_follower(rig, arm, list(ee), as_msg(T1.APPROACH[arm]),
                                     pol, cone, tries=8, want_tilt=True)
        rig.calls += 1
        if j is None:
            print("   cube_%d %-5s GRASP POSE DOES NOT SOLVE" % (i, arm))
            grasps.append(dict(cube=i, arm=arm, solved=False))
            bad_total += 1
            continue
        # THE TIPS AT THE OPENING THE CUBE NEEDS, NOT AT LEFTOVER STATE.
        #
        # This read them through /compute_fk, which uses the SIMULATION'S
        # CURRENT gripper joints -- whatever the last thing to touch the hand
        # left them at. The Robotiq's fingers swing on a four-bar, so the pad
        # midpoint moves 11.43 mm along the tool axis between a wide-open hand
        # and one closed on a 40 mm cube. Measured: the same check on the same
        # geometry read 0.00 mm on 2026-08-18 (open hand) and 13.52 mm on
        # 2026-08-23 (nearly shut). That is leftover state, not the robot.
        #
        # `srl_fk` walks the URDF including the mimic chain, so the opening is
        # STATED here and the answer cannot drift with the simulation.
        p = _off_fk.poses(arm, j,
                          ["end_effector_link",
                           "robotiq_85_left_finger_tip_link",
                           "robotiq_85_right_finger_tip_link"],
                          gripper=_GS.grip_for(int(T1.CUBE_M * 1000)))
        tips = [p[1][:3, 3], p[2][:3, 3]]
        import numpy as _np
        axis = tips[0] - tips[1]
        span = float(_np.linalg.norm(axis))
        axis = axis / max(1e-9, span)
        mid = (tips[0] + tips[1]) / 2.0
        miss = float(_np.linalg.norm(mid - _np.asarray(obj)))
        # A 40 mm cube's extent along an arbitrary unit axis
        across = float(T1.CUBE_M * sum(abs(v) for v in axis))
        low = float(min(t[2] for t in tips)) - T1.TABLE_TOP
        # TWO DIFFERENT CLAIMS, AND THEY NEED TWO DIFFERENT BOUNDS.
        #
        # "pad miss must be 0 BY CONSTRUCTION" is true only at zero tilt: the
        # task subtracts the pad offset to get the wrist, so at the commanded
        # orientation the two compose exactly. That is a 2 mm check on
        # arithmetic and it stays a 2 mm check.
        #
        # When the follower has to TILT -- and here it does, because T1's
        # table refuses the exact approach and the 15 deg cone rescues it at
        # 5 deg -- the pads necessarily leave the cube centre by
        # 2*|pad|*sin(tilt/2), which is 8.58 mm at 5 deg on a 98-110 mm pad
        # offset. That is not an arithmetic error and cannot be made zero
        # without changing the geometry. What it must stay inside is the thing
        # a grasp actually has to hit: the 30 mm capture gate.
        #
        # Loosening the 2 mm bound to cover the tilted case would be a
        # tolerance that binds nothing, which is a row in CLAUDE.md's own
        # table. So the bound is chosen by WHICH CLAIM is being made, the
        # tilt is recorded beside the miss, and a tilt appearing where there
        # was none is visible rather than absorbed.
        gate = 0.002 if tilt <= 1e-9 else CAPTURE_GATE_M
        ok = across <= 0.085 and miss <= gate and low > 0.0
        grasps.append(dict(cube=i, arm=arm, solved=True,
                           tool_axis_tilt_deg=round(tilt, 2),
                           gate_mm=round(gate * 1000, 1),
                           across_mm=round(across * 1000, 1),
                           pad_miss_mm=round(miss * 1000, 2),
                           lowest_tip_above_table_mm=round(low * 1000, 1),
                           finger_span_mm=round(span * 1000, 1),
                           closing_axis=[round(float(v), 4) for v in axis],
                           ok=ok))
        print("   cube_%d %-5s across the closing axis %5.1f mm (hand opens "
              "85)  pad miss %5.2f mm  lowest finger tip %+6.1f mm above the "
              "table  tilt %4.1f deg  %s"
              % (i, arm, across * 1000, miss * 1000, low * 1000, tilt,
                 "" if ok else "<-- PROBLEM"))
        if not ok:
            bad_total += 1
    res["grasps"] = grasps
    res["worst_pad_miss_mm"] = max((g.get("pad_miss_mm", 0.0)
                                    for g in grasps), default=0.0)
    res["worst_tool_axis_tilt_deg"] = max(
        (g.get("tool_axis_tilt_deg", 0.0) for g in grasps), default=0.0)
    res["capture_gate_mm"] = CAPTURE_GATE_M * 1000
    res["path_clean"] = all(
        v["ik_failures"] == 0 and v["floor_breaches"] == 0
        for v in res["arms"].values())
    res["orientation_policy"] = pol
    res["orientation_cone_deg"] = cone

    print("\n" + "=" * 72)
    print("VERDICT: %s" % ("CLEAN -- every waypoint of both arms solves and "
                           "keeps the floor" if bad_total == 0 else
                           "%d waypoint problems, listed above" % bad_total))
    res["clean"] = bad_total == 0
    res["ik_calls"] = rig.calls
    res["stage2_seed"] = a.stage2
    res["layout"] = dict(cubes=[list(c[:2]) for c in layout],
                         pads_for_cubes=[c[2] for c in layout],
                         planes=[list(p) for p in T1.T1_PLANES],
                         pair=dict(T1.T1_PAIR), z=T1.T1_Z,
                         table_top=T1.TABLE_TOP, near_y=T1.TABLE_NEAR_Y,
                         approach=dict(elev=T1.APPROACH_ELEV_DEG,
                                       head=T1.APPROACH_HEAD_DEG,
                                       roll=T1.APPROACH_ROLL_DEG),
                         offcentre_mm=oc)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    print("%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0 if bad_total == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
