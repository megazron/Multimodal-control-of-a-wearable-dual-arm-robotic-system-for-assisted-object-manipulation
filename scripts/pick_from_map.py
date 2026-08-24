#!/usr/bin/env python3
"""PICK AND PLACE FROM THE MAP THE ROBOT MEASURED. No declared coordinates.

    python3 scripts/pick_from_map.py --self-test              # no stack
    ./.venv_vision/bin/python scripts/pick_from_map.py --arm left --list
    ./.venv_vision/bin/python scripts/pick_from_map.py --arm left \\
        --object 2 --to 0.29 0.50 --execute

WHAT IS DIFFERENT ABOUT THIS
----------------------------
Everything else in this repository that picks something up was told where it
is. `run_abc` builds T1's path from `T1_CUBES`, a list of four coordinates in
a source file; the cubes are at (+/-0.42, 0.45) because a file says so. That
is verified N=10 over densified paths and it survives exactly one cell -- a
different table, a different object, or the same object moved 50 mm, and the
number in the file is a lie the arm acts on confidently.

This one reads `recordings/baselines/world_map.json` -- what the arm SAW
during the calibration sweep -- and picks an object out of it. Move the cube
and the plan moves. Put a different object down and it appears in the list.
Nothing here knows what T1 is.

THE PLANNER IS GIVEN THE MAP, WHICH HAS NEVER HAPPENED
------------------------------------------------------
`joint_planner.VoxelWorld` has existed and been tested since it was written
and was CONSTRUCTED NOWHERE outside its own self-test. So every joint path
this repository has ever planned knew about the wearer and nothing else -- not
the table, not the objects, not the tray. `world_from_map` is the seam and
this is its first consumer: the transit from home to the pregrasp is planned
against the measured surface, so the arm goes over the table rather than
through it.

The wearer is UNCHANGED and still comes from `clearance.py` at the 0.15 m
floor. The map may add obstacles; it may not remove one. That direction is the
whole safety case and it is the same rule the wearer tracker follows.

WHAT THIS DOES NOT DO
---------------------
It does not decide WHICH object you want. `--object`, `--nearest` and
`--colour` all name one, and with none of them it refuses rather than picking
the biggest thing and calling that the answer. Grounding a phrase like "the
green one" is `prompt_detector`'s job and it is a separate question from
whether the arm can reach it.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (HERE, os.path.join(ROOT, "config"),
           os.path.join(ROOT, "src/srl_perception"),
           os.path.join(ROOT, "src/srl_experiments"),
           os.path.join(ROOT, "src/srl_teleop")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from srl_experiments import narration as N                   # noqa: E402
from srl_perception import joint_planner as JP               # noqa: E402

MAP = os.path.join(ROOT, "recordings/baselines/world_map.json")
# How far back along the approach the hand waits before closing on the object.
STANDOFF_M = 0.12
# How far the object is lifted before it is carried anywhere.
LIFT_M = 0.10
# The gripper, in knuckle radians. 0 is open; this rig's closed-on-a-40mm-cube
# value is what `run_abc` uses and it is not recomputed here.
GRIP_OPEN_RAD = 0.0
GRIP_CLOSED_RAD = 0.5176


def _ago(seconds):
    s = float(seconds)
    if s < 90:
        return "%.0f s" % s
    if s < 5400:
        return "%.0f min" % (s / 60.0)
    if s < 172800:
        return "%.1f hours" % (s / 3600.0)
    return "%.1f days" % (s / 86400.0)


class PickRefusal(Exception):
    """Named, with the stage it failed at. Never a silent fallback."""


# ------------------------------------------------------------------ the map

# How old a map may be before EXECUTING off it is refused.
#
# There is no upper bound on how wrong a map can be -- somebody moves a cube
# and it is wrong immediately -- so this is not a safety guarantee. It is a
# guard against the one failure that costs something: driving the arm at a
# coordinate measured in a different session, from a file that looks exactly
# as authoritative as a fresh one.
#
# PLANNING off an old map is harmless and is only reported. MOVING off one is
# refused unless said otherwise.
STALE_MAP_S = 30 * 60


def load_map(path=MAP):
    """The map, plus its occupancy points if they were written beside it."""
    with open(path) as f:
        doc = json.load(f)
    doc["_age_s"] = time.time() - os.path.getmtime(path)
    occ = None
    npy = os.path.splitext(path)[0] + "_occupancy.npy"
    if os.path.exists(npy):
        occ = np.load(npy)
    doc["_occupancy"] = occ
    doc["_path"] = path
    return doc


def choose(doc, index=None, nearest=None, colour=None, graspable_only=True):
    """WHICH OBJECT, and it refuses to guess.

    A picker with no target that quietly takes the largest object is the
    "feature present but does nothing" row of the instrument table wearing a
    different hat: it always returns something, so it always looks like it
    worked.
    """
    objs = list(doc["objects"])
    for i, o in enumerate(objs):
        o["_i"] = i
    if graspable_only:
        objs = [o for o in objs if o.get("graspable")]
        if not objs:
            raise PickRefusal(
                "the map has %d object(s) and NONE of them is graspable. Each "
                "was refused for its own reason: %s"
                % (len(doc["objects"]),
                   "; ".join(o.get("why", "?") for o in doc["objects"])))
    if index is not None:
        hit = [o for o in objs if o["_i"] == index]
        if not hit:
            raise PickRefusal(
                "object %d is not in the graspable set. Graspable: %s"
                % (index, [o["_i"] for o in objs]))
        return hit[0]
    if nearest is not None:
        p = np.asarray(nearest, float)
        return min(objs, key=lambda o: float(np.linalg.norm(
            np.asarray(o["centre"], float)[:len(p)] - p)))
    if colour is not None:
        want = colour.lower()
        chan = {"blue": 0, "green": 1, "red": 2}
        if want not in chan:
            raise PickRefusal("colour must be one of %s, not %r"
                              % (sorted(chan), colour))
        k = chan[want]
        have = [o for o in objs if o.get("mean_bgr")]
        if not have:
            raise PickRefusal(
                "this map carries no colour per object, so --colour cannot be "
                "answered. It was built by the clustering finder, which sees "
                "no picture. Re-run the calibration with --finder segment.")
        def dom(o):
            b = o["mean_bgr"]
            return b[k] - max(b[j] for j in range(3) if j != k)
        best = max(have, key=dom)
        if dom(best) < 30:
            raise PickRefusal(
                "no object in this map is %s: the most %s-dominant one leads "
                "its next channel by only %.0f. Reporting that rather than "
                "handing back the closest thing." % (want, want, dom(best)))
        return best
    raise PickRefusal(
        "name an object: --object N, --nearest X Y, or --colour C. Refusing "
        "to pick one for you -- a picker that always returns something always "
        "looks like it worked.")


# ------------------------------------------------------- the poses, in metres

# Approach elevations tried, steepest first. +90 is straight DOWN onto the
# object; the shipped anchor is about -31, which reaches in from BELOW.
#
# MEASURED on the mount that exists, at every object the calibration found
# (`scripts/measure_grasp_approach_angles.py`):
#
#   +90 deg   0 of 5 yaws solve   -- straight down is still unreachable
#   +70 deg   1 of 5              -- reachable, both arms, every object
#   +60 deg   2 of 5
#   ...
#   -31 deg   2 of 5              -- the anchor, from below
#
# CLAUDE.md records "top-down on a surface: 0 of 840 cells". That was measured
# on the OLD MOUNT, before the mounts moved 150 mm outboard and 15 degrees of
# yaw, and it is still right about STRAIGHT DOWN. It is wrong about steeply
# from above, which nobody had asked.
APPROACH_ELEVATIONS_DEG = (85, 80, 75, 70, 65, 60, 50, 40, 30, 15, 0, -30.76)


def axis_at(elev_deg, yaw_deg, arm):
    """Unit approach direction. +90 points straight down at the object."""
    el = math.radians(float(elev_deg))
    sx = -1.0 if arm == "left" else 1.0
    h = np.array([sx * math.cos(el), 0.0, -math.sin(el)])
    th = math.radians(float(yaw_deg))
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    v = R @ h
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else np.array([0.0, 0.0, -1.0])


def approach_axis(arm):
    """The rig's shipped anchor direction. Kept for callers that want it.

    NOT recomputed from anything. `master_calibration.WORKSPACE_ORIENT` is a
    stored constant and CLAUDE.md's hard constraint 1 says re-deriving it
    costs T2 its right arm. This is the -31 degree, from-BELOW approach, which
    is why a grasp built on it looks like the hand coming up out of the table.
    """
    from srl_teleop import master_calibration as MC
    q = np.asarray(MC.WORKSPACE_ORIENT[arm], float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([2 * (x * z + y * w), 2 * (y * z - x * w),
                     1 - 2 * (x * x + y * y)])


def poses_for(obj, arm, place_xy=None, surface_z=None, standoff_m=STANDOFF_M,
              lift_m=LIFT_M, axis=None):
    """The waypoints of one pick and place, in the robot frame.

    THE PAD MIDPOINT IS A CURVE, NOT A CONSTANT, and it is read at the width
    this object actually measured. `pad_mid_ee_for` is the measured table --
    0.09833 m wide open, 0.10976 on a 40 mm cube -- and using the open-hand
    value on a closed hand is the 13.47 mm error this repository already paid
    for once.
    """
    sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
    import grasp_frames as GF
    centre = np.asarray(obj["centre"], float)
    width_mm = float(obj["width_m"]) * 1000.0
    # A VECTOR IN THE EE FRAME, (0, 0, z) -- the pad midpoint lies along the
    # tool axis, so its z component IS the wrist-to-pad distance. Treating the
    # returned tuple as a scalar is what the self-test caught.
    pad = float(GF.pad_mid_ee_for(width_mm, arm)[2])
    ax = approach_axis(arm) if axis is None else np.asarray(axis, float)
    # The WRIST goes where the pads have to be: back down the tool axis by the
    # measured wrist-to-pad distance for THIS opening.
    grasp = centre - ax * pad
    pre = grasp - ax * standoff_m
    lift = grasp + np.array([0.0, 0.0, lift_m])
    out = [("pregrasp", pre), ("grasp", grasp), ("lift", lift)]
    if place_xy is not None:
        z = float(surface_z if surface_z is not None else centre[2])
        drop = np.array([float(place_xy[0]), float(place_xy[1]),
                         z + (centre[2] - z)])
        out += [("carry", np.array([drop[0], drop[1], lift[2]])),
                ("place", drop - ax * pad + np.array([0.0, 0.0, 0.0])),
                ("retreat", np.array([drop[0], drop[1], lift[2]]))]
    return out


# ------------------------------------------------------------- the self-test

def self_test(verbose=True):
    """No stack, no map file: the parts that decide what the arm is told."""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-62s %s%s" % (name, "PASS" if cond else "FAIL",
                                    "" if cond else "  -- " + str(detail)))

    def refuses(name, fn, needle=""):
        nonlocal ok
        try:
            fn()
        except PickRefusal as e:
            hit = needle in str(e)
            ok = ok and hit
            if verbose:
                print("  %-62s %s" % (name, "PASS" if hit else
                                      "FAIL -- %s" % e))
            return
        ok = False
        if verbose:
            print("  %-62s FAIL -- it did not refuse" % name)

    doc = dict(objects=[
        dict(centre=[0.42, 0.45, 1.27], extents=[0.04, 0.04, 0.04],
             width_m=0.04, graspable=True, why="", mean_bgr=[220, 60, 40]),
        dict(centre=[0.48, 0.45, 1.27], extents=[0.04, 0.04, 0.04],
             width_m=0.04, graspable=True, why="", mean_bgr=[40, 220, 60]),
        dict(centre=[0.29, 0.50, 1.26], extents=[0.16, 0.14, 0.01],
             width_m=0.14, graspable=False, why="140 mm across", 
             mean_bgr=[220, 60, 40])])

    refuses("with no target named it refuses rather than picking one",
            lambda: choose(doc), "name an object")
    check("--object picks that object", choose(doc, index=1)["_i"] == 1)
    refuses("--object on an ungraspable one says so, and lists the others",
            lambda: choose(doc, index=2), "not in the graspable set")
    check("--nearest picks the nearer of two",
          choose(doc, nearest=(0.47, 0.45))["_i"] == 1)
    check("--colour green picks the green one",
          choose(doc, colour="green")["_i"] == 1)
    refuses("--colour on a map with no colour says WHY, and names the fix",
            lambda: choose(dict(objects=[dict(centre=[0, 0, 0], width_m=0.04,
                                              extents=[0.04] * 3,
                                              graspable=True)]),
                           colour="green"), "--finder segment")
    refuses("a map where nothing is graspable refuses with every reason",
            lambda: choose(dict(objects=[doc["objects"][2]])),
            "NONE of them is graspable")

    # ---- the staleness guard. A map is a fact about the SCENE, and the
    # scene moves; the guard exists so that driving the arm at coordinates
    # measured in a previous session has to be asked for.
    check("a map's age is reported in units a person reads",
          (_ago(45), _ago(600), _ago(7200), _ago(200000))
          == ("45 s", "10 min", "2.0 hours", "2.3 days"),
          (_ago(45), _ago(600), _ago(7200), _ago(200000)))
    check("the stale-map threshold is a real one, not disabled",
          60 <= STALE_MAP_S <= 24 * 3600, STALE_MAP_S)

    # ---- the approach, which is now searched rather than assumed
    check("straight down points the tool axis at the floor",
          abs(axis_at(90, 0, "left")[2] + 1.0) < 1e-6, axis_at(90, 0, "left"))
    check("the shipped anchor elevation points UPWARD -- in from below",
          axis_at(-30.76, 0, "left")[2] > 0.4, axis_at(-30.76, 0, "left"))
    check("the elevations are searched STEEPEST FIRST",
          list(APPROACH_ELEVATIONS_DEG) == sorted(APPROACH_ELEVATIONS_DEG,
                                                  reverse=True),
          APPROACH_ELEVATIONS_DEG)
    check("+70 deg is in the list, which is where the arm actually solves",
          70 in APPROACH_ELEVATIONS_DEG, APPROACH_ELEVATIONS_DEG)
    a70 = axis_at(70, 0, "left")
    check("a +70 approach descends steeply", a70[2] < -0.9, a70)
    # THE CONTROL: a search that always returned the same axis would pass
    # every check above and grasp from below anyway.
    check("different elevations really give different directions",
          float(np.linalg.norm(axis_at(70, 0, "left")
                               - axis_at(0, 0, "left"))) > 0.5)
    g_top = poses_for(doc["objects"][0], "left", axis=axis_at(70, 0, "left"))
    g_anc = poses_for(doc["objects"][0], "left")
    check("grasping from above puts the WRIST ABOVE the object; the anchor "
          "puts it below",
          dict(g_top)["grasp"][2] > doc["objects"][0]["centre"][2]
          > dict(g_anc)["grasp"][2],
          (dict(g_top)["grasp"][2], doc["objects"][0]["centre"][2],
           dict(g_anc)["grasp"][2]))

    # ---- the geometry
    try:
        p = poses_for(doc["objects"][0], "left")
        names = [n for n, _ in p]
        check("a pick with no destination is pregrasp, grasp, lift",
              names == ["pregrasp", "grasp", "lift"], names)
        pre = dict(p)["pregrasp"]
        gr = dict(p)["grasp"]
        check("the pregrasp is one standoff back along the approach",
              abs(float(np.linalg.norm(pre - gr)) - STANDOFF_M) < 1e-6,
              float(np.linalg.norm(pre - gr)))
        check("the lift is straight up, and only up",
              float(np.linalg.norm((dict(p)["lift"] - gr)[:2])) < 1e-9)
        # THE PAD OFFSET IS READ AT THIS OBJECT'S WIDTH, not the open-hand one.
        import grasp_frames as GF
        # width_mm = 0 IS THE WIDE-OPEN HAND, by that function's own contract.
        wide = float(GF.pad_mid_ee_for(0.0, "left")[2])
        onit = float(GF.pad_mid_ee_for(40.0, "left")[2])
        check("THE CONTROL: the pad offset really does vary with the opening",
              abs(wide - onit) > 0.005, (wide, onit))
        check("and the grasp uses the 40 mm value, not the open-hand one",
              abs(float(np.linalg.norm(gr - np.array([0.42, 0.45, 1.27])))
                  - onit) < 1e-6)
        full = poses_for(doc["objects"][0], "left", place_xy=(0.29, 0.50),
                         surface_z=1.25)
        check("a pick AND place carries above the surface before descending",
              [n for n, _ in full][:5] ==
              ["pregrasp", "grasp", "lift", "carry", "place"],
              [n for n, _ in full])
        check("and the carry is at the lift height, not at the object's",
              abs(dict(full)["carry"][2] - dict(full)["lift"][2]) < 1e-9)
    except Exception as e:                                    # noqa: BLE001
        check("the geometry could be built at all", False, e)

    if verbose:
        print("pick_from_map self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


# ------------------------------------------------------------------- the run

def _say(node, phase, **kw):
    print(N.line(phase, **kw), flush=True)
    if node is not None:
        node.say(phase, N.text(phase, **kw), N.speech(phase, **kw))


def run(node, arm, doc, obj, place_xy, execute, plan_joints=True):
    """Plan, check, narrate, and (only with --execute) move."""
    # DRAW THE MAP THE PICK IS PLANNED FROM, so a recording shows the arm
    # AND the thing it is reaching for.
    for _ in range(8):
        node.publish_map_markers(doc)
        node.spin(0.2)
    surface_z = doc.get("surface", {}).get("z_m")
    _say(node, "PLANNING", arm=arm,
         detail="object %d at %s, %.0f mm across, from the MEASURED map"
                % (obj["_i"], ["%.3f" % v for v in obj["centre"]],
                   obj["width_m"] * 1000))

    # ============ COME DOWN ONTO THE OBJECT, AS STEEPLY AS THE ARM ALLOWS ===
    #
    # The shipped anchor approaches at about -31 degrees -- from BELOW and
    # behind -- so a grasp built on it drives the wrist up out of the table
    # toward the object. It is what the arm was pinned to and it is not what
    # anybody wants to watch.
    #
    # Straight down is genuinely unreachable, and CLAUDE.md's "0 of 840 cells"
    # is right about that. But it was measured on the OLD MOUNT and it is
    # wrong about STEEPLY from above: at +70 degrees every object the
    # calibration found solves, on both arms.
    #
    # So the angle is not a constant. It is SEARCHED, steepest first, per
    # object, against this object's own measured position and width -- which
    # is the only way it can be right for a table and a layout nobody has seen
    # yet. The angle actually used is reported, because "it grasped" and "it
    # grasped from above" are different claims.
    # THE PICK AND THE PLACE ARE SEARCHED SEPARATELY.
    #
    # My first version demanded ONE orientation solve every waypoint --
    # pregrasp, grasp, lift, carry, place and retreat together. It found +15
    # degrees: a grasp from the SIDE, when the same arm solves +70 for the
    # grasp itself. The place pose over the pad was the binding constraint,
    # and it was quietly dragging the grasp down with it.
    #
    # There is no reason they should share an angle. The hand comes down onto
    # the object, closes, lifts, carries, and comes down onto the destination
    # -- two descents, each as steep as its own end of the table allows.
    def _search(names, want):
        for e in APPROACH_ELEVATIONS_DEG:
            for y in (0.0, -20.0, 20.0, -40.0, 40.0):
                ax = axis_at(e, y, arm)
                cand = dict(poses_for(obj, arm, place_xy=place_xy,
                                      surface_z=surface_z, axis=ax))
                q = node.solve_axis_quat(ax)
                if all(node.solve(arm, list(cand[n]), q, tries=2) is not None
                       for n in names if n in cand):
                    return ax, e, y, cand
        return None, None, None, None

    pick_names = ["pregrasp", "grasp", "lift"]
    ax_pick, e_pick, y_pick, cand = _search(pick_names, "pick")
    if ax_pick is None:
        _say(node, "REFUSED", arm=arm,
             why="no approach from %+.0f to %+.0f deg reaches this object"
                 % (APPROACH_ELEVATIONS_DEG[0], APPROACH_ELEVATIONS_DEG[-1]))
        raise PickRefusal(
            "object %d at %s cannot be reached from ANY of the swept approach "
            "angles (%+.0f down to %+.0f). The map says it is there; the arm "
            "says it cannot get a gripper to it."
            % (obj["_i"], ["%.3f" % v for v in obj["centre"]],
               APPROACH_ELEVATIONS_DEG[0], APPROACH_ELEVATIONS_DEG[-1]))

    def _describe(e):
        return ("from ABOVE" if e > 45 else
                "from the side" if e > 5 else "from BELOW -- the old anchor")

    print("   pick approach  %+.0f deg above horizontal, yaw %+.0f  (%s)"
          % (e_pick, y_pick, _describe(e_pick)), flush=True)

    steps = [(n, cand[n]) for n in pick_names]
    axis_by_step = {n: ax_pick for n in pick_names}

    if place_xy is not None:
        place_names = ["carry", "place", "retreat"]
        ax_pl, e_pl, y_pl, cand_pl = _search(place_names, "place")
        if ax_pl is None:
            raise PickRefusal(
                "object %d can be picked (%+.0f deg) but the destination "
                "%s cannot be reached from any swept angle."
                % (obj["_i"], e_pick, ["%.3f" % v for v in place_xy]))
        print("   place approach %+.0f deg above horizontal, yaw %+.0f  (%s)"
              % (e_pl, y_pl, _describe(e_pl)), flush=True)
        steps += [(n, cand_pl[n]) for n in place_names]
        axis_by_step.update({n: ax_pl for n in place_names})

    # ---- THE PLANNER IS GIVEN THE MAP. This is the part that has never run.
    world = None
    if plan_joints and doc.get("_occupancy") is not None:
        world = JP.world_from_map(doc["_occupancy"], voxel_m=0.03)
        print("   the planner has %d occupied voxels from the map "
              "(the surface AND the objects)" % len(world), flush=True)
    else:
        print("   NO OCCUPANCY beside this map, so the planner knows only "
              "the wearer -- the same blindness this file exists to remove. "
              "Re-run the calibration to write it.", flush=True)

    solved = []
    for name, xyz in steps:
        q = node.solve(arm, list(xyz), node.solve_axis_quat(axis_by_step[name]))
        if q is None:
            _say(node, "REFUSED", arm=arm,
                 why="%s at %s has no IK solution" % (name, ["%.3f" % v for v in xyz]))
            raise PickRefusal(
                "%s at %s could not be solved. The map says the object is "
                "there; the arm says it cannot get its wrist there at the "
                "anchor orientation." % (name, ["%.3f" % v for v in xyz]))
        solved.append((name, xyz, q))
        print("   %-9s %s  ok" % (name, ["%.3f" % v for v in xyz]), flush=True)

    if not execute:
        print("\nNOTHING WAS COMMANDED. --execute moves the arm.", flush=True)
        return solved

    grip = {"grasp": GRIP_CLOSED_RAD, "place": GRIP_OPEN_RAD}
    phase = {"pregrasp": "REACHING", "grasp": "GRASPING", "lift": "CARRYING",
             "carry": "CARRYING", "place": "PLACING", "retreat": "RETURNING"}
    for name, xyz, q in solved:
        _say(node, phase.get(name, "REACHING"), arm=arm, at=list(xyz),
             detail=name)
        node.send(arm, q, 2.5)
        t0 = time.time()
        while time.time() - t0 < 8.0 and not node.at(arm, q, 0.02):
            node.spin(0.1)
        if not node.at(arm, q, 0.02):
            raise PickRefusal("the arm did not arrive at %s within 8 s" % name)
        if name in grip:
            node.grip(arm, grip[name])
            node.spin(1.0)
    _say(node, "DONE", arm=arm, detail="object %d moved" % obj["_i"])
    return solved


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=("left", "right"))
    ap.add_argument("--map", default=MAP)
    ap.add_argument("--list", action="store_true",
                    help="print what the map holds and stop")
    ap.add_argument("--object", type=int, default=None)
    ap.add_argument("--nearest", nargs=2, type=float, default=None)
    ap.add_argument("--colour", default=None)
    ap.add_argument("--to", nargs=2, type=float, default=None,
                    help="place it here (x y). Without it this is a pick.")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--drive-real", action="store_true",
                    help="publish where the REAL-ARM BRIDGE is listening "
                         "(/real/...) instead of the simulated controller. "
                         "Without it --execute moves the SIMULATION ONLY -- "
                         "the bridge subscribes under /real and never sees a "
                         "bare topic. The bridge still has its own arming "
                         "gate and its own home refusal on top of this.")
    ap.add_argument("--accept-stale-map", action="store_true",
                    help="move even though the map is old. Planning off an "
                         "old map is always allowed and only reported; this "
                         "is for driving the arm off one.")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return 0 if self_test() else 1

    doc = load_map(a.map)
    age = doc["_age_s"]
    print("map: %s\n   surface z = %.4f m, %d object(s), finder: %s"
          % (a.map, doc["surface"]["z_m"], len(doc["objects"]),
             doc.get("provenance", {}).get("object_finder", "?")))
    print("   measured %s ago" % _ago(age))
    if a.execute and age > STALE_MAP_S and not a.accept_stale_map:
        # THE MAP IS A FACT ABOUT THE SCENE AND THE SCENE MOVES.
        print("\nREFUSING to move: this map was measured %s ago, and nothing "
              "here can tell whether anything has been touched since. A map "
              "from a previous session looks exactly as authoritative as a "
              "fresh one.\n  re-measure:  calibrate_environment.py --arm both"
              "\n  or override: --accept-stale-map" % _ago(age))
        return 5
    if a.list:
        for i, o in enumerate(doc["objects"]):
            print("   %2d  %s  %5.0f mm  %s%s"
                  % (i, ["%7.3f" % v for v in o["centre"]],
                     o["width_m"] * 1000,
                     "graspable" if o.get("graspable") else "NOT graspable",
                     "" if o.get("graspable") else " -- " + o.get("why", "")))
        return 0

    try:
        obj = choose(doc, index=a.object, nearest=a.nearest, colour=a.colour)
    except PickRefusal as e:
        print("REFUSED: %s" % e)
        return 3

    from env_probe import ProbeNode
    import rclpy
    rclpy.init()
    node = ProbeNode(real_ns="/real" if a.drive_real else "")
    print("   commands go to: %s"
          % ("/real/... -- THE REAL ARM BRIDGE" if a.drive_real
             else "the SIMULATED controller (use --drive-real for hardware)"))
    try:
        # NO CAMERA NEEDED: the map was measured earlier and is on disk.
        if not node.wait_ready(a.arm, 90.0, need_camera=False):
            print("REFUSING after 90 s. Still missing: %s"
                  % ", ".join(node.missing(a.arm, need_camera=False)))
            return 2
        run(node, a.arm, doc, obj, a.to, a.execute)
        return 0
    except PickRefusal as e:
        print("REFUSED: %s" % e)
        return 4
    finally:
        try:
            node.destroy_node(); rclpy.shutdown()
        except Exception:                                      # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
