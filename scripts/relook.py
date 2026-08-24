#!/usr/bin/env python3
"""RE-MEASURE ONLY WHAT CHANGED. Seconds, not half an hour.

    ./.venv_vision/bin/python scripts/relook.py                 # whole table
    ./.venv_vision/bin/python scripts/relook.py --changed-only  # scene camera
    ./.venv_vision/bin/python scripts/relook.py --region 0.35 0.55 0.40 0.55

WHY
---
The full sweep is 116 views and about half an hour, and it is the right thing
to do ONCE: it measures the support surface, learns which viewpoints the arm
can reach, and learns which few of them see the whole table. None of that
changes while the table stays where it is.

What changes is the objects. Somebody puts a different thing down, or moves
one 200 mm, and the map is wrong -- but the surface is not, the reachability
is not, and 95% of the table is not. Sweeping the volume again to find that
out is why the calibration was unusable.

So this does three things the sweep does not:

  1. it keeps the MEASURED PLANE instead of re-fitting one, because the table
     did not move;
  2. it visits only the OBSERVE SET -- the fewest viewpoints the full sweep
     proved cover the surface -- instead of every cell;
  3. with `--changed-only` it asks the SCENE CAMERA what moved and visits only
     the viewpoints that see those patches.

WHAT THE SCENE CAMERA IS FOR, AND WHAT IT CANNOT DO
---------------------------------------------------
It is a monocular colour camera: `/scene_camera/image_raw` plus intrinsics,
NO DEPTH. It cannot measure an object and this never asks it to. What a fixed
camera does better than anything else here is notice that something is
different, instantly, with no arm motion -- and intersecting its rays with a
plane it has been told about turns a changed patch of image into a patch of
TABLE. That is a place to go and look properly. The wrist cameras do the
measuring, as they always did.

WHAT IS CARRIED FORWARD AND WHAT IS REPLACED
--------------------------------------------
Objects inside the re-measured footprint are REPLACED by what was just seen --
including being deleted, because "it is not there any more" is the answer the
whole exercise exists to get. Objects outside it are carried forward untouched
and marked with the age of the observation they came from, so nothing silently
claims to have been re-measured when it was not.
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
from srl_perception import scene_change as SC                # noqa: E402
from srl_perception import table_scene as TS                 # noqa: E402
from srl_perception import world_model as WM                 # noqa: E402

MAP = os.path.join(ROOT, "recordings/baselines/world_map.json")
REF = os.path.join(ROOT, "recordings/baselines/scene_reference.npz")


class RelookRefusal(Exception):
    """Named, with the stage. Never a silent full sweep instead."""


def _say(node, phase, **kw):
    print(N.line(phase, **kw), flush=True)
    if node is not None:
        node.say(phase, N.text(phase, **kw), N.speech(phase, **kw))


def plane_from(doc):
    """The plane the full sweep measured. NOT re-fitted.

    Re-fitting from three views would be a worse plane than the one 116 views
    produced, and it would move -- so an object's height above the surface
    would change because the surface moved, not because the object did.
    """
    s = doc.get("surface")
    if not s or "normal" not in s:
        raise RelookRefusal(
            "this map carries no measured surface, so there is nothing to "
            "re-look against. Run the full calibration once first.")
    n = np.asarray(s["normal"], float)
    off = float(s["z_m"]) * abs(n[2]) if "offset" not in s else float(s["offset"])
    return TS.Plane(n, off, int(s.get("inliers", 0)), int(s.get("inliers", 0)))


def pick_viewpoints(doc, regions=None, cell_m=WM.FOOTPRINT_M):
    """Which of the learned observe-set viewpoints to visit.

    With no regions: all of them, which is still a handful against a hundred.
    With regions: only those whose footprint overlaps a changed patch.
    """
    obs = doc.get("provenance", {}).get("observe_set") or []
    if not obs:
        raise RelookRefusal(
            "this map has no observe set, so there is no short list of "
            "viewpoints to visit. It was written before the sweep learned "
            "one -- run the full calibration once to record it.")
    if not regions:
        return list(obs), None
    keep = []
    for o in obs:
        cells = {tuple(c) for c in o.get("footprint", [])}
        if not cells:
            keep.append(o)               # unknown footprint: do not skip it
            continue
        if any(SC.covers(r, cells, cell_m) for r in regions):
            keep.append(o)
    return keep, len(obs)


def same_object_radius(width_m, floor_m=0.05, k=1.5):
    """How far a thing may move and still be recognisably the same thing.

    A JUDGEMENT, AND IT HAS TO BE ONE. From a single before-and-after pair
    there is no way to tell "this cube moved 120 mm" from "that cube was taken
    away and a different one put down over there" -- nothing was watched in
    between, and both produce the identical pair of snapshots. Tracking would
    settle it; a re-look does not track.

    So the rule is stated rather than left to fall out of a threshold: an
    object that moved less than about its own size is the same object, and
    beyond that the honest report is that one went and another arrived. For a
    40 mm cube that is 60 mm.

    My first version used a flat 150 mm and duly reported a cube 120 mm away
    as "moved", which reads as a fact and is a guess.
    """
    return max(float(floor_m), float(k) * float(width_m))


def diff(old, new, moved_m=0.02):
    """What appeared, what went, what moved. In millimetres a person reads.

    Association is by `same_object_radius`, which is a stated judgement -- see
    there. Anything beyond it is reported as one object gone and another
    appeared, which is what a pair of snapshots can actually support.
    """
    used = set()
    appeared, moved, stayed = [], [], []
    for n in new:
        best, bd = None, 1e9
        for i, o in enumerate(old):
            if i in used:
                continue
            d = math.dist(o["centre"], n["centre"])
            if d < bd:
                best, bd = i, d
        radius = same_object_radius(
            max(float(n.get("width_m", 0.04)),
                float(old[best].get("width_m", 0.04)) if best is not None
                else 0.04))
        if best is not None and bd <= radius:
            used.add(best)
            (moved if bd > moved_m else stayed).append(
                dict(centre=n["centre"], moved_mm=round(bd * 1000, 1),
                     from_centre=old[best]["centre"]))
        else:
            appeared.append(dict(centre=n["centre"],
                                 width_mm=round(n["width_m"] * 1000, 1)))
    gone = [dict(centre=old[i]["centre"],
                 width_mm=round(old[i]["width_m"] * 1000, 1))
            for i in range(len(old)) if i not in used]
    return dict(appeared=appeared, gone=gone, moved=moved, unchanged=stayed)


def inside(centre, cells, cell_m=WM.FOOTPRINT_M):
    """Was this point inside the patch of table we just re-measured?"""
    i = int(math.floor(float(centre[0]) / cell_m))
    j = int(math.floor(float(centre[1]) / cell_m))
    return (i, j) in cells


def run(node, doc, viewpoints, plane, finder="segment", require_still=True):
    """Visit the chosen viewpoints, measure, and return (objects, footprint)."""
    sys.path.insert(0, HERE)
    from calibrate_environment import Calibrator
    views = []
    seen_cells = set()
    for k, vp in enumerate(viewpoints, 1):
        v = vp["viewpoint"]
        arm = v["arm"]
        cal = Calibrator(arm, node)
        cal.viewpoint = v
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x, q.y, q.z, q.w = (float(t) for t in v["quat"])
        _say(node, "REACHING", arm=arm, cell=k, of=len(viewpoints),
             at=v["cam"])
        if not cal.look_from(v["cam"], q):
            print("      not reached -- skipping", flush=True)
            continue
        if require_still:
            still, worst = node.is_still(arm)
            if not still:
                print("      arm still moving (%.4f rad) -- skipping" % worst,
                      flush=True)
                continue
        view = cal.capture("relook_%s_%02d" % (arm, k), finder=finder)
        if view is None:
            continue
        views.append(view)
        seen_cells |= WM.footprint(view.points, plane)
        print("      %d points, %d region(s)"
              % (view.n, len(view.objects or [])), flush=True)
    if not views:
        raise RelookRefusal(
            "not one of the %d viewpoint(s) produced a usable view. The map "
            "is unchanged rather than emptied." % len(viewpoints))

    # THE MEASURED PLANE, NOT A NEW ONE. Three views would fit a worse plane
    # than 116 did, and an object's height would then change because the
    # SURFACE moved.
    # THE EVIDENCE RULE SCALES WITH HOW MANY VIEWS WERE TAKEN. A re-look
    # visits three viewpoints on purpose; demanding two sightings there
    # emptied the map on the first run. See `world_model.views_needed`.
    objs, weak = WM._split_weak(WM._fuse_segments(views, plane), len(views))
    return objs, seen_cells, views


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=MAP)
    ap.add_argument("--out", default=None, help="default: overwrite --map")
    ap.add_argument("--changed-only", action="store_true",
                    help="ask the scene camera what moved and visit only the "
                         "viewpoints that see it")
    ap.add_argument("--region", nargs=4, type=float, default=None,
                    metavar=("X0", "X1", "Y0", "Y1"),
                    help="re-measure this patch of table and nothing else")
    ap.add_argument("--reference", default=REF,
                    help="the scene camera's reference frame, for --changed-only")
    ap.add_argument("--save-reference", action="store_true",
                    help="store the current scene frame as the reference and "
                         "stop. Do this when the table is as you want it.")
    ap.add_argument("--allow-moving-capture", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return 0 if self_test() else 1

    with open(a.map) as f:
        doc = json.load(f)
    plane = plane_from(doc)
    t0 = time.time()

    from env_probe import ProbeNode
    import rclpy
    rclpy.init()
    node = ProbeNode()
    try:
        arms = sorted({o["viewpoint"]["arm"]
                       for o in (doc.get("provenance", {}).get("observe_set")
                                 or []) if o.get("viewpoint")}) or ["left"]
        for arm in arms:
            if not node.wait_ready(arm, 120.0):
                print("REFUSING after 120 s. Still missing: %s"
                      % ", ".join(node.missing(arm)))
                return 2

        if a.save_reference:
            bgr, K, pose = node.scene_frame()
            if bgr is None:
                print("REFUSING: %s" % K)
                return 3
            np.savez(a.reference, bgr=bgr, K=np.asarray(K, float),
                     pose=np.asarray(pose, float), t=time.time())
            print("scene reference stored -> %s" % a.reference)
            return 0

        regions = None
        if a.region:
            regions = [dict(x=[a.region[0], a.region[1]],
                            y=[a.region[2], a.region[3]], area_px=0)]
            print("re-measuring the named region only: x %.3f..%.3f  "
                  "y %.3f..%.3f" % tuple(a.region))
        elif a.changed_only:
            if not os.path.exists(a.reference):
                print("REFUSING: no scene reference at %s. Store one with "
                      "--save-reference when the table is as you want it; "
                      "without it there is nothing to compare against and "
                      "'what changed' has no meaning." % a.reference)
                return 3
            ref = np.load(a.reference)
            bgr, K, pose = node.scene_frame()
            if bgr is None:
                print("REFUSING: %s" % K)
                return 3
            try:
                regions = SC.changed_regions(ref["bgr"], bgr, K, pose, plane)
            except SC.ChangeRefusal as e:
                print("REFUSING: the scene camera cannot answer: %s" % e)
                return 3
            age = time.time() - float(ref["t"])
            print("scene camera: %d region(s) changed since the reference "
                  "(%.0f min old)" % (len(regions), age / 60.0))
            for r in regions:
                print("   x %.3f..%.3f  y %.3f..%.3f   %d px"
                      % (r["x"][0], r["x"][1], r["y"][0], r["y"][1],
                         r["area_px"]))
            if not regions:
                print("\nNOTHING CHANGED. The map stands as it is; no arm "
                      "motion was needed to establish that.")
                return 0

        chosen, n_all = pick_viewpoints(doc, regions)
        _say(node, "CALIBRATING", arm="+".join(arms),
             detail="re-looking from %d viewpoint(s)%s, keeping the measured "
                    "surface" % (len(chosen),
                                 " of %d" % n_all if n_all else ""))
        objs, seen, views = run(node, doc, chosen, plane,
                                require_still=not a.allow_moving_capture)

        old = list(doc["objects"])
        kept = [o for o in old if not inside(o["centre"], seen)]
        for o in kept:
            o.setdefault("stale", True)
        new_all = [o.as_dict() for o in objs] + kept
        d = diff([o for o in old if inside(o["centre"], seen)],
                 [o.as_dict() for o in objs])

        doc["objects"] = new_all
        doc["relook"] = dict(
            viewpoints=len(chosen), of_observe_set=n_all,
            views_used=len(views), cells_remeasured=len(seen),
            seconds=round(time.time() - t0, 1),
            carried_forward=len(kept), regions=regions, diff=d,
            plane="the full sweep's, not re-fitted")
        with open(a.out or a.map, "w") as f:
            json.dump(doc, f, indent=2, sort_keys=True)

        print("\n%-10s %s" % ("appeared", d["appeared"] or "none"))
        print("%-10s %s" % ("gone", d["gone"] or "none"))
        print("%-10s %s" % ("moved", d["moved"] or "none"))
        print("%-10s %d object(s)" % ("unchanged", len(d["unchanged"])))
        print("%-10s %d object(s) outside the re-measured patch, marked stale"
              % ("carried", len(kept)))
        print("\n%d viewpoint(s), %.0f s -> %s"
              % (len(chosen), time.time() - t0, a.out or a.map))
        _say(node, "DONE", arm="+".join(arms),
             detail="re-look finished in %.0f s, %d object(s)"
                    % (time.time() - t0, len(new_all)))
        for _ in range(6):
            node.publish_map_markers(doc)
            node.spin(0.2)
        return 0
    except RelookRefusal as e:
        print("REFUSING: %s" % e)
        return 4
    finally:
        try:
            node.destroy_node(); rclpy.shutdown()
        except Exception:                                      # noqa: BLE001
            pass


def self_test(verbose=True):
    """No stack. The parts that decide what is re-measured and what is kept."""
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
        except RelookRefusal as e:
            hit = needle in str(e)
            ok = ok and hit
            if verbose:
                print("  %-62s %s" % (name, "PASS" if hit else "FAIL -- %s" % e))
            return
        ok = False
        if verbose:
            print("  %-62s FAIL -- it did not refuse" % name)

    check("the change detector agrees with itself", SC.self_test(verbose=False))

    refuses("a map with no measured surface is refused",
            lambda: plane_from({}), "no measured surface")
    refuses("a map with no observe set says so and names the fix",
            lambda: pick_viewpoints({"provenance": {}}),
            "run the full calibration once")

    # ---- which viewpoints a change selects
    vp = lambda i, cells: dict(  # noqa: E731
        source="v%d" % i, footprint=cells,
        viewpoint=dict(arm="left", cam=[0.5, 0.3, 1.5],
                       quat=[0, 0, 0, 1], facing_deg=0.0, layer=0,
                       elev_deg=38.9))
    doc = {"provenance": {"observe_set": [
        vp(0, [[8, 9], [8, 10]]),        # x ~0.40-0.45, y ~0.45-0.55
        vp(1, [[2, 3]]),                 # x ~0.10, y ~0.15 -- far away
    ]}}
    all_vps, _ = pick_viewpoints(doc)
    check("with no region, every observe-set viewpoint is visited",
          len(all_vps) == 2)
    region = [dict(x=[0.40, 0.46], y=[0.44, 0.52], area_px=500)]
    some, n_all = pick_viewpoints(doc, region)
    check("a change selects only the viewpoints that SEE it",
          len(some) == 1 and some[0]["source"] == "v0", [s["source"] for s in some])
    check("and it reports how many it skipped", n_all == 2, n_all)
    far = [dict(x=[-0.60, -0.50], y=[0.10, 0.20], area_px=500)]
    check("THE CONTROL: a change nothing sees selects nothing",
          len(pick_viewpoints(doc, far)[0]) == 0)
    unknown = {"provenance": {"observe_set": [
        dict(source="v9", viewpoint=vp(9, [])["viewpoint"])]}}
    check("a viewpoint with no recorded footprint is NOT skipped -- unknown "
          "is not 'sees nothing'",
          len(pick_viewpoints(unknown, far)[0]) == 1)

    # ---- what is kept and what is replaced
    seen = {(8, 9), (8, 10)}
    check("an object inside the re-measured patch is replaced",
          inside([0.42, 0.47, 1.28], seen))
    check("an object outside it is carried forward",
          not inside([-0.42, 0.47, 1.28], seen))

    # ---- the diff
    old = [dict(centre=[0.42, 0.45, 1.28], width_m=0.04),
           dict(centre=[0.48, 0.45, 1.28], width_m=0.04)]
    new = [dict(centre=[0.42, 0.45, 1.28], width_m=0.04),
           dict(centre=[0.60, 0.45, 1.28], width_m=0.04)]
    d = diff(old, new)
    check("a cube that vanished is reported GONE", len(d["gone"]) == 1, d)
    check("a cube that appeared is reported APPEARED", len(d["appeared"]) == 1, d)
    check("a cube that did not move is UNCHANGED", len(d["unchanged"]) == 1, d)
    d2 = diff(old, [dict(centre=[0.42, 0.50, 1.28], width_m=0.04),
                    dict(centre=[0.48, 0.45, 1.28], width_m=0.04)])
    check("a cube that moved 50 mm is reported MOVED with the distance",
          len(d2["moved"]) == 1 and abs(d2["moved"][0]["moved_mm"] - 50) < 1,
          d2["moved"])
    check("a 120 mm displacement is NOT called a move -- a snapshot pair "
          "cannot tell that from a swap",
          len(diff(old, new)["moved"]) == 0, diff(old, new)["moved"])
    check("the same-object radius scales with the object",
          same_object_radius(0.04) == 0.06 and same_object_radius(0.16) == 0.24,
          (same_object_radius(0.04), same_object_radius(0.16)))
    check("THE CONTROL: an identical scene reports no change at all",
          diff(old, [dict(o) for o in old])["appeared"] == []
          and diff(old, [dict(o) for o in old])["gone"] == []
          and diff(old, [dict(o) for o in old])["moved"] == [])

    if verbose:
        print("relook self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    sys.exit(main())
