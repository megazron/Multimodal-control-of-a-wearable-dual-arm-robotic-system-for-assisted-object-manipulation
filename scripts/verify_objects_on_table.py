#!/usr/bin/env python3
"""DOES EVERY OBJECT IN THE SCENE ACTUALLY REST ON A SOLID?

    python3 scripts/verify_objects_on_table.py [--task t1] [--self-test]

WHY THIS FILE EXISTS, AND WHY IT DID NOT.

TASK_SPEC T1-1 says the cubes rest on the table. They do not: the scene
carries TWO surface constants 150 mm apart -- `clip_tasks.BENCH_TOP` = 1.100,
which every object, pad and marking tile is positioned against, and
`clip_scene.TABLE_TOP` = 0.950, which is the only surface with geometry. The
raised bench that used to close that gap was deleted as scenery in the
"THE BENCH IS GONE. ONE TABLE." pass, and the objects were left where they
were.

Nothing caught it, because nothing looked. `measure_objects_on_the_table.py`
is a SWEEP -- it prices what would break if the slab came up -- and it needs a
live IK stack, so it cannot gate anything and never ran as a check.
`verify_rviz_clips.py` keys on rendered colour and has no opinion about z.
The floating is even stated as a design fact in a source comment
(`clip_scene.py`, "it is 170 mm below the work plane and nothing rests on
it"), which is how a defect survives a code read: it is documented.

So this is the missing check, and it is deliberately the CHEAPEST kind. It is
pure geometry over the same two functions the scene node draws from --
`clip_scene.furniture_boxes()` for the solids and this module's `objects_for()`
for what has to rest on them -- so it runs offline, in a unit test, with no
stack, no render and no IK. A check that needs a lab day is a check that runs
once.

WHAT IT ASSERTS, PER OBJECT.

  1. SUPPORTED: some solid's top surface is within --tol of the object's base.
     A gap is the floating defect; a negative gap is the object buried in the
     slab, which is just as wrong and is a separate message.
  2. INSIDE THE FOOTPRINT: the object's whole base rectangle lies over that
     solid's top rectangle. Resting on air 40 mm past the edge is not resting.

The workspace marking is checked too and reported separately. A participant is
told to work inside it; tiles drawn over air tell them a surface is there.

CONSTRUCTED GROUND TRUTH, NOT RENDERED. `--self-test` builds five synthetic
scenes whose answers follow from arithmetic -- resting, floating, buried,
overhanging, and supported-but-too-small -- and the check must return exactly
those five verdicts. This is the standing rule's allowed case for synthetic
data: the ground truth is constructed, not rendered. `--self-test` runs
FIRST on every invocation unless --no-self-test, so the instrument is cleared
against known answers before it reports on the real scene.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments"))

# 2 mm. The renderer's own hairlines are 3 mm and a cube is 40, so anything
# this check would call "resting" is under a millimetre of the pixel it draws.
TOL_M = 0.002

OK, FLOAT, BURIED, OVERHANG, UNSUPPORTED = (
    "OK", "FLOATING", "BURIED", "OVERHANGING", "UNSUPPORTED")
# A SIXTH VERDICT, AND IT IS THE ONE THAT KEEPS THIS CHECK USEFUL.
#
# T1-1 is BLOCKED and the block is geometric, not a bug: at the anchor, 0 of
# 3360 cells put an object resting on a surface within reach, and the highest
# slab that costs T1's pick paths nothing is 80 mm below the cubes' base. A
# gate that returns FAIL for that returns FAIL for ever, and a red row nobody
# believes carries no information -- which is the argument in
# `srl_experiments.by_design`, applied here.
#
# So a gap is BLOCKED (a known, measured cost) only when it equals
# `work_surface.FLOAT_GAP_M` to the millimetre. Any other gap is a FAILURE,
# including a SMALLER one: if the gap changes, either the surface moved or the
# objects did, and the numbers behind FLOAT_GAP_M were measured against the
# pair as they are.
BLOCKED = "BLOCKED"


def _top(solid):
    """(top_z, x0, x1, y0, y1) of a (name, xyz, size, colour) box."""
    _, xyz, size = solid[0], solid[1], solid[2]
    return (xyz[2] + size[2] / 2.0,
            xyz[0] - size[0] / 2.0, xyz[0] + size[0] / 2.0,
            xyz[1] - size[1] / 2.0, xyz[1] + size[1] / 2.0)


def documented_gap_m():
    """The one measured gap this repo accepts, or None if the module is gone."""
    try:
        from srl_experiments.work_surface import FLOAT_GAP_M
        return float(FLOAT_GAP_M)
    except Exception:                                    # pragma: no cover
        return None


def check_object(obj, solids, tol=TOL_M, blocked_gap=None, plane_z=None):
    """One object against every solid. Returns a dict, never raises.

    THE SUPPORT IS CHOSEN BY FOOTPRINT FIRST, HEIGHT SECOND, and that order
    matters: picking the nearest-in-z solid and then complaining about its
    footprint reports the table when the object is over the floor. Among the
    solids that actually cover the object's base, the verdict is about the
    closest one in z; if none covers it, the verdict is about the closest
    one that covers ANY of it, so the message can say how far off it is.
    """
    name, xyz, size = obj[0], obj[1], obj[2]
    base = xyz[2] - size[2] / 2.0
    ox0, ox1 = xyz[0] - size[0] / 2.0, xyz[0] + size[0] / 2.0
    oy0, oy1 = xyz[1] - size[1] / 2.0, xyz[1] + size[1] / 2.0

    covering, touching = [], []
    for s in solids:
        tz, sx0, sx1, sy0, sy1 = _top(s)
        # Overlap of the object's base with this solid's top, in area terms.
        ox = max(0.0, min(ox1, sx1) - max(ox0, sx0))
        oy = max(0.0, min(oy1, sy1) - max(oy0, sy0))
        if ox <= 0.0 or oy <= 0.0:
            continue
        frac = (ox * oy) / max(1e-12, (ox1 - ox0) * (oy1 - oy0))
        rec = dict(support=s[0], top_z=round(tz, 4),
                   gap_m=round(base - tz, 4), covered_frac=round(frac, 4),
                   # how far the object hangs past this solid, per edge
                   over_near_m=round(max(0.0, sy0 - oy0), 4),
                   over_far_m=round(max(0.0, oy1 - sy1), 4),
                   over_x_m=round(max(max(0.0, sx0 - ox0),
                                      max(0.0, ox1 - sx1)), 4))
        (covering if frac >= 1.0 - 1e-9 else touching).append(rec)

    pool = covering or touching
    if not pool:
        return dict(object=name, verdict=UNSUPPORTED, base_z=round(base, 4),
                    support=None, gap_m=None, covered_frac=0.0,
                    detail="no solid lies under any part of this object")
    best = min(pool, key=lambda r: abs(r["gap_m"]))
    out = dict(object=name, base_z=round(base, 4), **best)
    # REGISTRATION TO THE WORK PLANE, when one is given.
    #
    # The gap that matters is ONE scene-level number -- work plane minus
    # surface -- not a per-object one, and the first version of this got that
    # wrong: it compared every object's base to the surface and called the
    # coloured pads DRIFT for being 76 mm up instead of 80. They are 4 mm
    # thick and are drawn with their TOP flush to the work plane, on purpose,
    # so that a cube standing on a pad is at the same height as a cube
    # standing on the plane. That is registration, not drift.
    #
    # So per object the question is only: is it registered to the work plane,
    # base-flush (a cube) or top-flush (a mat)? The gap to the drawn surface
    # is then the same for everything and is checked once, by check_task.
    if plane_z is not None:
        top_o = xyz[2] + size[2] / 2.0
        if abs(base - plane_z) <= tol:
            out["registration"] = "base on the work plane"
        elif abs(top_o - plane_z) <= tol:
            out["registration"] = "top flush with the work plane (a mat)"
        else:
            out["verdict"] = (FLOAT if base > plane_z else BURIED)
            out["detail"] = ("neither its base (%.4f) nor its top (%.4f) is on "
                             "the work plane (%.4f) -- off by %.1f mm"
                             % (base, top_o, plane_z,
                                min(abs(base - plane_z),
                                    abs(top_o - plane_z)) * 1000.0))
            return out
    if abs(best["gap_m"]) > tol and plane_z is None:
        out["verdict"] = FLOAT if best["gap_m"] > 0 else BURIED
        out["detail"] = ("base %.4f is %.1f mm %s the top of %s (%.4f)"
                         % (base, abs(best["gap_m"]) * 1000.0,
                            "above" if best["gap_m"] > 0 else "below",
                            best["support"], best["top_z"]))
        if blocked_gap is not None and best["gap_m"] > 0:
            if abs(best["gap_m"] - blocked_gap) <= tol:
                out["verdict"] = BLOCKED
                out["detail"] = ("%.1f mm above %s -- the DOCUMENTED gap "
                                 "(work_surface.FLOAT_GAP_M). T1-1 is blocked "
                                 "by the pinned wrist, measured."
                                 % (best["gap_m"] * 1000.0, best["support"]))
            else:
                out["detail"] += (" -- the documented gap is %.1f mm, so this "
                                  "is DRIFT, not the known block."
                                  % (blocked_gap * 1000.0))
    elif not covering:
        out["verdict"] = OVERHANG
        out["detail"] = ("at the right height on %s but only %.1f%% of its "
                         "base is over it (near %.1f / far %.1f / x %.1f mm "
                         "past the edge)"
                         % (best["support"], best["covered_frac"] * 100.0,
                            best["over_near_m"] * 1000.0,
                            best["over_far_m"] * 1000.0,
                            best["over_x_m"] * 1000.0))
    else:
        out["verdict"] = OK
        if plane_z is not None:
            # NOT "rests on the table". It does not, and saying so here is
            # exactly the sentence a source comment used to carry while the
            # objects hung 150 mm up.
            out["detail"] = ("%s; inside %s's footprint; the work plane is "
                             "%.1f mm above it"
                             % (out.get("registration", "on the work plane"),
                                best["support"], best["gap_m"] * 1000.0))
        else:
            out["detail"] = "rests on %s" % best["support"]
    return out


# ---------------------------------------------------------------------------
# WHAT HAS TO REST, PER TASK. Read from the task modules, not written here,
# so a layout change reaches this check instead of sitting beside it.
# ---------------------------------------------------------------------------
def objects_for(task):
    """[(name, xyz, size)] for everything in `task` that must rest on a solid.

    Graspable items AND the non-graspable fixtures a viewer reads as lying on
    the surface. A coloured pad that floats is the same defect as a floating
    cube and is more visible, because it is 210 mm wide.
    """
    import clip_tasks as CT
    import msc_clip_tasks as MCT
    import clip_scene as CS
    out = []
    if task in ("t1", "t1s2"):
        cubes = MCT.T1_CUBES if task == "t1" else MCT.T1_CUBES
        for i, (cx, cy) in enumerate(cubes):
            out.append(("cube_%d" % i, [cx, cy, MCT.T1_Z],
                        [MCT.CUBE_M] * 3))
        arm = MCT.T1_ARM
        pw, pd = CS.PLANE_SIZE_BY_ARM.get(arm, (CS.PLANE_W, CS.PLANE_D))
        for i, (px, py) in enumerate(MCT.T1_PLANES):
            out.append(("plane_%d" % i,
                        [px, py, CT.BENCH_TOP - CS.PLANE_T / 2.0],
                        [pw, pd, CS.PLANE_T]))
    elif task == "t3":
        import task3 as T3
        out.append(("circuit_box", list(T3.BOX_OBJ), list(T3.BOX_SIZE)))
        out.append(("multimeter", list(T3.METER_OBJ), list(T3.METER_SIZE)))
    return out


def marking_tiles(task):
    """[(name, xyz, size)] for the workspace marking, or [] if none is drawn."""
    import clip_tasks as CT
    import clip_scene as CS
    out = []
    for arm in CS.marked_arms(task):
        # THE BARS THE SCENE ACTUALLY DRAWS, not the cells it draws them from.
        # Reading the cells would check a picture the scene no longer paints:
        # it draws the OUTLINE of the cells that are over the surface, on the
        # surface. This has to follow that or it verifies the old renderer.
        cells, _off = CS.cells_on_surface(task, arm)
        for j, (bx, by, sx, sy) in enumerate(
                CS.region_outline(cells, CS.REGION_STEP)):
            out.append(("mark_%s_%d" % (arm, j),
                        [bx, by, CS.TABLE_TOP + CS.MARK_T / 2.0],
                        [sx, sy, CS.MARK_T]))
    return out


def check_task(task, tol=TOL_M):
    import clip_scene as CS
    solids = CS.furniture_boxes(task)
    bg = documented_gap_m()
    plane = None
    try:
        from srl_experiments.work_surface import work_plane
        plane = work_plane()
    except Exception:                                    # pragma: no cover
        pass
    objs = [check_object(o, solids, tol, bg, plane)
            for o in objects_for(task)]
    # THE ONE GAP, CHECKED ONCE. Every object is registered to the work plane,
    # so the distance from the work plane to the highest surface under it is a
    # single scene fact. It must equal the documented, measured value.
    tops = [s[1][2] + s[2][2] / 2.0 for s in solids]
    gap = None if (plane is None or not tops) else round(plane - max(tops), 4)
    gap_ok = (gap is not None and bg is not None and abs(gap - bg) <= tol)
    # THE MARKING GETS NO BLOCKED ALLOWANCE. It is paint: there is no geometry
    # stopping it being drawn on the surface it describes, so a floating tile
    # is a defect with no excuse. This is the whole point of separating the two
    # -- the objects have a measured reason and the marking never did.
    tiles = [check_object(t, solids, tol) for t in marking_tiles(task)]
    return dict(task=task, tol_m=tol, documented_gap_m=bg,
                solids=[dict(name=s[0], top_z=round(_top(s)[0], 4),
                             x=[round(_top(s)[1], 4), round(_top(s)[2], 4)],
                             y=[round(_top(s)[3], 4), round(_top(s)[4], 4)])
                        for s in solids],
                objects=objs,
                marking=dict(
                    tiles=len(tiles),
                    bad=sum(1 for t in tiles if t["verdict"] != OK),
                    verdicts={v: sum(1 for t in tiles if t["verdict"] == v)
                              for v in (OK, FLOAT, BURIED, OVERHANG,
                                        UNSUPPORTED)}),
                work_plane_m=plane,
                work_plane_to_surface_m=gap,
                work_plane_gap_matches_documented=gap_ok,
                blocked=[o["object"] for o in objs
                         if o["verdict"] == BLOCKED],
                ok=all(o["verdict"] in (OK, BLOCKED) for o in objs)
                   and not any(t["verdict"] != OK for t in tiles)
                   and gap_ok)


# ---------------------------------------------------------------------------
# KNOWN ANSWERS. Arithmetic, not a render.
# ---------------------------------------------------------------------------
def self_test(tol=TOL_M):
    """Five constructed cases with answers that follow from the numbers.

    A slab 1.00 m square, top at exactly 1.000, and a 40 mm cube placed
    against it five ways. Each expected verdict is arithmetic: base - top is
    0, +0.150, -0.010; the footprint either fits or does not.
    """
    slab = ("slab", [0.0, 0.0, 0.9825], [1.0, 1.0, 0.035], None)
    small = ("small", [0.0, 0.0, 0.9825], [0.02, 0.02, 0.035], None)
    cases = [
        ("resting   base 0.980 on top 1.000... no, exactly 1.000",
         ("c", [0.0, 0.0, 1.020], [0.04] * 3), [slab], OK),
        ("floating  150 mm of air, the shipped T1 defect",
         ("c", [0.0, 0.0, 1.170], [0.04] * 3), [slab], FLOAT),
        ("buried    10 mm into the slab",
         ("c", [0.0, 0.0, 1.010], [0.04] * 3), [slab], BURIED),
        ("overhang  half its base past the near edge",
         ("c", [0.0, -0.50, 1.020], [0.04] * 3), [slab], OVERHANG),
        ("too small support, right height, 25%% of the base covered",
         ("c", [0.0, 0.0, 1.020], [0.04] * 3), [small], OVERHANG),
        ("no solid under it at all",
         ("c", [5.0, 5.0, 1.020], [0.04] * 3), [slab], UNSUPPORTED),
        # THE BLOCKED ALLOWANCE, AND THE PROOF IT IS NOT A BLANKET ONE. With
        # the documented gap set to exactly 80 mm, an object 80 mm up is
        # BLOCKED and an object 85 mm up is still FLOATING. A tolerance that
        # accepts anything above the surface would be the "so loose it binds
        # nothing" row of CLAUDE.md's table.
        ("blocked   exactly the documented 80 mm gap",
         ("c", [0.0, 0.0, 1.100], [0.04] * 3), [slab], BLOCKED, 0.080),
        ("drift     85 mm, 5 mm off the documented gap -- must NOT be excused",
         ("c", [0.0, 0.0, 1.105], [0.04] * 3), [slab], FLOAT, 0.080),
        ("drift     70 mm, SMALLER than documented -- still not excused",
         ("c", [0.0, 0.0, 1.090], [0.04] * 3), [slab], FLOAT, 0.080),
    ]
    bad = 0
    print("  KNOWN-ANSWER SELF-TEST (constructed geometry)")
    for case in cases:
        label, obj, solids, want = case[:4]
        bg = case[4] if len(case) > 4 else None
        got = check_object(obj, solids, tol, bg)["verdict"]
        mark = "ok " if got == want else "BAD"
        if got != want:
            bad += 1
        print("    %s  %-58s want %-11s got %s"
              % (mark, label, want, got))
    print("    -> %d of %d correct" % (len(cases) - bad, len(cases)))
    return bad == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="t1",
                    choices=["t1", "t1s2", "t3"])
    ap.add_argument("--tol", type=float, default=TOL_M)
    ap.add_argument("--no-self-test", action="store_true")
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    if not a.no_self_test:
        if not self_test(a.tol):
            print("\nSELF-TEST FAILED -- the check is wrong. Reporting "
                  "nothing about the real scene.")
            return 2
        print()

    r = check_task(a.task, a.tol)
    print("  %s -- objects that must rest on a solid" % a.task.upper())
    print("  solids in the scene:")
    for s in r["solids"]:
        print("    %-22s top z %.4f  x %.3f..%.3f  y %.3f..%.3f"
              % (s["name"], s["top_z"], s["x"][0], s["x"][1],
                 s["y"][0], s["y"][1]))
    if not r["solids"]:
        print("    (none)")
    print("  documented gap (work_surface.FLOAT_GAP_M): %s"
          % ("none" if r["documented_gap_m"] is None
             else "%.1f mm" % (r["documented_gap_m"] * 1000.0)))
    print("  objects:")
    for o in r["objects"]:
        print("    %-11s %-11s base z %.4f  %s"
              % (o["object"], o["verdict"], o["base_z"], o["detail"]))
    m = r["marking"]
    print("  workspace marking: %d tiles, %d not resting on a solid  %s"
          % (m["tiles"], m["bad"],
             {k: v for k, v in m["verdicts"].items() if v}))
    # THE SUMMARY KEYS ON THE GAP, NOT ON THE PER-OBJECT VERDICTS, because the
    # per-object verdicts went green the moment registration was introduced and
    # the summary happily printed "everything rests on a solid" over an 80 mm
    # gap. That sentence is the defect this file exists to catch; it must not be
    # reachable while the gap is open.
    gap = r["work_plane_to_surface_m"]
    print("  work plane %.4f, highest surface under it %.4f -> gap %s   %s"
          % (r["work_plane_m"], r["work_plane_m"] - (gap or 0.0),
             "%.1f mm" % (gap * 1000.0) if gap is not None else "unknown",
             "matches the documented value"
             if r["work_plane_gap_matches_documented"] else "DOES NOT MATCH"))
    if not r["ok"]:
        print("\n  FAIL -- see the verdicts above")
    elif gap is not None and gap > a.tol:
        print("\n  PASS WITH A KNOWN BLOCK. Everything is registered to the "
              "work plane and inside the surface's footprint, the marking is "
              "painted on the surface, and the work plane still stands %.1f mm "
              "ABOVE it." % (gap * 1000.0))
        print("  T1-1 IS NOT SATISFIED and this check does not claim it is: "
              "the objects do not rest on the table. That gap is blocked by "
              "the pinned wrist and is measured, not assumed --")
        print("  0 of 3360 cells at the anchor, and this surface is at the "
              "highest (top, near edge) pair that costs T1's FULL 171-waypoint "
              "path nothing -- see sweep_surface_vs_t1_path.py.")
    else:
        print("\n  PASS -- everything rests on a solid, inside its footprint")
    if a.json:
        json.dump(r, open(a.json, "w"), indent=2)
        print("  wrote %s" % a.json)
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
