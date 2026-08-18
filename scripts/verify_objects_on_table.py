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


PASS, BLOCK, DRIFT, BADGEOM = "PASS", "BLOCKED", "DRIFT", "BAD_GEOMETRY"
# exit codes, one per state. 0 is reserved for RESTING and nothing else.
CODES = {PASS: 0, BLOCK: 3, DRIFT: 1, BADGEOM: 1}


def verdict_for(gap_m, geometry_ok, documented_gap_m_, tol=TOL_M):
    """(state, exit_code) for one scene. A PURE FUNCTION, so it can be pinned.

    THIS IS THE COMPARISON THAT WAS INVERTED, so it is now a function with
    known answers instead of an expression buried in a reporter. The first
    version required `gap == FLOAT_GAP_M` for success, which meant

        objects floating 120 mm above the table   -> PASS
        objects RESTING on the table (gap 0.000)  -> FAIL

    i.e. it certified the defect and would have rejected the fix. The direction
    is the whole content of this check, so it is tested in both directions --
    see `self_test_verdicts()` -- and RESTING is the only state that returns 0.
    """
    if not geometry_ok:
        return BADGEOM, CODES[BADGEOM]
    if gap_m is None:
        return DRIFT, CODES[DRIFT]
    if abs(gap_m) <= tol:
        return PASS, CODES[PASS]
    if documented_gap_m_ is not None and abs(gap_m - documented_gap_m_) <= tol:
        return BLOCK, CODES[BLOCK]
    return DRIFT, CODES[DRIFT]


def self_test_verdicts(tol=TOL_M):
    """Known answers for `verdict_for`, in BOTH directions."""
    doc = 0.120
    cases = [
        ("objects RESTING on the table -- the only PASS",
         0.0, True, doc, PASS),
        ("resting within tolerance (1 mm)", 0.001, True, doc, PASS),
        ("floating at exactly the documented gap -> BLOCKED, exit 3",
         0.120, True, doc, BLOCK),
        ("floating 5 mm off the documented gap -> DRIFT",
         0.125, True, doc, DRIFT),
        ("floating with no documented gap at all -> DRIFT",
         0.120, True, None, DRIFT),
        ("surface ABOVE the work plane -> DRIFT, not a pass",
         -0.050, True, doc, DRIFT),
        ("resting but an object off the footprint -> BAD_GEOMETRY",
         0.0, False, doc, BADGEOM),
    ]
    bad = 0
    print("  KNOWN-ANSWER SELF-TEST (the verdict direction)")
    for label, gap, geom, doc_, want in cases:
        got, code = verdict_for(gap, geom, doc_, tol)
        okk = got == want and code == CODES[want]
        bad += not okk
        print("    %s  %-62s want %-11s got %-11s exit %d"
              % ("ok " if okk else "BAD", label, want, got, code))
    if CODES[PASS] != 0 or any(CODES[s] == 0 for s in (BLOCK, DRIFT, BADGEOM)):
        print("    BAD  exit 0 must mean RESTING and nothing else")
        bad += 1
    print("    -> %d of %d correct" % (len(cases) - bad, len(cases)))
    return bad == 0


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
def objects_for(task, seed=0):
    """[(name, xyz, size)] for everything in `task` that must rest on a solid.

    Graspable items AND the non-graspable fixtures a viewer reads as lying on
    the surface. A coloured pad that floats is the same defect as a floating
    cube and is more visible, because it is 210 mm wide.
    """
    import clip_tasks as CT
    import msc_clip_tasks as MCT
    import clip_scene as CS
    out = []
    if task == "t1":
        # T1'S OWN GEOMETRY AND ITS OWN SURFACE, since the 2026-08-17 rebuild.
        # Its objects REST on a table at 0.950 rather than floating over the
        # shared work plane at 1.100, and it has no single `T1_ARM` any more --
        # the two pads belong to different arms. The pads are mats lying ON the
        # table, so their own underside is the surface, not the plane above it.
        import t1_task as T1M
        for i, (cx, cy) in enumerate(T1M.T1_CUBES):
            out.append(("cube_%d" % i, [cx, cy, T1M.T1_Z],
                        [T1M.CUBE_M] * 3))
        for i, (px, py) in enumerate(T1M.T1_PLANES):
            out.append(("plane_%d" % i,
                        [px, py, T1M.TABLE_TOP + T1M.PLANE_T / 2.0],
                        [T1M.PAD_W, T1M.PAD_D, T1M.PLANE_T]))
    elif task == "t1s2":
        # STAGE 2'S OWN LAYOUT, AND IT HAD NEVER BEEN CHECKED.
        #
        # This branch read `MCT.T1_CUBES if task == "t1" else MCT.T1_CUBES` --
        # a ternary with the same answer on both sides -- so asking this check
        # about T1S2 measured STAGE 1's four cubes and stage 1's single-arm pad
        # pair. Stage 2 is a random draw over BOTH arms from its own seed and
        # has a pad pair per arm, none of which was ever looked at. The check
        # was crediting one task with another task's geometry, which is the
        # fault I had just finished fixing in `status_table`, in a file I wrote
        # the same day. Two namespaces, one key space, again.
        #
        # The seed matters: the scene node and the task are driven from ONE
        # seed (the sweep passes --seed to both) precisely so the picture and
        # the path cannot disagree, and 0 is what the sweep records.
        tgt = MCT.stage2_targets(seed)
        for arm in ("left", "right"):
            for i, cube in enumerate(tgt["cubes"][arm]):
                out.append(("cube_%s_%d" % (arm, i), list(cube),
                            [MCT.CUBE_M] * 3))
            # THE PADS ARE STAGE 1'S PADS, AT STAGE 1'S HEIGHT. They were
            # this arm's OWN pair, sized from `PLANE_SIZE_BY_ARM` and lying at
            # `BENCH_TOP` -- the geometry stage 2 had before the two stages
            # were joined. Left as it was, the check placed a 210 mm mat
            # 150 mm below the cubes it is supposed to be under.
            import t1_task as T1M
            for i, (px, py) in enumerate(MCT.T1_PLANES_BY_ARM[arm]):
                out.append(("plane_%s_%d" % (arm, i),
                            [px, py, T1M.TABLE_TOP + T1M.PLANE_T / 2.0],
                            [T1M.PAD_W, T1M.PAD_D, T1M.PLANE_T]))
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
                        [bx, by, CS.marking_z(arm, task)],
                        [sx, sy, CS.MARK_T]))
    return out


def check_task(task, tol=TOL_M, seed=0, inject_float_m=0.0):
    import clip_scene as CS
    solids = CS.furniture_boxes(task)
    bg = documented_gap_m()
    plane = None
    try:
        from srl_experiments.work_surface import work_plane
        plane = work_plane()
    except Exception:                                    # pragma: no cover
        pass
    # T1 HAS ITS OWN WORK PLANE AND ITS OWN TABLE, AND IT IS THE SAME SURFACE.
    #
    # Since the 2026-08-17 rebuild T1's objects rest ON the table rather than
    # floating over the shared work plane, so its base plane IS
    # `t1_task.TABLE_TOP` and its documented gap is 0, not
    # `work_surface.FLOAT_GAP_M`. Measuring it against the shared plane would
    # report a 150 mm float for a scene whose objects are touching the wood.
    doc_gap = bg
    if task in ("t1", "t1s2"):
        # BOTH STAGES, SINCE 2026-08-18. Stage 2 shared the floating work
        # plane until its paths were walked on stage 1's geometry; they have
        # been, eight seeds at N=10, so it stands its objects on the same
        # table and its documented gap is 0 like stage 1's.
        import t1_task as _T1
        plane = _T1.TABLE_TOP
        doc_gap = 0.0
    bg = doc_gap
    # THE NEGATIVE CONTROL, ON THE REAL SCENE.
    #
    # A check whose only evidence is that it reports BLOCKED today has proved
    # nothing about whether it can report anything else. This displaces every
    # object -- and the marking with them, since they are all registered to the
    # same plane -- by a known amount, so the verdict can be watched changing:
    #
    #     inject 0      -> BLOCKED at the documented 120.0 mm (exit 3)
    #     inject +5 mm  -> DRIFT, 125.0 mm is not the documented gap (exit 1)
    #     inject -120 mm-> PASS, the objects now rest on the table (exit 0)
    #
    # This is the same instrument answering three ways about one scene, which is
    # what "not passing vacuously" means. It is a diagnostic switch, never used
    # by a caller that gates on the result.
    # The displacement moves the WORK PLANE with the objects, because that is
    # what a scene whose work plane is elsewhere looks like -- shifting the
    # objects off their own plane would only test the registration check, and
    # the thing under test here is the GAP.
    if plane is not None:
        plane = round(plane + inject_float_m, 6)

    def _shift(o):
        if not inject_float_m:
            return o
        n, xyz, size = o[0], list(o[1]), o[2]
        xyz[2] += inject_float_m
        return (n, xyz, size)
    objs = [check_object(_shift(o), solids, tol, bg, plane)
            for o in objects_for(task, seed)]
    # THE ONE GAP, CHECKED ONCE. Every object is registered to the work plane,
    # so the distance from the work plane to the highest surface under it is a
    # single scene fact. It must equal the documented, measured value.
    # ==================================================================
    # RESTING IS THE PASS. THE DOCUMENTED GAP IS A BLOCK, NOT A REQUIREMENT.
    # ==================================================================
    # The first version of this got the comparison backwards, in the one
    # direction that makes a check worse than useless. It required the gap to
    # EQUAL `FLOAT_GAP_M` "in both directions, so moving either height needs a
    # re-measurement" -- with the effect that
    #
    #     objects floating 120 mm above the table   -> PASS
    #     objects RESTING on the table (gap 0.000)  -> FAIL
    #
    # so the check certified the defect it was written to catch and would have
    # rejected the fix. Verified against both scenes rather than argued.
    #
    # Now: `resting` is the pass. A gap equal to the documented one is BLOCKED
    # -- reported loudly, exit 3, never called a pass. Any other gap is drift
    # and fails. Moving the surface still cannot happen silently, because
    # `test_one_work_surface_height.py` pins the constants; that is where a
    # change needs a measurement, not here.
    tops = [s[1][2] + s[2][2] / 2.0 for s in solids]
    gap = None if (plane is None or not tops) else round(plane - max(tops), 4)
    resting = gap is not None and abs(gap) <= tol
    blocked_gap_here = (gap is not None and bg is not None
                        and not resting and abs(gap - bg) <= tol)
    gap_ok = resting or blocked_gap_here
    # THE MARKING IS CHECKED AGAINST THE WORK PLANE, LIKE THE OBJECTS.
    #
    # It used to be checked against the table, on the argument that paint has
    # no excuse for floating. That argument moved the marking DOWN to the table
    # and put the whole task outside its own boundary -- 116.5 mm below the
    # pads. The marking bounds the WORK, so it belongs at the plane the work
    # rests on and is registered to it exactly as a cube is. Its footprint is
    # still required to lie over a real solid, which is the part that stops a
    # boundary being drawn over thin air in plan view.
    tiles = [check_object(_shift(t), solids, tol, bg, plane)
             for t in marking_tiles(task)]
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
                # THREE DISTINCT STATES, so none of them can borrow another's
                # meaning. `resting` is the only one that satisfies T1-1.
                resting=bool(resting),
                blocked_at_documented_gap=bool(blocked_gap_here),
                gap_is_drift=bool(gap is not None and not gap_ok),
                blocked=[o["object"] for o in objs
                         if o["verdict"] == BLOCKED],
                geometry_ok=all(o["verdict"] in (OK, BLOCKED) for o in objs)
                            and not any(t["verdict"] != OK for t in tiles),
                ok=bool(resting
                        and all(o["verdict"] in (OK, BLOCKED) for o in objs)
                        and not any(t["verdict"] != OK for t in tiles)))


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
    ap.add_argument("--inject-float-mm", type=float, default=0.0,
                    help="displace every object by this many mm in z, to prove "
                         "the check can fail (negative control)")
    ap.add_argument("--seed", type=int, default=0,
                    help="t1s2's layout seed; the sweep records seed 0")
    ap.add_argument("--no-self-test", action="store_true")
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    if not a.no_self_test:
        if not self_test_verdicts(a.tol) or not self_test(a.tol):
            print("\nSELF-TEST FAILED -- the check is wrong. Reporting "
                  "nothing about the real scene.")
            return 2
        print()

    r = check_task(a.task, a.tol, a.seed, a.inject_float_mm / 1000.0)
    if a.inject_float_mm:
        print("  !! NEGATIVE CONTROL: every object and the marking displaced "
              "%+.1f mm in z. A correct check must NOT report PASS here "
              "unless the displacement lands them on the table."
              % a.inject_float_mm)
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
    print("  work plane %.4f, highest surface under it %.4f -> gap %s"
          % (r["work_plane_m"], r["work_plane_m"] - (gap or 0.0),
             "%.1f mm" % (gap * 1000.0) if gap is not None else "unknown"))
    state, code = verdict_for(gap, r["geometry_ok"], r["documented_gap_m"],
                              a.tol)
    r["state"] = state
    if state == BADGEOM:
        print("\n  FAIL (BAD GEOMETRY) -- see the verdicts above")
    elif state == PASS:
        print("\n  PASS -- every object RESTS on the surface (gap %.1f mm), "
              "inside its footprint, and the marking is painted on it. "
              "T1-1 satisfied." % ((gap or 0.0) * 1000.0))
    elif state == BLOCK:
        print("\n  BLOCKED -- NOT A PASS. Every object is registered to the "
              "work plane and inside the surface's footprint and the marking "
              "is painted on the surface, but the work plane stands %.1f mm "
              "ABOVE the surface, so the objects DO NOT REST ON THE TABLE."
              % (gap * 1000.0))
        print("  T1-1 is NOT satisfied. The gap equals the documented, "
              "measured value, so this is the known geometric block and not "
              "drift: 0 of 3360 cells at the anchor with objects resting, and "
              "this surface is the highest (top, near edge) pair that costs")
        print("  T1's full 171-waypoint path nothing "
              "(sweep_surface_vs_t1_path.py). Exit %d, not 0 -- a floating "
              "scene must never leave this script with a success code."
              % CODES[BLOCK])
    else:
        print("\n  FAIL (DRIFT) -- the gap is %.1f mm and the documented one "
              "is %s. Either the surface or the objects moved without a "
              "measurement."
              % ((gap or 0.0) * 1000.0,
                 "%.1f mm" % (r["documented_gap_m"] * 1000.0)
                 if r["documented_gap_m"] is not None else "unknown"))
    r["exit_code"] = code
    if a.json:
        json.dump(r, open(a.json, "w"), indent=2)
        print("  wrote %s" % a.json)
    return code


if __name__ == "__main__":
    sys.exit(main())
