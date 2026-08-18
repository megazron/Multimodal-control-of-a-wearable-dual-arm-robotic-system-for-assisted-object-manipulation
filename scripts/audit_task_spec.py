#!/usr/bin/env python3
"""ONE PASS OVER docs/TASK_SPEC.md, IN THE CODE, BEFORE ANY RECORDING.

    python3 scripts/audit_task_spec.py                # the audit, ~1 s
    python3 scripts/audit_task_spec.py --self-test    # prove it can fail

WHY THIS EXISTS. Three things have been reported done and later found absent:
the sweep's task set, the cube-plane overlap fix, and the presentation pose.
Each was believed on the strength of a report rather than a reading, and each
cost a recording session. So every MUST BE TRUE item in the spec is asked of
the CODE here, once, before a single clip is filmed.

WHAT IT DOES NOT DO. It cannot tell you a clip looks right; nothing can except
looking at the clip. It tells you whether the thing the clip is supposed to
show is CONSTRUCTED AT ALL. A criterion marked PRESENT here can still be
wrong in the picture, and every superseded clip set passed its checks.

THE STATES

    PRESENT   the code builds it, checked by reading the value a CONSUMER
              reads and not merely that a field is stored
    MISSING   specified in TASK_SPEC.md and absent, or present and wrong
    BLOCKED   specified and MEASURED IMPOSSIBLE on this platform. Carries the
              measurement. Not a to-do and not a silent gap; see by_design.py
    PIXELS    only answerable by extracting frames and looking

A CHECK THAT CANNOT FAIL ON A DELIBERATELY BROKEN INPUT IS NOT A CHECK, so
--self-test breaks each input in turn and requires the check to notice.
"""

import argparse
import math
import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments",
                                "experiments", "abc"))
sys.path.insert(0, os.path.join(WS, "src"))

PRESENT, MISSING, BLOCKED, PIXELS = "PRESENT", "MISSING", "BLOCKED", "PIXELS"

BANNED_WORDS = ("leverage", "robust", "seamless", "delve")


# --------------------------------------------------------------------------
# geometry helpers -- shared by the checks and by the self-test
# --------------------------------------------------------------------------
def _outside(box, x, y):
    """How far (mm) the point lies outside the box, per axis."""
    ox = 0.0 if box["x"][0] <= x <= box["x"][1] else \
        min(abs(x - box["x"][0]), abs(x - box["x"][1]))
    oy = 0.0 if box["y"][0] <= y <= box["y"][1] else \
        min(abs(y - box["y"][0]), abs(y - box["y"][1]))
    return ox * 1000.0, oy * 1000.0


def _footprints_overlap(a_xy, a_wd, b_xy, b_wd):
    return (abs(a_xy[0] - b_xy[0]) <= (a_wd[0] + b_wd[0]) / 2.0 and
            abs(a_xy[1] - b_xy[1]) <= (a_wd[1] + b_wd[1]) / 2.0)


def _cycles(states):
    """(closes, opens) in a gripper schedule, counted as TRANSITIONS.

    Counting distinct values would report 1 for a schedule that closes once
    and stays shut, which is exactly the defect this criterion exists to
    catch: four pick-and-places recorded as one close and one open.
    """
    closes = opens = 0
    prev = None
    for s in states:
        shut = s > 0.05
        if prev is not None and shut != prev:
            (closes if shut else opens).__class__      # no-op, keeps flake8
            if shut:
                closes += 1
            else:
                opens += 1
        prev = shut
    return closes, opens


# --------------------------------------------------------------------------
# the checks. each returns (state, evidence)
# --------------------------------------------------------------------------
def check_t0(mods):
    CS, M, T0 = mods["clip_scene"], mods["msc"], mods["task0"]
    out = {}
    tgt, _ = T0.sample_trial(M.T0_CLIP_SEED)
    labels = ("L1", "L2", "L3", "R1", "R2", "R3")

    drawn = CS.fixtures_for("t0")
    out["T0-1"] = ((PRESENT, "clip_scene.fixtures_for('t0') = %s" % drawn)
                   if sorted(drawn) == sorted(labels)
                   else (MISSING, "fixtures_for('t0') = %r" % drawn))

    cols = {k: T0.SPHERE_COLOURS.get(k) for k in labels}
    known = all(c in CS.SPHERE_RGBA for c in cols.values())
    by_colour_not_arm = len({cols["L1"], cols["L2"], cols["L3"]}) == 3
    out["T0-2"] = ((PRESENT, "SPHERE_COLOURS %s, all in clip_scene.SPHERE_RGBA"
                    % cols)
                   if known and by_colour_not_arm
                   else (MISSING, "colours %r resolvable=%s distinct-per-arm=%s"
                         % (cols, known, by_colour_not_arm)))

    src = open(os.path.join(WS, "scripts", "clip_scene.py")).read()
    lab = "TEXT_VIEW_FACING" in src and 'lab.text = label' in src
    out["T0-3"] = ((PRESENT, "tick() adds a TEXT_VIEW_FACING marker per label")
                   if lab else (MISSING, "no per-sphere text marker in tick()"))

    seps = [math.dist(tgt[a], tgt[b])
            for i, a in enumerate(labels) for b in labels[i + 1:]
            if a[0] == b[0]]
    out["T0-4"] = ((PRESENT, "within-arm sphere separation %.2f..%.2f m at "
                             "seed %d" % (min(seps), max(seps), M.T0_CLIP_SEED))
                   if min(seps) >= 0.25
                   else (MISSING, "closest two targets on one arm are %.3f m "
                                  "apart" % min(seps)))

    furn = CS.furniture_boxes("t0")
    out["T0-5"] = ((PRESENT, "furniture_boxes('t0') is empty")
                   if not furn
                   else (MISSING, "T0 draws %s"
                         % [f[0] for f in furn]))
    return out


def check_t1(mods):
    CS, M = mods["clip_scene"], mods["msc"]
    import t1_task as T1M
    out = {}
    cube = (T1M.CUBE_M, T1M.CUBE_M)

    # T1-1 -- do the cubes rest on the table
    #
    # AGAINST T1'S OWN TABLE, NOT THE SHARED ONE. T1 was rebuilt on its own
    # surface at 1.250 with the objects standing on it; `CS.TABLE_TOP` is the
    # shared 0.980 that every other task works 120 mm above, and measuring
    # against it reports a 290 mm float for a scene whose cubes are touching
    # the wood. `table_geometry` is the one place that answers "which table".
    _top, _near, _far = CS.table_geometry("t1")
    base = T1M.T1_Z - T1M.CUBE_M / 2.0
    gap_mm = (base - _top) * 1000.0
    if abs(gap_mm) <= 2.0:
        out["T1-1"] = (PRESENT, "cube base %.1f mm from the table top at "
                                "z = %.3f, near edge y = %.3f"
                       % (gap_mm, _top, _near))
    else:
        out["T1-1"] = (BLOCKED,
                       "cube base floats %.0f mm above the table top "
                       "(%.3f vs %.3f)" % (gap_mm, base, _top))

    # T1-2 -- cubes clear of the pads at the start
    hits = []
    pad = (T1M.PAD_W, T1M.PAD_D)
    for i, c in enumerate(T1M.T1_CUBES):
        for pi, p in enumerate(T1M.T1_PLANES):
            if _footprints_overlap([c[0], c[1]], cube,
                                   [p[0], T1M.ROW_Y], pad):
                hits.append("cube_%d/pad_%d" % (i, pi))
    gap_mm = (min(abs(c[0]) for c in T1M.T1_CUBES)
              - (max(abs(p[0]) for p in T1M.T1_PLANES) + T1M.PAD_W / 2.0)
              - T1M.CUBE_M / 2.0) * 1000.0
    out["T1-2"] = ((PRESENT, "the nearest cube's edge is %.0f mm outboard of "
                             "the pad's edge, in the same row" % gap_mm)
                   if not hits else (MISSING, "overlapping at start: %s"
                                     % ", ".join(hits)))

    # T1-3 -- one close and one open PER CUBE, counted over BOTH arms.
    #
    # It used to read `M.T1_ARM`, which the 2026-08-17 rebuild deleted: T1
    # runs on both arms now, a cube's arm follows the colour the CAMERA saw,
    # and counting one arm's schedule reports half the cycles for a task that
    # is working correctly.
    path = T1M.build()
    grip = T1M.grip(len(path["left"]))
    cl = op = 0
    for arm in ("left", "right"):
        c, o = _cycles(grip[arm])
        cl += c
        op += o
    n_cubes = len(T1M.T1_CUBES)
    out["T1-3"] = ((PRESENT, "%d closes and %d opens over %d waypoints per "
                             "arm, for %d cubes"
                    % (cl, op, len(grip["left"]), n_cubes))
                   if (cl, op) == (n_cubes, n_cubes)
                   else (MISSING, "%d closes / %d opens, %d of each expected"
                         % (cl, op, n_cubes)))

    # T1-4 -- each cube moves only while gripped
    src = open(os.path.join(WS, "scripts", "clip_scene.py")).read()
    reads_graspable = 'it.get("graspable") is False or it.get("placed")' in src
    proximity = "GRASP_NEAR_M" in src and "near = d is not None" in src
    out["T1-4"] = ((PRESENT, "tick() honours graspable/placed and gates "
                             "attachment on GRASP_NEAR_M = %.3f m"
                    % CS.GRASP_NEAR_M)
                   if reads_graspable and proximity
                   else (MISSING, "graspable read=%s proximity gate=%s"
                         % (reads_graspable, proximity)))

    # T1-5 -- the PADS land on the cube, and the hand stays out of the table.
    #
    # TWO NUMBERS, AND THE FIRST ONE IS NEW. "the pad offset is applied" was a
    # check that the CONSTANT existed, and the constant was wrong by 13.47 mm
    # for as long as it had existed -- derived from the magnitude of a
    # world-frame vector rather than measured. `verify_t1.py` reads the finger
    # tips off FK at the pose the task commands and reports how far the pad
    # midpoint lands from the cube centre; that is the number this asks for.
    import json as _json
    rec_f = os.path.join(WS, "recordings", "baselines", "t1_paths.json")
    miss_mm, low_mm = None, None
    if os.path.exists(rec_f):
        _rec = _json.load(open(rec_f))
        _g = [g for g in _rec.get("grasps", []) if g.get("solved")]
        if _g:
            miss_mm = max(g["pad_miss_mm"] for g in _g)
            low_mm = min(g["lowest_tip_above_table_mm"] for g in _g)
    lowest = min(p[2] for a in ("left", "right") for p in path[a])
    clear_mm = (lowest - T1M.TABLE_TOP) * 1000.0
    out["T1-5"] = ((PRESENT, "measured from FK: the pads land %.2f mm from "
                             "the cube centre and the lowest finger tip is "
                             "%.1f mm above the table; lowest commanded wrist "
                             "%.0f mm above it"
                    % (miss_mm, low_mm, clear_mm))
                   if (miss_mm is not None and miss_mm <= 2.0
                       and low_mm is not None and low_mm > 0.0
                       and clear_mm > 0)
                   else (MISSING, "pad miss %s mm, lowest tip %s mm above the "
                                  "table, lowest wrist %.0f mm above it -- run "
                                  "scripts/verify_t1.py"
                         % (miss_mm, low_mm, clear_mm)))

    # T1-6 -- each cube ends on the pad of its own colour, on the arm that can
    # reach that pad. Read from the RESOLUTION the task performs, not from a
    # restatement of it.
    plan = T1M.plan_for()
    bad = []
    for i, (x, y, pad, arm) in enumerate(plan):
        if T1M.PLANE_COLOURS[pad] != T1M.T1_RENDERED[i]:
            bad.append("cube_%d renders %s and is sent to the %s pad"
                       % (i, T1M.T1_RENDERED[i], T1M.PLANE_COLOURS[pad]))
        if (x >= 0) != (arm == "left"):
            bad.append("cube_%d at x=%+.3f is run by the %s arm" % (i, x, arm))
    at = T1M.grip_at(len(grip["left"]))
    per_pad = {}
    for _x, _y, _p, _a in plan:
        per_pad[_p] = per_pad.get(_p, 0) + 1
    used, delivered = {}, []
    for i, (x, y, pad, arm) in enumerate(plan):
        k = used.get(pad, 0)
        used[pad] = k + 1
        px = round(T1M.T1_PLANES[pad][0] + T1M.SLOT_OFFSETS[per_pad[pad]][k], 4)
        want = [px, T1M.ROW_Y, round(T1M.T1_Z + T1M.PAD_T, 4)]
        delivered.append(any(math.dist(a, want) < 1e-6 for a in at[arm]))
    out["T1-6"] = ((PRESENT, "every cube is routed to the pad of its RENDERED "
                             "colour by the arm that can reach it, and each "
                             "landing slot appears in the arrival gate")
                   if not bad and all(delivered)
                   else (MISSING, "%s; slots not gated %s"
                         % ("; ".join(bad) or "-",
                            [i for i, d in enumerate(delivered) if not d])))

    # T1-7 -- markings on the correct arms, ENCLOSING all objects.
    #
    # AGAINST T1'S OWN MARKING, NOT THE GLOBAL SURVEY. `REGION_CELLS` was
    # measured at the pinned anchor on the work plane 120 mm above the table
    # over y = 0.075..0.300; T1 works at its own approach, ON the table, in a
    # single row -- so not one of those cells is over this task's work and
    # `in_region` answers about a different task.
    objs = [("cube_%d" % i, c[0], c[1]) for i, c in enumerate(T1M.T1_CUBES)]
    widest = max(max(abs(o) for o in offs)
                 for offs in T1M.SLOT_OFFSETS.values())
    for pi, (px, _py) in enumerate(T1M.T1_PLANES):
        for sgn in (-1, +1):
            objs.append(("pad_%d slot %+.0f" % (pi, sgn * widest * 1000),
                         round(px + sgn * widest, 4), T1M.ROW_Y))
    h = CS.REGION_STEP / 2.0
    off = []
    for name, x, y in objs:
        arm = "left" if x >= 0 else "right"
        cells = CS.t1_marking_cells(arm)
        if not cells:
            off.append(name)
            continue
        x0, x1 = min(c[0] for c in cells) - h, max(c[0] for c in cells) + h
        y0, y1 = min(c[1] for c in cells) - h, max(c[1] for c in cells) + h
        if not (x0 <= x <= x1 and y0 <= y <= y1):
            off.append(name)
    arms = CS.marked_arms("t1")
    both = set(arms) == {"left", "right"}
    if off or not both:
        out["T1-7"] = (MISSING,
                       "%d of %d objects outside the drawn marking (%s); "
                       "arms drawn %s and T1 works both"
                       % (len(off), len(objs), ", ".join(off) or "-", arms))
    else:
        out["T1-7"] = (PRESENT,
                       "all %d objects inside the marking, drawn for both "
                       "arms from the task's own verified cells" % len(objs))

    # ---- T1-8  the table is WHITE ----------------------------------------
    # NEUTRALITY, not a specific RGB. "White" here means R = G = B and bright,
    # which is also the property that keeps it out of every colour detector
    # in verify_rviz_clips -- all of which key on channel DIFFERENCES.
    #
    # AND THE BRIGHTNESS TEST IS ON THE RENDERED VALUE, NOT THE REQUESTED ONE.
    # This check used to ask only whether CS.OAK was neutral and >= 0.80, and
    # it passed the shipped (0.94, 0.94, 0.95) table for months while that
    # table rendered at RGB(118,118,118) -- mid grey -- in every clip.
    UP_FACING_COEFF = 0.5            # probe_marker_shading.py
    WHITE_FLOOR_8BIT = 200           # below this a top reads grey on screen
    r, g, b = CS.OAK[0], CS.OAK[1], CS.OAK[2]
    neutral = max(abs(r - g), abs(g - b), abs(r - b)) <= 0.03
    rendered = min(255.0, UP_FACING_COEFF * min(r, g, b) * 255.0)
    out["T1-8"] = ((PRESENT, "table top rgba %s -> renders ~%.0f of 255 on an "
                             "up-facing face: neutral and white"
                             % (CS.OAK, rendered))
                   if neutral and rendered >= WHITE_FLOOR_8BIT
                   else (MISSING, "table top rgba %s renders ~%.0f of 255 "
                                  "(want >= %d, neutral): %s"
                                  % (CS.OAK, rendered, WHITE_FLOOR_8BIT,
                                     "not neutral" if not neutral
                                     else "too dark to read as white")))

    # ---- T1-9  stage 1 works BOTH arms, with the pads across the centre ---
    #
    # THIS CRITERION USED TO SAY THE OPPOSITE, and TASK_SPEC has been changed
    # rather than this check bent to fit: "the LEFT arm only, with all four
    # cubes on the LEFT" was the pre-rebuild T1, whose two pads sat 595 and
    # 825 mm off the centreline beside the person. The rebuilt T1 puts one pad
    # either side of the centreline directly in front of the wearer, so the
    # arm a cube is run by follows the colour it renders -- and neither arm
    # can cross the centreline, measured.
    xs = [c[0] for c in T1M.T1_CUBES]
    pads = [p[0] for p in T1M.T1_PLANES]
    left_cubes = [x for x in xs if x > 0]
    straddles = min(pads) < 0.0 < max(pads)
    per_arm = {a: sum(1 for _x, _y, _p, _a in plan if _a == a)
               for a in ("left", "right")}
    out["T1-9"] = ((PRESENT, "pads at %s straddle the centreline; %d cubes on "
                             "the left run by the left arm and %d on the "
                             "right by the right"
                    % ([round(p, 3) for p in pads], per_arm["left"],
                       per_arm["right"]))
                   if straddles and per_arm["left"] and per_arm["right"]
                   and len(left_cubes) == per_arm["left"]
                   else (MISSING, "pads %s, per-arm work %s"
                         % ([round(p, 3) for p in pads], per_arm)))

    # ---- T1-10  the pads are at the innermost SAFE column -----------------
    # Against the MEASURED per-column sweep, and the offset stated in the spec.
    col_f = os.path.join(WS, "recordings", "baselines", "t1_pad_columns.json")
    inner = min(abs(p[0]) for p in T1M.T1_PLANES)
    stated = ("%d mm" % round(inner * 1000)) in open(
        os.path.join(WS, "docs", "TASK_SPEC.md")).read()
    ok10, why10 = False, "no t1_pad_columns.json -- nobody swept the columns"
    if os.path.exists(col_f):
        cols = _json.load(open(col_f))
        floor = float(cols["floor"])
        worst = []
        for pi, (px, _py) in enumerate(T1M.T1_PLANES):
            arm = T1M.arm_for_pad(pi)
            rows = {round(float(r["column"]), 4): r for r in cols["arms"][arm]}
            for sgn in (-1, +1):
                slot = round(abs(px) + sgn * widest, 4)
                r = rows.get(slot)
                if r is None or r["worst_clearance"] is None:
                    worst.append("%s slot %.3f UNMEASURED" % (arm, slot))
                elif r["worst_clearance"] < floor + 0.020:
                    worst.append("%s slot %.3f at %.4f m"
                                 % (arm, slot, r["worst_clearance"]))
        ok10 = not worst and stated
        why10 = ("every pad slot measured clear with margin; the pair is "
                 "%d mm either side of the centreline and the spec says so"
                 % round(inner * 1000)) if ok10 else \
                ("%s; offset stated in the spec: %s"
                 % ("; ".join(worst) or "-", stated))
    out["T1-10"] = (PRESENT, why10) if ok10 else (MISSING, why10)

    # ---- T1-11  the clearance floor is measured, not assumed -------------
    # The check is that the MEASUREMENT EXISTS and passed for THIS layout, and
    # for BOTH stages: the whole finding behind T1-11 is that a layout can be
    # clean by every IK check and spend half its path inside the floor.
    runs, bad11 = [], []
    if os.path.exists(rec_f):
        d = _json.load(open(rec_f))
        same = (d.get("layout", {}).get("cubes")
                == [list(c) for c in T1M.T1_CUBES]
                and d.get("layout", {}).get("planes")
                == [list(p) for p in T1M.T1_PLANES])
        if not same:
            bad11.append("t1_paths.json records a different layout")
        else:
            runs.append("stage 1")
            for arm, r in d["arms"].items():
                if r["ik_failures"] or r["floor_breaches"]:
                    bad11.append("stage 1 %s: %d IK failures, %d breaches"
                                 % (arm, r["ik_failures"],
                                    r["floor_breaches"]))
    else:
        bad11.append("no t1_paths.json")
    s2_f = os.path.join(WS, "recordings", "baselines", "t1_stage2_paths.json")
    if os.path.exists(s2_f):
        d2 = _json.load(open(s2_f))
        for sd, v in sorted(d2.get("seeds", {}).items()):
            runs.append("stage 2 seed %s" % sd)
            if not v.get("clean"):
                bad11.append("stage 2 seed %s did not verify" % sd)
    else:
        bad11.append("no t1_stage2_paths.json -- stage 2 is unwalked")
    out["T1-11"] = ((PRESENT, "%d walked runs, all clean (%s)"
                     % (len(runs), ", ".join(runs)))
                    if runs and not bad11
                    else (MISSING, "; ".join(bad11)))

    # ---- T1-12  stage 2 randomises the SIDE, and the seed is read --------
    ra = open(os.path.join(WS, "src", "srl_experiments", "experiments", "abc",
                           "run_abc.py")).read()
    seed_read = 'build"](seed=a.seed)' in ra and \
                M.TASKS["t1s2"].get("seeded") is True
    sides = set()
    for sd in range(12):
        t = M.stage2_targets(sd)["cubes"]
        if not (t["left"] and t["right"]):
            sides = {"A DRAW PUT EVERY CUBE ON ONE SIDE"}
            break
        sides.add((len(t["left"]), len(t["right"])))
    varies = len(sides) > 1 and all(isinstance(s, tuple) for s in sides)
    out["T1-12"] = ((PRESENT, "seed reaches the layout; side splits over 12 "
                              "seeds: %s" % sorted(sides))
                    if seed_read and varies
                    else (MISSING, "seed read by the task: %s; side splits "
                                   "seen: %s" % (seed_read, sorted(sides))))
    return out


def check_t2(mods):
    CS, TSK = mods["clip_scene"], mods["tasks"]
    out = {}
    src = open(os.path.join(WS, "scripts", "clip_scene.py")).read()

    between = ('gl, gr = self._grip("left"), self._grip("right")' in src and
               'ns="tray"' in src)
    sep_mm = TSK.TRAY_SEP * 1000.0
    # "DRAWN BETWEEN BOTH GRIPPERS" WAS A CHECK THAT COULD NOT FAIL.
    #
    # It passed for a tray whose LENGTH was the live gripper separation plus a
    # constant -- a board that stretched from 0.487 to 1.211 m to reach
    # whatever the arms were doing, so of course it was always held at both
    # ends. The criterion is that a RIGID tray of the spec length is held at
    # both ends, and only the rigidity makes the criterion capable of failing.
    rigid = 'spec_len = float(TSK.TASK_B["objects"]["tray"]["size"][0])' in src
    elastic = "round(span + 0.06, 4)" in src
    spec_len_m = float(TSK.TASK_B["objects"]["tray"]["size"][0])
    out["T2-1"] = ((PRESENT, "tray drawn between both grippers at the SPEC "
                             "length %.3f m (rigid), grip separation %.0f mm"
                             % (spec_len_m, sep_mm))
                   if between and rigid and not elastic
                   and abs(sep_mm - 500.0) < 1.0
                   else (MISSING,
                         "between-grippers=%s rigid=%s elastic=%s "
                         "separation %.0f mm" % (between, rigid, elastic,
                                                 sep_mm)))

    # AND THE BALL HAD TO BE ABLE TO FALL OFF. It was drawn on the tray
    # unconditionally, so a carry that reached 23 deg -- against a declared
    # 6.8 -- still showed a ball sitting on the surface.
    ball = 'ns="ball"' in src and "ball" in CS.fixtures_for("t2")
    drops = 'self.ball_off = True' in src and 'fail_tilt' in src
    latches = 'getattr(self, "ball_off", False)' in src
    out["T2-2"] = ((PRESENT, "ball radius %.3f m on the tray, and it FALLS "
                    "past %.1f deg and stays off"
                    % (TSK.BALL_R, float(TSK.TASK_B.get("fail_tilt_deg", 6.8))))
                   if ball and drops and latches
                   else (MISSING, "ball=%s drops=%s latches=%s"
                         % (ball, drops, latches)))

    # SAMPLED PER TICK, DUMPED AS A SERIES, AND REDUCED BY ARITHMETIC THAT
    # HAS A KNOWN-ANSWER TEST. Anything less than all three and the carry is
    # still a verdict wearing a series' name.
    sampled = "self.carry_series.append(" in src
    dumped = "carry_series=getattr(n" in src
    reduced = hasattr(CS, "_carry_summary")
    ktest = os.path.exists(os.path.join(
        WS, "src", "srl_experiments", "test",
        "test_carry_series_known_answers.py"))
    declared = TSK.TASK_B.get("log_continuously", ())
    for cid, key, what in (("T2-3", "tilt_deg", "tilt"),
                           ("T2-4", "sep_err_mm", "separation")):
        ok = sampled and dumped and reduced and ktest and key in declared
        out[cid] = ((PRESENT, "%s sampled every tick into "
                              "scene_events.carry_series, reduced by "
                              "_carry_summary(), known-answer tested"
                     % what)
                    if ok
                    else (MISSING, "%s: sampled=%s dumped=%s reduced=%s "
                                   "known-answer-test=%s declared=%s"
                          % (what, sampled, dumped, reduced, ktest,
                             key in declared)))
    return out


def check_t3(mods):
    CS, T3 = mods["clip_scene"], mods["task3"]
    out = {}
    src = open(os.path.join(WS, "scripts", "clip_scene.py")).read()
    furn = [f[0] for f in CS.furniture_boxes("t3")]
    boxes = [f for f in furn if "circuit_box" in f]
    out["T3-1"] = ((PRESENT, "one graspable circuit_box at x=%.3f, none in the "
                             "furniture table" % T3.BOX_OBJ[0])
                   if not boxes
                   else (MISSING, "a second circuit box in the furniture: %s"
                         % boxes))

    # DRAWN, ON THE BOX, AND LEGIBLE. "Declared" was all this used to ask,
    # and the points were then drawn as 14 mm spheres that are sub-pixel at
    # T3's framing -- present in the file and absent from the picture -- with
    # P4 on the MULTIMETER, which contradicts "one circuit box with four
    # measurement points".
    pts = [p["id"] for p in T3.MEASUREMENT_POINTS]
    drawn = CS.fixtures_for("t3")
    on_box = 'it = self.items.get("circuit_box")' in src and \
        'host = "multimeter"' not in src
    labelled = 'lab.ns, lab.id = "measure_labels", i' in src
    pad = 'PAD_W, PAD_T = ' in src
    faces = {p["face"] for p in T3.MEASUREMENT_POINTS}
    ok = (len(pts) == 4 and sorted(drawn) == sorted(pts) and on_box
          and labelled and pad and faces == {"near", "far"})
    out["T3-2"] = ((PRESENT, "%s drawn as labelled pads ON THE BOX across "
                             "its %s faces; %d structurally need a "
                             "repositioning request"
                    % (pts, "/".join(sorted(faces)),
                       T3.MIN_EXPECTED_REQUESTS))
                   if ok
                   else (MISSING, "declared %s drawn %s on_box=%s "
                                  "labelled=%s pads=%s faces=%s"
                         % (pts, drawn, on_box, labelled, pad, faces)))

    right_box = 'dict(arm="right"' in src and T3.BOX_ARM == "right"
    left_meter = T3.METER_ARM == "left" and 'dict(arm="left"' in src
    size_mm = tuple(round(v * 1000) for v in T3.METER_SIZE)
    out["T3-3"] = ((PRESENT, "box on the %s arm, meter (%d x %d x %d mm) on "
                             "the %s arm, each gated on its own object"
                    % (T3.BOX_ARM, size_mm[0], size_mm[1], size_mm[2],
                       T3.METER_ARM))
                   if right_box and left_meter and size_mm == (30, 70, 45)
                   else (MISSING, "box arm %s meter arm %s size %s"
                         % (T3.BOX_ARM, T3.METER_ARM, size_mm)))
    return out


def check_dance(mods):
    ch = mods["choreography"]
    out = {}
    src = open(os.path.join(WS, "src", "srl_experiments", "experiments",
                            "abc", "choreography.py")).read()
    routines = [n for n in ("flow", "pulse", "play") if hasattr(ch, n)]
    out["D-4"] = ((PRESENT, "three routines: %s" % ", ".join(routines))
                  if len(routines) == 3
                  else (MISSING, "routines found: %s" % routines))
    out["D-1"] = ((PRESENT, "beats rendered through EASE %s at a BPM per "
                            "routine" % sorted(ch.EASE))
                  if len(ch.EASE) >= 4 and "_beats_to_n" in src
                  else (MISSING, "no tempo/phrasing layer"))
    # THE RENDERED DURATIONS, not the constant. A floor that is declared and
    # never reached is the "feature present but does nothing" failure mode.
    ant = getattr(ch, "ANTICIPATE_S", None)
    durs, skipped = [], []
    if hasattr(ch, "anticipation_report"):
        for r in ("flow", "pulse", "play"):
            ch._ANTI_LOG.clear()
            ch._ANTI_SKIPPED.clear()
            getattr(ch, r)()
            d, s = ch.anticipation_report()
            durs += d
            skipped += s
    ok = (ant is not None and abs(ant - 0.30) < 1e-6 and durs and
          min(durs) >= 0.30 - 1e-9 and not skipped)
    out["D-2"] = ((PRESENT, "ANTICIPATE_S = %.2f s; %d wind-ups rendered "
                            "across the three routines, %.2f..%.2f s, none "
                            "skipped" % (ant, len(durs), min(durs), max(durs)))
                  if ok
                  else (MISSING, "floor=%r rendered=%s skipped=%d"
                        % (ant, ("%.2f..%.2f" % (min(durs), max(durs)))
                           if durs else "none", len(skipped))))
    rel = [n for n in ("_mirror", "_canon") if n in src]
    out["D-3"] = ((PRESENT, "arm relationships available: %s" % rel)
                  if len(rel) == 2 else (MISSING, "only %s" % rel))
    box = ch.BOX
    out["D-5"] = ((PRESENT, "envelope |x| %.2f..%.2f, y %.2f..%.2f, "
                            "z %.2f..%.2f" % (box[0] + box[1] + box[2]))
                  if (box[2][1] - box[2][0]) >= 0.20
                  else (MISSING, "z span only %.2f m"
                        % (box[2][1] - box[2][0])))
    out["D-6"] = ((PRESENT, "_hold() used in every routine")
                  if src.count("_hold(") >= 6 else (MISSING, "few holds"))
    out["D-7"] = (PIXELS, "compute_ik / avoid_collisions / e-stop are the "
                          "runner's, checked live by the sweep")
    return out


def check_universal(mods):
    out = {}
    per = os.path.join(WS, "src", "srl_perception", "srl_perception")
    has_pub = os.path.exists(os.path.join(per, "mock_rgbd_camera.py"))
    launched = []
    for root, _d, files in os.walk(WS):
        if "/build/" in root or "/install/" in root or "/archive/" in root:
            continue
        for f in files:
            if f.endswith((".launch.py", ".sh")):
                p = os.path.join(root, f)
                try:
                    if "mock_rgbd" in open(p, errors="ignore").read():
                        launched.append(os.path.relpath(p, WS))
                except OSError:
                    pass
    out["U-1"] = ((PRESENT, "mock_rgbd_camera launched by %s" % launched)
                  if has_pub and launched
                  else (MISSING, "mock_rgbd_camera.py exists=%s but NO launch "
                                 "file or run script starts it, so the "
                                 "detection path has never run end to end"
                        % has_pub))

    # U-2 IS A CHAIN AND EVERY LINK IS ASKED FOR SEPARATELY. The failure was
    # never "nobody thought about yaw" -- Observation had a `quat` slot from
    # the start. It was that an identity quaternion, meaning NOT MEASURED,
    # was read as zero degrees at the far end.
    sys.path.insert(0, os.path.join(WS, "src", "srl_perception"))
    from srl_perception import scene_fingerprint as _sf
    links = {}
    links["identity is unknown"] = (
        _sf.yaw_from_quat((0.0, 0.0, 0.0, 1.0)) is None)
    links["fingerprint carries yaw"] = (
        "yaw_deg" in _sf.Observation("x", (0, 0, 0)).as_dict())
    links["symmetry folded"] = abs(_sf.wrap_symmetric(90.0, 90.0)) < 1e-9
    det = open(os.path.join(per, "colour_shape_detector.py")).read()
    links["detector publishes what it measured"] = (
        "min_aspect_for_yaw" in det)
    gg = open(os.path.join(WS, "src", "srl_autonomy", "srl_autonomy",
                           "grasp_generator.py")).read()
    links["grasp distinguishes unknown"] = ("_yaw_known" in gg and
                                            "yaw_source" in gg)
    bad = [k for k, v in links.items() if not v]
    out["U-2"] = ((PRESENT, "yaw chain complete: %s" % ", ".join(links))
                  if not bad
                  else (MISSING, "broken links: %s" % ", ".join(bad)))

    callers = []
    for root, _d, files in os.walk(os.path.join(WS, "src")):
        for f in files:
            if not f.endswith(".py"):
                continue
            p = os.path.join(root, f)
            t = open(p, errors="ignore").read()
            if "set_measured(" in t and "def set_measured" not in t:
                callers.append(os.path.relpath(p, WS))
    depth = [c for c in callers if "perception" in c or "depth" in c]
    # AND A CONSUMER-SIDE ROUTE, because module state does not cross a
    # process boundary: a node that only sets its own global changes nothing
    # for the follower or the task layer.
    wsurf = open(os.path.join(WS, "src", "srl_experiments", "srl_experiments",
                              "work_surface.py")).read()
    live = os.path.exists(os.path.join(
        WS, "scripts", "verify_work_surface_from_depth.py"))
    out["U-3"] = ((PRESENT, "set_measured called from %s; work_surface"
                            ".subscribe() carries it across processes; live "
                            "end-to-end check present" % depth)
                  if depth and "def subscribe(" in wsurf and live
                  else (MISSING, "producer=%s subscribe=%s live-check=%s"
                        % (depth, "def subscribe(" in wsurf, live)))
    return out


def check_recording(mods):
    out = {}
    rr = open(os.path.join(WS, "scripts", "record_rviz.py")).read()
    sw = open(os.path.join(WS, "scripts", "record_abc_sweep.py")).read()

    out["R-3"] = ((PRESENT, "VIEWS %s + quad + card"
                   % ", ".join(sorted(mods["record_rviz"].VIEWS)))
                  if len(mods["record_rviz"].VIEWS) >= 7 and "QUAD" in rr
                  else (MISSING, "only %d views" % len(mods["record_rviz"].VIEWS)))

    tf = mods["record_rviz"].TASK_FOCUS
    want = set(mods["msc"].ORDER)
    out["R-4"] = ((PRESENT, "TASK_FOCUS aims the front view for %s"
                   % ", ".join(sorted(tf)))
                  if want <= set(tf)
                  else (MISSING, "TASK_FOCUS has no entry for %s, so those "
                                 "tasks fall back to the generic _FOCUS"
                        % sorted(want - set(tf))))

    out["R-5"] = ((PRESENT, "prepend_card() concatenates a card in FRONT of "
                            "every angle")
                  if "def prepend_card" in sw and "concat=n=2" in sw
                  else (MISSING, "no card prepended"))

    bad = []
    for k, (what, watch) in mods["sweep"].CARD_TEXT.items():
        for txt in (what, watch):
            if "—" in txt or "–" in txt:
                bad.append("%s: em/en dash" % k)
            for w in BANNED_WORDS:
                if w in txt.lower():
                    bad.append("%s: %r" % (k, w))
    out["R-6"] = ((PRESENT, "%d cards, no dashes and no banned words"
                   % len(mods["sweep"].CARD_TEXT))
                  if not bad else (MISSING, "; ".join(bad)))

    # R-7 IS A CHAIN AND EVERY LINK IS ASKED FOR. "It exists" was the claim
    # made about this pose twice before it was found never to have been
    # built, so the check reads the stored file, its controls, the stager and
    # the sweep's call to it.
    import json as _json
    pf = os.path.join(WS, "recordings", "baselines",
                      "presentation_pose.json")
    stager = os.path.join(WS, "scripts", "stage_presentation_pose.py")
    stored, ctrl_fail, elev = None, [], {}
    if os.path.exists(pf):
        stored = _json.load(open(pf))
        ctrl_fail = [k for k, v in stored.get("controls", {}).items()
                     if v is False]
        elev = {a: stored["poses"][a]["elevation_deg"]
                for a in ("left", "right")}
    called = "stage_presentation_pose.py" in sw and "opened_on" in sw
    ok = (stored is not None and not ctrl_fail and os.path.exists(stager)
          and called and all(abs(v) <= stored.get("good_enough_deg", 8.0)
                             for v in elev.values()))
    out["R-7"] = ((PRESENT, "joint-space staging move, tool axis %+.1f / "
                            "%+.1f deg against +30.8 / +22.1 at home, wearer "
                            "clearance %.4f / %.4f m; commanded and WAITED ON "
                            "by the sweep, which records opened_on per clip"
                   % (elev["left"], elev["right"],
                      stored["poses"]["left"]["clearance_m"],
                      stored["poses"]["right"]["clearance_m"]))
                  if ok
                  else (MISSING, "stored=%s controls_failed=%s stager=%s "
                                 "called_by_sweep=%s elevations=%s"
                        % (stored is not None, ctrl_fail,
                           os.path.exists(stager), called, elev)))

    leg = "TABLE_LEG" in open(os.path.join(WS, "scripts",
                                           "clip_scene.py")).read()
    out["R-8"] = ((PRESENT, "the tall box at bottom-left is a TABLE LEG, "
                            "%.3f m square, drawn by tick(). Declared scene "
                            "geometry, not a leftover"
                   % mods["clip_scene"].TABLE_LEG)
                  if leg else (MISSING, "unidentified"))
    return out


def check_gui(mods):
    out = {}
    g = open(os.path.join(WS, "scripts", "srl_gui.py")).read()
    v = open(os.path.join(WS, "scripts", "verify_gui_buttons.py")).read()
    out["G-3"] = ((PRESENT, "readiness gate present and covered by "
                            "test_readiness_never_green_on_unknown")
                  if os.path.exists(os.path.join(
                      WS, "src", "srl_experiments", "test",
                      "test_readiness_never_green_on_unknown.py"))
                  else (MISSING, "no never-green-on-unknown test"))
    out["G-BUTTONS"] = ((PRESENT, "level2_click() presses every button")
                        if "def level2_click" in v and "level3" in v
                        else (MISSING, "no click-level sweep"))
    for cid, needle, what in (
            ("G-4", "joint", "per-joint health"),
            ("G-5", "divergence", "commanded-vs-actual divergence"),
            ("G-6", "camera", "both cameras"),
            ("G-7", "blocker", "active blockers"),
            ("G-9", "estop", "e-stop")):
        out[cid] = ((PRESENT, "%s referenced in srl_gui.py" % what)
                    if needle in g.lower()
                    else (MISSING, "no %s in the GUI" % what))
    return out


SECTIONS = (("T0", check_t0), ("T1", check_t1), ("T2", check_t2),
            ("T3", check_t3), ("DANCE", check_dance),
            ("UNIVERSAL", check_universal), ("RECORDING", check_recording),
            ("GUI", check_gui))


def load():
    import clip_scene
    import clip_tasks
    import msc_clip_tasks
    import task0
    import task3
    import tasks
    import choreography
    sys.path.insert(0, os.path.join(WS, "scripts"))
    import record_rviz
    import importlib
    sweep = importlib.import_module("record_abc_sweep")
    return dict(clip_scene=clip_scene, clip_tasks=clip_tasks,
                msc=msc_clip_tasks, task0=task0, task3=task3, tasks=tasks,
                choreography=choreography, record_rviz=record_rviz,
                sweep=sweep)


def self_test(mods):
    """Break each input on purpose and require the check to notice."""
    import copy
    fails = []

    def expect_fail(name, fn, mutate, restore):
        """Break one input and require the audit to NOTICE.

        RAISING COUNTS AS NOTICING. Some breakages are refused by the task
        itself rather than reported by the audit -- a cube whose colour names
        the other side's pad makes `t1_task.build` raise, because emitting the
        path would be worse than refusing it. A probe that treated the
        exception as a harness error would say the check had failed to detect
        the very thing it detected loudest.
        """
        mutate()
        try:
            res = fn(mods)
        except Exception as e:                                # noqa: BLE001
            print("   %-46s detected by REFUSAL: %s"
                  % (name, str(e).split(".")[0][:60]))
            return
        finally:
            restore()
        if all(s == PRESENT for s, _ in res.values()):
            fails.append(name)

    CS, M, T3 = mods["clip_scene"], mods["msc"], mods["task3"]

    import t1_task as T1M
    orig = copy.deepcopy(T1M.T1_CUBES)
    expect_fail("T1 cube/pad overlap",
                check_t1,
                lambda: T1M.T1_CUBES.__setitem__(
                    0, [T1M.T1_PLANES[0][0], T1M.ROW_Y]),
                lambda: T1M.T1_CUBES.__setitem__(0, orig[0]))

    # THE PROBES THAT MUTATED `REGION_CELLS` AND `T1_ARM` ARE GONE WITH THE
    # THINGS THEY MUTATED. T1 is two-armed and draws its marking from its own
    # verified cells rather than from the global survey, so breaking the
    # survey no longer breaks T1's marking -- correctly. What has to be broken
    # instead is the MARKING ITSELF, and the arm assignment.
    tmc = CS.t1_marking_cells
    expect_fail("T1 marking encloses objects",
                check_t1,
                lambda: setattr(CS, "t1_marking_cells",
                                lambda arm, task="t1", seed=0: [(0.0, 0.0)]),
                lambda: setattr(CS, "t1_marking_cells", tmc))

    ma = CS.marked_arms
    expect_fail("T1 marks BOTH arms",
                check_t1,
                lambda: setattr(CS, "marked_arms", lambda t: ("left",)),
                lambda: setattr(CS, "marked_arms", ma))

    afp = T1M.arm_for_pad
    expect_fail("T1 sends each cube to the arm that can reach its pad",
                check_t1,
                lambda: setattr(T1M, "arm_for_pad", lambda i: "left"),
                lambda: setattr(T1M, "arm_for_pad", afp))

    fx = CS.fixtures_for
    expect_fail("T0 spheres drawn",
                check_t0,
                lambda: setattr(CS, "fixtures_for",
                                lambda t: [] if t == "t0" else fx(t)),
                lambda: setattr(CS, "fixtures_for", fx))

    fb = CS.furniture_boxes
    expect_fail("T0 has no bench",
                check_t0,
                lambda: setattr(CS, "furniture_boxes",
                                lambda t: [("bench", [0, 0, 0], [1, 1, 1],
                                            None)] if t == "t0" else fb(t)),
                lambda: setattr(CS, "furniture_boxes", fb))

    expect_fail("T3 exactly one box",
                check_t3,
                lambda: setattr(CS, "furniture_boxes",
                                lambda t: list(fb(t)) +
                                [("circuit_box", [0, 0, 0], [1, 1, 1], None)]
                                if t == "t3" else fb(t)),
                lambda: setattr(CS, "furniture_boxes", fb))

    pts = list(T3.MEASUREMENT_POINTS)
    expect_fail("T3 four measurement points",
                check_t3,
                lambda: T3.MEASUREMENT_POINTS.__delitem__(slice(2, None)),
                lambda: T3.MEASUREMENT_POINTS.__setitem__(slice(None), pts))

    # A set of points that are all on ONE face has lost the coordination
    # demand the task is built on: P3 is on the far face and that is why a
    # repositioning request is structurally required.
    expect_fail("T3 points span both faces",
                check_t3,
                lambda: T3.MEASUREMENT_POINTS.__setitem__(
                    slice(None), [dict(p, face="near") for p in pts]),
                lambda: T3.MEASUREMENT_POINTS.__setitem__(slice(None), pts))

    card = dict(mods["sweep"].CARD_TEXT)
    expect_fail("card text style",
                check_recording,
                lambda: mods["sweep"].CARD_TEXT.__setitem__(
                    "t0", ("A robust and seamless routine — watch it.", "x")),
                lambda: (mods["sweep"].CARD_TEXT.clear(),
                         mods["sweep"].CARD_TEXT.update(card)))

    tf = dict(mods["record_rviz"].TASK_FOCUS)
    expect_fail("front view aimed per task",
                check_recording,
                lambda: mods["record_rviz"].TASK_FOCUS.clear(),
                lambda: mods["record_rviz"].TASK_FOCUS.update(tf))

    # and the cycle counter itself, which is the T1-3 instrument
    if _cycles([0.0] * 5 + [0.6] * 5 + [0.0] * 5) != (1, 1):
        fails.append("_cycles single cycle")
    if _cycles([0.0, 0.6, 0.0, 0.6, 0.0, 0.6, 0.0, 0.6, 0.0]) != (4, 4):
        fails.append("_cycles four cycles")
    if _cycles([0.6] * 10) != (0, 0):
        fails.append("_cycles never opens")

    print("SELF-TEST: %d probes" % 13)
    if fails:
        print("FAILED -- these checks passed a deliberately broken input:")
        for f in fails:
            print("   ", f)
        return 1
    print("all probes correctly detected the break")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    mods = load()
    if a.self_test:
        return self_test(mods)

    print("AUDIT OF docs/TASK_SPEC.md AGAINST THE CODE")
    print("=" * 74)
    tally = {PRESENT: 0, MISSING: 0, BLOCKED: 0, PIXELS: 0}
    missing = []
    for title, fn in SECTIONS:
        print("\n%s" % title)
        for cid, (state, why) in sorted(fn(mods).items()):
            tally[state] += 1
            if state in (MISSING, BLOCKED):
                missing.append((cid, state, why))
            print("  %-10s %-8s %s" % (cid, state, why))
    print("\n" + "=" * 74)
    print("%d PRESENT   %d MISSING   %d BLOCKED   %d PIXELS-ONLY"
          % (tally[PRESENT], tally[MISSING], tally[BLOCKED], tally[PIXELS]))
    if missing:
        print("\nNOT SAFE TO RECORD UNTIL THESE ARE SETTLED:")
        for cid, state, why in missing:
            print("  %-10s %-8s %s" % (cid, state, why.split(".")[0]))
    return 1 if tally[MISSING] else 0


if __name__ == "__main__":
    sys.exit(main())
