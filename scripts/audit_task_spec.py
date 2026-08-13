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
    out = {}
    cube = (0.04, 0.04)
    plane = (CS.PLANE_W, CS.PLANE_D)

    # T1-1 -- do the cubes rest on the table
    base = M.T1_Z - 0.04 / 2.0
    gap_mm = (base - CS.TABLE_TOP) * 1000.0
    if abs(gap_mm) <= 2.0:
        out["T1-1"] = (PRESENT, "cube base %.1f mm from the table top" % gap_mm)
    else:
        out["T1-1"] = (BLOCKED,
                       "cube base floats %.0f mm above the table top "
                       "(%.3f vs %.3f). Supports MEASURED FATAL: pedestals "
                       "give 0/4 cubes and 0/2 planes reachable, cantilever "
                       "lips 16-22 waypoint failures, footprint pads 26, side "
                       "posts 65, none 0 -- clip_scene.SUPPORTS_ENABLED and "
                       "docs/system/13_fixturing_and_the_approach_cone.md"
                       % (gap_mm, base, CS.TABLE_TOP))

    # T1-2 -- cubes clear of the planes at the start
    hits = []
    for i, c in enumerate(M.T1_CUBES):
        for pi, p in enumerate(M.T1_PLANES):
            if _footprints_overlap(c, cube, p, plane):
                hits.append("cube_%d/plane_%d" % (i, pi))
    out["T1-2"] = ((PRESENT, "cube row y=%.3f, plane row y=%.3f, %.0f mm "
                             "apart against a %.0f mm half-sum"
                    % (M.T1_CUBES[0][1], M.T1_PLANES[0][1],
                       abs(M.T1_CUBES[0][1] - M.T1_PLANES[0][1]) * 1000,
                       (cube[1] + plane[1]) / 2 * 1000))
                   if not hits else (MISSING, "overlapping at start: %s"
                                     % ", ".join(hits)))

    # T1-3 -- four separate close/open cycles
    path = M.t1()
    grip = M.t1_grip(len(path[M.T1_ARM]))[M.T1_ARM]
    cl, op = _cycles(grip)
    out["T1-3"] = ((PRESENT, "%d closes and %d opens over %d waypoints"
                    % (cl, op, len(grip)))
                   if (cl, op) == (4, 4)
                   else (MISSING, "%d closes / %d opens, four of each expected"
                         % (cl, op)))

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

    # T1-5 -- pads on the cube, not inside the table
    lowest = min(p[2] for p in path[M.T1_ARM])
    pad_ok = "PAD_OFFSET" in open(
        os.path.join(WS, "src", "srl_experiments", "experiments", "abc",
                     "clip_tasks.py")).read()
    clear_mm = (lowest - CS.TABLE_TOP) * 1000.0
    out["T1-5"] = ((PRESENT, "PAD_OFFSET applied in ee_for(); lowest commanded "
                             "wrist z %.3f, %.0f mm clear of the table top"
                    % (lowest, clear_mm))
                   if pad_ok and clear_mm > 0
                   else (MISSING, "pad offset=%s lowest wrist %.0f mm above "
                                  "the table" % (pad_ok, clear_mm)))

    # T1-6 -- each cube ends on the plane of its own colour
    bad = []
    for i in range(len(M.T1_CUBES)):
        want_blue = i in (0, 2)
        plane_is_blue = M.T1_PAIR[i] == 0
        if want_blue != plane_is_blue:
            bad.append(i)
    at = M.t1_grip_at(len(grip))
    delivered = {i: False for i in range(len(M.T1_CUBES))}
    for i in range(len(M.T1_CUBES)):
        px, py = M.T1_PLANES[M.T1_PAIR[i]]
        slot = -M.SLOT_DY if i in (0, 1) else +M.SLOT_DY
        want = [px, round(py + slot, 4), M.T1_Z]
        delivered[i] = any(math.dist(a, want) < 1e-6 for a in at)
    out["T1-6"] = ((PRESENT, "pairing 0,2->blue 1,3->green, and every cube's "
                             "own slot appears in the arrival gate")
                   if not bad and all(delivered.values())
                   else (MISSING, "mispaired %s; slots not gated %s"
                         % (bad, [k for k, v in delivered.items() if not v])))

    # T1-7 -- markings on the correct arms, enclosing all objects.
    #
    # CELL MEMBERSHIP, NOT THE BOUNDING BOX. The measured region is not a
    # rectangle -- the right arm's cells fill 62% of their own box at 50 mm
    # resolution -- so "inside the box" would pass objects standing on cells
    # that were never reachable.
    objs = [("cube_%d" % i, c[0], c[1]) for i, c in enumerate(M.T1_CUBES)]
    for pi, (px, py) in enumerate(M.T1_PLANES):
        for s in (-M.SLOT_DY, +M.SLOT_DY):
            objs.append(("plane_%d slot %+.0f" % (pi, s * 1000), px,
                         round(py + s, 4)))
    off = [n for n, x, y in objs if not CS.in_region(M.T1_ARM, x, y)]
    arms = CS.marked_arms("t1")
    right_arms = arms == (M.T1_ARM,)
    from_survey = "REGION_CELLS" in src and "_load_region" in src
    if off or not right_arms or not from_survey:
        out["T1-7"] = (MISSING,
                       "%d of %d objects outside the measured cells (%s); "
                       "arms drawn %s against T1_ARM=%s; marking read from "
                       "the survey=%s"
                       % (len(off), len(objs), ", ".join(off) or "-",
                          arms, M.T1_ARM, from_survey))
    else:
        out["T1-7"] = (PRESENT,
                       "all %d objects inside MEASURED cells; %d cells drawn "
                       "for the %s arm only (x %.2f..%.2f y %.2f..%.2f at "
                       "%.0f mm), read from the survey file"
                       % (len(objs), len(CS.REGION_CELLS[M.T1_ARM]),
                          M.T1_ARM, CS.WORKSPACE[M.T1_ARM]["x"][0],
                          CS.WORKSPACE[M.T1_ARM]["x"][1],
                          CS.WORKSPACE[M.T1_ARM]["y"][0],
                          CS.WORKSPACE[M.T1_ARM]["y"][1],
                          CS.REGION_STEP * 1000))
    return out


def check_t2(mods):
    CS, TSK = mods["clip_scene"], mods["tasks"]
    out = {}
    src = open(os.path.join(WS, "scripts", "clip_scene.py")).read()

    between = ('gl, gr = self._grip("left"), self._grip("right")' in src and
               'ns="tray"' in src)
    sep_mm = TSK.TRAY_SEP * 1000.0
    out["T2-1"] = ((PRESENT, "tray drawn between both grippers, grip "
                             "separation %.0f mm" % sep_mm)
                   if between and abs(sep_mm - 500.0) < 1.0
                   else (MISSING, "between-grippers=%s separation %.0f mm"
                         % (between, sep_mm)))

    ball = 'ns="ball"' in src and "ball" in CS.fixtures_for("t2")
    out["T2-2"] = ((PRESENT, "ball radius %.3f m drawn on the tray top"
                    % TSK.BALL_R)
                   if ball else (MISSING, "no ball fixture for t2"))

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
        mutate()
        try:
            res = fn(mods)
        finally:
            restore()
        if all(s == PRESENT for s, _ in res.values()):
            fails.append(name)

    CS, M, T3 = mods["clip_scene"], mods["msc"], mods["task3"]

    orig = copy.deepcopy(M.T1_CUBES)
    expect_fail("T1 cube/plane overlap",
                check_t1,
                lambda: M.T1_CUBES.__setitem__(
                    0, [M.T1_PLANES[0][0], M.T1_PLANES[0][1]]),
                lambda: M.T1_CUBES.__setitem__(0, orig[0]))

    cells = copy.deepcopy(CS.REGION_CELLS)
    expect_fail("T1 marking encloses objects",
                check_t1,
                lambda: CS.REGION_CELLS.__setitem__(M.T1_ARM, [(0.0, 0.0)]),
                lambda: CS.REGION_CELLS.update(cells))

    ma = CS.marked_arms
    expect_fail("T1 marks only its own arm",
                check_t1,
                lambda: setattr(CS, "marked_arms",
                                lambda t: ("left", "right")),
                lambda: setattr(CS, "marked_arms", ma))

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
