#!/usr/bin/env python3
"""What RViz has to show, built as markers, with no ROS in the way.

    python3 -m srl_teleop.rviz_master        # the self-test

WHY THIS FILE EXISTS AT ALL. The GUI's RViz was built before anybody had
driven a real arm. It shows COMMANDED beside ACTUAL and a divergence number,
which is the right shape and, after 2026-08-21, demonstrably not enough:
every one of the day's eight defects was invisible on that screen.
`docs/NEXT_SESSION_2026_08_22.md` lists what real arms showed it needs, and
this module builds exactly those five things:

  1  THREE STATES, NOT TWO. Commanded, actual, and PLANNED-BUT-REFUSED --
     the pose the checker rejected and the reason string. Every refusal on
     2026-08-21 was invisible unless somebody read a log, and two of them
     sent the operator to check a power fault that did not exist.
  2  THE WEARER FLOOR AS GEOMETRY. A clearance of "0.0962 m" next to a robot
     means nothing; the 0.15 m shell around the body does. The segment
     nearest the wearer is coloured by its own margin and LABELLED with the
     part it is nearest to -- `moving_chain_to` already carries that and
     nothing drew it.
  3  THE WHOLE PATH. The densified sweep between waypoints, not the
     endpoints. Defect 2 -- a pick that cleared three poses at 0.4 m and
     swept through the wearer between them -- existed because nobody could
     see the middle of a motion.
  4  LIVENESS ON THE FACE OF IT. Joint-state age per arm and the bridge's
     own error count. Four of the day's "the arm dropped off the network"
     calls were the optimiser holding the GIL and starving the spinner, and
     all four would have been settled in one glance at an age counter.
  5  THE WORKSPACE ENVELOPE, in three distinct volumes per arm: reachable,
     WEARER-LIMITED, and out of reach. Those are three different facts and
     `arm_reach_extents.json` collapsed them into "unreachable".

WHY IT IS PURE. Every function here takes plain numbers and returns plain
marker dictionaries. The ROS node is a thin shell over it, and the self-test
runs with no graph, no move_group and no display -- which is the only reason
a check on what the operator SEES can exist at all. The GUI's own button
audit hung on a modal dialog for weeks; a drawing layer that can only be
tested by looking at it gets tested by nobody.

THE COLOUR RULE, AND IT IS NOT DECORATION. A marker's ambient is 0.5 x its
colour and RViz does not clamp, which is why the table had to ask for 1.88 to
render white. More important: this repository has already shipped a clip
verifier that matched the REQUESTED colour rather than the RENDERED one. So
`colour_for_margin` is a pure function with a known-answer test, and the
thresholds it uses are the guard's own floor, read from one place -- never a
second copy that can drift from what the arm is actually stopped by.
"""
from __future__ import annotations

import math

# ONE SOURCE for the floor. participant_safety_node and mount_guard_node
# enforce it; a drawing that used its own copy would eventually draw a shell
# the arm is not actually stopped by, which is worse than drawing nothing.
DEFAULT_FLOOR_M = 0.15

# The three states, named once.
COMMANDED = "commanded"
ACTUAL = "actual"
REFUSED = "refused"

# Envelope classes. THREE, deliberately: "the arm cannot get there" and "it
# can, but not without going inside the person" are different facts, and the
# second one is actionable -- move the person, change the posture, re-mount.
REACHABLE = "reachable"
WEARER_LIMITED = "wearer_limited"
OUT_OF_REACH = "out_of_reach"

RGBA = {
    COMMANDED:      (0.20, 0.55, 1.00, 0.90),   # blue
    ACTUAL:         (1.00, 0.75, 0.10, 0.95),   # amber
    REFUSED:        (1.00, 0.15, 0.15, 0.85),   # red
    REACHABLE:      (0.20, 0.80, 0.35, 0.18),
    WEARER_LIMITED: (1.00, 0.55, 0.00, 0.28),
    OUT_OF_REACH:   (0.45, 0.45, 0.45, 0.10),
    "shell":        (0.90, 0.20, 0.65, 0.16),
    "text":         (1.00, 1.00, 1.00, 1.00),
}


class VizError(ValueError):
    """A drawing that cannot be built honestly, named rather than skipped."""


# ------------------------------------------------------------------- colour
def colour_for_margin(margin_m, floor_m=DEFAULT_FLOOR_M):
    """Green well clear, amber approaching the floor, red inside it.

    `margin_m` is the clearance MINUS nothing: it is the distance the guard
    itself reports. The breakpoints are the floor and twice the floor, so the
    picture changes at the value the arm actually stops at and not at a
    number chosen to look good.

    Returns (r, g, b, a). Pure arithmetic, so the self-test has a real answer
    to check rather than a rendering to squint at.
    """
    if margin_m is None or not _finite(margin_m):
        # UNKNOWN IS NOT SAFE. A missing clearance renders grey-blue and
        # never green: "nobody is disagreeing with me" is not agreement.
        return (0.35, 0.45, 0.60, 0.80)
    if margin_m < 0.0:
        return (1.00, 0.00, 0.00, 1.00)              # inside the person
    if margin_m < floor_m:
        return (1.00, 0.35, 0.00, 1.00)              # inside the floor
    if margin_m < 2.0 * floor_m:
        t = (margin_m - floor_m) / floor_m           # 0 -> 1 across the band
        return (1.0 - 0.8 * t, 0.55 + 0.35 * t, 0.10 + 0.15 * t, 1.00)
    return (0.20, 0.90, 0.25, 1.00)


def _finite(v):
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


# ------------------------------------------------------------------ markers
def _m(ns, mid, mtype, frame="world", **kw):
    d = {"ns": ns, "id": int(mid), "type": mtype, "frame_id": frame,
         "action": "add", "scale": (0.01, 0.01, 0.01),
         "colour": RGBA["text"], "pose": ((0, 0, 0), (0, 0, 0, 1)),
         "points": [], "text": ""}
    d.update(kw)
    return d


def delete_all(ns):
    """EVERY FRAME BEGINS WITH THIS. Ids are assigned per frame, so a frame
    with fewer markers than the last would otherwise leave the difference on
    screen -- a stale arm pose that looks live. `scene_markers` learned this
    the same way."""
    return _m(ns, 0, "deleteall", action="deleteall")


# ---------------------------------------------------------------- 1: states
def state_markers(state, points, label="", frame="world", floor_m=None):
    """One arm chain, drawn as a line strip plus its joints.

    `points` is the chain of link origins in world. `state` is one of
    COMMANDED / ACTUAL / REFUSED. A REFUSED state MUST carry a label -- the
    reason -- and refuses to be built without one, because a red arm on
    screen with no reason attached is exactly the log-reading this replaces.
    """
    if state not in (COMMANDED, ACTUAL, REFUSED):
        raise VizError("unknown state %r" % (state,))
    if state == REFUSED and not str(label).strip():
        raise VizError(
            "a REFUSED pose must carry the reason it was refused. Drawing "
            "the rejected path without the reason string is what the "
            "operator already had: a red arm and a log to go and read.")
    if len(points) < 2:
        raise VizError("a chain needs at least two points, got %d"
                       % len(points))
    out = [_m("srl_" + state, 1, "line_strip", frame=frame,
              colour=RGBA[state], scale=(0.012, 0.0, 0.0),
              points=[tuple(float(v) for v in p) for p in points])]
    for i, p in enumerate(points):
        out.append(_m("srl_" + state, 10 + i, "sphere", frame=frame,
                      colour=RGBA[state], scale=(0.022, 0.022, 0.022),
                      pose=(tuple(float(v) for v in p), (0, 0, 0, 1))))
    if str(label).strip():
        tip = points[-1]
        out.append(_m("srl_" + state + "_label", 1, "text", frame=frame,
                      colour=RGBA[state], scale=(0.0, 0.0, 0.035),
                      text=str(label),
                      pose=((float(tip[0]), float(tip[1]),
                             float(tip[2]) + 0.08), (0, 0, 0, 1))))
    return out


# ------------------------------------------------------- 2: the wearer floor
def wearer_shell(primitives, floor_m=DEFAULT_FLOOR_M, frame="world",
                 enforced_note=""):
    """The body, INFLATED by the floor, so the forbidden volume is visible.

    Drawn from whatever `mount_guard_node.wearer()` is enforcing right now --
    which is `fuse()`'s output, the mannequin with tracked parts substituted
    only where they are CLOSER. Drawing the mannequin while the guard
    enforces a measured body would put a shell on screen that the arm is not
    stopped by, and that is the defect this whole subsystem exists to close.
    """
    if not primitives:
        # AN EMPTY BODY IS A STATEMENT, NOT AN ABSENCE. `wearer_model()`
        # returns [] when the rig is declared empty, and a blank screen is
        # indistinguishable from a crashed node.
        return [_m("srl_wearer", 1, "text", frame=frame,
                   colour=(1.0, 0.6, 0.0, 1.0), scale=(0.0, 0.0, 0.06),
                   text="NO WEARER declared -- the floor is not being "
                        "enforced against anything",
                   pose=((0.0, 0.0, 1.6), (0, 0, 0, 1)))]
    out = []
    for i, prim in enumerate(primitives):
        name, kind, dims, centre, rpy = prim
        d = [float(v) for v in dims]
        if kind in ("capsule", "cylinder"):
            r, h = d[0] + floor_m, d[1]
            scale = (2 * r, 2 * r, h)
            mtype = "cylinder"
        elif kind == "sphere":
            r = d[0] + floor_m
            scale, mtype = (2 * r, 2 * r, 2 * r), "sphere"
        else:                                   # box, and anything box-like
            scale = tuple(v + 2 * floor_m for v in d[:3])
            mtype = "cube"
        out.append(_m("srl_wearer", 100 + i, mtype, frame=frame,
                      colour=RGBA["shell"], scale=scale,
                      pose=(tuple(float(v) for v in centre),
                            _q_from_rpy(rpy))))
    out.append(_m("srl_wearer_note", 1, "text", frame=frame,
                  colour=RGBA["text"], scale=(0.0, 0.0, 0.05),
                  text="wearer +%.0f mm floor%s"
                       % (floor_m * 1000,
                          ("  (%s)" % enforced_note) if enforced_note else ""),
                  pose=((0.0, 0.0, 1.85), (0, 0, 0, 1))))
    return out


def nearest_segment(a, b, margin_m, part_name, arm="", frame="world",
                    floor_m=DEFAULT_FLOOR_M):
    """The one arm segment closest to the wearer, coloured and LABELLED.

    The label names the part -- "L-upperarm", "torso" -- because "clearance
    0.0962" tells an operator nothing they can act on and "0.096 m to the
    wearer's LEFT UPPER ARM" tells them to move an elbow.
    """
    if not str(part_name).strip():
        raise VizError(
            "the nearest segment must name the wearer part it is nearest "
            "to. `moving_chain_to` carries it and drawing the distance "
            "without the part is the number nobody could act on.")
    col = colour_for_margin(margin_m, floor_m)
    mid = tuple((float(x) + float(y)) * 0.5 for x, y in zip(a, b))
    txt = ("%s %.3f m to %s" % (arm, margin_m, part_name)
           if _finite(margin_m)
           else "%s clearance UNKNOWN (to %s)" % (arm, part_name))
    return [
        _m("srl_nearest", 1, "line_strip", frame=frame, colour=col,
           scale=(0.03, 0.0, 0.0),
           points=[tuple(float(v) for v in a), tuple(float(v) for v in b)]),
        _m("srl_nearest", 2, "text", frame=frame, colour=col,
           scale=(0.0, 0.0, 0.04), text=txt,
           pose=((mid[0], mid[1], mid[2] + 0.06), (0, 0, 0, 1))),
    ]


# ------------------------------------------------------------ 3: whole path
def path_markers(samples, clearances=None, frame="world",
                 floor_m=DEFAULT_FLOOR_M, ns="srl_path", why=""):
    """The DENSIFIED sweep, one point per sample, coloured by its own
    clearance. Not the waypoints.

    Defect 2 of 2026-08-21: a pick cleared three poses at 0.4 m and commanded
    a joint-space sweep between them that went through the wearer. Endpoints
    drawn green with an invisible red middle is precisely the picture that
    permitted it, so this refuses a sample list short enough to be waypoints
    pretending to be a path.
    """
    n = len(samples)
    if n < 3:
        raise VizError(
            "a path of %d point(s) is a waypoint list, not a path. The "
            "defect this drawing exists for lived BETWEEN two cleared "
            "poses; densify before drawing." % n)
    if clearances is not None and len(clearances) != n:
        raise VizError(
            "%d samples and %d clearances. A path coloured by a clearance "
            "array of a different length is coloured by the wrong sample, "
            "which is worse than not colouring it." % (n, len(clearances)))
    out = [_m(ns, 1, "line_strip", frame=frame,
              colour=(0.6, 0.6, 0.6, 0.8), scale=(0.006, 0.0, 0.0),
              points=[tuple(float(v) for v in p) for p in samples])]
    worst_i, worst = None, None
    for i, p in enumerate(samples):
        c = None if clearances is None else clearances[i]
        if _finite(c) and (worst is None or float(c) < worst):
            worst, worst_i = float(c), i
        out.append(_m(ns, 100 + i, "sphere", frame=frame,
                      colour=colour_for_margin(c, floor_m),
                      scale=(0.014, 0.014, 0.014),
                      pose=(tuple(float(v) for v in p), (0, 0, 0, 1))))
    if worst_i is not None:
        p = samples[worst_i]
        # THE CHECKER'S OWN WORDS, on the picture. `check_path` already
        # produces a sentence naming the sample and the margin; putting the
        # number on screen without it sends the operator back to the log,
        # which is the state this whole layer exists to end.
        label = "worst on path %.3f m  (sample %d of %d)" % (worst,
                                                             worst_i + 1, n)
        if str(why).strip():
            label += "\n" + str(why)
        out.append(_m(ns + "_worst", 1, "text", frame=frame,
                      colour=colour_for_margin(worst, floor_m),
                      scale=(0.0, 0.0, 0.04), text=label,
                      pose=((float(p[0]), float(p[1]), float(p[2]) + 0.05),
                            (0, 0, 0, 1))))
    return out


# -------------------------------------------------------------- 4: liveness
def liveness_text(rows, frame="world", at=(0.0, -0.55, 1.95)):
    """Joint-state age per arm, session state, bridge errors -- on screen.

    `rows` is a list of (label, value, ok) where `ok` is True, False, or None
    for UNKNOWN. UNKNOWN never renders as OK: a missing measurement is
    reported as missing, which is the same rule `divergence.compare()`
    already follows and the reason it returns a STATUS rather than a number.
    """
    if not rows:
        raise VizError("a liveness panel with no rows is a panel that cannot "
                       "report the one thing it exists for")
    lines = []
    for label, value, ok in rows:
        mark = {True: "OK  ", False: "BAD ", None: "??  "}[ok]
        lines.append("%s%-22s %s" % (mark, label, value))
    bad = any(ok is False for _, _, ok in rows)
    unk = any(ok is None for _, _, ok in rows)
    col = ((1.0, 0.2, 0.2, 1.0) if bad
           else (0.9, 0.8, 0.3, 1.0) if unk
           else (0.3, 0.95, 0.4, 1.0))
    return [_m("srl_liveness", 1, "text", frame=frame, colour=col,
               scale=(0.0, 0.0, 0.045), text="\n".join(lines),
               pose=(tuple(float(v) for v in at), (0, 0, 0, 1)))]


# -------------------------------------------------------------- 5: envelope
def envelope_markers(origin, reaches, frame="world", ns="srl_envelope"):
    """The measured workspace, as three distinct volumes.

    `reaches` maps a unit direction to (distance_m, class) where class is
    REACHABLE / WEARER_LIMITED / OUT_OF_REACH. Drawn as a spoke per
    direction, coloured by class, from the sweep's own start point -- which
    is included because a reach without the point it was measured from is
    meaningless, and this repository has two baselines that disagree purely
    because they walked from different places.
    """
    if not reaches:
        raise VizError("no measured directions; run "
                       "scripts/measure_orientation_cost.py or "
                       "scripts/real_calibration/sweep_workspace.py first. "
                       "An envelope drawn from nothing is a claim about the "
                       "arm that nobody measured.")
    o = tuple(float(v) for v in origin)
    out = []
    for i, (d, (dist, klass)) in enumerate(sorted(reaches.items())):
        if klass not in (REACHABLE, WEARER_LIMITED, OUT_OF_REACH):
            raise VizError("direction %r has class %r, which is not one of "
                           "the three" % (d, klass))
        u = _unit(d)
        tip = tuple(o[k] + u[k] * float(dist) for k in range(3))
        out.append(_m(ns, 200 + i, "line_strip", frame=frame,
                      colour=RGBA[klass], scale=(0.008, 0.0, 0.0),
                      points=[o, tip]))
        out.append(_m(ns, 400 + i, "sphere", frame=frame,
                      colour=RGBA[klass], scale=(0.03, 0.03, 0.03),
                      pose=(tip, (0, 0, 0, 1))))
    out.append(_m(ns + "_origin", 1, "text", frame=frame,
                  colour=RGBA["text"], scale=(0.0, 0.0, 0.04),
                  text="envelope measured FROM (%.3f, %.3f, %.3f)" % o,
                  pose=((o[0], o[1], o[2] - 0.10), (0, 0, 0, 1))))
    return out


# ------------------------------------------------------------------- helpers
def _unit(v):
    n = math.sqrt(sum(float(x) ** 2 for x in v))
    if n < 1e-12:
        raise VizError("zero-length direction")
    return [float(x) / n for x in v]


def _q_from_rpy(rpy):
    r, p, y = (float(v) for v in (rpy or (0.0, 0.0, 0.0)))
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


# ---------------------------------------------------------------- self-test
def self_test(verbose=True):
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-52s %s%s" % (name, "OK" if cond else "*** FAILED ***",
                                    (" -- " + detail) if detail else ""))

    def refuses(name, fn):
        try:
            fn()
            check(name, False, "it did not refuse")
        except VizError:
            check(name, True)

    # -- colour is a known-answer function, not a look
    check("inside the person is pure red",
          colour_for_margin(-0.01)[:3] == (1.0, 0.0, 0.0))
    check("inside the floor is orange, not green",
          colour_for_margin(0.10)[1] < 0.5)
    check("well clear is green", colour_for_margin(0.9)[1] > 0.8)
    check("UNKNOWN clearance is never green",
          colour_for_margin(None)[1] < 0.6
          and colour_for_margin(float("nan"))[1] < 0.6)
    # monotone: more clearance is never redder
    reds = [colour_for_margin(x)[0] for x in
            (0.0, 0.05, 0.15, 0.20, 0.25, 0.30, 0.60)]
    check("colour is monotone in clearance",
          all(reds[i] >= reds[i + 1] - 1e-9 for i in range(len(reds) - 1)),
          "reds %s" % [round(r, 2) for r in reds])
    # the breakpoint IS the floor, not a number that looks nice
    check("the colour breaks at the floor and nowhere else",
          colour_for_margin(DEFAULT_FLOOR_M - 1e-6)[1] < 0.5
          and colour_for_margin(DEFAULT_FLOOR_M + 1e-6)[1] >= 0.5)

    # -- the three states
    chain = [(0.0, 0.0, 1.2), (0.2, 0.1, 1.25), (0.4, 0.2, 1.15)]
    for st in (COMMANDED, ACTUAL):
        ms = state_markers(st, chain)
        check("%s draws a strip and its joints" % st,
              sum(1 for m in ms if m["type"] == "line_strip") == 1
              and sum(1 for m in ms if m["type"] == "sphere") == 3)
    refuses("a REFUSED pose without a reason is refused",
            lambda: state_markers(REFUSED, chain))
    ms = state_markers(REFUSED, chain, "wearer clearance 0.096 m at step 28")
    check("a REFUSED pose carries its reason as text",
          any(m["type"] == "text" and "0.096" in m["text"] for m in ms))
    check("the three states use three different colours",
          len({RGBA[COMMANDED], RGBA[ACTUAL], RGBA[REFUSED]}) == 3)
    refuses("a one-point chain is refused",
            lambda: state_markers(COMMANDED, chain[:1]))

    # -- the shell is INFLATED by the floor, and that is measurable
    prim = [("torso", "box", (0.36, 0.20, 0.55), (0.0, 0.0, 1.30),
             (0, 0, 0))]
    m = wearer_shell(prim, floor_m=0.15)[0]
    check("the shell is the body plus 2x the floor on every axis",
          all(abs(m["scale"][i] - (prim[0][2][i] + 0.30)) < 1e-9
              for i in range(3)),
          "scale %s" % (tuple(round(v, 3) for v in m["scale"]),))
    m0 = wearer_shell(prim, floor_m=0.0)[0]
    check("a zero floor draws the body itself",
          all(abs(m0["scale"][i] - prim[0][2][i]) < 1e-9 for i in range(3)))
    check("an empty body SAYS so rather than drawing nothing",
          any("NO WEARER" in x["text"] for x in wearer_shell([])))

    # -- nearest segment must name the part
    refuses("an unnamed nearest part is refused",
            lambda: nearest_segment((0, 0, 1), (0, 0.1, 1), 0.09, ""))
    ms = nearest_segment((0, 0, 1), (0, 0.1, 1), 0.0962, "L-upperarm",
                         arm="left")
    check("the nearest segment names the part and the distance",
          any("L-upperarm" in x["text"] and "0.096" in x["text"]
              for x in ms if x["type"] == "text"))
    check("a breaching segment renders red, not amber",
          nearest_segment((0, 0, 1), (0, 0.1, 1), -0.01,
                          "torso")[0]["colour"][:3] == (1.0, 0.0, 0.0))

    # -- the path
    pts = [(0.0, 0.0, 1.2 + 0.01 * i) for i in range(12)]
    refuses("three waypoints pretending to be a path are refused",
            lambda: path_markers(pts[:2]))
    refuses("a clearance array of the wrong length is refused",
            lambda: path_markers(pts, clearances=[0.2] * 5))
    cl = [0.30] * 12
    cl[7] = 0.0405
    ms = path_markers(pts, clearances=cl)
    check("the path draws one point per SAMPLE",
          sum(1 for m in ms if m["type"] == "sphere") == 12)
    check("the worst point on the path is called out by index",
          any("sample 8 of 12" in m["text"] for m in ms
              if m["type"] == "text"))
    check("the worst point is red even though both ends are green",
          any(m["colour"][:3] == (1.0, 0.35, 0.0) or m["colour"][0] > 0.9
              for m in ms if m["type"] == "sphere"))
    ms = path_markers(pts, clearances=cl,
                      why="wearer clearance 0.0405 m at step 8/12, under "
                          "the 0.150 m floor")
    check("the checker's own reason is drawn beside the worst point",
          any("under the 0.150 m floor" in m["text"] for m in ms
              if m["type"] == "text"))

    # -- liveness
    refuses("an empty liveness panel is refused", lambda: liveness_text([]))
    t = liveness_text([("left joint states", "0.04 s", True),
                       ("right joint states", "2.41 s", False)])[0]
    check("a bad liveness row turns the whole panel red",
          t["colour"][:3] == (1.0, 0.2, 0.2))
    t = liveness_text([("kortex session", "unknown", None)])[0]
    check("an UNKNOWN liveness row is never green",
          t["colour"][1] < 0.9 and "??" in t["text"])

    # -- envelope
    refuses("an envelope with no measured direction is refused",
            lambda: envelope_markers((0, 0, 1), {}))
    refuses("an envelope class outside the three is refused",
            lambda: envelope_markers((0, 0, 1),
                                     {(1, 0, 0): (0.4, "probably")}))
    ms = envelope_markers((0.4, 0.175, 1.12), {
        (1, 0, 0): (0.40, WEARER_LIMITED),
        (0, 1, 0): (0.25, OUT_OF_REACH),
        (0, 0, 1): (0.80, REACHABLE)})
    check("the envelope draws the three classes in three colours",
          len({m["colour"] for m in ms if m["type"] == "line_strip"}) == 3)
    check("the envelope states the point it was measured FROM",
          any("measured FROM (0.400, 0.175, 1.120)" in m["text"]
              for m in ms if m["type"] == "text"))

    # -- every frame can be cleared
    check("delete_all is available and is an action, not a colour",
          delete_all("srl_path")["action"] == "deleteall")

    if verbose:
        print("rviz_master self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if self_test() else 1)
