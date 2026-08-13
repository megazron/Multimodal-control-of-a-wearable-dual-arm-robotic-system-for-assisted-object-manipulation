#!/usr/bin/env python3
"""NAMED PLACES -> A POSE THAT WAS MEASURED REACHABLE. Closes the MOVE_TO gap.

    from srl_autonomy.named_places import resolve, Unreachable

`voice_intent` has parsed "move to the front centre" since it was written, and
`NAMED_PLACES` there says which places are reachable and why. What was missing
is the other half: nothing turned a reachable named place into a POSE, so
`autonomy_executive` fell through to "I don't know how to 'goto'" and the
system refused a command it was designed to accept.

EVERY POSE HERE COMES OUT OF THE SURVEY, and that is the whole point. The
reachable region on this platform is nothing like the region a person expects
-- the two arms' sets are disjoint and the front centre is empty -- so a named
place resolved to a plausible-looking coordinate would be a guess wearing a
measurement's name. `recordings/baselines/work_surface_region.json` holds
every (x, y) that actually solved over the FULL pick path, and nothing else is
used.

THE CENTROID IS SNAPPED TO A REAL CELL, and that is not fussiness. The region
is not convex: at 50 mm resolution the right arm's cells filled 62% of their
own bounding box, so the centroid of the set can easily be a point that was
never tested and does not solve. Snapping to the nearest cell CENTRE returns a
point that was measured, which is the only kind this file is allowed to
return.

IT RAISES RATHER THAN RETURNING A FALLBACK. No survey, a survey done at the
grasp pose only, or a place that is not reachable all raise with the reason.
A fallback would be the guess this module exists to prevent.
"""

import json
import math
import os

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
REGION_FILE = os.path.join(WS, "recordings", "baselines",
                           "work_surface_region.json")


class SurveyUnavailable(RuntimeError):
    """No measured region to resolve against."""


class Unreachable(RuntimeError):
    """The place is real, named, and cannot be reached. Carries the number."""


def _region(path=None):
    p = path or REGION_FILE
    if not os.path.exists(p):
        raise SurveyUnavailable(
            "no surveyed region at %s -- run "
            "scripts/survey_work_surface.py --full-path. I will not invent a "
            "position for a named place." % p)
    d = json.load(open(p))
    if not d.get("full_path"):
        raise SurveyUnavailable(
            "%s was surveyed at the GRASP POSE ONLY. A cell that can be "
            "reached is not a cell that can be WORKED, and moving somewhere "
            "is the whole of this command." % p)
    return d


def _centroid_cell(cells):
    """The measured cell nearest the set's centroid.

    NOT the centroid itself. The region is not convex, so its centroid can
    lie in a hole -- a point that was never tested and need not solve.
    """
    cx = sum(c[0] for c in cells) / len(cells)
    cy = sum(c[1] for c in cells) / len(cells)
    return min(cells, key=lambda c: math.hypot(c[0] - cx, c[1] - cy))


def places(path=None):
    """Every place this resolver knows, and whether it is reachable.

    Built from the survey so it cannot drift from it. `voice_intent`'s
    NAMED_PLACES carries the same names for the PARSER's benefit; this is the
    geometry, and the test below requires the two to agree on reachability.
    """
    d = _region(path)
    cells = {a: [tuple(c) for c in d["cells"].get(a, [])]
             for a in ("left", "right")}
    z = float(d["z"])
    out = {}
    for arm in ("left", "right"):
        name = "%s side" % arm
        if cells[arm]:
            c = _centroid_cell(cells[arm])
            out[name] = dict(reachable=True, arm=arm,
                             position=[round(c[0], 4), round(c[1], 4), z],
                             n_cells=len(cells[arm]))
        else:
            out[name] = dict(
                reachable=False, arm=arm,
                why="the survey has no reachable cell for the %s arm" % arm)
    # THE FRONT CENTRE, and it is a measurement rather than an opinion.
    front = [c for c in cells["left"] + cells["right"] if abs(c[0]) <= 0.10]
    near = min((abs(c[0]) for c in cells["left"] + cells["right"]),
               default=None)
    out["front centre"] = dict(
        reachable=bool(front),
        arm=None,
        why=None if front else
        ("the front centre is not reachable by either arm: 0 of %d surveyed "
         "cells at |x| <= 0.10, and the nearest reachable x is %.2f"
         % (len(cells["left"]) + len(cells["right"]),
            near if near is not None else float("nan"))))
    out["front center"] = out["front centre"]
    # HOME is a JOINT-SPACE pose, not a point on the work plane. Returned
    # without a position on purpose: a caller that wants to go home must use
    # the home joint values, and inventing a Cartesian "home" would be a
    # third definition of a pose this project already has exactly one of.
    out["home"] = dict(reachable=True, arm=None, position=None,
                       joint_space=True)
    return out


def resolve(place, path=None):
    """A named place -> dict(position, arm, ...). Raises rather than guesses."""
    known = places(path)
    spec = known.get(str(place).strip().lower())
    if spec is None:
        raise Unreachable(
            "I don't know a place called %r. I know: %s"
            % (place, ", ".join(sorted(k for k in known
                                       if k != "front center"))))
    if not spec["reachable"]:
        raise Unreachable(spec["why"])
    return spec
