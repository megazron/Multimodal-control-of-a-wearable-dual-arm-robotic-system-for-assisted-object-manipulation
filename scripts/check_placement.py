#!/usr/bin/env python3
"""Did each cube end on its OWN plane, the WRONG plane, or off both?

    python3 scripts/check_placement.py [clip_dir ...]

THIS IS T1's ACTUAL SUCCESS CRITERION and it should not depend on frame
sampling. Sixteen sampled frames cannot tell whether four cubes ended
distinctly on their targets: a cube is 40 mm, a plane is 140 x 100 mm, and at
5-11 fps the moment of release may fall between frames entirely. Colour-pixel
counting -- how the clip verifier judges VISIBILITY -- is the wrong instrument
for a question about POSITION.

So the verdict comes from `scene_events.json`, which records every GRASPED and
RELEASED event with the item and the position it happened at. The final
position of each cube is the position of its LAST release (or its start
position if it was never grasped, which is itself a failure worth naming).

THE THREE VERDICTS ARE NOT TWO. "On its own plane" and "off both" are the
obvious pair; WRONG PLANE is the one that matters and the one a pass/fail
column would hide. T1's whole scoreable content is colour matching -- blue
cube to blue plane -- so a cube placed neatly and confidently on the wrong
mat is a task FAILURE that looks like a success in every other metric:
travel, grasp count, release count and clearance are all identical.

COLOUR PAIRING comes from the task definition (cubes 0 and 2 blue -> plane 0;
1 and 3 green -> plane 1), not from anything in the recording, so a recording
cannot quietly redefine what counts as correct.
"""
import glob
import json
import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments", "experiments",
                                "abc"))


def plane_bounds(plane_y=None):
    """Bounds of each plane. `plane_y` OVERRIDES the current layout.

    NECESSARY, NOT A CONVENIENCE. scene_events.json records fixtures by NAME
    ("plane_blue") and not by position, so a clip carries no record of where
    its own targets were. Judging a clip against whatever the module says
    TODAY silently re-scores it against a layout it never ran in: run against
    the current bounds, every cube in every pre-move clip reads OFF BOTH --
    39 of 39 -- purely because the planes were later moved from y 0.145 to
    y 0.320. That is the measuring instrument, not the robot.

    Clips recorded from here on will match the module. For older ones, pass
    the y they were recorded at.
    """
    import clip_scene as CS
    import msc_clip_tasks as MCT
    out = []
    for i, (px, py) in enumerate(MCT.T1_PLANES):
        if plane_y is not None:
            py = plane_y
        out.append((i, px - CS.PLANE_W / 2.0, px + CS.PLANE_W / 2.0,
                    py - CS.PLANE_D / 2.0, py + CS.PLANE_D / 2.0))
    return out


def own_plane(cube_index):
    """Cubes 0 and 2 are blue -> plane 0; 1 and 3 green -> plane 1."""
    return 0 if cube_index % 2 == 0 else 1


def classify(pos, planes, cube_index):
    x, y = pos[0], pos[1]
    on = [i for i, xl, xh, yl, yh in planes if xl <= x <= xh and yl <= y <= yh]
    want = own_plane(cube_index)
    if want in on:
        return "OWN", want
    if on:
        return "WRONG", on[0]
    return "OFF", None


def final_positions(ev):
    """Last RELEASE position per item; None if it was never released."""
    out, grabbed = {}, set()
    for e in ev.get("events", []):
        it = e.get("item")
        if not it:
            continue
        if e.get("ev") == "GRASPED":
            grabbed.add(it)
        if e.get("ev") == "RELEASED" and e.get("at"):
            out[it] = list(e["at"])
    return out, grabbed


def report(d, plane_y=None):
    f = os.path.join(d, "scene_events.json")
    if not os.path.exists(f):
        return None
    ev = json.load(open(f))
    if ev.get("task") == "t1s2":
        # STAGE 2 HAS ITS OWN TARGETS. Its cubes are per-arm and drawn from
        # stage2_targets(), not from T1_PLANES, so scoring it against T1's
        # two planes reports every cube OFF BOTH -- 19 of 19 -- for a reason
        # that has nothing to do with where the arm put them. Refusing is
        # correct; a wrong number is worse than no number.
        return [("(stage 2)", "NOT SCORED",
                 "own targets, not T1_PLANES", None)]
    if ev.get("task") != "t1":
        return None
    planes = plane_bounds(plane_y)
    finals, grabbed = final_positions(ev)
    rows = []
    # `items` is a LIST OF DICTS in some recordings and a dict in others, so
    # take names defensively rather than assuming either shape -- a crash
    # here would report "no placement data" for a clip that has it.
    raw = ev.get("items") or []
    if isinstance(raw, dict):
        items = list(raw)
    else:
        items = [(o.get("name") or o.get("id")) if isinstance(o, dict) else o
                 for o in raw]
    names = [x for x in list(finals) + items if x]
    cubes = sorted({x for x in names if "cube" in str(x)})
    for it in cubes:
        digits = "".join(c for c in str(it) if c.isdigit())
        idx = int(digits) if digits else 0
        pos = finals.get(it)
        if pos is None:
            rows.append((it, "NEVER RELEASED",
                         "grasped" if it in grabbed else "never grasped",
                         None))
            continue
        verdict, which = classify(pos, planes, idx)
        rows.append((it, verdict, "plane_%s" % which if which is not None
                     else "-", pos))
    return rows


def main():
    argv = sys.argv[1:]
    plane_y = None
    if "--plane-y" in argv:
        i = argv.index("--plane-y")
        plane_y = float(argv[i + 1])
        del argv[i:i + 2]
    dirs = argv
    if not dirs:
        dirs = sorted(set(os.path.dirname(p) for p in glob.glob(
            os.path.join(WS, "recordings", "**", "scene_events.json"),
            recursive=True)))
    print("=" * 74)
    print("T1 PLACEMENT -- from scene_events.json, NOT from frames")
    print("=" * 74)
    planes = plane_bounds(plane_y)
    if plane_y is not None:
        print("   plane y OVERRIDDEN to %.3f -- judging clips "
              "against the layout they RAN in" % plane_y)
    for i, xl, xh, yl, yh in planes:
        print("   plane_%d  x %.3f..%.3f  y %.3f..%.3f" % (i, xl, xh, yl, yh))
    any_rows = 0
    tally = {"OWN": 0, "WRONG": 0, "OFF": 0, "NEVER RELEASED": 0}
    for d in dirs:
        rows = report(d, plane_y)
        if not rows:
            continue
        any_rows += 1
        print("\n%s" % os.path.relpath(d, WS))
        for it, verdict, which, pos in rows:
            if verdict == "NOT SCORED":
                print("   %-16s %-14s %s" % (it, verdict, which))
                continue
            tally[verdict] = tally.get(verdict, 0) + 1
            where = ("(%.3f, %.3f)" % (pos[0], pos[1])) if pos else ""
            print("   %-16s %-14s %-9s %s" % (it, verdict, which, where))
    if not any_rows:
        print("\nNO T1 CLIPS FOUND with scene_events.json.")
        return 2
    print("\nTOTAL  own %d   WRONG %d   off %d   never released %d"
          % (tally["OWN"], tally["WRONG"], tally["OFF"],
             tally["NEVER RELEASED"]))
    return 0 if tally["WRONG"] == 0 and tally["OFF"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
