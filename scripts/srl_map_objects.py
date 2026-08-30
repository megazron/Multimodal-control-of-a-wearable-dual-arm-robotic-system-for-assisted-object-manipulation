#!/usr/bin/env python3
"""Name what the robot found, in one word each, from the map it measured.

    python3 scripts/srl_map_objects.py

WHY A NAME AND NOT AN INDEX. The window asked the operator for "object 0" --
a number they had to get from a separate LIST button, remember, and type into
a spin box before pressing pick. Nothing on screen connected the number to the
thing on the table, so picking the wrong one was a typo away and looked
exactly like picking the right one.

The map already carries what a name needs: `mean_bgr` (the colour the camera
actually saw, not a colour anybody declared) and `width_m` across the object's
narrowest axis. So the name is derived, never invented, and two objects that
genuinely look the same get `green` and `green 2` rather than a made-up
distinction.

GRASPABILITY IS CARRIED WITH THE NAME. The map says `graspable` and, when
false, `why` -- "139 mm across its narrowest axis and the jaws close on 85 mm".
A list that offers an ungraspable object as a button is a list that produces a
refusal the operator could have been shown up front.
"""
import json
import os
import sys

WS = "/home/gausms/kortex_ws"
MAP = os.path.join(WS, "recordings/baselines/world_map.json")

# (name, B, G, R). Compared in BGR because that is what the map stores.
_PALETTE = [
    ("red", (40, 40, 200)), ("green", (60, 170, 60)), ("blue", (190, 70, 60)),
    ("yellow", (60, 200, 210)), ("orange", (40, 120, 220)),
    ("white", (215, 215, 215)), ("grey", (128, 128, 128)),
    ("black", (35, 35, 35)), ("brown", (60, 90, 130)),
]


def colour_name(bgr):
    """Nearest palette name to a measured mean BGR."""
    if not bgr or len(bgr) < 3:
        return "object"
    b, g, r = float(bgr[0]), float(bgr[1]), float(bgr[2])
    best, bn = None, "object"
    for name, (pb, pg, pr) in _PALETTE:
        d = (b - pb) ** 2 + (g - pg) ** 2 + (r - pr) ** 2
        if best is None or d < best:
            best, bn = d, name
    return bn


def load(path=MAP):
    """[{index, name, graspable, why, width_mm, centre, colour}], or []."""
    if not os.path.exists(path):
        return []
    try:
        with open(path) as fh:
            doc = json.load(fh)
    except Exception:                                         # noqa: BLE001
        return []
    out, used = [], {}
    for i, o in enumerate(doc.get("objects") or []):
        col = colour_name(o.get("mean_bgr"))
        used[col] = used.get(col, 0) + 1
        # SECOND ONE GETS A NUMBER, the first does not. "green" and "green 2"
        # reads better than "green 1" and "green 2" when there is only one.
        name = col if used[col] == 1 else "%s %d" % (col, used[col])
        out.append(dict(
            index=i, name=name, colour=col,
            graspable=bool(o.get("graspable")),
            why=o.get("why") or "",
            width_mm=round(float(o.get("width_m") or 0.0) * 1000, 1),
            centre=[round(float(v), 4) for v in (o.get("centre") or [])],
            n_points=int(o.get("n_points") or 0)))
    return out


def age_s(path=MAP):
    import time
    return None if not os.path.exists(path) else time.time() - os.path.getmtime(path)


def main():
    objs = load()
    if not objs:
        print("no map yet -- run the scan first "
              "(recordings/baselines/world_map.json)")
        return 1
    a = age_s()
    print("%d object(s), map is %.0f s old" % (len(objs), a or 0))
    print("%-10s %-4s %9s %-10s %s"
          % ("name", "idx", "width_mm", "graspable", "why not"))
    for o in objs:
        print("%-10s %-4d %9.1f %-10s %s"
              % (o["name"], o["index"], o["width_mm"],
                 "yes" if o["graspable"] else "NO", o["why"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
