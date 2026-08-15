#!/usr/bin/env python3
"""Build the ONE region file from the clearance surveys, with provenance.

    python3 scripts/merge_work_surface_region.py

WHY A MERGE AND NOT ONE RUN. The region now spans |x| = 0.20 to 1.00 at 25 mm
over the whole pick path with an FK clearance call per waypoint, and a single
run of that does not finish inside this machine's ~20 minute process
lifetime -- the same reason the 2026-08-13 survey was merged from two runs.
Every input here used the same script, the same step, the same z, the same
scene and the same controls, and each recorded its own controls in its own
file; the merge refuses any input whose controls did not pass.

WHAT CHANGES ABOUT THE FILE ITSELF, and it is the point of the exercise:

  * `cells` is still the IK-reachable set, so nothing that reads the old key
    silently changes meaning;
  * `clear_cells` is new -- the cells that ALSO keep the 150 mm wearer
    clearance floor -- and it is what the marking and stage 2's sampler now
    use. The two differ by more than a trim: on the left arm 28 of 122 cells
    inside |x| <= 0.72 are IK-reachable and inside the floor, one of them
    with the tube 2.7 mm INSIDE the person.
  * the box is no longer |x| <= 0.70. That bound was the SURVEY BOX, not the
    arm: the direction walk puts the outboard limit at |x| = 0.975, and the
    300 mm beyond the old box is 100% clear of the wearer. The region a
    participant is shown has been understating the usable space by more than
    it was overstating it.
"""
import glob
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.join(ROOT, "recordings", "baselines")
OUT = os.path.join(BASE, "work_surface_region.json")
SOURCES = ["clearance_region_left_fine.json",
           "clearance_region_outboard.json",
           "clearance_region_inboard.json"]


def main():
    cells = {"left": set(), "right": set()}
    clear = {"left": set(), "right": set()}
    used, z, step, floor = [], None, None, None
    for name in SOURCES:
        p = os.path.join(BASE, name)
        if not os.path.exists(p):
            print("MISSING INPUT %s -- refusing to write a region file that "
                  "is short a survey without saying so." % name)
            return 2
        d = json.load(open(p))
        if d.get("refused"):
            print("%s REFUSED at run time (a control failed). Not merging it."
                  % name)
            return 3
        if z is None:
            z, step, floor = d["z"], d["step"], d["floor"]
        elif (d["z"], d["step"], d["floor"]) != (z, step, floor):
            print("%s was run at z=%s step=%s floor=%s against %s/%s/%s -- "
                  "refusing to merge surveys that do not describe the same "
                  "measurement." % (name, d["z"], d["step"], d["floor"],
                                    z, step, floor))
            return 4
        for arm in ("left", "right"):
            for c in d["per_arm"][arm]["ik_cells"]:
                cells[arm].add((round(c[0], 4), round(c[1], 4)))
            for c in d["per_arm"][arm]["clear_cells"]:
                clear[arm].add((round(c[0], 4), round(c[1], 4)))
        used.append(dict(file=name, x_range=d["x_range"], y_range=d["y_range"],
                         controls=d["controls"], ik_calls=d["ik_calls"]))

    out = dict(
        z=z, step=step, repeats=1, full_path=True, scene="t1",
        clearance_floor_m=floor, per_arm_pad_offset=True,
        cells={a: sorted([list(c) for c in cells[a]]) for a in cells},
        clear_cells={a: sorted([list(c) for c in clear[a]]) for a in clear},
        both=[], either=[], front_centre=[],
        sources=used,
        note="MERGED by scripts/merge_work_surface_region.py from clearance "
             "surveys run 2026-08-15. Two changes from every earlier version "
             "of this file. (1) `clear_cells` is new: the cells that keep the "
             "%.2f m wearer clearance floor as well as solving IK. The floor "
             "is NOT an IK constraint -- the SRDF excludes the wearer pairs a "
             "shoulder-mounted arm actually threatens, so collision-aware IK "
             "returns poses with the tube inside the person -- so it has to "
             "be measured separately and it never had been. (2) the box now "
             "runs to |x| = 1.00; the old |x| <= 0.70 was where the survey "
             "stopped looking, not where the arm stops." % floor)
    both = sorted(set(map(tuple, out["clear_cells"]["left"]))
                  & set(map(tuple, out["clear_cells"]["right"])))
    out["both"] = [list(c) for c in both]
    ei = sorted(set(map(tuple, out["clear_cells"]["left"]))
                | set(map(tuple, out["clear_cells"]["right"])))
    out["either"] = [list(c) for c in ei]
    out["front_centre"] = [list(c) for c in ei if abs(c[0]) <= 0.10]

    json.dump(out, open(OUT, "w"), indent=2)
    for arm in ("left", "right"):
        ik, cl = out["cells"][arm], out["clear_cells"][arm]
        X = [c[0] for c in cl]
        Y = [c[1] for c in cl]
        print("%-5s IK %4d cells   CLEAR OF THE FLOOR %4d   "
              "x %+.3f..%+.3f  y %.3f..%.3f"
              % (arm, len(ik), len(cl), min(X), max(X), min(Y), max(Y)))
    print("both arms %d   front centre (|x| <= 0.10) %d"
          % (len(out["both"]), len(out["front_centre"])))
    print("-> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
