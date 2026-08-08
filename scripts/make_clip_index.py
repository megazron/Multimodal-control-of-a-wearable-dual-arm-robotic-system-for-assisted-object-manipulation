#!/usr/bin/env python3
"""Build recordings/verification/INDEX.md -- every clip, and what it SHOULD show.

    python3 scripts/make_clip_index.py

Scans the per-run summary.json files rather than index.json: the sweep runs
one task per invocation and each invocation rewrites index.json, so that file
only ever holds the last task. Scanning the tree is the only complete view.

The "what it should show" column is the point of this file. A clip index that
only lists filenames tells a reviewer nothing -- they still have to work out
what correct looks like before they can tell whether they are seeing it.
"""
import glob
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/verification")

# What a correct clip looks like, per task. Written from the protocol, not
# from the recording -- otherwise it would just describe whatever happened.
EXPECT = {
    "t2": ("The FILL arm picks at the pick point, carries across, and opens "
           "over the container opening (green square). The HOLD arm stays "
           "put the whole time. The two arms never swap sides."),
    "t3": ("Both grippers rise together with the brown tray between them "
           "STRAIGHT and level -- |dz| should stay near 0 mm. Motion is a "
           "VERTICAL lift; nothing travels toward the wearer."),
    "t5": ("One arm (right, -x side) reaches the cradle, lifts, carries "
           "inboard toward the wearer and stops at the receive point (green "
           "star) at the wearer's side. The other arm never moves."),
    "t6": ("Same paths as T3 but the object is a SLING: it sags between the "
           "grippers. The ball sits at the lowest point and stays brown "
           "while retained; it turns RED if the sag drops below the ball "
           "diameter (separation past 340 mm)."),
    "t7": ("Each arm traces a small closed loop about its own centre -- a "
           "Lissajous on an 80 mm sphere. The two arms are INDEPENDENT. In "
           "the B1/B2 baselines only ONE arm moves."),
    "t8": ("The arm reaches out to a target (green star) far on its own "
           "side, at a position that is only reachable because the wearer "
           "has repositioned. The clip shows the arm AFTER the reposition."),
    "t9": ("The arm traces a circle whose radius is the sway amplitude. "
           "This is the arm HOLDING a world-fixed point while the base moves "
           "underneath it -- in the arm's own frame the target orbits. Radius "
           "should visibly grow across S1->S4 (0, 20, 60, 100 mm)."),
}
COND = {
    "direct": "no smoothing -- the command is the raw path",
    "assisted": "30% smoothing on the command, standing in for assistance",
    "shared": "55% smoothing, the most machine-shaped motion",
}


def main():
    rows = []
    for p in sorted(glob.glob(os.path.join(OUT, "*/*/*/summary.json"))):
        s = json.load(open(p))
        d = os.path.dirname(p)
        s["dir"] = os.path.relpath(d, ROOT)
        s["has_clip"] = os.path.exists(os.path.join(d, "clip.mp4"))
        s["has_plot"] = os.path.exists(os.path.join(d, "plot_metrics.png"))
        s["has_bag"] = os.path.isdir(os.path.join(d, "bag"))
        rows.append(s)

    tasks = sorted({r["task"] for r in rows})
    L = []
    L.append("# VERIFICATION CLIP INDEX\n")
    L.append("Every task x scenario x autonomy condition, driven in sim and "
             "recorded. **%d clips.**\n" % len(rows))
    L.append("""
## How to review one

Each folder holds `clip.mp4`, `plot_metrics.png`, `summary.json` and a
rosbag2 in `bag/`. The clip has two panels:

* **left** — 3-D view, wearer drawn as the collision primitives from
  `human_backpack.xacro`;
* **right** — **front view** (x-z), wearer facing you. This is the panel to
  judge from: the 3-D view cannot show whether two grippers are level, which
  is the whole of the T3/T6 metric.

The title bar carries live **minimum clearance** and, for T3/T6, live
**|dz|** between the grippers.

**Read the front view knowing y is projected away.** The wearer outline and
the arms overlap on screen whenever the arms are in front of the body — which
is where the tasks are. Clearance in the title bar is the real 3-D number;
trust it over the picture.

**The first ~1 s of every clip is the approach from home**, tagged
`[approach]` in the title. It is included because it is the largest motion and
the most collision-relevant, and it is **excluded from the tracking metric**
(the command is deliberately far ahead of the arm during it). `summary.json`
reports both: `tracking_rms_mm` (task phase) and
`tracking_rms_incl_approach_mm`.

## What the conditions are, honestly

`direct`, `assisted` and `shared` here differ **only in how much the commanded
path is smoothed** (%s). **The autonomy stack did not run.** These clips
verify GEOMETRY, MOTION and CLEARANCE for every scenario; they are not a
measurement of assistance, and no autonomy claim should be read off them.
""" % ", ".join("%s: %s" % (k, v) for k, v in COND.items()))

    # ---- headline checks
    through = [r for r in rows if r["passes_through_wearer"]]
    low = [r for r in rows if r["below_real_floor"]]
    still = [r for r in rows if r["ee_travel_m"] < 0.05]
    L.append("\n## Automatic checks across all %d clips\n" % len(rows))
    L.append("| check | result |")
    L.append("| --- | --- |")
    L.append("| any arm link inside the wearer | **%d** %s |"
             % (len(through), "" if not through else
                "-- " + ", ".join("%s/%s/%s" % (r["task"], r["scenario"],
                                                r["condition"])
                                  for r in through[:5])))
    L.append("| below the 120 mm `real_robot` clearance floor | **%d of %d** |"
             % (len(low), len(rows)))
    L.append("| clip shows no motion (EE travel < 50 mm) | **%d** |" % len(still))
    L.append("| clips / plots / bags present | %d / %d / %d |"
             % (sum(r["has_clip"] for r in rows),
                sum(r["has_plot"] for r in rows),
                sum(r["has_bag"] for r in rows)))

    for t in tasks:
        rs = [r for r in rows if r["task"] == t]
        L.append("\n---\n\n## %s — %d clips\n" % (t.upper(), len(rs)))
        L.append("**What it should show.** %s\n" % EXPECT.get(t, "—"))
        L.append("| scenario | cond | EE travel | min clearance | tracking RMS "
                 "| clip |")
        L.append("| --- | --- | --- | --- | --- | --- |")
        for r in sorted(rs, key=lambda r: (r["scenario"], r["condition"])):
            flag = ""
            if r["passes_through_wearer"]:
                flag = " **THROUGH WEARER**"
            elif r["below_real_floor"]:
                flag = " *(below 120 mm floor)*"
            L.append("| %s | %s | %.3f m | %.3f m%s | %.1f mm | `%s/clip.mp4` |"
                     % (r["scenario"], r["condition"], r["ee_travel_m"],
                        r["min_clearance_m"], flag, r["tracking_rms_mm"],
                        r["dir"]))
        note = rs[0].get("note", "")
        if note:
            L.append("\n%s\n" % note)

    L.append("""
---

## Regenerate

```bash
ros2 launch srl_moveit_config demo.launch.py      # sim, arms AT HOME
python3 scripts/record_verification.py --all
python3 scripts/make_clip_index.py
```

The recorder refuses nothing about home pose, but every scenario it replays
was verified from home — see `scripts/audit_scenario_reachability.py`.
""")
    p = os.path.join(OUT, "INDEX.md")
    open(p, "w").write("\n".join(L) + "\n")
    print("wrote %s  (%d clips, %d tasks)" % (p, len(rows), len(tasks)))
    print("  through wearer: %d   below floor: %d   no motion: %d"
          % (len(through), len(low), len(still)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
