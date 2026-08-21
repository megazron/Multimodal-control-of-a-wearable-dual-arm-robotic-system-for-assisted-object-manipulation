#!/usr/bin/env python3
"""Rebuild a workspace record from the per-point .npz captures.

Needed because `workspace.json` used to be written only at the END of a run,
so any interruption threw the whole measurement away -- while the .npz files,
written per point, survived. Every field the summary held is in them:
measured joints, commanded joints, the 4x4 EE pose, link positions,
clearance and a timestamp.

Points are recovered here rather than re-measured. Re-driving a real arm to
collect data you already have on disk is the expensive way to be tidy.
"""
import glob
import json
import os
import sys

import numpy as np


def main():
    d = sys.argv[1]
    arm = sys.argv[2] if len(sys.argv) > 2 else "right"
    files = [f for f in sorted(glob.glob(os.path.join(d, arm, "*.npz")))
             if "background" not in f]
    if not files:
        sys.exit("no captures in %s/%s" % (d, arm))
    pts = []
    for f in files:
        z = np.load(f, allow_pickle=True)
        base = os.path.basename(f)[:-4]
        nm, _, step = base.rpartition("_")
        rec = dict(dir=nm, step=int(step), file=base,
                   ee=[float(v) for v in z["ee"]],
                   q_meas=[float(v) for v in z["q"]])
        if "q_cmd" in z.files:
            rec["q_cmd"] = [float(v) for v in z["q_cmd"]]
        if "ee_pose" in z.files:
            rec["ee_pose_4x4"] = [[float(x) for x in r] for r in z["ee_pose"]]
        if "clearance" in z.files:
            rec["clearance_m"] = float(z["clearance"])
        if "link_names" in z.files and "link_xyz" in z.files:
            rec["link_positions"] = {
                str(n): [float(v) for v in p]
                for n, p in zip(z["link_names"], z["link_xyz"])}
        if "t_wall" in z.files:
            rec["t_wall"] = float(z["t_wall"])
        if "K" in z.files:
            rec["K"] = [float(v) for v in z["K"]]
        pts.append(rec)

    by_dir = {}
    for r in pts:
        by_dir.setdefault(r["dir"], []).append(r)
    dirs = {}
    for nm, rs in by_dir.items():
        rs.sort(key=lambda x: x["step"])
        far = max(rs, key=lambda x: x["step"])
        dirs[nm] = dict(points=len(rs), last_step=far["step"],
                        furthest_ee=far["ee"],
                        note="RECOVERED from captures; the run was "
                             "interrupted so the stop REASON was never "
                             "written and is not known for this direction")
    out = dict(arm=arm, recovered=True, n_points=len(pts),
               directions=dirs, points=pts)
    path = os.path.join(d, "recovered_%s.json" % arm)
    json.dump(out, open(path, "w"), indent=1)
    print("recovered %d points across %d directions -> %s"
          % (len(pts), len(dirs), path))
    for nm, v in sorted(dirs.items()):
        print("   %-8s %d points, furthest EE %s"
              % (nm, v["points"], np.round(v["furthest_ee"], 3).tolist()))
    print("\nNOTE: stop reasons are NOT recoverable from captures -- only the "
          "run summary held them, and that is what the interruption lost.")


if __name__ == "__main__":
    main()
