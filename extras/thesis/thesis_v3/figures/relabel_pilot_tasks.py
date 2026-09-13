#!/usr/bin/env python3
"""Correct the pilot task labels in the derived data files.

The pilot ran TWO tasks -- pick and place, and target reaching -- but the
session folders were named at the console with three different phrases
("object tracking", "target reaching", "position matching/reaching"), and
the first analysis pass took the folder names at face value and reported
three tasks. The operator who ran the sessions confirmed on 2026-09-07 that
"object tracking" sessions were the pick-and-place task and that "position
matching/reaching" and "target reaching" were the same target-reaching
task. This script applies that mapping to every derived data file the
figure scripts read, and recomputes the participant x task pairs, so every
figure and table downstream says the same thing.

Idempotent: running it twice changes nothing the second time.

    python3 extras/thesis/thesis_v3/figures/relabel_pilot_tasks.py
"""

import collections
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "pilot", "data")
TRAJ = os.path.join(HERE, "gallery", "trajectories", "trajectories.json")

MAP = {"Object tracking": "Pick and place",
       "Target reaching": "Target reaching",
       "Position matching": "Target reaching",
       "Position reaching": "Target reaching",
       "Pick and place": "Pick and place",
       "Unspecified": "Unspecified"}
PAIR_METRICS = ["duration_s", "ee_path_m", "clutch_engagements", "active_control_frac",
                "mean_lag_mm", "tracking_ok_frac", "controller_path_m"]


def relabel_list(items):
    n = 0
    for s in items:
        new = MAP[s["task"]]
        if new != s["task"]:
            s["task"] = new
            n += 1
    return n


def recompute_pairs(sessions):
    groups = collections.defaultdict(lambda: {"Direct": [], "Shared": []})
    for s in sessions:
        groups[(s["participant"], s["task"])][s["condition"]].append(s)
    pairs = []
    for (p, task), g in sorted(groups.items()):
        if not (g["Direct"] and g["Shared"]):
            continue
        rec = {"participant": p, "task": task}
        for m in PAIR_METRICS:
            if all(m in s and s[m] is not None for s in g["Direct"] + g["Shared"]):
                rec[m] = {"direct": round(sum(s[m] for s in g["Direct"]) / len(g["Direct"]), 4),
                          "shared": round(sum(s[m] for s in g["Shared"]) / len(g["Shared"]), 4)}
        rec["n_direct"] = len(g["Direct"])
        rec["n_shared"] = len(g["Shared"])
        pairs.append(rec)
    return pairs


def main():
    for name in ("vr_study_sessions.json", "master_arm_sessions.json"):
        p = os.path.join(DATA, name)
        d = json.load(open(p))
        n = relabel_list(d)
        json.dump(d, open(p, "w"), indent=1)
        print(name, n, "labels changed")

    p = os.path.join(DATA, "study_summary.json")
    d = json.load(open(p))
    n = relabel_list(d["sessions"])
    old_pairs = len(d.get("paired", []))
    d["paired"] = recompute_pairs(d["sessions"])
    if "cells" in d:  # participant x task x condition means, recomputed on the new labels
        groups = collections.defaultdict(list)
        for s_ in d["sessions"]:
            groups[(s_["participant"], s_["task"], s_["condition"])].append(s_)
        cells = []
        for (p_, task, cond), g in sorted(groups.items()):
            c = {"participant": p_, "task": task, "condition": cond, "n_trials": len(g)}
            for m in PAIR_METRICS:
                vals = [x[m] for x in g if x.get(m) is not None]
                if len(vals) == len(g):
                    c[m] = round(sum(vals) / len(vals), 4)
            cells.append(c)
        d["cells"] = cells
    d["task_note"] = ("Two tasks: 'Pick and place' (sessions the console named 'object tracking') and "
                      "'Target reaching' (named 'target reaching' or 'position matching/reaching'). "
                      "Relabelled by relabel_pilot_tasks.py on the operator's confirmation, 2026-09-07.")
    json.dump(d, open(p, "w"), indent=1)
    print("study_summary.json", n, "labels changed; pairs", old_pairs, "->", len(d["paired"]))
    for pr in d["paired"]:
        print("   pair", pr["participant"], pr["task"], "direct n=%d shared n=%d" % (pr["n_direct"], pr["n_shared"]),
              "time %.1f -> %.1f" % (pr["duration_s"]["direct"], pr["duration_s"]["shared"]))

    if os.path.exists(TRAJ):
        d = json.load(open(TRAJ))
        items = d["sessions"] if isinstance(d, dict) and "sessions" in d else d
        n = relabel_list(items)
        json.dump(d, open(TRAJ, "w"))
        print("trajectories.json", n, "labels changed")


if __name__ == "__main__":
    main()
