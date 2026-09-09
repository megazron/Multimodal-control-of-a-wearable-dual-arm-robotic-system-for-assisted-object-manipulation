#!/usr/bin/env python3
"""Assign the task of P3's seven sessions from the gripper record.

P3's sessions predate the per-participant folder naming, so no task label
was written at the console. The two tasks differ in one recorded quantity:
pick and place closes the gripper on the object and carries it, target
reaching never needs the gripper. The rule applied here is the one the
gripper table already reports: a session in which the gripper was held
closed for at least 5 % of the samples on either hand is pick and place;
otherwise it is target reaching. On the record this gives P3 one pick and
place and four target reaching under direct control, one of each under
shared autonomy.

Applied to every derived data file the figures read; idempotent.

    python3 thesis_v3/figures/classify_p3_tasks.py
"""
import json
import os

import relabel_pilot_tasks as R

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "pilot", "data")
TRAJ = os.path.join(HERE, "gallery", "trajectories", "trajectories.json")
CLOSED_FRAC = 0.05


def rule():
    out = {}
    for line in open(os.path.join(DATA, "gripper_by_session.jsonl")):
        r = json.loads(line)
        frac = max(h["target"]["closed_frac"] for h in r["hands"].values())
        out[r["session"]] = "Pick and place" if frac >= CLOSED_FRAC else "Target reaching"
    return out


def apply(items, task_of):
    n = 0
    for s in items:
        if s.get("participant") == "P3" and s["task"] == "Unspecified" and s["session"] in task_of:
            s["task"] = task_of[s["session"]]
            n += 1
    return n


def main():
    task_of = rule()
    p = os.path.join(DATA, "vr_study_sessions.json")
    d = json.load(open(p)); n = apply(d, task_of); json.dump(d, open(p, "w"), indent=1)
    print("vr_study_sessions.json", n, "sessions classified")

    p = os.path.join(DATA, "study_summary.json")
    d = json.load(open(p)); n = apply(d["sessions"], task_of)
    d["paired"] = R.recompute_pairs(d["sessions"])
    import collections
    groups = collections.defaultdict(list)
    for s_ in d["sessions"]:
        groups[(s_["participant"], s_["task"], s_["condition"])].append(s_)
    cells = []
    for (p_, task, cond), g in sorted(groups.items()):
        c = {"participant": p_, "task": task, "condition": cond, "n_trials": len(g)}
        for m in R.PAIR_METRICS:
            vals = [x[m] for x in g if x.get(m) is not None]
            if len(vals) == len(g):
                c[m] = round(sum(vals) / len(vals), 4)
        cells.append(c)
    d["cells"] = cells
    d["task_note"] = (d.get("task_note", "") + " P3's seven sessions carry no console label; their task "
                      "is assigned from the gripper record (closed >= 5 % of samples = pick and place), "
                      "classify_p3_tasks.py.")
    json.dump(d, open(p, "w"), indent=1)
    print("study_summary.json", n, "sessions classified;", len(d["paired"]), "pairs")

    if os.path.exists(TRAJ):
        # the trajectory records carry no session key: match P3's by condition and duration
        vr = json.load(open(os.path.join(DATA, "vr_study_sessions.json")))
        p3 = [v for v in vr if v["participant"] == "P3"]
        t = json.load(open(TRAJ))
        n = 0
        for s in t["sessions"]:
            if s.get("participant") == "P3" and s.get("task") == "Unspecified" and s.get("t"):
                dur = s["t"][-1] - s["t"][0]
                cands = [v for v in p3 if v["condition"] == s["condition"]]
                best = min(cands, key=lambda v: abs(v["duration_s"] - dur))
                if abs(best["duration_s"] - dur) < 15:
                    s["task"] = best["task"]; n += 1
        json.dump(t, open(TRAJ, "w"))
        print("trajectories.json", n, "sessions classified")
    for k, v in sorted(task_of.items()):
        if k.startswith("20260902_1"):
            print("  ", k, v)


if __name__ == "__main__":
    main()
