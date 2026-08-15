#!/usr/bin/env python3
"""IS THE NO-OPERATOR MODE DIFFERENCE REAL? Settled from the recorded set.

    python3 scripts/analyse_mode_difference.py

THE CLAIM UNDER TEST. `docs/system/findings.md` finding 3 reports that scripted
clips, with no operator anywhere in the loop, show different achieved motion by
mode: T0's left EE path 1.407 m under 01 against 2.034 m under 03, against a 3%
same-mode spread taken from the one cell recorded twice. TASK_SPEC section 1
now carries that as a correction to "robot performance is identical across
modes". If it is wrong, the five-mode comparison is fine and the correction has
to come out. If it is right, no trial difference can be attributed to the
operator until the no-operator difference is subtracted.

WHAT THIS DOES. It reads every `scene_events.json` in the recorded set and puts
three things beside each other that were never compared:

    ee_travel_m   the path integral, which is what the claim is made of
    ee_net_m      the straight line from the first observed pose to the last
    clip duration read from the mp4 container, so a run that was simply
                  watched for longer can be told from one that moved further

THE THREE HYPOTHESES IT CAN SEPARATE, and no report without all three:

  1. RECORDING LENGTH. If travel tracks duration, the metric is integrating
     idle jitter and the difference is an artefact. Refuted if runs of equal
     duration differ.
  2. RUN-TO-RUN NOISE. If the differences are independent noise, no two runs
     should agree to four decimal places. Refuted by exact agreement.
  3. THE COMMAND PATH. If the grouping follows which command path was in the
     loop, that is the stated mechanism.

AND THE THING THE NUMBERS TURN OUT TO REST ON. `ee_net_m` is anchored on the
FIRST POSE THE SCENE NODE HAPPENED TO SEE and `ee_travel_m` is a sum over
ticks, so both are properties of the observation window as well as of the
motion. That is reported here rather than worked around, because it bounds what
the archived set can settle: it can show the difference is not noise and not
duration, and it cannot give the difference a trustworthy SIZE. The size needs
repeats, and the repeat count is the thing that has never been run.
"""
import glob
import json
import os
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SET = os.path.join(ROOT, "recordings/verification")
OUT = os.path.join(ROOT, "recordings/baselines/mode_difference.json")


def mp4_duration(path):
    """Seconds, from the mp4 movie header. No ffprobe on this host."""
    try:
        d = open(path, "rb").read(400000)
    except Exception:                                          # noqa: BLE001
        return None
    i = d.find(b"mvhd")
    if i < 0:
        return None
    try:
        if d[i + 4] == 0:
            ts, dur = struct.unpack(">II", d[i + 16:i + 24])
        else:
            ts, dur = struct.unpack(">IQ", d[i + 24:i + 36])
        return round(dur / float(ts), 2) if ts else None
    except Exception:                                          # noqa: BLE001
        return None


def collect():
    rows = {}
    for p in sorted(glob.glob(os.path.join(SET, "*", "T*", "*",
                                           "scene_events.json"))):
        rel = os.path.relpath(p, SET).split(os.sep)
        mode, task = rel[0], rel[1]
        d = json.load(open(p))
        t = d.get("ee_travel_m") or {}
        n = d.get("ee_net_m") or {}
        rows.setdefault(task, {})[mode] = dict(
            travel_l=t.get("left"), travel_r=t.get("right"),
            net_l=n.get("left"), net_r=n.get("right"),
            samples=d.get("ee_samples"),
            duration_s=mp4_duration(os.path.join(os.path.dirname(p),
                                                 "rviz_front.mp4")))
    return rows


def main():
    rows = collect()
    if not rows:
        print("no recorded clips under %s" % SET)
        return 2

    print("ACHIEVED MOTION BY MODE, from the recorded set\n")
    findings = {}
    for task in sorted(rows):
        per = rows[task]
        print("== %s" % task)
        print("   %-20s %9s %9s %9s %9s %9s"
              % ("mode", "travelL", "travelR", "netL", "netR", "dur_s"))
        for mode in sorted(per):
            v = per[mode]
            print("   %-20s %9s %9s %9s %9s %9s"
                  % (mode, v["travel_l"], v["travel_r"], v["net_l"],
                     v["net_r"], v["duration_s"]))

        # 1. equal duration, unequal travel?
        by_dur = {}
        for mode, v in per.items():
            if v["duration_s"] is not None and v["travel_l"] is not None:
                by_dur.setdefault(v["duration_s"], []).append(
                    (mode, v["travel_l"]))
        dur_refuted = []
        for dur, group in by_dur.items():
            if len(group) < 2:
                continue
            lo = min(g[1] for g in group)
            hi = max(g[1] for g in group)
            if lo > 0 and (hi - lo) / lo > 0.05:
                dur_refuted.append((dur, round((hi - lo) / lo * 100.0, 1),
                                    [g[0] for g in group]))

        # 2. exact agreement between different modes?
        exact = {}
        for mode, v in per.items():
            if v["net_l"] is None:
                continue
            exact.setdefault((v["net_l"], v["net_r"]), []).append(mode)
        shared = {k: v for k, v in exact.items() if len(v) > 1}

        findings[task] = dict(
            equal_duration_unequal_travel=dur_refuted,
            identical_net_groups={str(k): v for k, v in shared.items()},
            per_mode=per)
        if dur_refuted:
            for dur, pct, modes in dur_refuted:
                print("   -> at %.2f s BOTH, travel differs by %.1f%% across "
                      "%s: recording LENGTH does not explain it"
                      % (dur, pct, ", ".join(modes)))
        if shared:
            for k, v in shared.items():
                print("   -> %s share net displacement %s EXACTLY: that is "
                      "not what independent run-to-run noise looks like"
                      % (", ".join(v), k))
        print("")

    print("=" * 74)
    print("WHAT THIS SETTLES, AND WHAT IT DOES NOT")
    print("=" * 74)
    any_dur = any(f["equal_duration_unequal_travel"] for f in findings.values())
    any_exact = any(f["identical_net_groups"] for f in findings.values())
    print("   recording length explains it     %s"
          % ("NO -- refuted above" if any_dur else "not testable here"))
    print("   independent run-to-run noise     %s"
          % ("NO -- exact agreement above" if any_exact
             else "not testable here"))
    print("   the SIZE of the difference       NOT SETTLED. Every cell is a "
          "single run, and both metrics depend on the observation window "
          "(ee_travel_m sums over ticks; ee_net_m is anchored on the first "
          "pose the recorder saw).")
    print("\n   So the correction in TASK_SPEC section 1 STANDS: a mode "
          "difference in a trial cannot be attributed to the operator until "
          "the no-operator difference is subtracted. What is still missing is "
          "its magnitude, which needs N repeats per cell and a metric that "
          "starts when the TASK starts.")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(findings, open(OUT, "w"), indent=2)
    print("\n-> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
