#!/usr/bin/env python3
"""EVERY SIMULATION THIS PROJECT CAN RUN, IN ONE COMMAND, WITH ITS NUMBERS.

    python3 scripts/run_simulation_campaign.py                 # no stack needed
    python3 scripts/run_simulation_campaign.py --with-stack     # + MoveIt ones
    python3 scripts/run_simulation_campaign.py --list
    python3 scripts/run_simulation_campaign.py --self-test

WHY THIS EXISTS
==========================================================================
The measurements in this repository were each run once, by hand, on the day
the thing they measure was built, and their numbers reached the thesis by
being copied out of a terminal.  That is reproducible in principle and not in
practice: nobody can re-run "the simulations" because there is no such thing
to run, only nineteen scripts with nineteen different argument lists, some of
which need a `move_group` and most of which do not, and no record of which
were run together or against which geometry.

This runs them all, records what each produced with its exit status and its
duration, and writes ONE directory a thesis chapter can be built from.

TWO TIERS, AND THE DISTINCTION IS LOAD-BEARING
==========================================================================
  STANDALONE   arithmetic, kinematics and analysis in this process. No ROS,
               no `move_group`, no cameras. These run anywhere and are the
               ones a reader can reproduce on a laptop.
  STACK        needs `/compute_ik` from a live MoveIt, so it needs the
               simulated platform brought up first. `--with-stack` launches
               one via `sim_session.py` and runs them inside it.

A stage that CANNOT run says so and says which tier it needed, rather than
being silently absent -- a missing row and a failing row must not look the
same, which is this repository's most repeated defect.

WHAT IS NOT SIMULATED, AND IS LABELLED SO
==========================================================================
Nothing here touches a physical arm. The sim-to-real stage ANALYSES recorded
hardware data rather than simulating it, and is tagged `recorded` so that its
numbers are never read as simulation output.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY = sys.executable or "python3"


class Stage:
    """One simulation: what it is, how to run it, and what it leaves behind."""

    def __init__(self, key, title, group, tier, argv, produces=(),
                 timeout=1800, note=""):
        self.key = key
        self.title = title
        self.group = group          # which chapter section it belongs to
        self.tier = tier            # "standalone" | "stack" | "recorded"
        self.argv = list(argv)
        self.produces = list(produces)
        self.timeout = timeout
        self.note = note


#: THE CAMPAIGN.  Grouped by the question each answers, because that is how
#: the results chapter is organised and a list ordered by filename would have
#: to be re-sorted by hand every time.
STAGES = [
    # ---------------------------------------------------- computer vision
    Stage("cv_arithmetic", "Vision arithmetic against a constructed scene",
          "computer vision", "standalone",
          [".venv_vision/bin/python", "scripts/cv_pickpose_visuals.py",
           "--self-test", "--fast",
           "--out", "recordings/simulations/{stamp}/cv_selftest"],
          produces=["recordings/simulations/{stamp}/cv_selftest"],
          timeout=1200,
          note="every geometric stage run against a depth frame whose plane, "
               "object heights and object widths were CHOSEN, so the answer "
               "is known before the stage runs"),
    Stage("cv_replay", "The full pipeline replayed on the recorded cameras",
          "computer vision", "standalone",
          [".venv_vision/bin/python", "scripts/cv_pickpose_visuals.py",
           "--replay", "recordings/vision_thesis/20260830_073141_upright",
           "--out", "recordings/simulations/{stamp}/cv_replay"],
          produces=["recordings/simulations/{stamp}/cv_replay/measurements.csv",
                    "recordings/simulations/{stamp}/cv_replay/manifest.json"],
          timeout=2400,
          note="the frames are real; the RUN is offline, so every constant "
               "can be changed and the whole set redrawn without a camera"),
    Stage("cv_orientation", "Which way up the scene camera is",
          "computer vision", "standalone",
          [".venv_vision/bin/python", "scripts/scene_rs_orientation.py",
           "--self-test"],
          note="12 known-answer checks on the rotation, its intrinsic map "
               "and the instrument that decides it"),

    # ------------------------------------------------------------- teleop
    Stage("teleop_motion", "Motion generation: three generators compared",
          "teleoperation", "standalone",
          [PY, "scripts/measure_teleop_motion.py"],
          produces=["recordings/baselines/teleop_motion.json"],
          note="off-path excursion, same-cycle excursion, peak commanded "
               "velocity against the declared limit, and arrival synchrony"),
    Stage("teleop_smoothing", "Master smoothing: 1-Euro against the fixed EMA",
          "teleoperation", "standalone",
          [PY, "scripts/measure_master_smoothing.py", "--self-test"],
          note="stillness and lag measured together, which is the pair a "
               "fixed alpha cannot win"),
    Stage("teleop_budget", "The positioning error budget",
          "teleoperation", "standalone",
          [PY, "scripts/measure_control_budget.py"],
          produces=["recordings/baselines/control_budget.json"],
          note="each term measured separately, combined in quadrature and "
               "added, against the 30 mm capture gate"),
    Stage("teleop_orientation_cost", "What the pinned wrist costs",
          "teleoperation", "stack",
          [PY, "scripts/measure_orientation_cost.py", "--dirs", "14",
           "--arm", "both", "--seeds", "10"],
          produces=["recordings/baselines/orientation_cost.json"],
          timeout=5400,
          note="mean reach per direction under each orientation policy"),

    # -------------------------------------------------------- robot tasks
    Stage("tasks_t1", "T1, both stages, over the whole densified path",
          "robot tasks", "stack",
          [PY, "scripts/verify_t1.py"],
          produces=["recordings/baselines/t1_paths.json"],
          timeout=2400,
          note="N=10 repeats per pose, densified to 20 mm, both arms "
               "confirmed at home before any sample"),
    Stage("tasks_msc", "The MSc task set, every coordinate",
          "robot tasks", "stack",
          [PY, "scripts/verify_msc_tasks.py"],
          timeout=3600,
          note="IK success and wearer clearance for every waypoint of "
               "T0, T1, T2 and T3"),
    Stage("tasks_dance", "The demonstration routines",
          "robot tasks", "stack",
          [PY, "scripts/verify_dance_paths.py"],
          timeout=2400,
          note="every waypoint of the three routines"),
    Stage("tasks_accuracy", "Accuracy table over the recorded clip set",
          "robot tasks", "standalone",
          [PY, "scripts/accuracy_table.py"],
          note="grasp success, positioning error at closure and placement "
               "error, per task and per mode, from committed clip data"),
    Stage("tasks_status", "Which cells of the task/mode matrix exist",
          "robot tasks", "standalone",
          [PY, "scripts/status_table.py"],
          note="planned cells against recorded cells, with by-design "
               "absences separated from real gaps"),

    # ---------------------------------- shared autonomy and full autonomy
    Stage("autonomy_instructions",
          "Full autonomy: 75 phrasings through the grammar",
          "autonomy", "standalone",
          [PY, "scripts/sweep_t1_instructions.py",
           "--json", "recordings/simulations/{stamp}/instructions.json"],
          produces=["recordings/simulations/{stamp}/instructions.json"],
          timeout=1200,
          note="correct / asked / refused / MISUNDERSTOOD, against the "
               "committed earlier grammar on the same cases"),
    Stage("autonomy_arbitration",
          "Shared autonomy: the arbiter over its whole input range",
          "autonomy", "standalone",
          [PY, "scripts/measure_shared_autonomy.py",
           "--out", "recordings/simulations/{stamp}/arbitration.json"],
          produces=["recordings/simulations/{stamp}/arbitration.json"],
          note="what fraction of the confidence range assists, what the "
               "position path does in each state, and the tie case"),

    # ------------------------------------------------- study and analysis
    Stage("study_power", "The pre-registered analysis on constructed data",
          "study", "standalone",
          [PY, "scripts/validate_analysis_pipeline.py", "--trials", "20000"],
          produces=["recordings/baselines/analysis_pipeline_validation.json"],
          timeout=1800,
          note="power and false-positive rate of the procedure that will be "
               "applied to the participant data"),

    # ------------------------------------------- recorded, NOT simulation
    Stage("sim_to_real", "The simulation-to-hardware gap",
          "transfer", "recorded",
          [PY, "scripts/measure_sim_to_real_gap.py"],
          produces=["recordings/baselines/sim_to_real_gap.json"],
          note="ANALYSIS OF RECORDED HARDWARE DATA, not a simulation. "
               "Tagged separately so its numbers are never read as one"),
]


def _fill(s, stamp):
    return s.replace("{stamp}", stamp)


def run_stage(st, outdir, stamp, dry=False):
    argv = [_fill(a, stamp) for a in st.argv]
    exe = argv[0]
    if exe.startswith(".venv") or exe.startswith("scripts/"):
        argv[0] = os.path.join(WS, exe)
    rec = dict(key=st.key, title=st.title, group=st.group, tier=st.tier,
               argv=argv, note=st.note, produces=[])
    if dry:
        rec["status"] = "listed"
        return rec
    if not os.path.exists(argv[0]) and os.path.sep in argv[0]:
        rec.update(status="missing",
                   reason="%s does not exist" % argv[0], seconds=0.0)
        return rec
    t0 = time.time()
    try:
        p = subprocess.run(argv, cwd=WS, capture_output=True, text=True,
                           timeout=st.timeout)
        out, code = (p.stdout or "") + (p.stderr or ""), p.returncode
    except subprocess.TimeoutExpired:
        out, code = "", -9
        rec["reason"] = "timed out after %d s" % st.timeout
    except Exception as exc:                                 # noqa: BLE001
        out, code = str(exc), -1
        rec["reason"] = "could not start: %s" % exc
    rec["seconds"] = round(time.time() - t0, 1)
    rec["exit"] = code
    rec["status"] = "ok" if code == 0 else "failed"
    # The tail is what a reader wants; the whole log is kept on disk.
    lines = out.splitlines()
    rec["tail"] = "\n".join(lines[-40:])
    logp = os.path.join(outdir, "logs", st.key + ".log")
    os.makedirs(os.path.dirname(logp), exist_ok=True)
    with open(logp, "w") as fh:
        fh.write(out)
    rec["log"] = os.path.relpath(logp, WS)
    for pat in st.produces:
        path = os.path.join(WS, _fill(pat, stamp))
        rec["produces"].append(dict(
            path=_fill(pat, stamp), exists=os.path.exists(path),
            bytes=(os.path.getsize(path)
                   if os.path.isfile(path) else None)))
    return rec


def self_test():
    """The campaign definition itself, checked without running anything."""
    ok = fail = 0

    def check(name, cond):
        nonlocal ok, fail
        if cond:
            ok += 1
            print("  ok    %s" % name)
        else:
            fail += 1
            print("  FAIL  %s" % name)

    keys = [s.key for s in STAGES]
    check("every stage key is unique", len(keys) == len(set(keys)))
    check("every stage declares a known tier",
          all(s.tier in ("standalone", "stack", "recorded") for s in STAGES))
    check("every stage says what it measures", all(s.note for s in STAGES))
    check("there is at least one stage per group",
          {s.group for s in STAGES} >= {"computer vision", "teleoperation",
                                        "robot tasks", "autonomy", "study"})
    missing = [s.key for s in STAGES
               if os.path.sep in s.argv[0].replace("/", os.path.sep)
               and not os.path.exists(os.path.join(WS, s.argv[0]))]
    # The interpreter entries are absolute or bare; only repo paths count.
    missing = [s.key for s in STAGES
               if s.argv[1].startswith("scripts/")
               and not os.path.exists(os.path.join(WS, s.argv[1]))]
    check("every stage's script exists: %s" % (missing or "all present"),
          not missing)
    # A campaign that reports a missing stage as a pass would be worthless.
    probe = Stage("probe", "t", "g", "standalone",
                  [PY, "scripts/does_not_exist_xyz.py"], note="n")
    r = run_stage(probe, "/tmp", "x")
    check("a stage that cannot run reports failed, not ok",
          r["status"] == "failed")
    print("\n%d checks, %d failed" % (ok + fail, fail))
    return 1 if fail else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--with-stack", action="store_true",
                    help="also run the stages that need a live /compute_ik. "
                         "Brings a MoveIt stack up via sim_session.py.")
    ap.add_argument("--only", default="",
                    help="comma separated stage keys")
    ap.add_argument("--group", default="",
                    help="comma separated groups")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)

    if a.self_test:
        return self_test()

    only = {s.strip() for s in a.only.split(",") if s.strip()}
    groups = {s.strip() for s in a.group.split(",") if s.strip()}
    picked = [s for s in STAGES
              if (not only or s.key in only)
              and (not groups or s.group in groups)]
    if a.list:
        for s in picked:
            print("%-24s %-16s %-11s %s"
                  % (s.key, s.group, s.tier, s.title))
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = a.out or os.path.join(WS, "recordings", "simulations", stamp)
    os.makedirs(outdir, exist_ok=True)

    standalone = [s for s in picked if s.tier in ("standalone", "recorded")]
    stack = [s for s in picked if s.tier == "stack"]

    run = dict(started=datetime.now().isoformat(timespec="seconds"),
               stamp=stamp, argv=sys.argv, out=os.path.relpath(outdir, WS),
               interpreter=PY, stages=[])

    print("SIMULATION CAMPAIGN %s" % stamp)
    print("  %d standalone, %d needing a stack (%s)\n"
          % (len(standalone), len(stack),
             "will be run" if a.with_stack else "SKIPPED, pass --with-stack"))

    for i, st in enumerate(standalone):
        print("[%d/%d] %-22s %s" % (i + 1, len(standalone), st.key, st.title))
        rec = run_stage(st, outdir, stamp)
        print("        %s in %.1f s" % (rec["status"], rec.get("seconds", 0)))
        run["stages"].append(rec)

    if stack:
        if not a.with_stack:
            for st in stack:
                run["stages"].append(dict(
                    key=st.key, title=st.title, group=st.group, tier=st.tier,
                    status="skipped", note=st.note,
                    reason="needs a live /compute_ik; re-run with "
                           "--with-stack"))
        else:
            run["stages"].extend(_run_in_stack(stack, outdir, stamp))

    run["finished"] = datetime.now().isoformat(timespec="seconds")
    counts = {}
    for r in run["stages"]:
        counts[r["status"]] = counts.get(r["status"], 0) + 1
    run["totals"] = counts
    with open(os.path.join(outdir, "campaign.json"), "w") as fh:
        json.dump(run, fh, indent=2)
    _report(run, outdir)
    print("\n%s" % ", ".join("%d %s" % (v, k) for k, v in sorted(counts.items())))
    print("written to %s" % os.path.relpath(outdir, WS))
    return 0 if counts.get("failed", 0) == 0 else 1


def _run_in_stack(stack, outdir, stamp):
    """Bring up one MoveIt stack and run every stack stage inside it.

    ONE stack for all of them, deliberately: HARD CONSTRAINT 3 forbids two,
    and two verifiers against one `move_group` have already produced a
    measurement where one run's furniture was in the other run's scene.
    """
    inner = os.path.join(outdir, "stack_runner.py")
    payload = [dict(key=s.key, title=s.title, group=s.group, tier=s.tier,
                    argv=[_fill(x, stamp) for x in s.argv],
                    produces=[_fill(x, stamp) for x in s.produces],
                    timeout=s.timeout, note=s.note) for s in stack]
    with open(inner, "w") as fh:
        fh.write("#!/usr/bin/env python3\n"
                 "# Generated by run_simulation_campaign.py. Runs INSIDE the\n"
                 "# session sim_session.py brought up, so every stage sees\n"
                 "# the same /compute_ik and the same scene.\n"
                 "import json, os, subprocess, sys, time\n"
                 "WS = %r\n"
                 "STAGES = json.loads(%r)\n"
                 "OUT = %r\n" % (WS, json.dumps(payload), outdir))
        fh.write('''
recs = []
for i, st in enumerate(STAGES):
    print("[stack %d/%d] %s" % (i + 1, len(STAGES), st["key"]), flush=True)
    t0 = time.time()
    try:
        p = subprocess.run(st["argv"], cwd=WS, capture_output=True,
                           text=True, timeout=st["timeout"])
        out, code = (p.stdout or "") + (p.stderr or ""), p.returncode
    except subprocess.TimeoutExpired:
        out, code = "", -9
    except Exception as exc:
        out, code = str(exc), -1
    rec = dict(st)
    rec["seconds"] = round(time.time() - t0, 1)
    rec["exit"] = code
    rec["status"] = "ok" if code == 0 else "failed"
    rec["tail"] = "\\n".join(out.splitlines()[-40:])
    lp = os.path.join(OUT, "logs", st["key"] + ".log")
    os.makedirs(os.path.dirname(lp), exist_ok=True)
    open(lp, "w").write(out)
    rec["log"] = os.path.relpath(lp, WS)
    rec["produces"] = [dict(path=q, exists=os.path.exists(os.path.join(WS, q)))
                       for q in st["produces"]]
    print("    %s in %.1f s" % (rec["status"], rec["seconds"]), flush=True)
    recs.append(rec)
open(os.path.join(OUT, "stack_stages.json"), "w").write(json.dumps(recs, indent=2))
''')
    cmd = [PY, os.path.join(WS, "scripts", "sim_session.py"),
           "--stack", "moveit", "--skip-tests", "--run-timeout", "10800",
           "--", PY, inner]
    print("\nbringing up a MoveIt stack for %d stage(s) ..." % len(stack))
    p = subprocess.run(cmd, cwd=WS, capture_output=True, text=True)
    with open(os.path.join(outdir, "logs", "sim_session.log"), "w") as fh:
        fh.write((p.stdout or "") + (p.stderr or ""))
    got = os.path.join(outdir, "stack_stages.json")
    if os.path.exists(got):
        with open(got) as fh:
            return json.load(fh)
    return [dict(key=s.key, title=s.title, group=s.group, tier=s.tier,
                 status="failed", note=s.note,
                 reason="the simulated stack did not come up; see "
                        "logs/sim_session.log")
            for s in stack]


def _report(run, outdir):
    lines = ["# Simulation campaign %s" % run["stamp"], "",
             "Started %s, finished %s." % (run["started"], run["finished"]),
             "", "| stage | group | tier | status | s |",
             "| --- | --- | --- | --- | --- |"]
    for r in run["stages"]:
        lines.append("| `%s` | %s | %s | **%s** | %s |"
                     % (r["key"], r["group"], r["tier"], r["status"],
                        r.get("seconds", "-")))
    for r in run["stages"]:
        lines += ["", "## `%s` -- %s" % (r["key"], r["title"]), "",
                  "*%s*" % r["note"], ""]
        if r.get("reason"):
            lines += ["> %s" % r["reason"], ""]
        if r.get("tail"):
            lines += ["```", r["tail"], "```"]
    with open(os.path.join(outdir, "report.md"), "w") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
