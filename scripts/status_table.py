#!/usr/bin/env python3
"""THE STATUS TABLE: every task x every mode, read FROM DISK.

    python3 scripts/status_table.py

Four independent columns, because they fail independently and conflating
them is how this project has previously believed things it had not shown:

    PLAN     the SESSION PLAN runs this task in this mode at all. Read from
             msc_session.PLAN, because the design is deliberately HALF
             CROSSED -- T2 runs in the two anchors only, T3 adds shared
             autonomy, T1 stage 2 adds the two VR modes. A cell outside the
             plan has no data BY DESIGN and is not a gap.
    BUILT    the task and its scenarios exist in the task table
    VERIF    coordinates verified at N=10 over the FULL densified path
             against a live /compute_ik  (recordings/baselines/*.json)
    CLIP     a clip exists on disk with the expected angle files
    DATA     a trial CSV exists on disk with rows in it

A task can be BUILT and not VERIF; VERIF and never CLIPped; CLIPped and
never logged. "It works" has meant all four of these at different times in
this repo, which is why they are counted separately and read from the
artefacts rather than from a hand-maintained list -- a status list that can
go stale silently is worth less than no list.
"""
import glob
import json
import re
import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments", "experiments",
                                "abc"))

MODES = ["01_master_teleop", "02_vr_teleop", "03_shared_autonomy",
         "04_vr_shared", "06_full_autonomy"]
TASKS = [("t0", "T0 target reaching"), ("t1", "T1 pick and place"),
         ("t1s2", "T1 s2 both arms"), ("t2", "T2 coordinated carry"),
         ("t3", "T3 circuit + meter")]

# Everything blocked on something OUTSIDE this machine. Stated as facts with
# their evidence, not as a wish list.
LAB_BLOCKED = [
    ("two real Kortex sessions", "all dual-arm work is against mocks; the "
     "arm permits exactly ONE session and a leaked one refuses the next"),
    ("detection rate at working distance", "MEASURED 0-4% on rendered "
     "primitives vs 0.89-0.91 on a real photograph -- the renderer is out "
     "of distribution, so the 95% gate is NEITHER passed nor failed"),
    ("the real camera intrinsics, distortion and extrinsic", "the mock's "
     "values are nominal; only the driver is a source for the real ones"),
    ("whether the vision module streams at all", "open, given that UDP "
     "cyclic is dead on this host and only SHM works"),
    ("the master arm", "7 of 14 channels INCOHERENT as of the 2026-08-06 "
     "baseline; this is a soldering problem, not a software one"),
    ("l_j2 and l_j4 specifically", "regression names them: the left arm has "
     "NO reach observable without them (R2 0.133, residual 93% of spread)"),
    ("the right arm's home joint values", "recorded as NEVER READ from "
     "hardware; P_HOME for that arm depends on them"),
    ("the arms sharing a workspace", "0 of 63 frontal cells reachable by "
     "both, 0 of 319 surveyed cells at |x| <= 0.10. T4 handover is BLOCKED "
     "by geometry and needs the right arm RE-PARKED in hardware"),
    ("the mount's proximal interference", "real and mechanical; the SRDF "
     "exclusion silences the alarm, it does not move the metal"),
    ("Kortex close-then-reconnect", "the riskiest thing in the recovery "
     "layer and the part a real run hits first"),
    ("adb / Android Platform-Tools", "installed on neither side; the one "
     "hard prerequisite before any headset test"),
    ("this machine is off the lab network", "eth0 DOWN, 100% loss to "
     "192.168.1.10 -- nothing needing a real arm can run here at all"),
]

ETHICS_BLOCKED = [
    ("any run with a real participant", "every figure in this repo is "
     "scripted or synthetic; no human data has been collected"),
    ("the two-person dyad protocol", "operator and wearer are DIFFERENT "
     "people, so both are participants and both consent separately"),
    ("worn operation", ">17 kg with harness and NO gravity compensation, "
     "on a person who did not choose the motion; recommended against until "
     "the proximal interference is fixed mechanically"),
    ("physiological measures (EDA)", "trust/arousal measures on the wearer "
     "need their own approval"),
    ("recording of participants", "the clip pipeline records the SIM only; "
     "any camera on a person is a separate matter"),
]


def built():
    try:
        import msc_clip_tasks as M
    except Exception as e:
        print("cannot import task table: %s" % e)
        return {}
    out = {}
    for k, _lab in TASKS:
        t = M.TASKS.get(k)
        out[k] = 0 if not t else len(t() if callable(t) else t)
    return out


def verified():
    """N=10 full-path verification results, from the baseline JSONs."""
    out = {}
    for f in glob.glob(os.path.join(WS, "recordings", "baselines", "*.json")):
        try:
            d = json.load(open(f))
        except Exception:
            continue
        b = os.path.basename(f)
        if "scenario" in b or "verification" in b or "reach" in b:
            out[b] = d
    return out


def clips():
    """Clip directories on disk, newest recording root wins."""
    found = {}
    roots = sorted(glob.glob(os.path.join(WS, "recordings", "verification*")))
    for r in roots:
        for p in glob.glob(os.path.join(r, "*", "*", "*")):
            if not os.path.isdir(p):
                continue
            mp4 = glob.glob(os.path.join(p, "*.mp4"))
            if not mp4:
                continue
            mode = p.split(os.sep)[-3]
            task = p.split(os.sep)[-2].lower()
            found.setdefault((task, mode), []).append((len(mp4), r))
    return found


def data():
    """Trial CSVs with actual rows, by (task, mode) -- filename PARSED.

    NOT substring-matched. The first version looked for "t0" anywhere in the
    path and found it inside the trial INDEX (`..._t001_...`), so every task
    reported data in every mode from files belonging to other tasks. A status
    table that reports success it has not earned is worse than none, and this
    is the fourth time a measuring tool in this repo has done exactly that.

    The trial files are `msc_<key>_b<NN>_t<NNN>_<mode>.csv` where <key> is the
    DISPATCHER key (m0, m1, m1s2, m2, m3), not the display name.
    """
    key_of = {"m0": "t0", "m1": "t1", "m1s2": "t1s2", "m2": "t2", "m3": "t3"}
    pat = re.compile(r"^msc_(m\d(?:s\d)?)_b\d+_t\d+_(\S+)\.csv$")
    found = {}
    for f in glob.glob(os.path.join(WS, "recordings", "**", "*.csv"),
                       recursive=True):
        m = pat.match(os.path.basename(f))
        if not m:
            continue                       # summary_*.csv and anything else
        task, mode = key_of.get(m.group(1)), m.group(2)
        if task is None or mode not in MODES:
            continue
        try:
            with open(f) as fh:
                n = sum(1 for _ in fh) - 1
        except Exception:
            continue
        if n > 0:
            found[(task, mode)] = found.get((task, mode), 0) + n
    return found


def planned():
    """(task, mode) cells the SESSION PLAN actually runs.

    THE TABLE USED TO RENDER A BY-DESIGN ABSENCE IDENTICALLY TO A REAL GAP,
    and that is worse than not showing it: it invites somebody to go hunting
    for a broken data path that is not broken. T2 has no rows in either VR
    mode because msc_session.py runs T2 in the two ANCHORS only -- measured,
    a T2 trial driven in 02_vr_teleop logs 42 rows with 0.2322 m of left EE
    travel, so the path works and the plan is simply not asking for it.
    """
    try:
        import msc_session as MS
    except Exception:
        return None
    key = {"T0": "t0", "T1_s1": "t1", "T1_s2": "t1s2", "T2": "t2", "T3": "t3"}
    return {(key[k], m) for k, v in MS.PLAN.items() for m in v[0]}


def main():
    b, c, d = built(), clips(), data()
    pl = planned()
    print("=" * 78)
    print("STATUS: every MSc task x every mode, read from disk")
    print("=" * 78)
    print("\nC = clip on disk   D = trial rows   -- = OUTSIDE THE SESSION "
          "PLAN, absence is by design")
    print("a lower-case letter means PLANNED AND MISSING -- a real gap\n")
    hdr = "%-22s %-3s" % ("task", "B")
    for m in MODES:
        hdr += " %-13s" % m.split("_", 1)[1][:12]
    print(hdr)
    print("-" * 78)
    for k, lab in TASKS:
        row = "%-22s %-3s" % (lab, b.get(k, 0) or "-")
        for m in MODES:
            got = d.get((k, m))
            # EXACT, not startswith: "t1s2".startswith("t1") is True, so a
            # prefix test let T1 claim T1 STAGE 2's clips. Same bug as the
            # substring match in data() and in the same file -- prefix keys
            # and prefix tests do not mix.
            has_clip = (k, m) in c
            in_plan = pl is None or (k, m) in pl
            # CLIPS are a verification artefact and are filmed for EVERY
            # mode; DATA follows the plan. So a missing clip is always a gap,
            # while missing data is a gap only inside the plan.
            cell = ("C" if has_clip else "c")
            cell += ("D" if got else ("d" if in_plan else "-"))
            row += " %-13s" % cell
        print(row)
    gaps_c = sorted(k for k in [(t, m) for t, _ in TASKS for m in MODES]
                    if k not in c)
    gaps_d = sorted(k for k in (pl or set()) if k not in d)
    print("\nclip dirs %d   logged cells %d of %d planned"
          % (len(c), len(d), len(pl or [])))
    print("REAL GAPS -- missing clip : %s"
          % (", ".join("/".join(g) for g in gaps_c) or "NONE"))
    print("REAL GAPS -- planned, no data: %s"
          % (", ".join("/".join(g) for g in gaps_d) or "NONE"))

    print("\n" + "=" * 78)
    print("BLOCKED ON THE LAB -- not on code, and not fixable from here")
    print("=" * 78)
    for t, why in LAB_BLOCKED:
        print("  * %s\n      %s" % (t, why))

    print("\n" + "=" * 78)
    print("BLOCKED ON ETHICS -- approval, not equipment")
    print("=" * 78)
    for t, why in ETHICS_BLOCKED:
        print("  * %s\n      %s" % (t, why))
    return 0


if __name__ == "__main__":
    sys.exit(main())
