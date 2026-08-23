#!/usr/bin/env python3
"""THE ACCURACY TABLE, derived from recorded data. Mode by metric.

    python3 scripts/accuracy_table.py [--root recordings/<dir>]
    python3 scripts/accuracy_table.py --self-test

WHY THIS FILE EXISTS. The figures quoted for mode accuracy -- 100% grasp,
0.000 mm placement, zero variance -- have lived only in session reports. They
could not be re-derived from this repository, which means they could not be
checked, and a number that cannot be reproduced does not belong in a thesis.
This script computes them from `scene_events.json`, which every clip writes,
so the table is a consequence of committed data rather than a recollection.

WHAT EACH NUMBER IS, AND HOW IT IS MEASURED. State this whenever the table is
quoted, because "0.000 mm" means nothing without it:

  grasp success    fraction of the task's objects that were GRASPED at all,
                   from the GRASPED events. An object never grasped is a
                   failure, not a missing sample.
  positioning err  `min_pad_obj_closed_m`: how far the pads were from the
                   object AT THE MOMENT THE FINGERS CLOSED. Not the closest
                   approach, which flatters a run that got near and then
                   missed.
  placement err    distance from an object's final resting position to its
                   declared target, in the plane. Height is excluded because
                   the objects rest on a surface and the vertical component
                   is set by that surface, not by the operator.
  repeat variance  population standard deviation of placement error across
                   the trials in a cell.
  smallest object  the smallest `width_mm` that was grasped successfully in
                   that mode.

WHY THE MODE AXIS IS EXPECTED TO BE FLAT. `run_abc.py` builds its trajectory
from the TASK spec with no mode argument; mode selects only the topic, the
isolation check and the logging labels. So identical numbers across modes are
the architecture showing through, not a suspicious result -- and that is
precisely the point of the table. It is what establishes that the manipulator
is not a confound, so a null result on the human-factors axis cannot be
confused with a manipulator that varied between conditions.

THE INSTRUMENT IS VALIDATED BEFORE IT REPORTS. `--self-test` feeds synthetic
records with known answers, INCLUDING deliberately broken ones, and the script
refuses to print a table if any of them come back wrong. A check that cannot
fail on a bad input is not a check.
"""
import argparse
import glob
import json
import math
import os
import statistics
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments", "experiments",
                                "abc"))


def targets_for(task):
    """Declared placement targets, from the TASK definition.

    Read from the task rather than from the recording, so a recording cannot
    quietly redefine what counts as correct.
    """
    try:
        import msc_clip_tasks as MCT
        import t1_task as T1M
    except Exception:
        return {}
    if task in ("t1",):
        # WHICH PAD EACH CUBE GOES TO IS `t1_task.T1_PAIR`, AND THIS HAD ITS
        # OWN COPY OF IT.
        #
        # It read `plane[0 if i % 2 == 0 else 1]` -- cubes 0,2 to the first
        # pad and 1,3 to the second. That was true of a single-arm T1 with an
        # alternating row. T1 became TWO-ARMED on 2026-08-16: the pads
        # straddle the centreline at +/-0.290, each cube is picked by the arm
        # on its OWN side, and neither arm can cross -- 0 of 10 IK solutions
        # at every cross-side slot. `T1_PAIR` is {0:0, 1:0, 2:1, 3:1}.
        #
        # So this scored cube_1 and cube_2 against the pad on the WRONG SIDE,
        # 0.58 m away. Measured on the 2026-08-23 re-record: **319.7 mm mean
        # placement error with 264 mm of variance**, on clips whose own scene
        # log says "placed 56 mm from target". The robot put every cube on the
        # pad of its own colour; the table was measuring the distance to the
        # other one.
        #
        # The file's own header says the numbers must be re-derivable from
        # committed data. A copy of a task fact is not a derivation.
        return {"cube_%d" % i: MCT.T1_PLANES[T1M.T1_PAIR[i]]
                for i in range(len(MCT.T1_CUBES))
                if i in T1M.T1_PAIR}
    return {}


def cell_metrics(ev):
    """Metrics for one recorded cell. Returns a dict, or None if unusable."""
    task = ev.get("task")
    items = ev.get("items") or []
    if not isinstance(items, list) or not items:
        return None
    grasped = set()
    for e in ev.get("events", []):
        if e.get("ev") == "GRASPED" and e.get("item"):
            grasped.add(e["item"])

    tgt = targets_for(task)
    n, ok, pos_err, place_err, widths = 0, 0, [], [], []
    for it in items:
        name = it.get("item")
        if not name:
            continue
        n += 1
        was = name in grasped
        if was:
            ok += 1
            if it.get("width_mm"):
                widths.append(float(it["width_mm"]))
        d = it.get("min_pad_obj_closed_m")
        if d is not None:
            pos_err.append(float(d))
        t = tgt.get(name)
        f = it.get("final")
        if t and f:
            place_err.append(math.dist((f[0], f[1]), (t[0], t[1])))
    if not n:
        return None
    return dict(task=task, n=n, grasped=ok,
                grasp_rate=ok / float(n),
                pos_err=pos_err, place_err=place_err,
                smallest=min(widths) if widths else None)


def collect(root):
    out = {}
    for f in glob.glob(os.path.join(root, "**", "scene_events.json"),
                       recursive=True):
        parts = os.path.relpath(f, root).split(os.sep)
        if len(parts) < 2:
            continue
        mode = parts[0]
        try:
            ev = json.load(open(f))
        except Exception:
            continue
        m = cell_metrics(ev)
        if m:
            out.setdefault(mode, []).append(m)
    return out


def _fmt_mm(v):
    return "--" if v is None else "%.3f" % (v * 1000.0)


def table(by_mode):
    print("%-20s %7s %8s %11s %11s %10s %9s"
          % ("mode", "cells", "grasp %", "pos err mm", "place mm",
             "var mm", "min obj"))
    print("-" * 82)
    for mode in sorted(by_mode):
        cells = by_mode[mode]
        gr = [c["grasp_rate"] for c in cells]
        pe = [x for c in cells for x in c["pos_err"]]
        pl = [x for c in cells for x in c["place_err"]]
        sm = [c["smallest"] for c in cells if c["smallest"] is not None]
        var = (statistics.pstdev(pl) if len(pl) > 1 else 0.0) if pl else None
        print("%-20s %7d %7.1f%% %11s %11s %10s %9s"
              % (mode, len(cells), 100.0 * sum(gr) / len(gr),
                 _fmt_mm(sum(pe) / len(pe)) if pe else "--",
                 _fmt_mm(sum(pl) / len(pl)) if pl else "--",
                 _fmt_mm(var) if var is not None else "--",
                 "%.0f mm" % min(sm) if sm else "--"))


def self_test():
    """Known answers, including inputs the metrics MUST reject."""
    bad = []

    def chk(label, got, want):
        if got != want:
            bad.append("%s: got %r want %r" % (label, got, want))

    two_of_four = dict(task="none", events=[
        {"ev": "GRASPED", "item": "a"}, {"ev": "GRASPED", "item": "b"}],
        items=[{"item": x, "width_mm": 40} for x in "abcd"])
    m = cell_metrics(two_of_four)
    chk("2 of 4 grasped", round(m["grasp_rate"], 6), 0.5)

    none = dict(task="none", events=[],
                items=[{"item": "a", "width_mm": 40}])
    chk("nothing grasped is 0%, not missing",
        cell_metrics(none)["grasp_rate"], 0.0)

    chk("no items is unusable", cell_metrics(dict(task="t1", items=[])), None)

    # A cell where every object was grasped must NOT come back 0.5, and a
    # cell where none was must NOT come back 1.0. Both directions, because a
    # metric that only ever returns 1.0 passes the first test alone.
    allg = dict(task="none",
                events=[{"ev": "GRASPED", "item": x} for x in "ab"],
                items=[{"item": x, "width_mm": 40} for x in "ab"])
    chk("all grasped", cell_metrics(allg)["grasp_rate"], 1.0)

    # Smallest object must come from GRASPED items only: an ungrasped 10 mm
    # object must not be reported as the smallest thing the mode can handle.
    mixed = dict(task="none", events=[{"ev": "GRASPED", "item": "big"}],
                 items=[{"item": "big", "width_mm": 40},
                        {"item": "tiny", "width_mm": 10}])
    chk("smallest counts grasped only", cell_metrics(mixed)["smallest"], 40.0)

    # THE CUBE -> PAD MAPPING COMES FROM THE TASK, NOT FROM THIS FILE.
    #
    # It carried `plane[i % 2]` -- a single-arm alternating row -- while T1
    # has been two-armed since 2026-08-16, with the pads straddling the
    # centreline at +/-0.290 and neither arm able to cross it. So cube_1 and
    # cube_2 were scored against the pad 0.58 m away, and the 2026-08-23
    # re-record reported **319.7 mm** of placement error with 264 mm of
    # variance on clips whose own scene log says "placed 56 mm from target".
    # The robot put every cube on the pad of its own colour; the table was
    # measuring the distance to the other one.
    #
    # A CHECK THAT CANNOT FAIL IS NOT A CHECK, so this requires the old map to
    # have been WRONG as well as the new one to be right. If they ever agree,
    # the second assertion fires and says the control has gone inert.
    try:
        import msc_clip_tasks as _MCT
        import t1_task as _T1M
        tg = targets_for("t1")
        chk("cube->pad is the task's own T1_PAIR",
            all(list(tg["cube_%d" % i]) == list(_MCT.T1_PLANES[_T1M.T1_PAIR[i]])
                for i in _T1M.T1_PAIR), True)
        legacy = {"cube_%d" % i: _MCT.T1_PLANES[0 if i % 2 == 0 else 1]
                  for i in _T1M.T1_PAIR}
        chk("the pre-2026-08-23 map really was different",
            sum(1 for k in tg if list(tg[k]) != list(legacy[k])), 2)
    except Exception as _e:                                    # noqa: BLE001
        bad.append("cube->pad map: %r" % (_e,))

    if bad:
        print("INSTRUMENT SELF-TEST FAILED:")
        for b in bad:
            print("   " + b)
        return 1
    print("instrument self-test passed (%d known answers, including two that "
          "must FAIL on bad input)" % 7)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    rc = self_test()
    if a.self_test:
        return rc
    if rc:
        print("\nREFUSING to report a table from an instrument that failed "
              "its own known-answer test.")
        return rc
    roots = ([os.path.join(WS, a.root)] if a.root else
             sorted(glob.glob(os.path.join(WS, "recordings", "verification*"))))
    for r in roots:
        by = collect(r)
        if not by:
            continue
        print("\n%s" % os.path.relpath(r, WS))
        table(by)
        # PLACEMENT IS SCORED AGAINST TARGETS READ FROM THE TASK AS IT
        # STANDS NOW. scene_events.json records fixtures by NAME and never by
        # position, so a clip carries no record of where its own targets sat.
        # Any clip recorded before the planes moved (y 0.145 -> 0.320) is
        # being scored against coordinates it never ran in, which is why the
        # placement column reads ~187 mm rather than a few mm. GRASP RATE and
        # POSITIONING ERROR do not depend on target position and are sound.
        print("   NOTE: placement is scored against the CURRENT targets. "
              "Clips recorded before the\n"
              "   planes moved are scored against coordinates they never ran "
              "in, so treat the\n"
              "   placement and variance columns as INVALID for those. Grasp "
              "rate and positioning\n"
              "   error are independent of target position and are sound.")
        out = os.path.join(r, "accuracy_table.json")
        json.dump({m: [dict(task=c["task"], n=c["n"], grasped=c["grasped"],
                            pos_err=c["pos_err"], place_err=c["place_err"],
                            smallest=c["smallest"]) for c in cs]
                   for m, cs in by.items()}, open(out, "w"), indent=2)
        print("-> %s" % os.path.relpath(out, WS))
    return 0


if __name__ == "__main__":
    sys.exit(main())
