#!/usr/bin/env python3
"""channel_report.py — the verdict table, from a capture CSV. Nothing else.

    python3 channel_report.py <all_segments.csv> [--baseline base.json]
                              [--save-baseline base.json]

This is the objective acceptance test for the wiring repair. Run it before a
repair attempt, run it after, diff the table.

THE RULES, and why each exists.

  Samples are cleaned first: exact 0.0 is the firmware's dropout marker, and
  anything outside [0,360] is a serial parse glitch (0.2-0.8% of the
  2026-08-06 capture, in bursts up to 27 samples).

  RANGE is CIRCULAR -- the smallest arc containing every sample, bounded by
  360 by construction. Unwrap-and-subtract is wrong for a rotary sensor:
  across a dropout gap the unwrap cannot know how far the joint travelled and
  the offset runs away. That is what produced 314363 deg.

  STEPS are computed between DISTINCT sensor updates that are ADJACENT in
  time. The recorder samples faster than the pots update, so most consecutive
  rows are bit-identical; including them drove the reported noise floor to
  exactly 0.000 on all 14 channels, which is a property of the recorder.

  VERDICT, in order:
    DEAD          circular range < 5 deg, or dropouts > 90%.
    INCOHERENT    more than 5% of updates jump > 60 deg. Values arrive and
                  span a wide range but consecutive samples are unrelated --
                  the channel is NOT tracking. Calling this "alive with a big
                  range" is how a broken channel gets trusted.
    INTERMITTENT  dropouts > 2%, otherwise coherent.
    ALIVE         coherent, range >= 20 deg, dropouts <= 2%.
    SUSPECT       anything else, named rather than bucketed silently.
"""
import argparse
import csv
import json
import sys

import numpy as np

JUMP_DEG = 60.0
POTS = [(arm, j) for arm in ('left', 'right') for j in range(1, 8)]
PFX = {'left': 'l', 'right': 'r'}


def load(path):
    with open(path) as fh:
        r = csv.reader(fh)
        hdr = next(r)
        rows = list(r)
    idx = {k: i for i, k in enumerate(hdr)}
    out = {}
    for k, i in idx.items():
        col = [row[i] if i < len(row) else '' for row in rows]
        try:
            out[k] = np.array([float(x) if x != '' else np.nan for x in col])
        except ValueError:
            out[k] = np.array(col, dtype=object)
    return out


def seg_slices(d):
    seg = d['segment']
    out, cur, start = {}, None, 0
    for i, s in enumerate(seg):
        if s != cur:
            if cur is not None:
                out[cur] = (start, i)
            cur, start = s, i
    if cur is not None:
        out[cur] = (start, len(seg))
    return out


def circ_range(v):
    v = v[np.isfinite(v) & (v != 0.0) & (v >= 0) & (v <= 360)]
    if v.size < 2:
        return 0.0
    a = np.sort(np.mod(v, 360.0))
    gaps = np.diff(np.concatenate([a, [a[0] + 360.0]]))
    return float(360.0 - gaps.max())


def steps(v):
    """|delta| between distinct, time-adjacent, valid updates."""
    good = np.isfinite(v) & (v != 0.0) & (v >= 0) & (v <= 360)
    idx = np.flatnonzero(good)
    out = []
    prev_i = prev_v = None
    for i in idx:
        if prev_i is None:
            prev_i, prev_v = i, v[i]
            continue
        if v[i] != prev_v:
            if good[prev_i:i + 1].all():
                dv = abs(v[i] - prev_v)
                out.append(min(dv, 360.0 - dv))
            prev_v = v[i]
        prev_i = i
    return np.array(out)


def analyse(d, segs):
    res = {}
    for arm, j in POTS:
        ch = '%s_j%d' % (PFX[arm], j)
        seg = 'A_%s_j%d' % (arm, j)
        if ch not in d or seg not in segs:
            res[ch] = dict(verdict='NO DATA', why='segment %s absent' % seg)
            continue
        a, b = segs[seg]
        v = d[ch][a:b]
        n = v.size
        drop = float((v == 0.0).sum()) / max(1, n)
        rng = circ_range(v)
        st = steps(v)
        rest = np.concatenate([steps(d[ch][x:y])
                               for s2, (x, y) in segs.items()
                               if s2.startswith('A_') and s2 != seg] or
                              [np.array([])])
        rest = rest[rest < 20.0] if rest.size else rest
        noise = float(np.median(rest)) if rest.size else float('nan')
        if st.size < 5 or rng < 5.0:
            v_, w = 'DEAD', 'range %.1f deg, %d updates' % (rng, st.size)
        elif drop > 0.90:
            v_, w = 'DEAD', '%.0f%% dropouts' % (100 * drop)
        elif (st > JUMP_DEG).mean() > 0.05:
            v_, w = 'INCOHERENT', ('%.0f%% of updates jump >%.0f deg'
                                   % (100 * (st > JUMP_DEG).mean(), JUMP_DEG))
        elif drop > 0.02:
            v_, w = 'INTERMITTENT', '%.1f%% dropouts' % (100 * drop)
        elif rng >= 20.0:
            v_, w = 'ALIVE', 'range %.0f deg' % rng
        else:
            v_, w = 'SUSPECT', 'range only %.1f deg' % rng
        res[ch] = dict(verdict=v_, why=w, updates=int(st.size),
                       drop_pct=round(100 * drop, 1), range_deg=round(rng, 1),
                       jump_pct=round(100 * float((st > JUMP_DEG).mean()), 1)
                       if st.size else None,
                       step_p50=round(float(np.median(st)), 2) if st.size else None,
                       noise_deg=round(noise, 3) if noise == noise else None)
    return res


def show(res, baseline=None):
    print("  chan   upd  drop%   range   jump%  p50step  noise   VERDICT"
          + ("      vs BASELINE" if baseline else ""))
    changed = 0
    for arm, j in POTS:
        ch = '%s_j%d' % (PFX[arm], j)
        r = res[ch]
        diff = ""
        if baseline and ch in baseline:
            was = baseline[ch]['verdict']
            if was != r['verdict']:
                diff = "   %s -> %s  CHANGED" % (was, r['verdict'])
                changed += 1
            else:
                diff = "   (%s)" % was
        print("  %-5s %4s %6s %7s %7s %8s %6s   %-12s %s%s"
              % (ch, r.get('updates', '-'), r.get('drop_pct', '-'),
                 r.get('range_deg', '-'), r.get('jump_pct', '-'),
                 r.get('step_p50', '-'), r.get('noise_deg', '-'),
                 r['verdict'], r['why'], diff))
    if baseline:
        print("\n  %d channel(s) changed verdict vs baseline" % changed)
    n = {}
    for r in res.values():
        n[r['verdict']] = n.get(r['verdict'], 0) + 1
    print("\n  summary: " + ", ".join("%s %d" % kv for kv in sorted(n.items())))
    usable = sum(v for k, v in n.items() if k in ('ALIVE', 'INTERMITTENT'))
    print("  usable (ALIVE or INTERMITTENT): %d of 14" % usable)
    return changed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--baseline', default='')
    ap.add_argument('--save-baseline', default='')
    a = ap.parse_args()
    d = load(a.csv)
    segs = seg_slices(d)
    res = analyse(d, segs)
    base = json.load(open(a.baseline)) if a.baseline else None
    print("CHANNEL VERDICT TABLE  (%s)" % a.csv)
    print()
    show(res, base)
    if a.save_baseline:
        json.dump(res, open(a.save_baseline, 'w'), indent=1)
        print("\n  baseline saved -> %s" % a.save_baseline)
    return 0


if __name__ == '__main__':
    sys.exit(main())
