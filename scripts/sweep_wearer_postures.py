#!/usr/bin/env python3
"""Run the centre sweep once per wearer arm posture, one stack each, and merge.

    python3 scripts/sweep_wearer_postures.py
    python3 scripts/sweep_wearer_postures.py --postures down folded

ONE STACK PER POSTURE, and that is not caution for its own sake. The posture
lives in the URDF, which move_group loads once at launch; changing the
environment variable under a running stack changes nothing at all, and the
sweep would report the same wearer under five names. Each run therefore brings
its own stack up with `SRL_WEARER_ARMS` set, and the measurement refuses to
report unless the description it reads back holds the posture it was asked for.

`down` is the regression control and it runs first: it is the shipped wearer,
so it must return the shipped answer. If it does not, the instrument changed
and every other row in the table is about the instrument, not about postures.

WHICH SHIPPED ANSWER, THOUGH -- and this bit was not free. `centre_vs_height.json`
records left 0.425 / right 0.400 at z = 1.120, and this sweep returns
0.325 / 0.450. A result that contradicts an earlier measurement is an
instrument check until the two are reconciled, so the reconciliation was run:
`measure_centre_vs_height.py` itself, unchanged, over the same range at the
same height, returns **0.325 / 0.450** today. The two instruments agree with
each other and disagree with the stored file, which is therefore stale.
"""
import argparse
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/srl_teleop'))

from srl_teleop import wearer_posture as WP            # noqa: E402

MERGED = os.path.join(ROOT, 'recordings/baselines/centre_vs_wearer_posture.json')
# What `down` must reproduce. These are NOT the numbers in
# centre_vs_height.json, which say 0.425 / 0.400: that file is stale. Re-running
# ITS OWN script unchanged at z = 1.120 today returns 0.325 / 0.450, matching
# this sweep exactly, so the two instruments agree and the stored file does not.
# The likeliest cause is the 2026-08-15 home change, which moved the IK seed and
# therefore which null-space branch the solver lands in. See findings.md.
PUBLISHED = {'left': 0.325, 'right': 0.450}


def run_one(posture, extra, timeout):
    env = dict(os.environ)
    env[WP.ENV_VAR] = posture
    out = os.path.join(ROOT,
                       'recordings/baselines/centre_posture_%s.json' % posture)
    cmd = [sys.executable, os.path.join(ROOT, 'scripts/sim_session.py'),
           '--stack', 'moveit', '--run-timeout', str(timeout), '--',
           sys.executable,
           os.path.join(ROOT, 'scripts/measure_centre_vs_wearer_posture.py'),
           '--posture', posture] + list(extra)
    print('\n' + '=' * 72)
    print('POSTURE %s -- %s' % (posture, WP.POSTURES[posture]['doc']))
    print('=' * 72, flush=True)
    r = subprocess.run(cmd, env=env, cwd=ROOT)
    if r.returncode != 0:
        print('posture %s: the run exited %d' % (posture, r.returncode))
    if not os.path.exists(out):
        return None
    with open(out) as fh:
        return json.load(fh)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--postures', nargs='*',
                    default=['down', 'behind', 'folded', 'out', 'none'])
    ap.add_argument('--full-repeats', type=int, default=10)
    ap.add_argument('--run-timeout', type=int, default=7200)
    ap.add_argument('--merge-only', action='store_true',
                    help='rebuild the comparison from the per-posture files '
                         'already on disk, without running anything. Used when '
                         'a posture had to be re-run on its own.')
    a = ap.parse_args()

    extra = ['--full-repeats', str(a.full_repeats)]
    results = {}
    for posture in a.postures:
        if a.merge_only:
            path = os.path.join(
                ROOT, 'recordings/baselines/centre_posture_%s.json' % posture)
            results[posture] = (json.load(open(path))
                                if os.path.exists(path) else None)
            continue
        results[posture] = run_one(posture, extra, a.run_timeout)

    print('\n' + '=' * 72)
    print('MIN |x| THAT IS REACHABLE AND CLEAR OF THE 150 mm FLOOR, BY POSTURE')
    print('=' * 72)
    print('%-8s %-24s %-10s %-10s %-10s %s'
          % ('posture', '', 'left s1', 'left full', 'right s1', 'right full'))
    rows = {}
    for posture, d in results.items():
        if not d or d.get('refused'):
            print('%-8s %-24s  REFUSED: %s'
                  % (posture, '', (d or {}).get('refused', 'no result')))
            continue
        z = '%.3f' % d['zs'][0]
        s1 = {arm: d['stage1'][z][arm]['innermost_safe_x'] for arm in
              ('left', 'right')}
        full = d['innermost_safe_confirmed']
        rows[posture] = dict(stage1=s1, full=full,
                             binds={arm: d['stage1'][z][arm]
                                    ['binds_at_innermost_safe']
                                    for arm in ('left', 'right')},
                             centre_cells=len(d['centre_cells']),
                             home=d['home_clearance_m'])
        fmt = lambda v: '----' if v is None else '%.3f' % v      # noqa: E731
        print('%-8s %-24s %-10s %-10s %-10s %s'
              % (posture, d['posture_doc'][:24], fmt(s1['left']),
                 fmt(full['left']), fmt(s1['right']), fmt(full['right'])))

    ctl_ok = None
    if 'down' in rows:
        got_s1, got_full = rows['down']['stage1'], rows['down']['full']
        ctl_ok = all(got_s1[arm] is not None
                     and abs(got_s1[arm] - PUBLISHED[arm]) < 1e-6
                     and got_full[arm] is not None
                     and abs(got_full[arm] - PUBLISHED[arm]) < 1e-6
                     for arm in PUBLISHED)
        print('\nREGRESSION CONTROL: `down` must reproduce the published '
              'columns %s, at BOTH stages -- centre_vs_height.json confirms '
              'them over the full path too.' % PUBLISHED)
        print('   stage 1 %s ; full path %s -- %s'
              % (got_s1, got_full, 'MATCH' if ctl_ok else 'DOES NOT MATCH'))
        if not ctl_ok:
            print('   The instrument moved. Every other row of this table is '
                  'about the instrument until this is reconciled.')

    if 'folded' in rows and 'down' in rows:
        f, d = rows['folded']['full'], rows['down']['full']
        worse = all((f[arm] is None) or (d[arm] is not None
                                         and f[arm] >= d[arm] - 1e-9)
                    for arm in ('left', 'right'))
        print('\nDIRECTION CONTROL: `folded` puts the wearer\'s forearms in '
              'the work volume, so it cannot be BETTER than `down`.')
        print('   folded %s vs down %s -- %s'
              % (f, d, 'as expected' if worse else 'BACKWARDS, so the posture '
                 'is not reaching the measurement'))

    out = dict(published_down=PUBLISHED, rows=rows,
               down_reproduces_published=ctl_ok,
               postures={p: WP.POSTURES[p]['doc'] for p in results},
               per_posture_files={
                   p: 'recordings/baselines/centre_posture_%s.json' % p
                   for p in results})
    with open(MERGED, 'w') as fh:
        json.dump(out, fh, indent=2)
    print('\n-> %s' % MERGED)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
