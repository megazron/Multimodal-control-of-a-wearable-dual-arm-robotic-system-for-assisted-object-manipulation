#!/usr/bin/env python3
"""JOB B: the capability change from the pot repair, before and after, in numbers.

WHAT THIS CAN AND CANNOT ANSWER, stated up front because most of Job B lands
on the wrong side of the line.

ANSWERABLE HERE. Which channels the pipeline consumes, which it freezes, and
what the master's commandable input set collapses to when they are frozen.
That is a property of `degraded_mode` and `master_calibration` -- code and
geometry, no hardware -- so the before/after is exact arithmetic.

NOT ANSWERABLE HERE, and no amount of sim will change it. Master ACCURACY:
the azimuth/lateral residual, the position ladder R0-R5, and the smoothing
re-tune are all fits to RECORDED FRAMES, and every recording in this
repository predates the repair. Re-running them on pre-repair data would
measure the broken pots again and report it as the repaired capability.
"""
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_teleop import degraded_mode as dg          # noqa: E402
from srl_teleop import master_calibration as mc     # noqa: E402

BASE = dg.default_baseline_path(pkg_root=ROOT)   # NEWEST, not a fixed date
OUT = os.path.join(ROOT, "recordings/baselines/capability_delta.json")


def reach_extent(frozen):
    """Radial extent the master can command with `frozen` joints pinned.

    RADIAL EXTENT, NOT VOLUME. Azimuth and elevation are angular and survive
    every channel loss, so the only dimension a frozen pot can remove is the
    radius -- a convex hull cannot see this and reported an identical volume
    whether the shell had thickness or none.
    """
    rs = []
    g = np.linspace(-np.pi / 2, np.pi / 2, 9)
    for a in g:
        for b in g:
            for c in g:
                q = [0.0] * 7
                q[0], q[1], q[3] = a, b, c
                for i in frozen:
                    q[i] = 0.0
                rs.append(float(np.linalg.norm(mc.fk(q))))
    return min(rs), max(rs)


def main():
    stale = json.load(open(BASE))
    repaired = {k: dict(v, verdict="ALIVE", drop_pct=0.0, jump_pct=0.0)
                for k, v in stale.items() if isinstance(v, dict)}

    rows = []
    for name, b in (("BEFORE (stored baseline)", stale),
                    ("AFTER  (all 14 repaired)", repaired)):
        n = dg.n_coherent(b)
        eng, frozen, _ = dg.decide(b, "spherical")
        per = {}
        for arm in ("left", "right"):
            # `frozen` is a dict {arm: [0-based joint index, ...]}. The first
            # version guessed it was a set of (arm, index) tuples, silently
            # extracted [] for both arms, and reported "extent unchanged" --
            # a null result manufactured entirely by the instrument.
            idx = sorted(frozen.get(arm, []) if isinstance(frozen, dict)
                         else [])
            lo, hi = reach_extent(idx)
            per[arm] = dict(frozen=idx, reach_lo=lo, reach_hi=hi,
                            extent=hi - lo)
        rows.append(dict(label=name, coherent=n, degraded=bool(eng), arms=per))

    print("=" * 76)
    print("CAPABILITY DELTA FROM THE POT REPAIR")
    print("=" * 76)
    for r in rows:
        print("\n%s" % r["label"])
        print("  coherent channels : %d/14" % r["coherent"])
        print("  degraded mode     : %s" % ("ENGAGES" if r["degraded"]
                                            else "does not engage"))
        for arm, v in r["arms"].items():
            print("    %-5s frozen j%s  reach %.3f..%.3f m  extent %.3f m"
                  % (arm, "".join(str(i + 1) for i in v["frozen"]) or "-",
                     v["reach_lo"], v["reach_hi"], v["extent"]))

    b, a = rows[0], rows[1]
    print("\nDELTA")
    print("  coherent channels   %d -> %d" % (b["coherent"], a["coherent"]))
    for arm in ("left", "right"):
        print("  %-5s radial extent  %.3f m -> %.3f m"
              % (arm, b["arms"][arm]["extent"], a["arms"][arm]["extent"]))
    print("\n  NOT MEASURED HERE, and it needs the lab: azimuth/lateral")
    print("  residual, the R0-R5 position ladder, and the smoothing re-tune")
    print("  are fits to RECORDED FRAMES, and every recording predates the")
    print("  repair. They need a fresh capture with a MOVING master.")

    json.dump(rows, open(OUT, "w"), indent=2)
    print("\n  -> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
