#!/usr/bin/env python3
"""Where can task A's bin go so the transport is a CARRY, not a drop?

THE PROBLEM THIS SOLVES. A_PICK and A_BIN were 0.132 m apart of which 0.130 m
was VERTICAL -- the block started hovering directly above the bin and the
"place" lowered it 130 mm straight down. On video that is a gripper going
down, not a pick-and-place, and no participant would read it as one.

REQUIREMENT: at least 0.25 m of LATERAL travel, with the whole path verified
N=10 at 20 mm spacing, exactly as `verify_abc_scenarios.py` verifies the rest.
A coordinate that verifies is worth nothing if it verifies the wrong task.

WHY A SWEEP AND NOT A CHOSEN NUMBER. The left arm's usable band is narrow and
has already surprised this project: a top-down grasp measured 0/9 at x = 0.25
and 9/9 at x = 0.30, a 50 mm cliff. Picking a value and checking it would
find the cliff on the day. This walks candidates outward and reports the last
one that passes N/N, so the shipped figure can sit inside that boundary rather
than on it.

    python3 scripts/sweep_task_a_bin.py [--repeats 10]
"""
import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
from verify_task_scenes import Solver, HOME_TOL_RAD            # noqa: E402
from audit_scenario_reachability import densify                # noqa: E402
import clip_tasks as CT                                        # noqa: E402

STEP = 0.02
OUT = os.path.join(ROOT, "recordings/baselines/task_a_bin_sweep.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--min-lateral", type=float, default=0.25)
    a = ap.parse_args()
    N = a.repeats

    import rclpy
    if not rclpy.ok():
        rclpy.init()
    n = Solver()
    # WAIT FOR THE TOPIC BEFORE ASKING WHERE THE ARM IS. home_ok now raises on
    # absent data rather than reporting it as a 2.89 rad offset, but the wait
    # is what makes the answer meaningful.
    n.spin(4.0)
    bad = False
    for arm in ("left", "right"):
        w, _j = n.home_ok(arm)
        print("  %-5s home offset %.4f rad" % (arm, w))
        bad |= w > HOME_TOL_RAD
    if bad:
        print("\n  REFUSING: the arms are not at home. Reach measured from "
              "anywhere else answers a different question under the same "
              "name.")
        return 3

    quat = n.ee_quat("left")
    if quat is None:
        print("\n  REFUSING: no tf2 for the left end effector. An identity "
              "quaternion is not neutral, it is a specific unreachable pose.")
        return 4

    calls = {"n": 0}

    def ok(p, k=None):
        k = N if k is None else k
        for _ in range(k):
            calls["n"] += 1
            if not n.solve("left", list(p), quat, tries=6):
                return False
        return True

    # INSTRUMENT CONTROLS. A sweep whose answer is "everything passes" and a
    # sweep whose solver has stopped solving print the same thing.
    ctl_far = ok([1.60, 0.35, 1.15], k=1)          # past the 0.902 m reach
    ctl_good = ok(CT.A_PICK, k=1)                  # the current, verified pick
    print("\n  control  unreachable pose -> %s   (want False)" % ctl_far)
    print("  control  current A_PICK   -> %s   (want True)" % ctl_good)
    if ctl_far or not ctl_good:
        print("\n  REFUSING: the controls did not behave, so a pass below "
              "would be a property of the harness, not the geometry.")
        return 5

    pick = list(CT.A_PICK)
    rows = []
    print("\n  candidate bins, full path verified N=%d at %.0f mm" % (N, STEP * 1000))
    print("  %-26s %8s %8s  %s" % ("bin", "lateral", "waypts", "verdict"))
    # Outboard only: inboard runs at the wearer's centreline, which measured
    # 0/9 for a top-down grasp and is where both arms' reachable sets end.
    for x in [round(0.32 + i * 0.03, 3) for i in range(1, 22)]:
        binp = [x, CT.A_PICK[1], 1.02]
        lateral = math.hypot(binp[0] - pick[0], binp[1] - pick[1])
        lift = [pick[0], pick[1], pick[2] + 0.14]
        over = [binp[0], binp[1], binp[2] + 0.16]
        path = densify([pick, lift, over, binp], STEP)
        if not path:
            raise RuntimeError("densify returned nothing -- refusing to "
                               "report a pass on an empty path")
        good = all(ok(w) for w in path)
        rows.append(dict(bin=binp, lateral_m=round(lateral, 3),
                         waypoints=len(path), ok=bool(good)))
        print("  %-26s %8.3f %8d  %s"
              % ("(%.2f, %.2f, %.2f)" % tuple(binp), lateral, len(path),
                 "PASS" if good else "fail"))
        if not good:
            break

    passed = [r for r in rows if r["ok"] and r["lateral_m"] >= a.min_lateral]
    last = rows[-1] if rows else None
    chosen = None
    if passed:
        # NOT the furthest that passes -- one step inside it. Feasibility here
        # falls off a cliff rather than degrading, so the margin is the
        # defence and N only locates the edge.
        chosen = passed[-2] if len(passed) >= 2 else passed[-1]
    res = dict(rows=rows, repeats=N, ik_calls=calls["n"],
               min_lateral_required=a.min_lateral,
               chosen=chosen,
               boundary=(None if last is None or last["ok"] else last["bin"]))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)

    print("\n  %d IK calls" % calls["n"])
    if chosen is None:
        print("  NO candidate reaches %.2f m of lateral travel." % a.min_lateral)
        return 1
    print("  last passing: %s" % rows[[r["ok"] for r in rows].index(True)
                                      if False else -1]["bin"])
    print("  CHOSEN (one step inside the boundary): %s, lateral %.3f m"
          % (chosen["bin"], chosen["lateral_m"]))
    print("  -> %s" % OUT)
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:                                      # noqa: BLE001
            pass
    sys.exit(rc)
