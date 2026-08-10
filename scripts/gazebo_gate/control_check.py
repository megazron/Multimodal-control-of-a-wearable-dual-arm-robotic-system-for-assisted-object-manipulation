#!/usr/bin/env python3
"""POSITIVE CONTROL for the gate's command path.

The gate produced one complete run of confident numbers while every setpoint
was landing on a topic with no subscriber, and the gripper appeared to track a
closing sweep because the fingers were drooping under gravity at roughly the
rate the sweep was stepping.  So before any gate result is believed, this must
pass: command a joint, read it back, and require it to be where it was sent.

    python3 control_check.py [--engine bullet-featherstone]
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gate  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engine", default="bullet-featherstone")
    ap.add_argument("--step", type=float, default=0.001)
    a = ap.parse_args()

    out = gate.SCRATCH / ("control_%s" % a.engine)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    sdf = gate.build_model_sdf(out, None, 2.076)
    world = gate.build_world(out, gate.model_body(sdf), 1.0, a.engine, a.step,
                             cube_pose="5 5 0.02 0 0 0", pedestal_top=None)
    sim = gate.Sim(world, out / "gz.log")
    if not sim.wait_ready():
        sim.stop()
        sys.exit("gz sim never came up")

    checks = []
    try:
        # Load every setpoint while the world is PAUSED, then run.  No race.
        sim.hold_all()
        sim.play()
        time.sleep(6.0)
        rest = sim.read_joints()
        checks.append({"what": "joint_state readable",
                       "ok": len(rest) > 5, "n_joints": len(rest)})
        # Held at zero after a settle - the failure that started all this.
        held = {k: v for k, v in rest.items() if k in gate.ARM_JOINTS}
        worst = max((abs(p) for p, _ in held.values()), default=99)
        fastest = max((abs(v) for _, v in held.values()), default=99)
        checks.append({"what": "arm holds at zero",
                       "ok": worst < 0.05 and fastest < 0.05,
                       "worst_pos_rad": worst, "worst_vel_rad_s": fastest,
                       "per_joint": {k: [round(v[0], 4), round(v[1], 4)]
                                     for k, v in sorted(held.items())}})

        for j, q in [("joint_1", 0.50), ("joint_4", -0.40),
                     (gate.KNUCKLE, 0.40)]:
            r = sim.cmd_verified(j, q, tol=0.06, settle=4.0)
            r["what"] = "command reaches %s" % j
            checks.append(r)

        # NEGATIVE CONTROL: the topic the first version used.  It must NOT
        # move the joint -- if it does, the diagnosis above was wrong.
        js = sim.read_joints()
        before = js.get("joint_1", (float("nan"), 0))[0]
        gate.sh("%s && gz topic -t /gate/cmd/joint_1 -m gz.msgs.Double "
                "-p 'data: -0.9'" % gate.SRC)
        time.sleep(4.0)
        after = sim.read_joints().get("joint_1", (99, 0))[0]
        checks.append({"what": "wrong topic /gate/cmd/joint_1 does NOTHING",
                       "ok": (before == before) and abs(after - before) < 0.03,
                       "before": before, "after": after})
    finally:
        sim.stop()

    res = {"engine": a.engine, "checks": checks,
           "verdict": "PASS" if all(c["ok"] for c in checks) else "FAIL"}
    (out / "control.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
