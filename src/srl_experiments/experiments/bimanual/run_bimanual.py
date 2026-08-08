#!/usr/bin/env python3
"""THE runner for the current experiment set: T2, T3, T5, T6, T7.

    ros2 run srl_experiments run_bimanual --task t3 --participant P01 \
        --condition direct --scenario S1
    ros2 run srl_experiments run_bimanual --task t7 --participant P01 --dry-run

Before this existed the bimanual tasks had protocols, layouts, verified
scenarios and analysis scripts but NO RUNNER -- every GUI button that claimed
to start one exited 2 against `run_experiment.sh`, which only ever accepted
e1..e6. The tasks looked runnable and were not.

WHICH SET IS CURRENT
--------------------
T2/T3/T5/T6/T7 are current. E1-E6 are SUPERSEDED as protocols; their
`run_*.py` scripts still work and are kept because several are the analysis
backend a T-task reuses (E2's autonomy-level machinery, E3's divided
attention). `run_experiment.sh` now dispatches both and says which is which.

T1 is subsumed by T7. T4 is geometrically blocked: 0 of 16 transfer points
are reachable by both arms, which is geometry and not wrist orientation.

SCENARIOS ARE REFUSED UNLESS VERIFIED
-------------------------------------
Every scenario in `scenarios_verified.yaml` carries `verified: true|false`
from a live `/compute_ik` check with the arms at home. This runner REFUSES an
unverified scenario rather than discovering it fails IK with a participant in
the harness -- a scenario that fails on the day wastes a session.
"""
import argparse
import os
import sys
import time

import numpy as np
import yaml

def _bimanual_dir():
    """Locate the SOURCE bimanual folder.

    `ros2 run` installs run_*.py and analyse_*.py as executables but does NOT
    copy their sibling modules, so the executed copy sits in a directory
    without coupled_metrics.py, targets.py or scenarios_verified.yaml. Resolve
    the source tree instead of assuming __file__'s neighbours exist.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [here]
    ws = os.environ.get("SRL_WS", os.path.expanduser("~/kortex_ws"))
    cands.append(os.path.join(ws, "src", "srl_experiments", "experiments",
                              "bimanual"))
    # build/ copy -> walk back to the workspace root
    parts = here.split(os.sep)
    if "build" in parts:
        root = os.sep.join(parts[:parts.index("build")])
        cands.append(os.path.join(root, "src", "srl_experiments",
                                  "experiments", "bimanual"))
    for c in cands:
        if os.path.exists(os.path.join(c, "coupled_metrics.py")):
            return c
    raise SystemExit(
        "cannot find the bimanual source folder (looked in: %s). Set SRL_WS "
        "to the workspace root." % ", ".join(cands))


HERE = _bimanual_dir()
PKG = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, PKG)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "t7_pursuit"))

import coupled_metrics as cm                                  # noqa: E402
from srl_experiments.conditions import assign_order           # noqa: E402
from srl_experiments.trial_logger import TrialLogger          # noqa: E402

TASKS = {
    "t2": ("T2 hold and fill", "t2_hold_and_fill", "discrete"),
    "t3": ("T3 rigid coupled carry", "t3_coordinated_carry", "transport"),
    "t5": ("T5 handover to wearer", "t5_handover_to_wearer", "handover"),
    "t6": ("T6 compliant coupled carry", "t6_compliant_carry", "transport"),
    "t7": ("T7 bimanual pursuit", "t7_pursuit", "pursuit"),
}
SCENARIO_KEY = {"t3": "T3_rigid", "t6": "T6_compliant", "t7": "T7_pursuit",
                "t5": "T5_handover", "t2": "T2_hold_fill"}
CONDITIONS = ("direct", "assisted", "shared")


def load_scenarios():
    p = os.path.join(HERE, "scenarios_verified.yaml")
    if not os.path.exists(p):
        raise SystemExit("scenarios_verified.yaml missing — run "
                         "scripts/verify_scenarios.py first")
    return yaml.safe_load(open(p))


def resolve_scenario(spec, task, want):
    """Return (name, dict) or raise with the reason. Never silently picks."""
    key = SCENARIO_KEY[task]
    tasks = spec.get("tasks", {})
    if key not in tasks:
        raise SystemExit(
            "no verified scenarios for %s. Present: %s"
            % (task, ", ".join(sorted(tasks))))
    pool = tasks[key]
    matches = [(n, s) for n, s in pool.items() if n.startswith(want + "_")
               or n == want]
    if not matches:
        raise SystemExit(
            "scenario %r not defined for %s. Available: %s"
            % (want, task, ", ".join(sorted(pool))))
    name, sc = matches[0]
    if not sc.get("verified"):
        raise SystemExit(
            "scenario %s for %s is NOT VERIFIED (%s). Refusing to run it: a "
            "scenario that fails IK on the day wastes a participant. Re-run "
            "scripts/verify_scenarios.py." % (name, task,
                                              sc.get("reason", "no reason")))
    return name, sc


# --------------------------------------------------------------- execution
class Session:
    """Owns TF/joint-state sampling. Falls back to a scripted operator when
    --scripted is given, so the whole pipeline is exercisable without a
    participant and without hardware."""

    def __init__(self, scripted=False):
        self.scripted = scripted
        self.node = None
        self.buf = None
        if scripted:
            return
        import rclpy
        from rclpy.node import Node
        import tf2_ros
        rclpy.init(args=None)
        self.rclpy = rclpy
        self.node = Node("run_bimanual")
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self.node)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def ee(self, arm, t=0.0, scenario=None):
        if self.scripted:
            return self._scripted_ee(arm, t, scenario)
        import rclpy.time
        try:
            tr = self.buf.lookup_transform(
                "world", "%s_end_effector_link" % arm, rclpy.time.Time())
            v = tr.transform.translation
            return np.array([v.x, v.y, v.z])
        except Exception:                                     # noqa: BLE001
            return None

    def _scripted_ee(self, arm, t, scenario):
        """A deterministic stand-in operator. Not a model of a human — it
        exists so the logging, metric and validity path can be exercised end
        to end without a participant."""
        sep = float((scenario or {}).get("sep", cm.SLING_NOMINAL_SEP))
        side = -0.5 if arm == "left" else 0.5
        z = 1.15 + 0.05 * math.sin(0.6 * t)
        wob = 0.004 * math.sin(1.7 * t + (0.0 if arm == "left" else 1.1))
        return np.array([side * sep, 0.35, z + wob])

    def spin(self, dt):
        if self.scripted:
            time.sleep(dt)
        else:
            t0 = time.monotonic()
            while time.monotonic() - t0 < dt:
                self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def close(self):
        if not self.scripted and self.node is not None:
            self.node.destroy_node()
            self.rclpy.shutdown()


import math                                                   # noqa: E402


def run_transport(sess, sc, dur, dt, log, coupling):
    """T3 / T6. Identical paths; only the coupling differs."""
    t, L, R = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        now = time.monotonic() - t0
        a, b = sess.ee("left", now, sc), sess.ee("right", now, sc)
        t.append(now)
        L.append(a)
        R.append(b)
        log.sample()
        sess.spin(dt)
    nominal = float(sc.get("sep", cm.SLING_NOMINAL_SEP))
    good = [(x, a, b) for x, a, b in zip(t, L, R)
            if a is not None and b is not None]
    if len(good) < 5:
        return None, "no EE data — is the sim running and TF publishing?"
    t = [g[0] for g in good]
    L = [g[1] for g in good]
    R = [g[2] for g in good]
    return cm.summarise_transport(np.array(t), L, R, nominal, coupling), None


def run_pursuit(sess, sc, dur, dt, log):
    """T7. Targets from the verified centres; error against the EE."""
    import targets as tg
    tt, TL, TR = tg.trial_targets(sc, dur, dt)
    EL, ER = [], []
    t0 = time.monotonic()
    for k in range(len(tt)):
        now = time.monotonic() - t0
        EL.append(sess.ee("left", now, sc))
        ER.append(sess.ee("right", now, sc))
        log.sample()
        sess.spin(dt)
    okL = [i for i, e in enumerate(EL) if e is not None]
    if len(okL) < 5:
        return None, "no EE data — is the sim running and TF publishing?"
    sl = cm.summarise_pursuit(tt[okL], [EL[i] for i in okL],
                              [TL[i] for i in okL])
    okR = [i for i, e in enumerate(ER) if e is not None]
    sr = cm.summarise_pursuit(tt[okR], [ER[i] for i in okR],
                              [TR[i] for i in okR]) if len(okR) >= 5 else {}
    out = {"rms_error_mm_left": sl["rms_error_mm"],
           "max_error_mm_left": sl["max_error_mm"],
           "phase_lag_s_left": sl["phase_lag_s"],
           "frac_on_target_left": sl["frac_on_target"],
           "rms_error_mm_right": sr.get("rms_error_mm", float("nan")),
           "speed_left": sc.get("speed_left", 0.0),
           "speed_right": sc.get("speed_right", 0.0),
           "unimanual": sc.get("unimanual", ""),
           "duration_s": sl["duration_s"]}
    return out, None


def run_single_arm(sess, sc, dur, dt, log, kind):
    """T5 handover, T2 hold-and-fill. Both are reach-and-place at heart; the
    metric that distinguishes them is discrete, and neither is instrumented
    for grasp detection yet — see the caveat printed at the end."""
    t0 = time.monotonic()
    n = 0
    while time.monotonic() - t0 < dur:
        log.sample()
        sess.spin(dt)
        n += 1
    return {"samples": n, "duration_s": time.monotonic() - t0,
            "kind": kind}, None


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Runner for the current bimanual set (T2/T3/T5/T6/T7).")
    ap.add_argument("--task", required=True, choices=sorted(TASKS))
    ap.add_argument("--participant", required=True,
                    help="anonymised ID. Names are refused by write_manifest.")
    ap.add_argument("--condition", default=None, choices=CONDITIONS,
                    help="omit to run all three in counterbalanced order")
    ap.add_argument("--scenario", default="S1")
    ap.add_argument("--participant-index", type=int, default=0)
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--results", default=None)
    ap.add_argument("--scripted", action="store_true",
                    help="scripted stand-in operator; no participant needed")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and exit; touches no hardware")
    a = ap.parse_args(argv)

    label, folder, kind = TASKS[a.task]
    spec = load_scenarios()
    sname, sc = resolve_scenario(spec, a.task, a.scenario)
    conds = ([a.condition] if a.condition
             else assign_order(list(CONDITIONS), a.participant_index))
    results = a.results or os.path.join(HERE, folder, "results")

    print("=" * 66)
    print("  %s" % label)
    print("=" * 66)
    print("  participant   %s" % a.participant)
    print("  scenario      %s   (verified)" % sname)
    print("  conditions    %s" % ", ".join(conds))
    print("  repeats       %d   duration %.0f s   rate %.0f Hz"
          % (a.repeats, a.duration, a.rate))
    print("  results       %s" % results)
    if kind == "transport":
        nominal = float(sc.get("sep", cm.SLING_NOMINAL_SEP))
        print("  coupling      %s   nominal separation %.0f mm"
              % ("compliant sling" if a.task == "t6" else "rigid tray",
                 1000 * nominal))
        if a.task == "t6":
            print("  sling         L=%.0f mm, ball %.0f mm, retained to "
                  "%.0f mm separation"
                  % (1000 * cm.SLING_L, 2000 * cm.BALL_R,
                     1000 * cm.max_secure_separation()))
        print("  waypoints     %s" % sc.get("path"))
    elif kind == "pursuit":
        print("  target speed  left %.2f  right %.2f m/s"
              % (sc.get("speed_left", 0), sc.get("speed_right", 0)))
        print("  centres       L %s  R %s"
              % (sc.get("centre_left"), sc.get("centre_right")))
    if a.dry_run:
        print("\n  DRY RUN — nothing launched, no hardware touched.")
        return 0

    sess = Session(scripted=a.scripted)
    dt = 1.0 / a.rate
    log = TrialLogger(results, a.participant, a.task, session=None)
    log.write_manifest()
    idx = 0
    rows = []
    try:
        for block_i, cond in enumerate(conds):
            for rep in range(a.repeats):
                idx += 1
                print("\n  trial %d — %s, %s, rep %d"
                      % (idx, cond, sname, rep + 1))
                # block is an INTEGER index in the filename template, not
                # the condition name.
                log.start(trial_index=idx, condition=cond, scenario=sname,
                          target_id=sname, arm="both", block=block_i)
                if kind == "transport":
                    m, err = run_transport(sess, sc, a.duration, dt, log,
                                           "compliant" if a.task == "t6"
                                           else "rigid")
                elif kind == "pursuit":
                    m, err = run_pursuit(sess, sc, a.duration, dt, log)
                else:
                    m, err = run_single_arm(sess, sc, a.duration, dt, log,
                                            kind)
                if err:
                    log.invalidate(err)
                    print("    INVALID: %s" % err)
                else:
                    for k, v in (m or {}).items():
                        print("    %-24s %s" % (k, v))
                    rows.append(dict(m, condition=cond, scenario=sname))
                log.finish()
    finally:
        sess.close()

    print("\n  %d valid trial(s) written under %s" % (len(rows), results))
    if kind in ("handover", "discrete"):
        print("  NOTE: %s's discrete outcome (grasp success, blocks placed) "
              "is NOT yet instrumented — this logs trajectories and timing "
              "only. Do not report a success rate from it." % a.task.upper())
    return 0


if __name__ == "__main__":
    sys.exit(main())
