#!/usr/bin/env python3
"""Run E1-E6 in sim with a fault injected MID-TRIAL, and check the consequences.

    python3 scripts/inject_experiment_faults.py            # all six
    python3 scripts/inject_experiment_faults.py e2 e6

This is the acceptance test for the whole hardening effort. Everything else
proves a mechanism fires; this proves the fired mechanism ends up in the data
correctly. For each experiment it asserts four things:

  1. the trial running when the fault landed is marked INVALID, with the
     CAUSE recorded -- not merely a shorter trial;
  2. the SESSION CONTINUES to the next trial rather than dying, because a
     session that aborts on the first rig hiccup costs the participant's
     whole visit;
  3. the analyser EXCLUDES invalid trials exactly as pre-registered, and says
     how many and why;
  4. a session in which EVERY trial is invalidated FAILS LOUDLY instead of
     producing an empty result set, which is indistinguishable from a null
     finding by anyone reading the output.

The fault is injected by publishing the topic the real fault would affect, so
nothing in the experiment code knows it is being tested.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
EXPERIMENTS = {
    "e1": ("e1_fitts_characterisation", "run_fitts.py", "analyse_fitts.py"),
    "e2": ("e2_autonomy_level", "run_autonomy_level.py",
           "analyse_autonomy_level.py"),
    "e3": ("e3_divided_attention", "run_divided_attention.py",
           "analyse_divided_attention.py"),
    "e4": ("e4_dof_recovery", "run_dof_recovery.py", "analyse_dof_recovery.py"),
    "e5": ("e5_intent_inference", "run_intent_inference.py",
           "analyse_intent_inference.py"),
    "e6": ("e6_vr_vs_mannequin", "run_vr_vs_mannequin.py",
           "analyse_vr_vs_mannequin.py"),
}
# Enough trials that invalidating one or two stays under the pre-registered
# 50% exclusion ceiling. Scripted trials run 0.4-0.7 s, so a fault held long
# enough to reliably overlap one would wipe out a 4-trial session entirely and
# the analyser would (correctly) refuse it.
N_TRIALS = 8
INJECT_AT_S = 6.0          # into the run, i.e. during a trial, not between


def env():
    e = dict(os.environ)
    e["ROS_DOMAIN_ID"] = "0"
    e.pop("ROS_LOCALHOST_ONLY", None)
    e["PYTHONUNBUFFERED"] = "1"
    return e


class Injector:
    """One ROS node for the whole run, reused across all six experiments.

    Three things this has to get right, each learned by getting it wrong:

    * A FIXED DELAY IS NOT MID-TRIAL. The six experiments have different
      startup costs and trial lengths, so one offset landed inside a trial in
      some runs and between trials in others, quietly testing nothing. The
      logger opens one CSV per trial at trial start, so the appearance of the
      Nth per-trial file is an unambiguous "a trial is running now" signal
      that needs no coupling into the experiment code.

    * THE PUBLISHER MUST ALREADY BE DISCOVERED. Building the node after the
      trigger costs ~1 s of DDS discovery, longer than a scripted trial, so
      the fault arrived after the trial it was meant to hit.

    * STATE MUST BE RESET BETWEEN EXPERIMENTS. recovery_manager raises each
      fault ONCE and ignores a re-raise until it resolves, and the e-stop
      LATCHES until /estop_reset. So the first experiment's fault silently
      disarmed the injection for the other five, and the table showed one
      invalid trial and five clean runs -- which looks like five passes.
    """

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger
        self._rclpy = rclpy
        self._String = String
        rclpy.init()
        self.node = Node("experiment_fault_injector")
        self.link = self.node.create_publisher(String, "/arm_link_status", 10)
        self.estop_state = None
        self.node.create_subscription(
            Bool, "/estop_state",
            lambda m: setattr(self, "estop_state", bool(m.data)), 10)
        self.reset_cli = self.node.create_client(Trigger, "/estop_reset")
        self.trial = None
        self.node.create_subscription(
            String, "/trial_state",
            lambda m: setattr(self, "trial", json.loads(m.data)), 20)
        self.spin(3.0)                       # settle discovery ONCE, up front

    def spin(self, t):
        t0 = time.monotonic()
        while time.monotonic() - t0 < t:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)

    def publish_link(self, payload, dur):
        m = self._String()
        m.data = json.dumps(payload)
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            self.link.publish(m)
            self._rclpy.spin_once(self.node, timeout_sec=0.05)

    def reset(self):
        """Known-good before each experiment: links up, e-stop cleared.

        Also drops the last /trial_state. It persists across experiments, so
        the previous run's final trial index still satisfied "trial 3 is
        running" and the fault fired during the NEXT experiment's startup --
        landing before any trial existed and invalidating nothing, which the
        table then scored as a pass.
        """
        self.trial = None
        from std_srvs.srv import Trigger
        self.publish_link(MID_TRIAL_FAULT["clear_payload"], 2.0)
        if self.reset_cli.wait_for_service(timeout_sec=5.0):
            f = self.reset_cli.call_async(Trigger.Request())
            t0 = time.monotonic()
            while not f.done() and time.monotonic() - t0 < 8.0:
                self._rclpy.spin_once(self.node, timeout_sec=0.05)
        self.publish_link(MID_TRIAL_FAULT["clear_payload"], 1.5)
        return self.estop_state is not True

    def arm_on_trial(self, experiment, trial_no=3, give_up_s=300.0):
        """Fire while trial `trial_no` is DEMONSTRABLY running.

        Earlier versions triggered on the per-trial log file appearing, which
        is the trial's START but says nothing about whether it is still going.
        Scripted trials run 0.4-2.5 s with gaps between them, so the fault
        repeatedly landed in a gap, invalidated nothing, and the table scored
        a pass for an injection that never happened. /trial_state is published
        throughout a trial, so waiting for it removes the guesswork.
        """
        def go():
            t0 = time.monotonic()
            while time.monotonic() - t0 < give_up_s:
                t = self.trial
                if (t and t.get("running")
                        and t.get("experiment") == experiment
                        and (t.get("trial_index") or 0) >= trial_no - 1):
                    break
                self._rclpy.spin_once(self.node, timeout_sec=0.02)
            else:
                return
            self.publish_link(MID_TRIAL_FAULT["payload"],
                              MID_TRIAL_FAULT["hold_s"])
            self.publish_link(MID_TRIAL_FAULT["clear_payload"], 2.0)
        t = threading.Thread(target=go, daemon=True)
        t.start()
        return t

    def close(self):
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# The fault used mid-trial. Arm-link loss is chosen as the representative
# runtime fault: it freezes BOTH arms, so it is unambiguous in the data, and
# it is recoverable, so the session can be checked for continuing afterwards.
MID_TRIAL_FAULT = dict(
    topic="/arm_link_status",
    payload=dict(arms=dict(left=dict(ip="192.168.1.10", up=True),
                           right=dict(ip="192.168.1.9", up=False,
                                      consecutive_failures=3))),
    clear_payload=dict(arms=dict(left=dict(ip="192.168.1.10", up=True),
                                 right=dict(ip="192.168.1.9", up=True))),
    # Long enough to certainly overlap a whole trial. Scripted trials run
    # 0.4-2.5 s with gaps between them, so a shorter fault repeatedly landed
    # in a gap and invalidated nothing -- which reads as a pass. With 8 trials
    # this invalidates 1-2, well under the pre-registered 50% ceiling.
    hold_s=3.0)


def run_experiment(key, results_dir, inj=None, timeout=420):
    folder, runner, _ = EXPERIMENTS[key]
    d = ROOT / "src/srl_experiments/experiments" / folder
    cmd = [sys.executable, str(d / runner),
           "--participant", "FAULTTEST", "--scripted",
           "--max-trials", str(N_TRIALS), "--results", str(results_dir)]
    th = inj.arm_on_trial(folder) if inj else None
    p = subprocess.run(cmd, capture_output=True, text=True, env=env(),
                       timeout=timeout, cwd=str(ROOT))
    if th:
        th.join(timeout=10.0)
    return p


def summary_files(results_dir):
    return sorted(Path(results_dir).rglob("*summary*.csv"))


def read_summary(results_dir):
    rows = []
    for f in summary_files(results_dir):
        with open(f) as fh:
            rows += list(csv.DictReader(fh))
    return rows


def analyse(key, results_dir):
    folder, _, analyser = EXPERIMENTS[key]
    d = ROOT / "src/srl_experiments/experiments" / folder
    files = [str(f) for f in summary_files(results_dir)]
    if not files:
        return None
    return subprocess.run([sys.executable, str(d / analyser)] + files,
                          capture_output=True, text=True, env=env(),
                          timeout=180, cwd=str(ROOT))


def all_invalid_copy(results_dir, dest):
    """A session where EVERY trial is invalid, built from the real one."""
    dest.mkdir(parents=True, exist_ok=True)
    out = dest / "all_invalid_summary.csv"
    rows = read_summary(results_dir)
    if not rows:
        return None
    for r in rows:
        r["valid"] = "0"
        r["invalid_reason"] = r.get("invalid_reason") or "rig fault, whole session"
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("which", nargs="*", default=sorted(EXPERIMENTS))
    ap.add_argument("--outdir", default="")
    a = ap.parse_args()

    base = Path(a.outdir or (ROOT / "recordings" / "fault_acceptance"))
    base.mkdir(parents=True, exist_ok=True)
    results = []
    inj = Injector()

    for key in a.which:
        rdir = base / key
        if rdir.exists():
            for f in rdir.rglob("*"):
                if f.is_file():
                    f.unlink()
        rdir.mkdir(parents=True, exist_ok=True)
        row = dict(exp=key, ran=False, trials=0, invalid=0, cause="",
                   continued=False, excluded=None, all_invalid_loud=None,
                   note="")
        try:
            inj.reset()
            p = run_experiment(key, rdir, inj)
            row["ran"] = (p.returncode == 0)
            if p.returncode != 0:
                row["note"] = (p.stderr or p.stdout).strip().splitlines()[-1][:110]
        except subprocess.TimeoutExpired:
            row["note"] = "run timed out"
            results.append(row)
            continue

        rows = read_summary(rdir)
        row["trials"] = len(rows)
        bad = [r for r in rows if str(r.get("valid", "1")) in ("0", "")]
        row["invalid"] = len(bad)
        # The invalid trial must name the INJECTED fault. Without this an
        # unrelated standing fault (the master goes stale in a sim run with no
        # Teensy) would invalidate a trial and the table would score it as a
        # pass for an injection that never landed.
        hit = [r for r in bad if "link DOWN" in (r.get("invalid_reason") or "")]
        row["cause_matches"] = bool(hit)
        if bad:
            row["cause"] = ((hit[0] if hit else bad[0])
                            .get("invalid_reason") or "")[:70]
        # The session must reach the end of the schedule despite the fault.
        row["continued"] = len(rows) >= N_TRIALS

        an = analyse(key, rdir)
        if an is not None:
            txt = (an.stdout or "") + (an.stderr or "")
            row["excluded"] = ("EXCLUDED as invalid" in txt
                               or "excluded" in txt.lower())
            row["analyser_rc"] = an.returncode

        # Every trial invalid must FAIL LOUDLY, not analyse to nothing.
        dest = base / (key + "_all_invalid")
        f = all_invalid_copy(rdir, dest)
        if f is not None:
            folder, _, analyser = EXPERIMENTS[key]
            d = ROOT / "src/srl_experiments/experiments" / folder
            q = subprocess.run([sys.executable, str(d / analyser), str(f)],
                               capture_output=True, text=True, env=env(),
                               timeout=180, cwd=str(ROOT))
            txt = (q.stdout or "") + (q.stderr or "")
            row["all_invalid_loud"] = (q.returncode != 0
                                       and "REFUSING TO ANALYSE" in txt)
        results.append(row)

    inj.close()
    print()
    print("SIX-EXPERIMENT MID-TRIAL FAULT INJECTION")
    print("fault injected WHILE TRIAL 3 IS RUNNING in each run "
          "(triggered off /trial_state): "
          "arm link DOWN to the right arm")
    print()
    hdr = ("%-4s %-6s %-7s %-8s %-10s %-9s %-11s %s"
           % ("EXP", "RAN", "TRIALS", "INVALID", "CONTINUED", "EXCLUDED",
              "ALLINV-LOUD", "CAUSE RECORDED"))
    print(hdr)
    print("-" * len(hdr))
    ok = True
    for r in results:
        good = (r["ran"] and r["invalid"] >= 1 and r.get("cause_matches")
                and r["continued"] and r["excluded"] and r["all_invalid_loud"])
        ok &= bool(good)
        print("%-4s %-6s %-7d %-8d %-10s %-9s %-11s %s"
              % (r["exp"], "yes" if r["ran"] else "NO", r["trials"],
                 r["invalid"], "yes" if r["continued"] else "NO",
                 {True: "yes", False: "NO", None: "-"}[r["excluded"]],
                 {True: "yes", False: "NO", None: "-"}[r["all_invalid_loud"]],
                 r["cause"] or r["note"] or "-"))
    print()
    print("ALL PASS" if ok else "FAILURES ABOVE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
