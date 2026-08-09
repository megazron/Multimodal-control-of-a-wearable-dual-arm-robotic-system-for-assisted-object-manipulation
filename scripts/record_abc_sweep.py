#!/usr/bin/env python3
"""PART 5 -- the 7-angle sweep: three tasks x four modes, launched THROUGH THE GUI.

    python3 scripts/record_abc_sweep.py [--resume] [--only 04_shared_autonomy]

WHAT MAKES THIS DIFFERENT FROM EVERY EARLIER RECORDING. Job A found that no
clip in this repository had ever been recorded through a control mode: they
all came from a recorder that calls /compute_ik directly, so no follower, no
clutch, no anchor and no safety guard was in the path, and the mode axis of
the tree was empty because nothing could fill it. Here the motion is started
by pressing the GUI's own button -- `Gui.on_launch(spec)` on the real manifest
-- so a clip filed under a mode really did travel that mode's command path,
and the recording doubles as proof the GUI drives the system.

THE ISOLATION RULE IS ENFORCED HERE, NOT DOCUMENTED.
======================================================================
Measured: after a VR run, modes 01, 04 and 06 every one reported 0.0000 m of
travel, and killing `vr_pose_mapper` restored them to 0.2000 m immediately.
The mapper stays ENGAGED when its run ends and keeps publishing on
/master_arm_pose_<arm> -- the same topic mode 01 uses -- so it holds the arm
at its last command. That is the project's one-source-at-a-time rule at the
PROCESS level.

Left as a documented step, this would fail silently and expensively: every
mode after the first records a stationary arm, all 84 clips render, the
verifier passes them on colour, and the sweep looks complete. So before each
mode records anything:

  1. TEAR DOWN every upstream this mode does not need (procscan.kill_all,
     which cannot kill its own shell).
  2. START only the upstreams this mode does need, and wait for them.
  3. COUNT PUBLISHERS on the follower's input topic and REFUSE TO RECORD
     unless the count is exactly what this mode predicts. Not "check it is
     up" -- count it, and compare against a number written down in advance.

A wrong count aborts the mode with the number in the message. It is not a
warning: a warning here produces a directory full of plausible, worthless
clips.

Xvfb, NEVER WSLg's :0 -- x11grab there records BLACK, measured, because the
compositor never puts window contents in the X root window. Seven displays,
one per view, so the motion runs ONCE and all seven angles see the same run.
"""
import argparse
import json
import os
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))
from srl_teleop import procscan                              # noqa: E402
from srl_teleop import gui_launch_specs as gls               # noqa: E402
import record_rviz as rr                                     # noqa: E402

OUT = os.path.join(WS, "recordings/verification")
PROGRESS = os.path.join(OUT, "abc_sweep_progress.json")

TASKS = ("a", "b", "c")
SCENARIO = {"a": "S3_both", "b": "S2_full_lift", "c": "S1_both_slow"}

# Per mode: which upstream nodes it needs, and how many publishers the
# FOLLOWER's input topic must have while it runs. The expected counts are the
# whole point -- they are predictions, written before the run, that the sweep
# refuses to proceed without.
MODES = {
    "01_master_teleop": dict(
        needs=[], follower_topic="/master_arm_pose_%s", expect_pubs=1,
        note="the runner itself is the only publisher"),
    "02_vr_teleop": dict(
        needs=[("vr_pose_mapper",
                ["ros2", "run", "srl_vr_teleop", "vr_pose_mapper"])],
        follower_topic="/master_arm_pose_%s", expect_pubs=1,
        note="the MAPPER is the only publisher; the runner drives it "
             "upstream on /vr/controller_pose_*"),
    "04_shared_autonomy": dict(
        needs=[], follower_topic="/autonomy/assist_pose_%s", expect_pubs=1,
        note="the runner itself"),
    "06_full_autonomy": dict(
        needs=[], follower_topic="/autonomy/assist_pose_%s", expect_pubs=1,
        note="the runner itself, commanded by a spoken instruction"),
}

# Every upstream any mode can start. Anything not in a mode's `needs` is torn
# down before that mode runs -- listed explicitly so a new upstream cannot be
# forgotten by omission.
ALL_UPSTREAMS = ["lib/srl_vr_teleop/vr_pose_mapper",
                 "lib/srl_autonomy/autonomy_executive",
                 "lib/srl_vr_teleop/quest_vendor_bridge"]


def log(msg):
    print(msg, flush=True)


def load_progress():
    try:
        return json.load(open(PROGRESS))
    except Exception:                                         # noqa: BLE001
        return {}


def save_progress(p):
    os.makedirs(os.path.dirname(PROGRESS), exist_ok=True)
    tmp = PROGRESS + ".tmp"
    json.dump(p, open(tmp, "w"), indent=2)
    os.replace(tmp, PROGRESS)      # atomic: a killed sweep never truncates it


# ===========================================================================
#  ISOLATION
# ===========================================================================
class Graph:
    """A tiny ROS node used only to COUNT publishers. Nothing else."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.n = Node("abc_sweep_graph")

    def pubs(self, topic):
        """Publisher count, WITHOUT spinning.

        `count_publishers` reads the middleware's graph cache, which the RMW
        keeps current on its own thread -- it needs time, not a spin. Spinning
        here raised "Executor is already spinning" because the GUI's Bus owns
        the default executor on another thread, which is the same collision
        that killed grasp_generator at startup once.

        The maximum over several samples is deliberate: discovery is
        asynchronous, and a count taken before matching completes is a count
        of the question rather than the answer.
        """
        best = 0
        for _ in range(8):
            time.sleep(0.25)
            best = max(best, self.n.count_publishers(topic))
        return best

    def close(self):
        try:
            self.n.destroy_node()
        except Exception:                                     # noqa: BLE001
            pass


def isolate(mode, graph, started):
    """Make `mode` the only source. Returns (ok, message).

    Tears down every upstream this mode does not need, starts the ones it
    does, then COUNTS publishers on the follower's input topic and compares
    against the number this mode predicted.
    """
    spec = MODES[mode]
    need_pats = {n[0] for n in spec["needs"]}

    killed = []
    for pat in ALL_UPSTREAMS:
        if any(p in pat for p in need_pats):
            continue
        t, _ = procscan.kill_all(pat)
        if t:
            killed.append("%s x%d" % (pat.rsplit("/", 1)[-1], len(t)))
            started.pop(pat, None)
    if killed:
        log("   torn down: %s" % ", ".join(killed))
        time.sleep(2.0)

    for name, argv in spec["needs"]:
        pat = "lib/srl_vr_teleop/%s" % name if "vr" in name else name
        if procscan.count(pat) == 0:
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            p = subprocess.Popen(argv, env=env, start_new_session=True,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
            started[pat] = p
            log("   started %s (pid %d)" % (name, p.pid))
            time.sleep(9.0)

    # THE COUNT. Before the runner starts, the follower's input topic should
    # carry ONLY this mode's upstream -- 0 for the modes whose upstream IS the
    # runner, 1 for VR where the mapper publishes it.
    expect_idle = 1 if mode == "02_vr_teleop" else 0
    bad = []
    for arm in ("left", "right"):
        topic = spec["follower_topic"] % arm
        n = graph.pubs(topic)
        if n != expect_idle:
            bad.append("%s has %d publisher(s), expected %d before the run"
                       % (topic, n, expect_idle))
    if bad:
        return False, "; ".join(bad)
    return True, ("isolated: %s idle publishers as predicted (%s)"
                  % (expect_idle, spec["note"]))


# ===========================================================================
#  THE SWEEP
# ===========================================================================
def build_gui():
    """The real GUI, offscreen. Clips are started by pressing ITS buttons."""
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    import rclpy
    import threading
    import srl_gui
    from PyQt5.QtWidgets import QApplication
    if not rclpy.ok():
        rclpy.init()
    bus = srl_gui.Bus()
    threading.Thread(target=lambda: rclpy.spin(bus), daemon=True).start()
    app = QApplication.instance() or QApplication([sys.argv[0]])

    class A:
        no_rviz, single_rviz, self_test_exit = True, False, False
        embed_rviz, dual_rviz = False, False
    g = srl_gui.Gui(bus, A())
    return app, g


def run_one(app, gui, task, mode, out_dir):
    """Press the GUI button for (task, mode) and wait for it to finish."""
    key = "abc_%s_%s" % (task, mode[:2])
    spec = next((s for s in gui.specs if s.key == key), None)
    if spec is None:
        return False, "no GUI spec %r" % key
    if not spec.enabled:
        return False, "GUI button disabled: %s" % spec.disabled_reason
    before = len(gui.jobs)
    gui.on_launch(spec)
    app.processEvents()
    if len(gui.jobs) == before:
        return False, "the GUI refused to launch it (see its log)"
    label, proc = gui.jobs[-1]
    t0 = time.monotonic()
    while proc.poll() is None and time.monotonic() - t0 < 300:
        app.processEvents()
        time.sleep(0.25)
    rc = proc.poll()
    if rc is None:
        proc.kill()
        return False, "timed out after 300 s"
    # THE RUNNER EXITS NON-ZERO IF NO ARM MOVED. That is the whole reason the
    # clip can be trusted: a stationary arm is not recorded as a success.
    return rc == 0, ("run exited %s" % rc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--only", default=None, help="one mode key")
    ap.add_argument("--tasks", default="abc")
    ap.add_argument("--settle-s", type=float, default=3.0)
    a = ap.parse_args()

    modes = [a.only] if a.only else list(MODES)
    tasks = [t for t in TASKS if t in a.tasks]
    prog = load_progress() if a.resume else {}

    log("=" * 74)
    log("ABC SWEEP -- %d modes x %d tasks x %d angles"
        % (len(modes), len(tasks), len(rr.VIEWS) + 1))
    log("=" * 74)

    graph = Graph()
    started = {}
    app, gui = build_gui()
    done = failed = skipped = 0
    try:
        for mode in modes:
            log("\n== MODE %s" % mode)
            ok, why = isolate(mode, graph, started)
            log("   %s" % why)
            if not ok:
                # NOT A WARNING. Recording here would fill a directory with
                # plausible clips of a stationary arm.
                log("   ABORTING THIS MODE -- isolation not satisfied.")
                failed += len(tasks)
                continue
            for task in tasks:
                scen = SCENARIO[task]
                out_dir = os.path.join(OUT, mode, task.upper(), scen)
                pkey = "%s/%s/%s" % (mode, task, scen)
                if a.resume and prog.get(pkey, {}).get("ok"):
                    log("   %-28s SKIP (already recorded)" % pkey)
                    skipped += 1
                    continue
                os.makedirs(out_dir, exist_ok=True)
                log("   %-28s recording..." % pkey)
                rr.ensure_display(os.path.join(out_dir, "rviz"),
                                  gripper_arm="left")
                time.sleep(a.settle_s)
                grabs = rr.start_grabs(out_dir)
                time.sleep(1.0)
                good, msg = run_one(app, gui, task, mode, out_dir)
                time.sleep(1.0)
                rr.stop_grabs(grabs)
                quad = rr.make_quad(out_dir)
                files = sorted(f for f in os.listdir(out_dir)
                               if f.endswith(".mp4"))
                prog[pkey] = dict(ok=bool(good), msg=msg, mode=mode,
                                  task=task, scenario=scen, quad=bool(quad),
                                  files=files, dir=os.path.relpath(out_dir, WS))
                save_progress(prog)          # AFTER EVERY CLIP
                log("      %s  %s  (%d files%s)"
                    % ("OK  " if good else "FAIL", msg, len(files),
                       ", quad" if quad else ""))
                done += bool(good)
                failed += (not good)
    finally:
        for pat, p in started.items():
            try:
                procscan.kill_all(pat)
            except Exception:                                 # noqa: BLE001
                pass
        graph.close()

    log("\n" + "=" * 74)
    log("%d recorded, %d failed, %d skipped   -> %s"
        % (done, failed, skipped, PROGRESS))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
