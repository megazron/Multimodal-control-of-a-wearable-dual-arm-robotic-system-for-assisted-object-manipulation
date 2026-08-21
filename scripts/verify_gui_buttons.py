#!/usr/bin/env python3
"""PART 4 -- PRESS EVERY BUTTON IN THE GUI AND REQUIRE EVIDENCE IT DID SOMETHING.

THE FAILURE THIS EXISTS TO CATCH. Five experiment buttons in an earlier build
exited 2 the instant they were pressed while appearing to launch: the label
said one thing, the dispatcher accepted another, and the process died before
anything drew. From the operator's side a button that launches nothing and a
button that launches something invisible are the same button.

THREE LEVELS, because one check cannot cover all of them honestly:

  1. DISPATCH -- for every task button, actually RUN the dispatcher with
     `--dry-run` and require exit 0. This is the direct regression: the old
     failure was exit 2 from `run_experiment.sh`, and only really executing
     it can prove that is gone. Nothing moves; --dry-run is the runners' own
     flag.

  2. CLICK -- construct the real GUI offscreen and press EVERY button through
     Qt, with launching intercepted. Each press must produce an observable
     outcome: a launch record, or a REFUSAL NAMING A REASON. Silence is a
     failure, and so is a traceback. Launching is intercepted rather than
     performed because pressing "Sim teleop only" for real would start a
     stack, and pressing it twice would start the second stack this project
     spent a day paying for.

  3. CONTROLS -- press the control buttons for real against whatever ROS graph
     is present. With no stack up, each must fail LOUDLY and by name
     ("no set_parameters service -- value UNCHANGED"), never silently. A
     control that quietly does nothing is worse than one that errors, because
     the operator believes the limit was applied.

WHY LEVEL 2 IS NOT ENOUGH ON ITS OWN, stated so nobody removes level 1: a
dry-launch proves the click path works and proves nothing about whether the
command would survive contact with the dispatcher. That is precisely the gap
the original bug lived in.

    python3 scripts/verify_gui_buttons.py
"""
import os
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))
sys.path.insert(0, os.path.join(WS, "scripts"))

RESULTS = []


def check(level, name, ok, detail):
    RESULTS.append((level, name, bool(ok), detail))
    print("  [%s] %-40s %s   %s"
          % (level, name[:40], "PASS" if ok else "FAIL", detail))


# ---------------------------------------------------------------- level 1
def level1_dispatch():
    print("\n-- LEVEL 1: run the dispatcher for real, --dry-run, require exit 0")
    from srl_teleop import gui_launch_specs as gls
    accepted = gls.dispatcher_tasks()
    if not accepted:
        check("1", "dispatcher task list parsed", False,
              "parsed ZERO tasks -- refusing to report a pass on an empty set")
        return
    check("1", "dispatcher task list parsed", True,
          "%d tasks: %s" % (len(accepted), " ".join(sorted(accepted))))

    # DISPATCH THE ARGV THE BUTTON ACTUALLY SENDS, not a shortened one.
    #
    # This used to run `run_experiment.sh <task> --dry-run` with NO --mode,
    # which is a command line no button in the GUI produces. It failed on m1,
    # correctly and unhelpfully: T1 runs under 06 only, the dispatcher said so
    # in a sentence, and the audit filed it as a broken button. The button was
    # never broken; the check was testing an argv that does not exist.
    #
    # So: one dispatch per ENABLED task and demo button, using its own argv.
    # A disabled button is not dispatched, because a disabled button cannot be
    # pressed -- but it is counted, so "0 enabled buttons" cannot pass.
    specs = [x for x in gls.all_specs()
             if x.group in ("task", "demo") and x.enabled]
    disabled = [x for x in gls.all_specs()
                if x.group in ("task", "demo") and not x.enabled]
    check("1", "enabled task/demo buttons to dispatch", bool(specs),
          "%d enabled, %d disabled with a stated reason" % (len(specs),
                                                            len(disabled)))
    for sp in specs:
        argv = list(sp.argv) + ["--dry-run"]
        try:
            p = subprocess.run(argv, capture_output=True, text=True,
                               timeout=120)
        except subprocess.TimeoutExpired:
            check("1", "dispatch %s" % sp.key, False, "timed out after 120 s")
            continue
        tail = (p.stderr or p.stdout or "").strip().splitlines()
        tail = tail[-1][:70] if tail else ""
        check("1", "dispatch %s" % sp.key, p.returncode == 0,
              "exit %d  %s" % (p.returncode, tail))

    # AND THE OTHER HALF: a DISABLED button must be disabled for a reason that
    # is TRUE. A button greyed out with a false explanation is as bad as a
    # button that exits 2, and harder to notice -- it simply looks unavailable
    # for ever. So every mode-locked button is dispatched too, and is required
    # to be REFUSED.
    locked = [x for x in disabled if "runs under" in (x.disabled_reason or "")]
    for sp in locked:
        argv = list(sp.argv) + ["--dry-run"]
        try:
            p = subprocess.run(argv, capture_output=True, text=True,
                               timeout=120)
        except subprocess.TimeoutExpired:
            check("1", "refusal is real: %s" % sp.key, False, "timed out")
            continue
        out = (p.stderr or "") + (p.stdout or "")
        check("1", "refusal is real: %s" % sp.key,
              p.returncode != 0 and "REFUSING" in out,
              "exit %d  %s" % (p.returncode,
                               out.strip().splitlines()[0][:60] if out.strip()
                               else ""))


# ---------------------------------------------------------------- level 2
def level2_click():
    print("\n-- LEVEL 2: construct the GUI offscreen and press every button")
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    import rclpy
    import threading
    import srl_gui

    # INTERCEPT THE LAUNCH. The click path runs in full -- preflight, refusal,
    # logging -- and only the final Popen is replaced. Pressing these for real
    # would start stacks, and the second-stack refusal that would then fire is
    # correct behaviour that would mask everything after it.
    launched = []
    real_launch = srl_gui.Gui.on_launch

    def fake_launch(self, spec):
        fails = self._preflight(spec)
        if fails:
            self.bus.note("REFUSED %s: %s" % (spec.label, "; ".join(fails)),
                          bad=True)
            launched.append((spec.key, "refused", "; ".join(fails)))
            return
        launched.append((spec.key, "would launch", " ".join(spec.argv)))
        self.bus.note("would launch %s" % spec.label)
    srl_gui.Gui.on_launch = fake_launch

    # AND `_run_raw`, WHICH WAS NOT INTERCEPTED AND SHOULD ALWAYS HAVE BEEN.
    #
    # `on_launch` covers the manifest Specs. `_run_raw` is the OTHER launch
    # path -- the ported console buttons and the real-arm panel -- and it went
    # straight to Popen during an audit. Measured 2026-08-21: pressing
    # `1. START REAL ARMS` ran scripts/start_real.sh for real, whose first act
    # is a teardown, and it CLOSED BOTH LIVE KORTEX SESSIONS mid-session. The
    # left arm then sagged under gravity.
    #
    # An audit that can move a robot is not an audit. Same principle as
    # above: the whole click path runs, only the final Popen is replaced.
    raw_launched = []

    def fake_run_raw(self, label, argv):
        raw_launched.append((label, " ".join(argv)))
        self.bus.note("would run %s" % label)
    srl_gui.Gui._run_raw = fake_run_raw

    # AND THE PROMPT PANEL'S OWN LAUNCHES, for the same reason and no other.
    # LOOK moves the arm to the observe pose and VOICE opens a listener; both
    # are real actions with real side effects, and pressing them here would
    # move a robot in the middle of a button audit. The click path -- the
    # validation, the refusal, the state label, the log line -- runs in full.
    spawned = []

    def fake_start(self, argv, done):
        spawned.append(list(argv))
        self._inst_say("would run: %s" % " ".join(argv[-4:]))
        done(0)

    def fake_spawn(self, argv):
        spawned.append(list(argv))
        self._inst_say("would run: %s" % " ".join(argv[:4]))
        return None
    srl_gui.Gui._inst_start = fake_start
    srl_gui.Gui._inst_spawn = fake_spawn

    # THE CONSENT GATE IS MODAL, AND IT IS SUPPOSED TO BE.
    #
    # START SESSION walks `session.CONSENT_STEPS` with a modal QMessageBox per
    # step and refuses to begin unless every one is confirmed. With nobody to
    # click it, that dialog blocks for ever -- which is why this level never
    # finished: measured 2026-08-18, the audit sat on button 05 of 69 until it
    # was killed, and every button after it went unpressed while the run
    # reported nothing at all.
    #
    # THE ANSWER IS `No`, NOT `Yes`, and that is the point rather than a
    # convenience. Answering Yes would have an audit consenting on a
    # participant's behalf, which is the one thing this dialog exists to
    # prevent. Answering No exercises the DECLINE path -- the session must not
    # start, and the refusal must be logged -- which is the branch worth
    # checking automatically anyway.
    from PyQt5.QtWidgets import QMessageBox as _QMB
    _QMB.question = staticmethod(lambda *a, **k: _QMB.No)
    _QMB.information = staticmethod(lambda *a, **k: _QMB.Ok)

    # THE CONNECTION PANEL'S REPAIRS, INTERCEPTED FOR THE SAME REASON AS THE
    # LAUNCHES AND NO OTHER. Three of them are real repairs to the machine
    # this audit is running on: restarting the discovery helper takes several
    # seconds and would be reported here as a frozen window, and restarting
    # the master arm reader would leave a node running after the audit
    # finished. The click path -- the diagnosis, the wrapper, the log line,
    # the re-check -- runs in full; only the last step is replaced.
    #
    # NOT INTERCEPTED, deliberately: clearing the leftover memory blocks.
    # That button only exists when there ARE leftovers and NOTHING is
    # running, which is precisely the state in which clearing them is the
    # correct action, and the repair refuses on its own if a stack appears.
    repaired = []

    def _stub(name, msg):
        def go():
            repaired.append(name)
            return True, msg
        return go
    srl_gui.rad.fix_reset_daemon = _stub(
        "reset_daemon", "would restart the discovery helper")
    srl_gui.rad.fix_kill_second_stack = _stub(
        "kill_second_stack", "would stop the duplicate")
    srl_gui.rad.fix_kill_stray_rsp = _stub(
        "kill_stray_rsp", "would stop the leftover")
    # AND THE ONE BUTTON. Pressing START VR TELEOP for real inside a button
    # audit would bring up a whole stack -- and pressing it twice would be the
    # second stack this project spent a day paying for. The click path runs in
    # full; only the worker's step execution is replaced.
    def _fake_vr(self):
        self.log("VR bring-up: would start (intercepted by the audit)")
        self.vr_head.setText("INTERCEPTED")
    srl_gui.Gui.on_vr_start = _fake_vr

    srl_gui.Gui._fix_teensy_repoint = lambda self: (
        repaired.append("teensy_repoint"),
        (True, "would restart the master arm reader"))[1]

    from PyQt5.QtWidgets import QApplication, QPushButton, QCheckBox, QSlider

    rclpy.init()
    bus = srl_gui.Bus()
    threading.Thread(target=lambda: rclpy.spin(bus), daemon=True).start()
    app = QApplication([sys.argv[0]])

    class Args:
        no_rviz, single_rviz, self_test_exit = True, False, False
    g = srl_gui.Gui(bus, Args())
    g.show()
    app.processEvents()

    # THE CONNECTION PANEL'S ROWS ARE BUILT FROM A DIAGNOSIS, so they do not
    # exist until one has been made -- and the first one is scheduled 1.2 s
    # after the window opens, deliberately, because the checks shell out and a
    # window that does not appear for eight seconds reads as a crash.
    #
    # Collecting the buttons before that ran would have audited a panel with
    # no rows in it and reported a pass. This is the same defect as the modal
    # dialog that hid five buttons for months: the audit walking a window that
    # is not finished.
    g.on_doctor_check()
    checks = g.doctor_wait(45)
    check("2", "connection panel rendered its checks", len(checks) >= 7,
          "%d row(s): %s" % (len(checks), ", ".join(c.key for c in checks)))
    _states = {c.state for c in checks}
    check("2", "connection panel never green on unknown",
          not (srl_gui.rad.verdict(checks)[0] == srl_gui.rad.OK
               and srl_gui.rad.UNKNOWN in _states),
          "verdict %s over states %s" % (srl_gui.rad.verdict(checks)[0],
                                         "/".join(sorted(_states))))

    buttons = g.findChildren(QPushButton)
    # A BUTTON THAT DOES NOT RETURN IS A FAILURE, reported as one rather than
    # hanging the audit. Anything slower than this is a modal dialog or a
    # blocking service call, and either way the operator's window is frozen --
    # which for the panel carrying the e-stop is the worst outcome this GUI
    # has.
    SLOW_S = 5.0
    check("2", "GUI constructed and has buttons", len(buttons) >= 15,
          "%d QPushButton" % len(buttons))

    n_err = 0
    for b in buttons:
        label = b.text()
        if not b.isEnabled():
            # A DISABLED BUTTON MUST CARRY ITS REASON. That is the whole
            # difference between "this cannot run" and "this did nothing".
            ok = bool(b.toolTip().strip())
            check("2", "disabled: %s" % label, ok,
                  (b.toolTip()[:60] + "...") if ok else "NO REASON GIVEN")
            continue
        before = len(bus.log)
        _t0 = time.time()
        try:
            b.click()
            app.processEvents()
            time.sleep(0.12)
            for _ in range(6):
                app.processEvents()
                time.sleep(0.05)
        except Exception as e:                                # noqa: BLE001
            n_err += 1
            check("2", "click: %s" % label, False, "RAISED %r" % (e,))
            continue
        _dt = time.time() - _t0
        if _dt > SLOW_S:
            check("2", "click returns: %s" % label, False,
                  "took %.1f s -- the window was frozen for that long" % _dt)
        after = len(bus.log)
        # EVERY PRESS MUST LEAVE A TRACE. Silence is indistinguishable from a
        # disconnected signal, which is the bug class this file exists for.
        check("2", "click: %s" % label, after > before,
              "%d log entr%s" % (after - before,
                                 "y" if after - before == 1 else "ies"))

    # Sliders and the checkbox are controls too, and they were never clicked
    # by anything before.
    for s in g.findChildren(QSlider):
        before = len(bus.log)
        s.setValue(max(s.minimum(), s.value() - 10))
        s.sliderReleased.emit()
        app.processEvents()
        time.sleep(0.2)
        for _ in range(4):
            app.processEvents()
            time.sleep(0.05)
        check("2", "slider %d..%d" % (s.minimum(), s.maximum()),
              len(bus.log) > before, "%d entries" % (len(bus.log) - before))
    for c in g.findChildren(QCheckBox):
        before = len(bus.log)
        c.setChecked(not c.isChecked())
        app.processEvents()
        time.sleep(0.2)
        for _ in range(4):
            app.processEvents()
            time.sleep(0.05)
        check("2", "checkbox %s" % c.text(), len(bus.log) > before,
              "%d entries" % (len(bus.log) - before))

    # DROP-DOWNS ARE CONTROLS TOO, and none of them was ever exercised. The
    # ACTUAL panel's view selector is one: it changes what the operator is
    # looking at, and a selector wired to nothing looks exactly like a view
    # that has only one angle.
    from PyQt5.QtWidgets import QComboBox
    for cb in g.findChildren(QComboBox):
        if cb.count() < 2:
            continue
        before = len(bus.log)
        cb.setCurrentIndex((cb.currentIndex() + 1) % cb.count())
        app.processEvents()
        time.sleep(0.2)
        for _ in range(4):
            app.processEvents()
            time.sleep(0.05)
        check("2", "drop-down -> %s" % cb.currentText()[:28],
              len(bus.log) > before, "%d entries" % (len(bus.log) - before))

    # ------------------------------------------------------------ level 2b
    # EVERY REPAIR BUTTON, INCLUDING THE ONES THE MACHINE IS TOO HEALTHY TO
    # SHOW.
    #
    # The connection panel draws a row's fix button only when that row is
    # BAD, so a walk of the live window presses whichever repairs the box
    # happens to need that day -- one of eight, on a healthy machine. Six
    # buttons would then go unpressed for ever while the audit reported that
    # every button had been pressed, which is the "a check that cannot fail"
    # shape.
    #
    # So each fault is forced into the panel through the SAME render path the
    # live diagnosis uses, and the resulting button is pressed for real. The
    # three repairs that would disturb this machine are already stubbed above;
    # the rest run, and the modal ones are answered No, which exercises the
    # decline branch.
    print("\n-- LEVEL 2b: force each connection fault and press its repair")
    rad = srl_gui.rad

    class _Broken(rad.Probe):
        """A world with exactly one thing wrong in it."""

        def __init__(self, **kw):
            self.kw = kw

        def my_env(self):
            return self.kw.get("env", dict(rad.EXPECTED))

        def stack_pids(self):
            return self.kw.get("stack", [])

        def live_pids(self):
            return self.kw.get("live", [])

        def env_of(self, pid):
            return self.kw.get("envs", {}).get(pid)

        def shm_segments(self):
            return self.kw.get("shm", [])

        def daemon_nodes(self, timeout_s=8):
            return self.kw.get("daemon", (["/x"], False))

        def find(self, rx):
            for k, v in self.kw.get("procs", {}).items():
                if k in rx:
                    return v
            return []

        def orphan_nodes(self):
            return self.kw.get("orphans", [])

        def tty_candidates(self):
            return self.kw.get("ttys", ["/dev/ttyACM0"])

        def master_port_param(self):
            return self.kw.get("port", "")

        def joint_states(self):
            return None, self.kw.get("real")

        def home_radians(self, arm):
            return [0.0] * 7

        def kortex_procs(self):
            return []

        def kortex_log_tail(self):
            return self.kw.get("ktail", "kortex session closed cleanly")

    WORLDS = [
        ("discovery", dict(env=dict(rad.EXPECTED,
                                    FASTDDS_BUILTIN_TRANSPORTS=""))),
        ("shm", dict(shm=["/dev/shm/fastrtps_synthetic"], live=[])),
        ("daemon", dict(daemon=([], True))),
        ("second_stack", dict(procs={"master_pose_node": [
            (1, "a/lib/srl_teleop/master_pose_node"),
            (2, "b/lib/srl_teleop/master_pose_node")]})),
        ("orphans", dict(orphans=[(1, "/opt/ros/jazzy/lib/rviz2/rviz2")])),
        ("teensy-absent", dict(ttys=[])),
        ("teensy-moved", dict(ttys=["/dev/ttyACM1"], port="/dev/ttyACM0")),
        ("home", dict(real={"left_joint_%d" % (i + 1): (1.94 if i == 6 else 0.0)
                            for i in range(7)})),
        ("kortex", dict(ktail="the process died\n")),
    ]
    pressed_fixes = []
    # HOLD THE LIVE RE-CHECK OFF. Otherwise the 30 s timer and the 600 ms
    # post-repair re-check land in the middle of this loop and replace the
    # injected rows with the real machine's, destroying the button about to
    # be pressed. It showed up as "missing: daemon, home" -- two repairs
    # reported unpressed that had never been offered.
    g.doctor_frozen = True
    g.doc_timer.stop()
    for name, kw in WORLDS:
        checks = rad.run_all(_Broken(**kw), g._doctor_fixes())
        bad = [c for c in checks if c.state == rad.BAD and c.has_fix]
        g._doctor_render(checks)
        app.processEvents()
        if not bad:
            check("2b", "fault appears: %s" % name, False,
                  "no BAD row with a repair -- the fault was not reproduced")
            continue
        ok_all, detail = True, []
        for c in bad:
            row = g.doc_rows.get(c.key)
            if row is None:
                ok_all = False
                detail.append("%s has no row" % c.key)
                continue
            btn = row[3]
            if not btn.isVisible() and btn.text() != c.fix_label:
                ok_all = False
                detail.append("%s button not shown" % c.key)
                continue
            before = len(bus.log)
            t0 = time.time()
            try:
                btn.click()
                app.processEvents()
                time.sleep(0.15)
                for _ in range(6):
                    app.processEvents()
                    time.sleep(0.05)
            except Exception as e:                            # noqa: BLE001
                ok_all = False
                detail.append("%s RAISED %r" % (c.key, e))
                continue
            dt = time.time() - t0
            if dt > SLOW_S:
                ok_all = False
                detail.append("%s took %.1f s" % (c.key, dt))
            if len(bus.log) <= before:
                ok_all = False
                detail.append("%s pressed SILENTLY" % c.key)
            else:
                pressed_fixes.append(c.key)
                # [1] is the MESSAGE. [0] is the timestamp, and printing
                # that showed a clock beside every repair instead of what the
                # repair said it had done.
                detail.append("%s: %s" % (c.fix_label[:22],
                                          bus.log[-1][1][:34]))
        check("2b", "repair pressed: %s" % name, ok_all,
              "; ".join(detail)[:110])

    # AND EVERY ONE OF THEM, not just the ones this machine happened to need.
    want = {"discovery", "shm", "daemon", "second_stack", "orphans", "teensy",
            "home", "kortex"}
    check("2b", "every repair button pressed at least once",
          want <= set(pressed_fixes),
          "missing: %s" % (", ".join(sorted(want - set(pressed_fixes)))
                           or "none"))

    # A HEALTHY WORLD MUST SHOW NO REPAIR BUTTONS AT ALL. Without this the
    # test above passes on a panel that shows every button all the time.
    healthy = rad.run_all(_Broken(), g._doctor_fixes())
    g._doctor_render(healthy)
    app.processEvents()
    shown = [k for k, row in g.doc_rows.items() if row[3].isVisible()]
    check("2b", "no repair offered when nothing is wrong", not shown,
          "buttons shown: %s" % (", ".join(shown) or "none"))
    g.doctor_frozen = False

    # Every enabled launch spec must have been pressed.
    enabled = {s.key for s in g.specs if s.enabled}
    pressed = {k for k, _, _ in launched}
    check("2", "every enabled launch button pressed", enabled <= pressed,
          "%d/%d (%s)" % (len(pressed & enabled), len(enabled),
                          ",".join(sorted(enabled - pressed)) or "none missing"))

    # The self-test itself is the negative-control-of-negative-controls.
    check("2", "indicator self-test passes", g.on_self_test(),
          g.selftest_lbl.text()[:70])

    srl_gui.Gui.on_launch = real_launch
    g.close()
    app.processEvents()
    bus.destroy_node()
    rclpy.shutdown()
    return launched


# ---------------------------------------------------------------- level 3
def level3_controls_fail_loudly(launched):
    print("\n-- LEVEL 3: with no stack up, controls must fail BY NAME")
    refusals = [d for k, st, d in launched if st == "refused"]
    named = [d for d in refusals if d and len(d) > 10]
    check("3", "every refusal carries a reason", len(named) == len(refusals),
          "%d refusals, %d with a stated reason"
          % (len(refusals), len(named)))
    # A run with a stack up would legitimately have zero refusals, so this is
    # informational rather than a required count.
    print("     (%d launch specs were refused by preflight in this "
          "environment)" % len(refusals))


def main():
    print("=" * 76)
    print("GUI BUTTONS -- press every one, require evidence")
    print("=" * 76)
    level1_dispatch()
    launched = level2_click() or []
    level3_controls_fail_loudly(launched)

    print("\n" + "=" * 76)
    if not RESULTS:
        print("NO CHECKS RAN. That is a failure, not a pass.")
        return 2
    bad = [(lv, n) for lv, n, ok, _ in RESULTS if not ok]
    print("%d checks, %d PASS, %d FAIL"
          % (len(RESULTS), len(RESULTS) - len(bad), len(bad)))
    for lv, n in bad:
        print("  FAILED [%s] %s" % (lv, n))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
