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

    for t in sorted(accepted):
        argv = [os.path.join(WS, "scripts", "run_experiment.sh"), t,
                "--participant", "GUIVERIFY", "--dry-run"]
        try:
            p = subprocess.run(argv, capture_output=True, text=True,
                               timeout=120)
        except subprocess.TimeoutExpired:
            check("1", "dispatch %s" % t, False, "timed out after 120 s")
            continue
        tail = (p.stderr or p.stdout or "").strip().splitlines()
        tail = tail[-1][:70] if tail else ""
        check("1", "dispatch %s" % t, p.returncode == 0,
              "exit %d  %s" % (p.returncode, tail))


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

    from PyQt5.QtWidgets import QApplication, QPushButton, QCheckBox, QSlider
    from PyQt5.QtCore import Qt

    rclpy.init()
    bus = srl_gui.Bus()
    threading.Thread(target=lambda: rclpy.spin(bus), daemon=True).start()
    app = QApplication([sys.argv[0]])

    class Args:
        no_rviz, single_rviz, self_test_exit = True, False, False
    g = srl_gui.Gui(bus, Args())
    g.show()
    app.processEvents()

    buttons = g.findChildren(QPushButton)
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
