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

#: Every process a button tried to start during an audit, refused and
#: recorded. Module level because the check that reports it runs in a
#: different function from the one that installs the block.
blocked_popen = []
import sys
import threading
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
    # MARK EVERYTHING THIS AUDIT DOES AS AUTOMATED. It presses the
    # "I am working alone" checkbox, and that writes to the observer-bypass
    # log -- the record of a human choosing to work without an observer.
    # Thirty-four indistinguishable entries appeared there on 2026-08-22.
    os.environ["SRL_AUDIT"] = "1"
    import rclpy
    import threading
    import srl_gui

    # INTERCEPT THE LAUNCH. The click path runs in full -- preflight, refusal,
    # logging -- and only the final Popen is replaced. Pressing these for real
    # would start stacks, and the second-stack refusal that would then fire is
    # correct behaviour that would mask everything after it.
    # ONLY THE SPAWN IS REPLACED, and that is a correction.
    #
    # This used to replace `on_launch` itself with a stub that reimplemented
    # the preflight and the refusal. Everything the real `on_launch` does
    # before the Popen was therefore invisible to this audit -- and stayed
    # invisible when it grew a step that rewrites the command line for the
    # session's settings. A stub that duplicates production logic drifts from
    # it silently. `on_launch` now ends in `_spawn`, this replaces `_spawn`
    # alone, and the whole click path runs for real.
    launched = []
    seen_argv = []          # every spec that reached the preflight, with argv
    real_pre = srl_gui.Gui._preflight
    real_spawn = srl_gui.Gui._spawn

    def watched_pre(self, spec):
        seen_argv.append((spec.key, list(spec.argv)))
        fails = real_pre(self, spec)
        if fails:
            launched.append((spec.key, "refused", "; ".join(fails)))
        return fails

    def fake_spawn(self, spec):
        launched.append((spec.key, "would launch", " ".join(spec.argv)))
        self.bus.note("would launch %s" % spec.label)
    srl_gui.Gui._preflight = watched_pre
    srl_gui.Gui._spawn = fake_spawn

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
    # AND EVERY OTHER Popen, BECAUSE FAKING TWO NAMED PATHS IS NOT ENOUGH.
    #
    # `_spawn` and `_run_raw` were the two launch paths when this was
    # written. They are no longer the only ones: the camera watchdog and the
    # camera doctor shell out directly, and on 2026-08-30 an audit run
    # therefore started REAL nodes on its own ROS_DOMAIN_ID -- including
    # `kortex_highlevel_bridge`, which OPENED A SESSION ON EACH REAL ARM.
    # The arm permits exactly one (HARD CONSTRAINT 2), so the operator's own
    # stack could not connect and the arms would not move, for the better
    # part of an hour, while every diagnosis pointed at the teleoperation
    # chain.
    #
    # Naming the paths to block is a list that goes stale the moment somebody
    # adds a third. Blocking the SYSCALL cannot: anything that tries to start
    # a process during an audit is recorded and refused, whoever calls it and
    # whenever it was added. The audit still exercises the whole click path;
    # only the fork is replaced.
    global blocked_popen
    blocked_popen = []
    _real_popen = subprocess.Popen

    class _RefusedPopen:
        """Stands in for a process that was never started."""

        def __init__(self, argv, *a, **k):
            blocked_popen.append(" ".join(map(str, argv))
                                 if isinstance(argv, (list, tuple))
                                 else str(argv))
            self.pid = -1
            self.args = argv

        def poll(self):
            return 0

        def wait(self, timeout=None):
            return 0

        def kill(self):
            return None

        def terminate(self):
            return None

        def communicate(self, *a, **k):
            return (b"", b"")

    subprocess.Popen = _RefusedPopen
    srl_gui.subprocess.Popen = _RefusedPopen

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

    # THE SCAN AND PICK BUTTONS DRIVE BOTH ARMS ACROSS A TABLE.
    #
    # Pressing them for real in the middle of an audit is exactly the sort of
    # thing this file exists to avoid, and it is the same treatment LOOK,
    # VOICE and the launchers already get: replace ONLY the Popen, and leave
    # the whole click path -- the missing-file refusal, the venv choice, the
    # note line, the log entry -- running for real.
    seq_spawned = []

    def fake_seq_spawn(self, argv):
        seq_spawned.append(list(argv))
        return None
    srl_gui.Gui._seq_spawn = fake_seq_spawn

    # THE ARM BUTTONS OPEN AND CLOSE KORTEX SESSIONS.
    #
    # CONNECT starts a real bridge; DISCONNECT signals one. Doing either for
    # real mid-audit would open a session on the physical arm, or SIGINT the
    # one an operator is using. Replace the spawn and the signal, and leave
    # the rest of the path -- the missing-script refusal, the "no bridge is
    # running" case, the note line -- running for real.
    arm_spawned, arm_signalled = [], []

    def fake_arm_spawn(self, argv):
        arm_spawned.append(list(argv))
        return None
    srl_gui.Gui._arm_spawn = fake_arm_spawn

    # No bridge exists during an audit, so DISCONNECT takes its "nothing was
    # running" branch -- which is the branch worth exercising anyway.
    srl_gui.Gui._bridge_pids = staticmethod(lambda arm: [])

    # THE POSE BUTTONS COMMAND JOINT TRAJECTORIES. Same reasoning: the values
    # and the refusal path are what matter, not whether an arm moves.
    posed = []

    def fake_send(self, arm, q, secs=5.0):
        # RETURNS THE PAIR THE REAL ONE RETURNS. It returned a bare True while
        # production returned (ok, topic); the caller then unpacked True and
        # the audit would have passed on a signature that no longer existed.
        posed.append((arm, [round(float(x), 4) for x in q], secs))
        return (True, "/real/%s_arm_controller/joint_trajectory" % arm)
    srl_gui.Bus.send_joint_pose = fake_send

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

    from PyQt5.QtCore import Qt
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
    # ---- NOTHING IN THE CONTROL COLUMN MAY BE CUT OFF.
    #
    # Horizontal scrolling is off in that column, so a page wider than its
    # viewport is not scrollable -- it is silently truncated, and what goes
    # missing is the right-hand end of every button label and every sentence.
    # It shipped that way: "E-STOP -- HALT BOTH ARMS" rendered as "HALT BOTH
    # ARM", and a `setMaximumWidth(392)` on the column meant no amount of
    # widening the splitter could ever help.
    #
    # This audit pressed all 203 buttons and passed throughout, because a
    # button whose label is cut is still a button that clicks. Checking the
    # PIXELS is the only way to see it, and this is the cheap version of
    # that: ask each scroll page whether its content fits.
    g._fit_left_column()
    app.processEvents()
    clipped = g.clipped_pages()
    check("2", "no control-column page is cut off", not clipped,
          "; ".join("%s: viewport %d px, content needs %d"
                    % (n, v, w) for n, v, w in clipped)
          or "all pages fit; column %d px" % g.split.sizes()[0])

    # ---- AND EVERY MODE HAS A WAY IN.
    #
    # The contract changed on 2026-08-27: the per-mode SIM/MOCK/REAL grid
    # was absorbed into the three-click panel. Every mode still has ONE
    # start button, and the sim view, the mock rehearsal and the real
    # cascade are shared steps that apply to whichever mode is live --
    # which is how the machine actually works (start_mode adopts what is
    # running). The audit therefore proves: five mode starts, the instruct
    # way in, and the three shared steps, all enabled.
    rows = getattr(g, "mode_btn", {})
    want = {"teleop", "vr", "shared", "shared_vr", "full",
            "__instruct__", "__sim_view__", "__real__", "__mock__"}
    check("2", "every mode has a start, plus sim view / mock / real",
          set(rows) >= want and all(rows[k].isEnabled() for k in want),
          "missing: %s" % ", ".join(sorted(want - set(rows)))
          if set(rows) < want else "%d buttons, all enabled" % len(rows))
    check("2", "full autonomy has a prompt reachable from RUN",
          "__instruct__" in rows and rows["__instruct__"].isEnabled()
          and getattr(g, "instruct_tab_index", None) is not None,
          "the Instruct panel is what drives mode 06 from a sentence")

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

    # ================================================================
    # EVERY PRESS LEFT A TRACE. THAT IS NOT THE SAME AS WORKING.
    # ================================================================
    #
    # The check above passes when a click adds a log line. It catches a
    # disconnected signal, which is what it was written for, and it passed
    # 203/203 while `E-STOP -- HALT BOTH ARMS` did nothing at all: it shelled
    # out to `ros2 service call /estop`, which on an absent service does not
    # exit but WAITS FOR EVER, so the 2.5 s return-code check saw None and
    # said nothing, and the panel printed "E-STOP SENT -- latched until
    # reset" in red. A log line appeared. The arms were never told.
    #
    # So the buttons whose effect is observable in this process get their
    # EFFECT checked, not their noise.
    print("\n-- LEVEL 2b: buttons with an observable effect must have it")

    from rclpy.node import Node as _Node
    from std_msgs.msg import Bool as _Bool
    # THE SPY NEEDS ITS OWN EXECUTOR.
    #
    # `rclpy.spin(node)` uses the GLOBAL default executor, and `bus` is
    # already being spun on it. A second `rclpy.spin` in another thread
    # raises "Executor is already spinning" inside that thread, where nothing
    # is watching -- so the subscription never fires and every check below
    # reads "received []".
    #
    # That happened, and it read exactly like the defect it was written to
    # catch: an e-stop that logs and does not publish. The publisher was
    # matched, the GUI logged "E-STOP published", and the message was
    # delivered to a callback nobody was pumping. docs/ENGINEERING_LOG.md's standing rule,
    # in the audit itself -- a surprising failure is evidence about the
    # INSTRUMENT until the instrument has been cleared.
    from rclpy.executors import SingleThreadedExecutor as _Exec
    spy = _Node("gui_audit_spy")
    seen = []
    spy.create_subscription(_Bool, "/estop", lambda m: seen.append(m.data), 10)
    _spy_exec = _Exec()
    _spy_exec.add_node(spy)
    threading.Thread(target=_spy_exec.spin, daemon=True).start()

    def _settle(n=30):
        for _ in range(n):
            app.processEvents()
            time.sleep(0.05)

    # WAIT FOR THE MATCH, AND ASSERT IT. A brand-new subscriber has not
    # finished DDS discovery with an existing publisher, so clicking
    # immediately tests nothing and fails for a reason that has nothing to do
    # with the button. Worse, without this assertion the check could pass
    # trivially once the timing changed, having never observed anything.
    _estop_pub = bus._pubs.get(("/estop", "Bool"))
    _matched = False
    for _ in range(100):
        if _estop_pub is not None and _estop_pub.get_subscription_count() >= 1:
            _matched = True
            break
        _settle(2)
    check("2b", "the audit's own /estop subscriber matched the GUI", _matched,
          "%d subscriber(s) seen by the GUI's publisher. Without a match the "
          "e-stop checks below would test nothing."
          % (_estop_pub.get_subscription_count() if _estop_pub else -1))

    by_text = {b.text().strip(): b for b in g.findChildren(QPushButton)
               if b.text().strip()}

    # 1 -- EVERY e-stop button must put True on /estop. Both of them: the
    #      bottom-bar one always did, the real-arm one never did.
    estops = [k for k in by_text if k.startswith("E-STOP")]
    check("2b", "there is more than one e-stop button", len(estops) >= 2,
          ", ".join(estops))
    for k in estops:
        seen.clear()
        by_text[k].click()
        _settle()
        check("2b", "e-stop reaches /estop: %s" % k, seen == [True],
              "received %r -- a stop button that logs and does not publish "
              "is the worst defect this window can have" % (seen,))

    # 2 -- NO BUTTON MAY SHELL OUT TO `ros2 service call`. It hangs for ever
    #      on an absent service, so the caller can never learn it failed.
    gui_src = open(os.path.join(WS, "scripts/srl_gui.py")).read()
    check("2b", "nothing shells out to `ros2 service call`",
          '"service", "call"' not in gui_src
          and "'service', 'call'" not in gui_src,
          "use bus.call_trigger, which checks service_is_ready first and "
          "reports the real result")

    # 3 -- WITH NO STACK UP, a service button must SAY the service is absent.
    #      This is the difference between "it did not work" and "it worked".
    # `2. HOME BOTH ARMS` was one of FOUR real-arm start buttons and is gone:
    # they were consolidated into `MOVE THE REAL ARMS` on 2026-08-29, because
    # an operator holding four buttons that each did part of the job and
    # refused for a different reason pressed all of them and the arm never
    # moved. Homing is now a stage inside that sequencer. What this check is
    # FOR -- a service button must say so when its service is absent -- is
    # unchanged, and the two remaining service buttons still carry it.
    for k in ("reset e-stop", "stop real arms"):
        b = by_text.get(k)
        if b is None:
            check("2b", "service button present: %s" % k, False, "missing")
            continue
        n0 = len(bus.log)
        b.click()
        _settle()
        said = [str(x) for x in list(bus.log)[n0:]]
        check("2b", "says the service is absent: %s" % k,
              any("not present" in x for x in said),
              "; ".join(x[:60] for x in said[-2:]) or "SAID NOTHING")

    # 4 -- FULL AUTONOMY must actually bring the prompt to the front.
    b = getattr(g, "mode_btn", {}).get("__instruct__")
    if b is None:
        check("2b", "the full-autonomy prompt has a button", False,
              "no mode_btn['__instruct__']")
    else:
        g.mid_tabs.setCurrentIndex(0)
        app.processEvents()
        b.click()
        _settle(6)
        check("2b", "FULL AUTONOMY switches to the Instruct panel",
              g.mid_tabs.currentIndex() == g.instruct_tab_index,
              "tab %d, Instruct is %d"
              % (g.mid_tabs.currentIndex(), g.instruct_tab_index))

    # 4b -- THE FULL-AUTONOMY PROMPT MUST GATE ON LOOK.
    #
    # Mode 06 picks its destination from the colour the CAMERA sees, so a
    # sentence typed before anything has looked has nothing to plan against.
    # The failure to avoid is a parse that succeeds on stale or absent
    # detections and then commands an arm. So: SEND must echo the sentence,
    # refuse by NAMING LOOK, and leave CONFIRM disabled -- for a sentence the
    # grammar understands and for one it does not.
    #
    # WHAT THIS CANNOT CHECK, and it is most of the interesting part: whether
    # the parse is RIGHT. That needs LOOK to succeed, which needs the
    # perception stack. `scripts/sweep_t1_instructions.py` scores 75
    # phrasings and is where that lives.
    if hasattr(g, "inst_edit"):
        inst_btns = {b.text().strip(): b
                     for b in g.mid_tabs.widget(g.instruct_tab_index)
                     .findChildren(QPushButton) if b.text().strip()}
        for sentence in ("put the red cube on the red pad",
                         "wibble the frobnicator"):
            g.inst_edit.setText(sentence)
            app.processEvents()
            n0 = len(bus.log)
            inst_btns["SEND"].click()
            _settle()
            said = " | ".join(str(x) for x in list(bus.log)[n0:])
            check("2b", "instruct echoes and gates on LOOK: %r"
                  % sentence[:22],
                  sentence in said and "LOOK" in said,
                  said[:100] or "SAID NOTHING")
        check("2b", "CONFIRM stays disabled until something is planned",
              not inst_btns["CONFIRM AND RUN"].isEnabled(),
              "a confirm that is live with no plan commands an arm from "
              "nothing")
    else:
        check("2b", "the instruct prompt exists", False,
              "no inst_edit -- mode 06 has no way in")

    # 4c -- THE EXPERIMENTS PANEL BUILDS A COMMAND THE DISPATCHER ACCEPTS.
    #
    # Defaults alone must produce a runnable trial: the panel's whole claim
    # is that you can press RUN TRIAL without filling anything in.
    if hasattr(g, "_exp_argv"):
        argv = g._exp_argv()
        check("2b", "experiments panel builds a complete command",
              argv[0] == "bash" and "--participant" in argv
              and "--session" in argv and "--trial-index" in argv,
              " ".join(argv[-8:]))
        # ...and the dispatcher must actually take it. --dry-run so nothing
        # moves; a panel that composes an argv nobody accepts is the "exits
        # 2 and looks like it launched" defect with extra fields.
        try:
            pr = subprocess.run(argv + ["--dry-run"], capture_output=True,
                                text=True, timeout=180)
            ok_run = pr.returncode == 0
            tail = ((pr.stdout or pr.stderr or "").strip().splitlines()
                    or [""])[-1][:70]
        except Exception as e:                            # noqa: BLE001
            ok_run, tail = False, repr(e)
        check("2b", "the dispatcher accepts the panel's own defaults",
              ok_run, tail)
        # the trial number must ADVANCE, or two runs share one identity
        before = g.exp_trial.value()
        g.exp_dry.setChecked(True)
        g.on_run_trial()
        _settle(10)
        check("2b", "RUN TRIAL advances the trial number",
              g.exp_trial.value() == before + 1,
              "%d -> %d" % (before, g.exp_trial.value()))
    else:
        check("2b", "the experiments panel exists", False, "no _exp_argv")

    # 4d -- THE 3-D VIEW IS THE DEFAULT AND CAN BE RESET.
    if hasattr(g, "arm_view"):
        import srl_arm_view as _av
        check("2b", "the ACTUAL panel offers a 3-D view",
              _av.VIEW_3D in [g.view_pick.itemText(i)
                              for i in range(g.view_pick.count())]
              and g.view_pick.currentText() == _av.VIEW_3D,
              "current: %s" % g.view_pick.currentText())
        g.arm_view.yaw, g.arm_view.pitch = 123.0, -45.0
        g.arm_view.reset_view()
        check("2b", "reset 3-D view returns the shipped angles",
              (g.arm_view.yaw, g.arm_view.pitch)
              == (_av.DEFAULT_YAW_DEG, _av.DEFAULT_PITCH_DEG),
              "yaw %.1f pitch %.1f" % (g.arm_view.yaw, g.arm_view.pitch))

    # 4e -- EVERY READOUT IS SELECTABLE, so it can be copied.
    from PyQt5.QtWidgets import QLabel as _QLabel
    labs = [x for x in g.findChildren(_QLabel) if x.pixmap() is None]
    unsel = [x for x in labs
             if not (x.textInteractionFlags() & Qt.TextSelectableByMouse)]
    check("2b", "every text readout can be selected and copied",
          not unsel, "%d labels, %d not selectable" % (len(labs), len(unsel)))

    # 4f -- THE INSTRUCTION BOX TAKES THE KEYBOARD.
    #
    # It is a QLineEdit that was never read-only and never disabled, and it
    # still could not be typed into: the embedded RViz is a separate X client
    # and once it holds the input focus every keystroke goes to it.
    if hasattr(g, "inst_edit"):
        g.mid_tabs.setCurrentIndex(0)
        app.processEvents()
        g.on_show_instruct()
        _settle(6)
        check("2b", "opening Instruct puts the keyboard in the box",
              app.focusWidget() is g.inst_edit,
              "focus is on %r" % type(app.focusWidget()).__name__)
        from PyQt5.QtTest import QTest
        g.inst_edit.clear()
        QTest.keyClicks(g.inst_edit, "pick up the blue one")
        _settle(4)
        check("2b", "the instruction box accepts typing",
              g.inst_edit.text() == "pick up the blue one",
              repr(g.inst_edit.text()))

    # 4g -- THE MOTION GENERATOR CHOOSER MUST REACH A LAUNCH.
    #
    # THE PRESS IS NOT THE POINT. A combo box that logs a new value and then
    # does not change the command line is the "feature present but does
    # nothing" row of docs/ENGINEERING_LOG.md's table, and it is invisible from a return
    # code -- the button still launches, the stack still comes up, and the
    # followers run the default. So the check is on the ARGV: select the
    # legacy generator and require that a stack-starting spec picks it up,
    # and that a spec whose launch file does not declare the argument does
    # NOT (`ros2 launch` fails outright on an argument it does not know, so
    # appending one there would kill the button rather than degrade it).
    if hasattr(g, "motion_gen"):
        from srl_teleop import gui_launch_specs as _gls
        keys = [g.motion_gen.itemData(i) for i in range(g.motion_gen.count())]
        check("2b", "the motion generator chooser offers all three",
              keys == ["ruckig", "clamp", "legacy"], "%s" % (keys,))
        note0 = g.motion_note.text()
        g.motion_gen.setCurrentIndex(keys.index("legacy"))
        _settle(4)
        check("2b", "choosing a non-default generator changes the warning",
              g.motion_note.text() != note0 and "51.3 mm"
              in g.motion_note.text(), g.motion_note.text()[:60])
        check("2b", "and it is logged as a change to the next run",
              any("motion generator for the NEXT launch" in txt
                  for _, txt, _ in g.bus.log),
              "%d event log lines searched" % len(g.bus.log))
        # DRIVE THE REAL CLICK PATH and read the argv the spawn WOULD have
        # been given -- `seen_argv` records every spec that reached the
        # preflight, which is after the rewrite. Reading the manifest entry
        # instead would test `with_argv`, not the button.
        want = "motion_generator:=legacy"
        base = len(seen_argv)
        g.on_launch(next(x for x in _gls.MODES if x.key == "sim"))
        g.on_launch(next(x for x in _gls.MODES if x.key == "vr"))
        got = dict(seen_argv[base:])
        check("2b", "the chosen generator reaches the launch command line",
              want in got.get("sim", []), "sim argv: %s" % (got.get("sim"),))
        check("2b", "and NOT a launch file that cannot take the argument",
              want not in got.get("vr", []), "vr argv: %s" % (got.get("vr"),))
        g.motion_gen.setCurrentIndex(keys.index("ruckig"))
        _settle(2)
        base = len(seen_argv)
        g.on_launch(next(x for x in _gls.MODES if x.key == "sim"))
        got = dict(seen_argv[base:])
        check("2b", "back on the default, nothing is appended",
              not any("motion_generator" in a for a in got.get("sim", [])),
              "%s" % (got.get("sim"),))

    # 5 -- HIDDEN BUTTONS ARE NOT OPERATOR-REACHABLE, and counting them as
    #      passes inflates the audit. Report the split so the number means
    #      something, and require every hidden one to be unlabelled -- a
    #      hidden button WITH a label is a control somebody meant to show.
    hid = [b for b in g.findChildren(QPushButton) if b.isHidden()]
    hid_lab = [b.text().strip() for b in hid if b.text().strip()]
    check("2b", "no LABELLED button is hidden from the operator",
          not hid_lab,
          "%d hidden, all unlabelled (per-row fix buttons)" % len(hid)
          if not hid_lab else "hidden but labelled: %s" % ", ".join(hid_lab))

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

        def stale_shm_segments(self):
            """THE ONE `check_shm` ACTUALLY BRANCHES ON.

            This override was missing, so the injected segment reached
            `shm_segments()` and then `check_shm` asked the REAL
            `stale_shm_segments()`, which globs the live /dev/shm and consults
            /proc. On a machine with nothing stale that returns [], the check
            took its "segments in use by what is running" branch and reported
            OK -- so the shm fault could never be reproduced and its repair
            button could never be pressed.

            It is the injection missing its consumer: the world said one
            thing and the code under test read another, which is why a fault
            injector needs its own control. `stale` is deliberately DERIVED
            from the injected `shm` here rather than being a second
            independent knob, so the two cannot drift apart again.
            """
            if "shm" not in self.kw:
                return []
            return list(self.kw["shm"])

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

    # ---------------------------------------------------- the task runner
    # The task/demo wall of one-button-per-(task x mode) is gone -- the
    # operator called 55 buttons in one panel what it was. The runner is
    # two selectors and RUN, so the audit drives it the way an operator
    # does: every pair the manifest offers is selected and RUN pressed.
    pairs = getattr(g, "_task_pairs", {})
    task_demo = {s.key for s in g.specs if s.group in ("task", "demo")}
    check("2", "every task/demo spec reachable from the selectors",
          {sp.key for sp in pairs.values()} == task_demo,
          "%d pairs vs %d specs" % (len(pairs), len(task_demo)))
    refused_greys = None
    for (base, short), sp in sorted(pairs.items()):
        bi = g.task_pick.findData(base)
        mi = g.mode_pick.findData(short)
        if bi < 0 or mi < 0:
            check("2", "selector offers %s / %s" % (base, short), False,
                  "missing from a combo")
            continue
        g.task_pick.setCurrentIndex(bi)
        g.mode_pick.setCurrentIndex(mi)
        app.processEvents()
        if sp.enabled:
            g.task_run_btn.click()
            _settle(2)
        elif refused_greys is None:
            # A pair the dispatcher refuses must grey RUN with the reason
            # ON it before any press -- a RUN that exits 2 looks exactly
            # like one that launched something invisible.
            refused_greys = (not g.task_run_btn.isEnabled()
                            and bool(g.task_run_note.text().strip()))
    if refused_greys is not None:
        check("2", "a refused pair greys RUN and says why", refused_greys,
              g.task_run_note.text()[:60])

    # Every enabled launch spec must have been LAUNCHED -- mode and diag
    # specs from their buttons, task and demo specs through the runner.
    enabled = {s.key for s in g.specs if s.enabled}
    pressed = {k for k, _, _ in launched}
    check("2", "every enabled launch spec launched", enabled <= pressed,
          "%d/%d (%s)" % (len(pressed & enabled), len(enabled),
                          ",".join(sorted(enabled - pressed)) or "none missing"))

    # The self-test itself is the negative-control-of-negative-controls.
    check("2", "indicator self-test passes", g.on_self_test(),
          g.selftest_lbl.text()[:70])

    # ================================================================
    # LEVEL 2c: PRESSING THINGS MUST NOT BREAK THE WINDOW
    # ================================================================
    #
    # Every button has now been pressed, most of them in an order no operator
    # would use. The window has to still BE a working window: the layout not
    # cut, the tabs still switchable, the e-stop still connected, the
    # instruction box still typable.
    #
    # A control that works on a fresh window and stops working after somebody
    # explored the panel is a glitch, and a press-everything audit is exactly
    # the thing positioned to catch it -- which this one never did, because
    # it checked each press in isolation and never asked afterwards.
    print("\n-- LEVEL 2c: after pressing everything, is the window still sane")

    g._fit_left_column()
    _settle(6)
    check("2c", "layout still fits after every button was pressed",
          not g.clipped_pages(),
          "; ".join("%s: %d/%d" % (n, v, w) for n, v, w in g.clipped_pages())
          or "column %d px" % g.split.sizes()[0])

    n_tabs_ok = True
    for i in range(g.mid_tabs.count()):
        g.mid_tabs.setCurrentIndex(i)
        app.processEvents()
        n_tabs_ok &= (g.mid_tabs.currentIndex() == i)
    check("2c", "every middle tab still switches", n_tabs_ok,
          "%d tabs" % g.mid_tabs.count())
    for i in range(g.act_tabs.count()):
        g.act_tabs.setCurrentIndex(i)
        app.processEvents()
    check("2c", "every activity tab still switches",
          g.act_tabs.currentIndex() == g.act_tabs.count() - 1,
          "%d tabs" % g.act_tabs.count())

    seen.clear()
    by_text["E-STOP"].click()
    _settle()
    check("2c", "the e-stop still reaches /estop after everything else",
          seen == [True], "received %r" % (seen,))

    if hasattr(g, "inst_edit"):
        g.on_show_instruct()
        _settle(6)
        from PyQt5.QtTest import QTest as _QT
        g.inst_edit.clear()
        _QT.keyClicks(g.inst_edit, "still typable")
        _settle(4)
        check("2c", "the instruction box still takes typing",
              g.inst_edit.text() == "still typable", repr(g.inst_edit.text()))

    check("2c", "no button raised during the sweep", n_err == 0,
          "%d raised" % n_err)

    srl_gui.Gui._preflight = real_pre
    srl_gui.Gui._spawn = real_spawn
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
    if blocked_popen:
        uniq = sorted(set(blocked_popen))
        check("2", "no button started a real process during the audit",
              True,
              "%d refused: %s" % (len(uniq), "; ".join(u[:60] for u in uniq[:3])))
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
