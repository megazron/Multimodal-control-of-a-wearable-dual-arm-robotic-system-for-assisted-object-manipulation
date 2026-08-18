#!/usr/bin/env python3
"""THE GUI'S PROMPT BOX: DRIVE IT, AND REQUIRE THAT IT DRIVES THE ARM.

    python3 scripts/verify_prompt_panel.py

THE FAILURE THIS EXISTS TO CATCH, in the brief's own words: "a prompt box that
looks right and sends nothing". A panel can display a parse, a detection table
and a confirm button, and still be a picture -- the only thing that makes it
real is that the argv it would hand to the runner is the argv a recorded run
uses, carrying the operator's own sentence.

So this constructs the REAL GUI offscreen, types into the REAL box, presses the
REAL buttons, and intercepts exactly one thing: the `Popen`. Everything before
it -- the grounding, the question, the answer, the announced intention, the
confirm gate, the state labels -- runs for real, because that is what is being
checked.

WHAT IS CHECKED, in the order an operator meets it:

  1. with NO detections, SEND refuses and says why. It must not plan from the
     task file, which is the fallback TASK_SPEC P-4 forbids;
  2. after a look, the panel shows every cube, its CLASSIFIED colour and the
     classifier's own margin;
  3. a plain instruction plans, announces what it will do, and arms CONFIRM;
  4. CONFIRM hands the runner an argv containing this task, mode 06, the
     detections file and the OPERATOR'S OWN SENTENCE. This is the check the
     panel exists to pass;
  5. an ambiguous instruction ASKS and does NOT arm CONFIRM -- the arm cannot
     be started on a guess;
  6. the answer typed into the SAME box resolves it, and two different answers
     give two different plans, so the dialogue is real and not decorative;
  7. an unrepresentable instruction is REFUSED in plain words;
  8. a progress line from the runner reaches the state label, so "which cube
     it is on" is live rather than a post-mortem.

THE DETECTIONS ARE A FIXTURE, and that is deliberate: this measures the PANEL,
not the camera. `scripts/verify_look_then_grasp.py` and
`scripts/verify_vision_drives_grasp.py` measure the camera, against a rendered
scene, and neither can tell you whether a button is wired up.
"""
import json
import os
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
for p in (os.path.join(ROOT, "src/srl_experiments/experiments/abc"),
          os.path.join(ROOT, "src/srl_autonomy"),
          os.path.join(ROOT, "config")):
    sys.path.insert(0, p)

FAILED = []


def check(name, ok, detail=""):
    print("   %-58s %s   %s" % (name, "ok" if ok else "FAILED", detail))
    if not ok:
        FAILED.append(name)
    return ok


def main():
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    import threading

    import rclpy
    import srl_gui
    from PyQt5.QtWidgets import QApplication

    import t1_task as T1M

    # ---- THE ONE THING THAT IS FAKED -----------------------------------
    started = []

    def fake_start(self, argv, done):
        started.append(list(argv))
        done(0)

    def fake_spawn(self, argv):
        started.append(list(argv))
        return None
    srl_gui.Gui._inst_start = fake_start
    srl_gui.Gui._inst_spawn = fake_spawn

    rclpy.init()
    bus = srl_gui.Bus()
    threading.Thread(target=lambda: rclpy.spin(bus), daemon=True).start()
    app = QApplication([sys.argv[0]])

    class Args:
        no_rviz, single_rviz, self_test_exit, dual_rviz = True, False, False, False
        tab = "instruct"
    g = srl_gui.Gui(bus, Args())
    g.show()
    app.processEvents()

    tmp = tempfile.mkdtemp(prefix="promptpanel_")
    det = os.path.join(tmp, "detections.json")
    g.DETECTIONS = det
    srl_gui.Gui.DETECTIONS = det

    def send(text):
        g.inst_edit.setText(text)
        g.on_instruct_send()
        for _ in range(4):
            app.processEvents()
            time.sleep(0.02)

    print("\n-- 1. NO DETECTIONS: it must refuse, not fall back")
    send("put the blue ones on the blue pad")
    check("refuses with no detections",
          "NO DETECTIONS" in g.inst_state.text(), g.inst_state.text())
    check("CONFIRM stays disabled", not g.inst_go.isEnabled())

    # ---- a look, as the staged detector would have written it -----------
    layout = [(cx, cy, T1M.T1_PAIR[i])
              for i, (cx, cy) in enumerate(T1M.T1_CUBES)]
    colours = T1M.PLANE_COLOURS
    rec = dict(
        arm=T1M.LOOK_ARM, task="t1", seed=0,
        cubes=[[c[0], c[1], c[2]] for c in layout],
        timing=dict(seen=[dict(colour=colours[c[2]], x=c[0], y=c[1],
                               z=T1M.T1_Z, blob_px=41.0, expected_px=39.5,
                               depth_m=0.72, hsv_median=[110, 200, 180],
                               hsv_margin_min=[21, 29, 25],
                               confidence_counts=21) for c in layout]),
        layout=dict(T1_CUBES=[[c[0], c[1]] for c in layout],
                    T1_PLANES=[list(p) for p in T1M.T1_PLANES],
                    T1_Z=T1M.T1_Z))
    json.dump(rec, open(det, "w"))

    print("\n-- 2. WHAT THE CAMERA SAW, shown per cube")
    g._inst_load_detections()
    app.processEvents()
    txt = g.inst_seen_lbl.text()
    check("one line per cube", txt.count("cube ") == len(layout), txt[:60])
    check("each line names the CLASSIFIED colour",
          all(c in txt for c in colours), txt[:60])
    check("each line carries a confidence",
          txt.count("confidence") == len(layout))

    print("\n-- 3. A PLAIN INSTRUCTION PLANS AND ANNOUNCES")
    send("put the blue ones on the blue pad")
    check("state is AWAITING CONFIRMATION",
          g.inst_state.text() == "AWAITING CONFIRMATION", g.inst_state.text())
    check("CONFIRM is armed", g.inst_go.isEnabled())
    check("it announced what it will do",
          "blue pad" in g.inst_intent.text(), g.inst_intent.text()[:60])
    check("the parse is shown before anything moves",
          g.inst_parse["verb"].text() == "put_on"
          and "blue" in g.inst_parse["destination"].text(),
          "%s / %s" % (g.inst_parse["verb"].text(),
                       g.inst_parse["destination"].text()))

    print("\n-- 4. CONFIRM HANDS THE RUNNER THE OPERATOR'S OWN SENTENCE")
    started.clear()
    g.on_instruct_run()
    for _ in range(4):
        app.processEvents()
        time.sleep(0.02)
    check("it started something", len(started) == 1,
          "%d" % len(started))
    argv = started[0] if started else []
    joined = " ".join(argv)
    check("it is run_experiment.sh", argv and argv[0].endswith(
        "run_experiment.sh"), argv[0] if argv else "-")
    check("with this task", "m1" in argv, joined[:70])
    check("under 06_full_autonomy", "06_full_autonomy" in argv)
    check("with the DETECTIONS it was planned from", det in argv)
    check("carrying the typed sentence verbatim",
          "put the blue ones on the blue pad" in argv, joined[-70:])

    print("\n-- 5. AN AMBIGUOUS INSTRUCTION ASKS AND ARMS NOTHING")
    started.clear()
    send("pick up the blue cube")
    check("state is ASKING", g.inst_state.text() == "ASKING",
          g.inst_state.text())
    check("CONFIRM is NOT armed", not g.inst_go.isEnabled())
    check("nothing was started", not started)
    q = g.inst_log.toPlainText().strip().splitlines()[-1]
    check("the question is in plain words",
          "which one" in q.lower() or "?" in q, q[-60:])

    print("\n-- 6. THE ANSWER GOES IN THE SAME BOX, AND IT DECIDES")
    send("the leftmost")
    check("it planned on the answer",
          g.inst_state.text() == "AWAITING CONFIRMATION", g.inst_state.text())
    left_plan = [tuple(p) for p in g._inst_outcome.picks]
    send("pick up the blue cube")
    send("the rightmost")
    right_plan = [tuple(p) for p in g._inst_outcome.picks]
    check("two answers give two different plans", left_plan != right_plan,
          "%s vs %s" % (left_plan, right_plan))

    print("\n-- 7. AN UNREPRESENTABLE INSTRUCTION IS REFUSED, WITH A REASON")
    send("do not put the blue ones on the blue pad")
    check("state is REFUSED", g.inst_state.text() == "REFUSED",
          g.inst_state.text())
    check("CONFIRM is NOT armed", not g.inst_go.isEnabled())
    last = g.inst_log.toPlainText().strip().splitlines()[-1]
    check("it says why", "negat" in last.lower(), last[-60:])

    print("\n-- 8. LIVE STATE FROM THE RUNNER'S OWN PROGRESS LINES")
    g._inst_lines = ["[progress] left arm CLOSED on at (0.420, 0.450, 1.270)"]
    g._inst_done_cb = None
    g._inst_drain()
    app.processEvents()
    check("the state label follows the run",
          "CLOSED" in g.inst_state.text(), g.inst_state.text())

    print("\n-- 9. STAGE 2 IS SELECTABLE AND CARRIES ITS SEED")
    g.inst_task.setCurrentIndex(1)
    g.inst_seed.setText("3")
    check("task key follows the box", g._inst_task_key() == "m1s2")
    started.clear()
    g.on_instruct_look()
    for _ in range(4):
        app.processEvents()
        time.sleep(0.02)
    look = " ".join(started[0]) if started else ""
    check("the look is told which stage", "--task t1s2" in look, look[-60:])
    check("and which seed", "--seed 3" in look, look[-40:])

    shutil.rmtree(tmp, ignore_errors=True)
    print("\n" + "=" * 70)
    if FAILED:
        print("%d CHECK(S) FAILED:" % len(FAILED))
        for f in FAILED:
            print("   %s" % f)
        return 1
    print("EVERY CHECK PASSED -- the prompt box grounds, asks, answers, "
          "refuses,\nand hands the runner the operator's own sentence.")
    return 0


if __name__ == "__main__":
    rc = main()
    # HARD EXIT, and it is about the exit CODE rather than about tidiness.
    #
    # A Qt application and a spinning rclpy executor tear down on two threads,
    # and this one ends with "terminate called without an active exception"
    # and a core dump AFTER every check has printed and passed. A verifier
    # whose exit code says failure when its output says success is worse than
    # no verifier: whichever a caller reads, the other one is a lie.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
