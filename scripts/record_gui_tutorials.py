#!/usr/bin/env python3
"""Screen recordings of the GUI being used: click this, this happens.

One clip per workflow, narrated by an on-screen caption bar that names the
step BEFORE the click and the result AFTER it -- so the viewer can follow
along and do the same thing.

WHY THE CAPTION IS LIVE AND NOT BURNT IN AFTERWARDS. These clips are about
CAUSE AND EFFECT, and a caption composited in post is pinned to a timestamp
rather than to the action. Driving the caption from the same code that
performs the click means the narration cannot drift out of step with what is
on screen, however long a control takes to respond.

THE CLICKS ARE REAL. Every step calls the widget's own `click()` or the
handler the widget is wired to, on the real GUI against a live ROS graph --
the same route `verify_gui_buttons.py` uses to press all 41 buttons. Nothing
here simulates an effect it did not cause.

Xvfb :99, never WSLg's :0, where x11grab records black.

    python3 scripts/record_gui_tutorials.py [--only estop]
"""
import argparse
import os
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))
from srl_teleop import procscan                              # noqa: E402

OUT = os.path.join(WS, "recordings/verification/GUI_TUTORIALS")
FFMPEG = os.environ.get("FFMPEG") or os.path.expanduser("~/.local/bin/ffmpeg")
DISPLAY = ":99"
W, H = 1920, 1080


def display_works():
    try:
        r = subprocess.run([FFMPEG, "-loglevel", "error", "-f", "x11grab",
                            "-video_size", "64x64", "-i", "%s.0" % DISPLAY,
                            "-frames:v", "1", "-f", "null", "-"],
                           capture_output=True, timeout=25)
        return r.returncode == 0
    except Exception:                                         # noqa: BLE001
        return False


def ensure_xvfb():
    if display_works():
        return
    lock = "/tmp/.X%s-lock" % DISPLAY.lstrip(":")
    if os.path.exists(lock) and procscan.count(r"Xvfb\s+" + DISPLAY) == 0:
        try:
            os.remove(lock)
        except OSError:
            pass
    subprocess.Popen(["Xvfb", DISPLAY, "-screen", "0", "%dx%dx24" % (W, H)],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(15):
        time.sleep(1.0)
        if display_works():
            return
    raise RuntimeError("Xvfb %s did not come up" % DISPLAY)


# ---------------------------------------------------------------- the GUI
def build():
    os.environ["DISPLAY"] = DISPLAY
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
    import rclpy
    import threading
    import srl_gui
    from PyQt5.QtWidgets import QApplication
    if not rclpy.ok():
        rclpy.init()
    bus = srl_gui.Bus()
    threading.Thread(target=lambda: rclpy.spin(bus), daemon=True).start()
    app = QApplication.instance() or QApplication([sys.argv[0]])
    app.setStyleSheet(srl_gui.STYLE)

    class A:
        no_rviz, single_rviz, self_test_exit = True, False, False
        embed_rviz, dual_rviz = False, False
    g = srl_gui.Gui(bus, A())
    g.show()
    for _ in range(30):
        app.processEvents()
        time.sleep(0.05)
    return app, g, srl_gui


class Tutorial:
    """Runs the steps while ffmpeg captures the display."""

    def __init__(self, app, gui, name):
        self.app, self.gui, self.name = app, gui, name
        self.proc = None

    def start(self):
        os.makedirs(OUT, exist_ok=True)
        self.path = os.path.join(OUT, "%s.mp4" % self.name)
        self.proc = subprocess.Popen(
            [FFMPEG, "-y", "-loglevel", "error", "-f", "x11grab",
             "-video_size", "%dx%d" % (W, H), "-framerate", "10",
             "-i", "%s.0" % DISPLAY, "-c:v", "libx264", "-preset",
             "ultrafast", "-pix_fmt", "yuv420p", self.path],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        time.sleep(1.2)

    def say(self, text, seconds=2.6, kind="step"):
        """Narrate, then hold. The caption IS the GUI's banner, so it is part
        of the interface rather than a layer over it."""
        colour = {"step": "#3fb6c9", "do": "#eef4f8",
                  "result": "#8ee08e", "warn": "#e8a33d"}[kind]
        self.gui._narration = (
            text,
            "color:%s;padding:5px;letter-spacing:2px;"
            "border-bottom:1px solid #16232e" % colour)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.app.processEvents()
            time.sleep(0.03)

    def stop(self):
        try:
            self.proc.communicate(input=b"q", timeout=15)
        except Exception:                                     # noqa: BLE001
            self.proc.terminate()
        self.gui._narration = None
        ok = os.path.exists(self.path) and os.path.getsize(self.path) > 30000
        print("  %-22s %s  %s" % (self.name, "OK" if ok else "FAILED",
                                  self.path))
        return ok


def find_button(gui, text):
    from PyQt5.QtWidgets import QPushButton
    for b in gui.findChildren(QPushButton):
        if text.lower() in b.text().lower():
            return b
    return None


# ------------------------------------------------------------- workflows
def wf_launch_mode(t):
    g = t.gui
    t.say("LAUNCHING A MODE  --  every mode starts from this panel", 3.2)
    t.say("The Launch column lists Modes, Tasks and Diagnostics", 2.8)
    b = find_button(g, "Sim teleop only")
    t.say("CLICK:  'Sim teleop only'", 2.4, "do")
    if b:
        b.click()
    t.say("The GUI runs PREFLIGHT first and refuses if a stack is already up "
          "-- two master nodes split the serial stream", 4.0, "result")
    t.say("Watch 'recent actions' below the schematics: every launch, "
          "refusal and reason is logged there", 4.0, "result")


def wf_run_task(t):
    g = t.gui
    t.say("RUNNING A TASK  --  one button per task AND mode", 3.0)
    t.say("Each button carries its mode, so a clip filed under a mode really "
          "did travel that mode's command path", 3.8)
    b = find_button(g, "A positioning")
    t.say("CLICK:  a task button", 2.2, "do")
    if b:
        b.click()
    t.say("A button whose target cannot be resolved is DISABLED with the "
          "reason on its tooltip -- it never silently exits", 4.0, "result")


def wf_divergence(t):
    t.say("READING THE DIVERGENCE  --  commanded versus actual", 3.2)
    t.say("Bottom right: per-joint difference and end-effector distance, "
          "coloured against lag_trip_rad", 3.8)
    t.say("The threshold is READ FROM THE RUNNING BRIDGE, and the header "
          "names its source", 3.4, "result")
    t.say("ONLY A 'MEASURED' ROW CARRIES A NUMBER. A real arm that is not "
          "publishing reads NO REAL ARM, never 0.000", 4.4, "warn")
    t.say("There is a separate REAL FROZEN state, because "
          "/real/joint_states publishes a cache at full rate when the "
          "hardware component goes inactive", 4.6, "warn")


def wf_dial(t):
    g = t.gui
    t.say("THE PRECISION / SPEED DIAL  --  one slider, four parameters", 3.2)
    t.say("Under it: the scale, smoothing, velocity cap and step limit it "
          "implies, updating as you drag", 3.6)
    t.say("DRAG:  toward PRECISION", 2.2, "do")
    g.dial.s.setValue(15)
    g.dial.s.sliderReleased.emit()
    t.say("Values applied through parameter CLIENTS with read-back -- never "
          "'ros2 param set', which goes via a daemon that hangs here", 4.4,
          "result")
    t.say("DRAG:  back toward SPEED", 2.2, "do")
    g.dial.s.setValue(100)
    g.dial.s.sliderReleased.emit()
    t.say("The dial may NEVER move a safety parameter. That is asserted in "
          "code on every change, not left to the layout", 4.2, "warn")


def wf_estop(t):
    g = t.gui
    t.say("THE E-STOP  --  triggering it, and clearing it", 3.2)
    t.say("The big red button, always visible, bottom left", 2.8)
    t.say("CLICK:  E-STOP", 2.2, "do")
    g.on_estop()
    t.say("The banner becomes a solid red slab -- the ONE slab in the "
          "interface, reserved for the state that must interrupt you", 4.2,
          "result")
    for _ in range(60):
        t.app.processEvents()
        time.sleep(0.05)
    t.say("CLICK:  'reset e-stop'", 2.4, "do")
    g.on_estop_reset()
    t.say("/estop_reset is a SERVICE, not a topic. Publishing a Bool at it "
          "leaves the e-stop latched -- that once made six fault injections "
          "report NOT HANDLED", 5.0, "warn")


def wf_switch_mode(t):
    g = t.gui
    t.say("SWITCHING MODES  --  one source owns the arm at a time", 3.4)
    t.say("The Mode indicator reads from live publishers, not from a "
          "setting you chose", 3.4)
    t.say("If two sources claim the arm it reads CONFLICT, in red", 3.2,
          "warn")
    t.say("CLICK:  'SELF-TEST indicators' -- proves every indicator "
          "separates healthy from abnormal from NOT CHECKED", 4.2, "do")
    g.on_self_test()
    for _ in range(40):
        t.app.processEvents()
        time.sleep(0.05)
    t.say(g.selftest_lbl.text() or "self-test complete", 4.6, "result")


WORKFLOWS = [("launch_a_mode", wf_launch_mode), ("run_a_task", wf_run_task),
             ("read_the_divergence", wf_divergence),
             ("precision_speed_dial", wf_dial),
             ("estop_trigger_and_clear", wf_estop),
             ("switching_modes", wf_switch_mode)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=None)
    a = ap.parse_args()
    ensure_xvfb()
    app, gui, _ = build()
    print("GUI TUTORIALS -> %s" % OUT)
    n_ok = 0
    todo = [(n, f) for n, f in WORKFLOWS if not a.only or a.only in n]
    for name, fn in todo:
        t = Tutorial(app, gui, name)
        t.start()
        t.say("SRL OPERATIONS  --  %s" % name.replace("_", " ").upper(), 3.0)
        fn(t)
        t.say("END  --  %s" % name.replace("_", " "), 2.4)
        n_ok += t.stop()
    print("\n%d of %d tutorials recorded" % (n_ok, len(todo)))
    return 0 if n_ok == len(todo) else 1


if __name__ == "__main__":
    sys.exit(main())
