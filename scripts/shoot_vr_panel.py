#!/usr/bin/env python3
"""Photograph the VR panel on its own, so it can be LOOKED at.

    python3 scripts/shoot_vr_panel.py [out.png]

WHY NOT x11grab. The operations window is 1880x1060 and opens at y=156 on this
display, so its lower third is off the bottom of the screen -- x11grab captures
what the compositor has, and clicking a scrollbar at a coordinate that is not
on the screen lands somewhere else entirely. Three attempts at scrolling the
control column with xdotool moved nothing and produced three identical
screenshots, which is exactly the sort of "evidence" that reads as a pass.

Grabbing the WIDGET renders it through Qt regardless of where it sits on the
screen or whether it is scrolled into view, so what comes out is what the
operator would see once they scrolled to it. That is the point of GUI RULE 3:
a control can pass an audit -- return codes and all -- while being invisible,
cut off, or below the fold, and none of that is visible from a return code.
"""
import sys
import threading

import rclpy

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
import srl_gui  # noqa: E402


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/vr_panel.png"
    from PyQt5.QtWidgets import QApplication

    rclpy.init()
    bus = srl_gui.Bus()
    threading.Thread(target=lambda: rclpy.spin(bus), daemon=True).start()
    app = QApplication([sys.argv[0]])

    class Args:
        no_rviz, single_rviz, self_test_exit = True, False, False

    g = srl_gui.Gui(bus, Args())
    g.show()
    for _ in range(60):
        app.processEvents()

    # The panel is built by _vr_panel(); find it by the widget the new link
    # display lives on rather than by walking the layout tree, which changes.
    w = g.vr_url_lbl
    while w is not None and not (w.__class__.__name__ == "QGroupBox"
                                 and w.title().startswith("VR")):
        w = w.parent()
    if w is None:
        print("could not find the VR group box", file=sys.stderr)
        return 2
    w.adjustSize()
    for _ in range(20):
        app.processEvents()
    pix = w.grab()
    pix.save(out)
    print("wrote %s  (%dx%d)" % (out, pix.width(), pix.height()))

    # AND SAY WHAT IS ACTUALLY IN IT, so a blank or truncated grab is caught
    # by something other than my own eyes.
    print("link labels:")
    print("   %s" % g.vr_url_lbl.text())
    print("   %s" % g.vr_headset_lbl.text())
    print("smoothing controls: law=%s  min_cutoff=%.2f  beta=%.1f  wrist=%s"
          % (g.vr_smooth.currentText(), g.vr_mincut.value(),
             g.vr_beta.value(), g.vr_rot_smooth.isChecked()))
    print("   state line: %r" % g.vr_smooth_state.text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
