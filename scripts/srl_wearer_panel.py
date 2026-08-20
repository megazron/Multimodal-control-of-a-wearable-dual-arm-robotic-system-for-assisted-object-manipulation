#!/usr/bin/env python3
"""The WEARER tab: the scene camera, the tracked body over it, and which body
the robot is actually planning against.

WHY THIS PANEL EXISTS AND WHAT IT MUST NEVER DO. The tracker can substitute a
measured body for the mannequin in the collision model. That is the most
consequential thing any perception in this project does -- it changes the
geometry the clearance floor is measured against, and the clearance floor is
the last thing between the arms and a person's chest. So the operator has to
be able to see, at a glance and without interpreting anything:

  * the picture the tracker is working from, or NOTHING if it is stale;
  * where the tracker thinks the body is, drawn ON that picture, so a body
    estimate in the wrong place is visible as a skeleton in the wrong place;
  * where the ROBOT ARMS are in that same picture, projected from their own
    encoders -- which is the only check on the camera transform there is;
  * per segment, the confidence and, when a segment is refused, WHY;
  * and, in the largest text on the panel, whether the planning scene holds
    the TRACKED body or the MANNEQUIN.

The last of those is the one that matters. Everything else on this panel is
diagnosis; that line is the safety state.

NEVER PAINTS A STALE FRAME, for the same reason the wrist camera panels do
not: a frozen picture of a room cannot be told from a live picture of a room
where nobody is moving, and here the consequence is a body model that is
believed while the person has walked away.
"""
import json
import time

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                             QPushButton, QVBoxLayout, QWidget)

STALE_S = 1.0


class FeedView(QWidget):
    """The camera picture with the skeleton and the projected arms on it."""

    def __init__(self, colours, parent=None):
        super().__init__(parent)
        self.c = colours
        self.frame = None            # (w, h, bytes, step, encoding)
        self.frame_t = 0.0
        self.overlay = None
        self.note = "no scene camera"
        self.state = "unknown"
        self.setMinimumSize(320, 190)

    def set_frame(self, raw, t):
        self.frame, self.frame_t = raw, t
        self.update()

    def set_overlay(self, o, note, state):
        self.overlay, self.note, self.state = o, note, state
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(self.c["bg"]))
        fresh = self.frame is not None and (time.monotonic() - self.frame_t) < STALE_S
        box = (0, 0, w, h - 18)
        if fresh:
            fw, fh, buf, step, enc = self.frame
            fmt = (QImage.Format_BGR888 if enc == "bgr8"
                   else QImage.Format_RGB888)
            qi = QImage(buf, fw, fh, step, fmt)
            pm = QPixmap.fromImage(qi).scaled(box[2], box[3],
                                              Qt.KeepAspectRatio,
                                              Qt.SmoothTransformation)
            ox = (box[2] - pm.width()) // 2
            oy = (box[3] - pm.height()) // 2
            p.drawPixmap(ox, oy, pm)
            self._draw_overlay(p, ox, oy, pm.width(), pm.height())
        else:
            # NOT DIMMED, NOT HELD. A stale frame is not shown at all.
            p.setPen(QPen(QColor(self.c["unknown"])))
            f = p.font()
            f.setPointSize(12)
            p.setFont(f)
            p.drawText(box[0], box[1], box[2], box[3], Qt.AlignCenter,
                       "NO SCENE CAMERA" if self.frame is None else "STALE")
        col = {"ok": self.c["accent"], "bad": self.c["bad"],
               "unknown": self.c["unknown"]}.get(self.state, self.c["unknown"])
        p.setPen(QPen(QColor(col)))
        f = p.font()
        f.setPointSize(8)
        p.setFont(f)
        p.drawText(4, h - 5, self.note[:120])
        p.end()

    def _draw_overlay(self, p, ox, oy, pw, ph):
        o = self.overlay
        if not o:
            return
        sx = pw / float(max(1, o.get("width", 1)))
        sy = ph / float(max(1, o.get("height", 1)))

        # THE BODY, coloured by the confidence of its weaker end. A uniformly
        # coloured skeleton hides exactly the case the gates exist for: a
        # torso seen perfectly with elbows at 0.10.
        for seg in o.get("skeleton", []):
            x1, y1, x2, y2, conf = seg
            col = (self.c["accent"] if conf >= 0.5 else self.c["bad"])
            pen = QPen(QColor(col))
            pen.setWidth(3 if conf >= 0.5 else 1)
            if conf < 0.5:
                pen.setStyle(Qt.DotLine)
            p.setPen(pen)
            p.drawLine(int(ox + x1 * sx), int(oy + y1 * sy),
                       int(ox + x2 * sx), int(oy + y2 * sy))

        # THE ROBOT ARMS, projected from their own encoders. If these do not
        # land on the arms in the picture, the camera transform is wrong --
        # and every body position is wrong with it. This is the check.
        for arm, poly in (o.get("arms") or {}).items():
            if len(poly) < 2:
                continue
            pen = QPen(QColor(self.c["warn"]))
            pen.setWidth(2)
            p.setPen(pen)
            for i in range(len(poly) - 1):
                p.drawLine(int(ox + poly[i][0] * sx), int(oy + poly[i][1] * sy),
                           int(ox + poly[i + 1][0] * sx),
                           int(oy + poly[i + 1][1] * sy))
            p.drawText(int(ox + poly[-1][0] * sx) + 4,
                       int(oy + poly[-1][1] * sy), arm)


class WearerPanel(QWidget):
    """Feed, per-segment confidence, and the planning-scene verdict."""

    PARTS = ("torso", "head", "hips", "L-upperarm", "L-forearm", "L-hand",
             "R-upperarm", "R-forearm", "R-hand")

    def __init__(self, colours, on_recalibrate=None, parent=None):
        super().__init__(parent)
        self.c = colours
        v = QVBoxLayout(self)
        v.setContentsMargins(4, 4, 4, 4)

        # THE SAFETY LINE, FIRST AND BIGGEST.
        self.verdict = QLabel("MANNEQUIN -- nothing is being measured")
        f = self.verdict.font()
        f.setPointSize(12)
        f.setBold(True)
        self.verdict.setFont(f)
        self.verdict.setWordWrap(True)
        self.verdict.setStyleSheet("color:%s" % colours["unknown"])
        v.addWidget(self.verdict)

        self.sub = QLabel("")
        sf = self.sub.font()
        sf.setPointSize(8)
        self.sub.setFont(sf)
        self.sub.setWordWrap(True)
        self.sub.setStyleSheet("color:%s" % colours["muted"])
        v.addWidget(self.sub)

        self.feed = FeedView(colours)
        v.addWidget(self.feed, 1)

        g = QGroupBox("Per segment -- what is measured, and why not")
        gl = QGridLayout(g)
        gl.setVerticalSpacing(1)
        self.rows = {}
        for i, part in enumerate(self.PARTS):
            nm = QLabel(part)
            nf = nm.font()
            nf.setPointSize(8)
            nm.setFont(nf)
            src = QLabel("mannequin")
            src.setFont(nf)
            why = QLabel("")
            why.setFont(nf)
            why.setStyleSheet("color:%s" % colours["muted"])
            why.setWordWrap(True)
            gl.addWidget(nm, i, 0)
            gl.addWidget(src, i, 1)
            gl.addWidget(why, i, 2)
            gl.setColumnStretch(2, 1)
            self.rows[part] = (nm, src, why)
        v.addWidget(g)

        row = QHBoxLayout()
        b = QPushButton("what would fix this")
        b.setToolTip("Prints, in the event log, the one thing that would move "
                     "the most parts from the mannequin to measured.")
        b.clicked.connect(self._advise)
        row.addWidget(b)
        if on_recalibrate:
            rb = QPushButton("re-check the camera transform")
            rb.clicked.connect(on_recalibrate)
            row.addWidget(rb)
        row.addStretch(1)
        v.addLayout(row)
        self._advice_cb = None
        self.last = {}

    def set_advice_sink(self, fn):
        self._advice_cb = fn

    def _advise(self):
        if not self._advice_cb:
            return
        self._advice_cb(self.advice())

    def advice(self):
        """The single most useful next action, from what the rows say."""
        d = self.last.get("decisions") or {}
        if not d:
            return ("Nothing is publishing a wearer estimate. Start the "
                    "tracker with scripts/run_wearer_tracker.sh.")
        why = [v[1] for v in d.values() if v[0] == "mannequin"]
        if not why:
            return "Every trackable part is measured. Nothing to fix."
        for key, text in (
                ("position relative to the robot",
                 "The camera's position in the robot frame is not known. Run "
                 "scripts/calibrate_scene_camera_extrinsics.py."),
                ("intrinsic",
                 "The camera has no intrinsic calibration. Run "
                 "scripts/calibrate_scene_camera.py."),
                ("too dark", "The room is too dark for the scene camera."),
                ("blank", "The scene camera is covered, or its driver is "
                          "repeating one frame."),
                ("more than one person",
                 "More than one person is in view. The tracker will not guess "
                 "which one is wearing the arms."),
                ("too old", "The scene camera has stopped delivering."),
                ("moved", "A joint jumped impossibly fast -- the detector "
                          "lost the person for a moment."),
                ("not clearly enough seen",
                 "Parts of the wearer are not clearly visible. Move the "
                 "camera so the whole body is in frame."),
                ("further away than assumed",
                 "The camera sees the wearer further from the robot than the "
                 "model assumes. That is allowed to change nothing -- the "
                 "camera may only make the wearer bigger.")):
            if any(key in w for w in why):
                return text
        return "; ".join(sorted(set(why))[:2])

    # ----------------------------------------------------------- refresh
    def update_from(self, est_json, overlay_json, frame, frame_t, cam_json):
        if frame is not None:
            self.feed.set_frame(frame, frame_t)
        est = self._parse(est_json)
        ovl = self._parse(overlay_json)
        cam = self._parse(cam_json) or {}
        self.last = est or {}

        state = (est or {}).get("state", "")
        if state.startswith("TRACKED"):
            col, s = self.c["accent"], "ok"
        elif state.startswith("PARTLY"):
            col, s = self.c["warn"], "bad"
        else:
            col, s = self.c["unknown"], "unknown"
        self.verdict.setText(
            ("PLANNING SCENE: " + (state or "MANNEQUIN -- no tracker"))[:110])
        self.verdict.setStyleSheet("color:%s" % col)

        bits = []
        if cam:
            bits.append("camera %s" % ("live" if cam.get("live") else
                                       (cam.get("reason") or "not running")))
            if not cam.get("calibrated", False):
                bits.append("NOT CALIBRATED")
        if est and est.get("note"):
            bits.append(est["note"])
        self.sub.setText("   ".join(bits)[:160])
        self.feed.set_overlay(ovl, (est or {}).get("note", "") or
                              (cam.get("reason", "") if cam else ""), s)

        dec = (est or {}).get("decisions") or {}
        segs = (est or {}).get("segments") or {}
        for part, (nm, src, why) in self.rows.items():
            which, reason = dec.get(part, ("mannequin", "no tracker running"))
            seg = segs.get(part) or {}
            conf = seg.get("conf")
            if which == "tracked":
                src.setText("MEASURED %s" % ("" if conf is None
                                             else "%.2f" % conf))
                src.setStyleSheet("color:%s" % self.c["accent"])
                why.setText("")
            else:
                src.setText("mannequin")
                src.setStyleSheet("color:%s" % self.c["muted"])
                why.setText(reason[:70])

    @staticmethod
    def _parse(s):
        if not s:
            return None
        try:
            return json.loads(s)
        except Exception:                                     # noqa: BLE001
            return None
