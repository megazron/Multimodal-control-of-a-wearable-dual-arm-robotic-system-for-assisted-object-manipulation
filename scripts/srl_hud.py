#!/usr/bin/env python3
"""The dark HUD: palette, type, and the schematic widgets.

THE DESIGN ARGUMENT, because it is the reason for every choice below.

Film-prop interfaces read as advanced because of RESTRAINT -- a dark field,
one accent hue, thin geometric linework, glow used once. They can afford that
because nothing on them has to be diagnosed. This one does, so the restraint
is kept and turned into a diagnostic device:

  GLOW AND SATURATION ARE RESERVED FOR THE ABNORMAL. A healthy rig is drawn
  almost entirely in one desaturated cyan on near-black. Nothing pulses,
  nothing is red, nothing is bright. The moment a channel goes incoherent or a
  clearance closes, that element -- and only that element -- gains saturation
  and a halo. If everything glows, nothing reads as urgent; that is ISA-101's
  point and it survives the aesthetic unchanged.

  UNKNOWN IS NOT A SHADE OF BAD. Violet, distinct from both the accent and the
  alarm, because "I have no data" is a different instruction to the operator
  than "this is wrong" -- and because this project has repeatedly shipped
  indicators that rendered no-data identically to healthy.

  THREE TYPE LEVELS ONLY, and every number is monospace. A proportional font
  reflows as digits change, so a value updating at 10 Hz jitters sideways and
  the eye chases it. Fixed-width numerics hold still, which is what makes a
  changing digit visible at the edge of vision.

THE SCHEMATICS EXIST BECAUSE A TABLE OF FOURTEEN NUMBERS DOES NOT ANSWER THE
QUESTION. "Which channels are driving the robot, and which are frozen" was
readable only by cross-referencing a verdict table against a stale JSON file,
which is exactly how eight repaired channels stayed frozen without anyone
noticing. Drawn as the actual kinematic chain -- roll, bend, roll, bend, roll,
bend, roll -- with each joint lit by its own health, the answer is
pre-attentive.
"""
import math

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import (QBrush, QColor, QFont, QLinearGradient, QPainter,
                         QPainterPath, QPen, QRadialGradient)
from PyQt5.QtWidgets import QSizePolicy, QWidget

# ---------------------------------------------------------------- palette
BG = "#070b0f"            # near black, very slightly blue
PANEL = "#0d141b"
LINE = "#16232e"          # structural rule: visible, never assertive
ACCENT = "#3fb6c9"        # the ONE hue a healthy system is drawn in
ACCENT_DIM = "#1f5a66"
TEXT = "#b9ccd6"
MUTED = "#5c7382"
WARN = "#e8a33d"
BAD = "#ff4d5e"
UNKNOWN = "#a06ce0"       # not a shade of bad: no data is its own state
OK_SOFT = "#2f7d6a"

# Health -> (colour, glow). Glow is the scarce channel: only the abnormal
# states get one, and DEAD deliberately does not -- a dead channel is a
# settled fact, not an alarm. FROZEN does, because a channel frozen by a
# stale baseline is a live fault wearing a calm face.
HEALTH = {
    "ALIVE":        (ACCENT, 0.0),
    "INTERMITTENT": (WARN, 0.35),
    "INCOHERENT":   (BAD, 0.85),
    "DEAD":         (MUTED, 0.0),
    "FROZEN":       (UNKNOWN, 0.7),
    "UNKNOWN":      (UNKNOWN, 0.25),
}


# ---------------------------------------------------------------- type
# HELVETICA, ALIASED. It is licensed and not installed; Liberation Sans is
# metric-compatible, so text laid out for Helvetica keeps its line breaks and
# column widths. ~/.config/fontconfig/fonts.conf states the alias explicitly
# rather than leaning on fontconfig's implicit metric rule, so `fc-match
# Helvetica` is a check that can fail on a machine without it.
#
# VERIFIED AT BOTH LAYERS, because fontconfig resolving is not evidence Qt
# resolves:
#     fc-match Helvetica            -> LiberationSans-Regular.ttf
#     QFontInfo(QFont("Helvetica")) -> "Liberation Sans"
#
# "Helvetica is not resolving" turned out to be that NOTHING EVER ASKED FOR
# IT: this module requested "DejaVu Sans" and "DejaVu Sans Mono" throughout.
# The request is now Helvetica, and it resolves.
SANS = "Helvetica"
# Numerics are fixed-width so a changing value does not shift the digits
# beside it. Liberation Mono is the metric-compatible monospace partner.
MONO = "Liberation Mono"

# Live scale, driven by the GUI's own control. 1.0 is the base size, chosen
# to be readable at arm's length on a 1920x1080 panel.
SCALE = {"k": 1.0}


def set_scale(k):
    """Set the global type scale. Clamped: below 0.7 the labels stop being
    readable at arm's length, above 1.8 the panels overflow."""
    SCALE["k"] = max(0.7, min(1.8, float(k)))
    return SCALE["k"]


def mono(size=11, bold=False):
    f = QFont(MONO, max(6, int(round(size * SCALE["k"]))))
    f.setBold(bold)
    f.setStyleHint(QFont.Monospace)
    # Fixed advance width, so 0.199 -> 0.200 does not move anything.
    f.setFixedPitch(True)
    return f


def sans(size=11, bold=False):
    f = QFont(SANS, max(6, int(round(size * SCALE["k"]))))
    f.setBold(bold)
    f.setStyleHint(QFont.Helvetica)
    return f


def _c(x, a=255):
    q = QColor(x)
    q.setAlpha(a)
    return q


class Card(QWidget):
    """A panel. One hairline, no fill beyond the panel tone, generous inset."""

    TITLE_H = 20

    def __init__(self, title=""):
        super().__init__()
        self.title = title
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paint_frame(self, p):
        r = self.rect().adjusted(0, 0, -1, -1)
        p.fillRect(r, _c(PANEL))
        p.setPen(QPen(_c(LINE), 1))
        p.drawRect(r)
        if self.title:
            p.setFont(sans(8, True))
            p.setPen(_c(MUTED))
            p.drawText(QRectF(10, 4, r.width() - 20, self.TITLE_H),
                       Qt.AlignLeft | Qt.AlignVCenter, self.title.upper())
            p.setPen(QPen(_c(LINE), 1))
            p.drawLine(10, self.TITLE_H + 4, r.width() - 10, self.TITLE_H + 4)


def glow_dot(p, x, y, rad, colour, strength):
    """A joint node. The halo is the alarm channel and is drawn ONLY when
    `strength` > 0, so a healthy chain has no glow anywhere on it."""
    if strength > 0:
        g = QRadialGradient(QPointF(x, y), rad * 4.2)
        g.setColorAt(0.0, _c(colour, int(150 * strength)))
        g.setColorAt(0.45, _c(colour, int(55 * strength)))
        g.setColorAt(1.0, _c(colour, 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(g))
        p.drawEllipse(QPointF(x, y), rad * 4.2, rad * 4.2)
    p.setPen(QPen(_c(colour), 1.4))
    p.setBrush(QBrush(_c(BG)))
    p.drawEllipse(QPointF(x, y), rad, rad)


class MasterArmSchematic(Card):
    """Both master arms as the real kinematic chain, lit by channel health.

    THE CHAIN IS DRAWN AS IT IS, not as a row of boxes: J1 roll, J2 bend,
    J3 roll, J4 bend, J5 roll, J6 bend, J7 roll. A ROLL is drawn as a ring
    around the link axis and a BEND as a hinge across it, so the alternation
    is visible without reading a single label. That alternation is what makes
    "both bends are dead so there is no reach observable" something you can
    SEE -- it was previously a regression result buried in a commit message.
    """

    ROLL, BEND = "roll", "bend"
    KIND = [ROLL, BEND, ROLL, BEND, ROLL, BEND, ROLL]

    def __init__(self):
        super().__init__("master arm")
        self.snap = {}
        self.setMinimumHeight(272)

    def set_data(self, d):
        self.snap = d or {}
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        self.paint_frame(p)
        top = self.TITLE_H + 14
        h = max(60, (self.height() - top - 26) / 2.0)
        for i, arm in enumerate(("left", "right")):
            self._arm(p, arm, top + i * h, h)
        p.setFont(mono(7))
        p.setPen(_c(MUTED))
        p.drawText(QRectF(10, self.height() - 16, self.width() - 20, 12),
                   Qt.AlignLeft,
                   "ring = roll   hinge = bend   halo = abnormal")

    def _arm(self, p, arm, y0, h):
        d = self.snap.get(arm) or {}
        joints = d.get("joints") or []
        SIDE = 196.0                 # fixed sidebar; the chain takes the rest
        x0 = 68.0
        x1 = max(x0 + 60.0, self.width() - SIDE - 22.0)
        cy = y0 + h * 0.50
        step = (x1 - x0) / 6.0

        p.setFont(sans(9, True))
        p.setPen(_c(TEXT))
        p.drawText(QRectF(10, cy - 9, 56, 18), Qt.AlignLeft | Qt.AlignVCenter,
                   arm.upper())

        # the link line, drawn segment by segment so a frozen segment reads
        # as a broken chain rather than a coloured dot on a solid one
        for k in range(6):
            a = joints[k] if k < len(joints) else {}
            b = joints[k + 1] if k + 1 < len(joints) else {}
            col = HEALTH.get(a.get("health", "UNKNOWN"), HEALTH["UNKNOWN"])[0]
            col2 = HEALTH.get(b.get("health", "UNKNOWN"), HEALTH["UNKNOWN"])[0]
            grad = QLinearGradient(x0 + k * step, cy, x0 + (k + 1) * step, cy)
            grad.setColorAt(0.0, _c(col, 150))
            grad.setColorAt(1.0, _c(col2, 150))
            p.setPen(QPen(QBrush(grad), 1.6))
            p.drawLine(QPointF(x0 + k * step, cy), QPointF(x0 + (k + 1) * step, cy))

        for k in range(7):
            j = joints[k] if k < len(joints) else {}
            health = j.get("health", "UNKNOWN")
            col, glow = HEALTH.get(health, HEALTH["UNKNOWN"])
            x = x0 + k * step
            kind = self.KIND[k]
            if kind == self.ROLL:
                if glow > 0:
                    glow_dot(p, x, cy, 6.0, col, glow)
                p.setPen(QPen(_c(col), 1.3))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(QPointF(x, cy), 8.5, 8.5)
                p.drawEllipse(QPointF(x, cy), 3.0, 3.0)
            else:
                if glow > 0:
                    glow_dot(p, x, cy, 6.0, col, glow)
                p.setPen(QPen(_c(col), 1.6))
                p.setBrush(QBrush(_c(BG)))
                path = QPainterPath()
                path.moveTo(x - 7.0, cy - 9.5)
                path.lineTo(x + 7.0, cy - 9.5)
                path.lineTo(x + 7.0, cy + 9.5)
                path.lineTo(x - 7.0, cy + 9.5)
                path.closeSubpath()
                p.drawPath(path)
                p.setPen(QPen(_c(col), 1.1))
                p.drawLine(QPointF(x - 7.0, cy), QPointF(x + 7.0, cy))

            # FROZEN gets a strike-through: the channel exists, is healthy,
            # and is not being used. That distinction is the one that hid a
            # whole repair behind a stale baseline file.
            if health == "FROZEN":
                p.setPen(QPen(_c(UNKNOWN), 1.2, Qt.DashLine))
                p.drawLine(QPointF(x - 9, cy - 9), QPointF(x + 9, cy + 9))

            p.setFont(mono(7))
            p.setPen(_c(MUTED))
            p.drawText(QRectF(x - 16, cy - 30, 32, 10), Qt.AlignHCenter,
                       "J%d" % (k + 1))
            ang = j.get("deg")
            p.setFont(mono(9))
            p.setPen(_c(TEXT if ang is not None else MUTED))
            p.drawText(QRectF(x - 24, cy + 15, 48, 12), Qt.AlignHCenter,
                       "--" if ang is None else "%+.0f" % ang)
            age = j.get("age_s")
            p.setFont(mono(7))
            stale = (age is not None and age > 2.0)
            p.setPen(_c(BAD if stale else MUTED))
            p.drawText(QRectF(x - 24, cy + 27, 48, 10), Qt.AlignHCenter,
                       "--" if age is None else
                       ("%.1fs" % age if age < 100 else ">99s"))

        self._sidebar(p, d, x1 + 22, cy - 66, h)

    def _sidebar(self, p, d, x, y, h):
        """IMU gate, FSR thresholds, buttons and the capability rung.

        Clamped into this arm's own half: the block is ~150 px tall and ran
        over the next arm's IMU row when the card was capped, which put two
        arms' numbers on one line -- the one thing a status panel may never
        do.
        """
        y = max(y, 0.0)
        w = max(120.0, self.width() - x - 12)
        p.setFont(mono(7))

        # ---- IMU: the 1 +/- 0.15 g gate DRAWN, with the live magnitude on it
        g = d.get("accel_g")
        lo, hi = 0.85, 1.15
        bx, bw = x, max(60.0, w - 8)
        p.setPen(_c(MUTED))
        p.drawText(QRectF(x, y, w, 10), Qt.AlignLeft, "IMU |a|")
        by = y + 12
        p.setPen(QPen(_c(LINE), 1))
        p.drawLine(QPointF(bx, by + 4), QPointF(bx + bw, by + 4))
        # the gate band
        def gx(v):
            return bx + bw * min(1.0, max(0.0, (v - 0.5) / 1.0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(_c(ACCENT_DIM, 90)))
        p.drawRect(QRectF(gx(lo), by, gx(hi) - gx(lo), 9))
        if g is not None:
            inside = lo <= g <= hi
            col = ACCENT if inside else BAD
            if not inside:
                glow_dot(p, gx(g), by + 4, 2.6, col, 0.8)
            p.setPen(QPen(_c(col), 1.6))
            p.drawLine(QPointF(gx(g), by - 3), QPointF(gx(g), by + 11))
        p.setFont(mono(7))
        p.setPen(_c(TEXT if g is not None else MUTED))
        p.drawText(QRectF(x, by + 12, w, 10), Qt.AlignLeft,
                   "--" if g is None else "%.3f g  gate %s"
                   % (g, "OPEN" if lo <= g <= hi else "SHUT"))
        gy = d.get("gyro_dps")
        p.setPen(_c(MUTED))
        p.drawText(QRectF(x, by + 22, w, 10), Qt.AlignLeft,
                   "gyro --" if gy is None else "gyro %6.1f d/s" % gy)

        # ---- FSR with the deadband and latch thresholds marked
        fy = by + 36
        p.setPen(_c(MUTED))
        p.drawText(QRectF(x, fy, w, 10), Qt.AlignLeft, "FSR")
        raw = d.get("fsr")
        fb = fy + 12
        full = 3603.0
        p.setPen(QPen(_c(LINE), 1))
        p.drawLine(QPointF(bx, fb + 4), QPointF(bx + bw, fb + 4))
        for val, lab, col in ((250, "db", MUTED), (400, "rel", ACCENT_DIM),
                              (1200, "latch", ACCENT)):
            fx = bx + bw * min(1.0, val / full)
            p.setPen(QPen(_c(col), 1, Qt.DotLine))
            p.drawLine(QPointF(fx, fb - 2), QPointF(fx, fb + 10))
        if raw is not None:
            fx = bx + bw * min(1.0, raw / full)
            latched = d.get("latched")
            col = ACCENT if not latched else WARN
            if latched:
                glow_dot(p, fx, fb + 4, 2.4, col, 0.5)
            p.setPen(QPen(_c(col), 1.8))
            p.drawLine(QPointF(fx, fb - 3), QPointF(fx, fb + 11))
        p.setFont(mono(7))
        p.setPen(_c(TEXT if raw is not None else MUTED))
        p.drawText(QRectF(x, fb + 12, w, 10), Qt.AlignLeft,
                   "--" if raw is None else "%4.0f  %s"
                   % (raw, "LATCHED" if d.get("latched") else "open"))

        # ---- clutch + button
        cy2 = fb + 24
        cl = d.get("clutch")
        col = MUTED if cl is None else (ACCENT if cl else WARN)
        p.setPen(_c(col))
        p.drawText(QRectF(x, cy2, w, 10), Qt.AlignLeft,
                   "clutch %s" % ("--" if cl is None
                                  else ("ENGAGED" if cl else "released")))
        btn = d.get("button")
        p.setPen(_c(ACCENT if btn else MUTED))
        p.drawText(QRectF(x, cy2 + 10, w, 10), Qt.AlignLeft,
                   "button %s" % ("DOWN" if btn else "up"))

        # ---- capability rung and what the next repair would buy
        p.setFont(mono(7, True))
        rung = d.get("rung")
        p.setPen(_c(TEXT if rung else UNKNOWN))
        p.drawText(QRectF(x, cy2 + 22, w, 10), Qt.AlignLeft,
                   "rung %s" % (rung or "--"))
        p.setFont(mono(7))
        p.setPen(_c(MUTED))
        p.drawText(QRectF(x, cy2 + 31, w, 26), Qt.AlignLeft | Qt.TextWordWrap,
                   d.get("regain") or "")


class RobotSchematic(Card):
    """Seven joints per arm with LIMITS DRAWN, wrap counts, aperture, clearance.

    A joint angle is only meaningful against its limit, and this rig has
    joints that sit at 44-58% of travel while the arm is nonetheless
    unreachable -- a number alone invites the wrong conclusion. The bar shows
    where in its range each joint actually is.
    """

    CONT = (0, 2, 4, 6)          # joints 1,3,5,7 are continuous

    def __init__(self):
        super().__init__("robot")
        self.snap = {}
        self.setMinimumHeight(210)

    def set_data(self, d):
        self.snap = d or {}
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        self.paint_frame(p)
        top = self.TITLE_H + 14
        h = max(50, (self.height() - top - 16) / 2.0)
        for i, arm in enumerate(("left", "right")):
            self._arm(p, arm, top + i * h, h)

    def _arm(self, p, arm, y0, h):
        d = self.snap.get(arm) or {}
        joints = d.get("joints") or []
        x0 = 60.0
        x1 = self.width() - 190.0
        step = (x1 - x0) / 7.0
        bar_h = min(38.0, h * 0.46)
        cy = y0 + max(6.0, (h - bar_h - 24) * 0.42)

        p.setFont(sans(9, True))
        p.setPen(_c(TEXT))
        p.drawText(QRectF(10, cy + bar_h * 0.3, 46, 16),
                   Qt.AlignLeft | Qt.AlignVCenter, arm.upper())

        for k in range(7):
            j = joints[k] if k < len(joints) else {}
            x = x0 + k * step
            w = step * 0.62
            p.setPen(QPen(_c(LINE), 1))
            p.setBrush(Qt.NoBrush)
            p.drawRect(QRectF(x, cy, w, bar_h))
            frac = j.get("frac")            # 0..1 position within the limits
            if frac is not None:
                near = abs(frac - 0.5) > 0.44        # within 6% of a stop
                col = BAD if near else ACCENT
                yy = cy + bar_h * (1.0 - frac)
                if near:
                    glow_dot(p, x + w / 2.0, yy, 2.6, col, 0.75)
                p.setPen(QPen(_c(col), 1.8))
                p.drawLine(QPointF(x + 1, yy), QPointF(x + w - 1, yy))
            p.setFont(mono(7))
            p.setPen(_c(MUTED))
            p.drawText(QRectF(x - 4, cy + bar_h + 1, w + 8, 10),
                       Qt.AlignHCenter, "J%d" % (k + 1))
            if k in self.CONT:
                nw = j.get("wraps")
                p.setPen(_c(WARN if nw else MUTED))
                p.drawText(QRectF(x - 4, cy + bar_h + 10, w + 8, 10),
                           Qt.AlignHCenter,
                           "w%d" % nw if nw else "w0")

        self._side(p, d, x1 + 14, cy, h)

    def _side(self, p, d, x, y, h):
        w = self.width() - x - 12
        p.setFont(mono(7))
        # aperture against the object's width
        ap = d.get("aperture_rad")
        need = d.get("object_rad")
        p.setPen(_c(MUTED))
        p.drawText(QRectF(x, y, w, 10), Qt.AlignLeft, "aperture")
        by = y + 12
        p.setPen(QPen(_c(LINE), 1))
        p.drawRect(QRectF(x, by, w - 6, 9))
        if need is not None:
            nx = x + (w - 6) * min(1.0, need / 0.8)
            p.setPen(QPen(_c(UNKNOWN), 1, Qt.DotLine))
            p.drawLine(QPointF(nx, by - 2), QPointF(nx, by + 11))
        if ap is not None:
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(_c(ACCENT, 120)))
            p.drawRect(QRectF(x + 1, by + 1, (w - 8) * min(1.0, ap / 0.8), 7))
        p.setPen(_c(TEXT if ap is not None else MUTED))
        p.drawText(QRectF(x, by + 12, w, 10), Qt.AlignLeft,
                   "--" if ap is None else "%.2f rad" % ap)

        # clearance as a MARGIN against the floor, not a bare number
        cy2 = by + 26
        clr = d.get("clearance_m")
        floor = d.get("floor_m", 0.12)
        p.setPen(_c(MUTED))
        p.drawText(QRectF(x, cy2, w, 10), Qt.AlignLeft, "clearance")
        cb = cy2 + 12
        p.setPen(QPen(_c(LINE), 1))
        p.drawRect(QRectF(x, cb, w - 6, 9))
        fx = x + (w - 6) * min(1.0, floor / 0.5)
        p.setPen(QPen(_c(BAD), 1, Qt.DashLine))
        p.drawLine(QPointF(fx, cb - 2), QPointF(fx, cb + 11))
        if clr is not None and clr >= 0:
            below = clr < floor
            col = BAD if below else ACCENT
            if below:
                glow_dot(p, x + 6, cb + 4, 3.0, BAD, 1.0)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(_c(col, 130)))
            p.drawRect(QRectF(x + 1, cb + 1, (w - 8) * min(1.0, clr / 0.5), 7))
        p.setPen(_c(TEXT if (clr is not None and clr >= 0) else UNKNOWN))
        p.drawText(QRectF(x, cb + 12, w, 10), Qt.AlignLeft,
                   "--" if (clr is None or clr < 0) else
                   "%.3f m  floor %.2f" % (clr, floor))


class Strip(Card):
    """A rolling time-series strip, one line per arm. The console's Charts.

    FOUR SERIES, chosen because each is a quantity you cannot judge from an
    instantaneous number: EE height (is it drifting?), clearance (is it
    closing?), and sim-to-real lag (is it diverging?). A single sample of any
    of those answers nothing; the shape over sixty seconds answers it at a
    glance, which is the whole reason the console had plots and the rewrite
    should not have dropped them.
    """

    WINDOW_S = 60.0

    def __init__(self, title, unit, floor=None):
        super().__init__(title)
        self.unit = unit
        self.floor = floor
        self.hist = {"left": [], "right": []}
        self.setMinimumHeight(74)

    def push(self, t, left, right):
        for k, v in (("left", left), ("right", right)):
            if v is None:
                continue
            h = self.hist[k]
            h.append((t, float(v)))
            cut = t - self.WINDOW_S
            while h and h[0][0] < cut:
                h.pop(0)
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        self.paint_frame(p)
        x0, y0 = 44.0, self.TITLE_H + 10.0
        w = self.width() - x0 - 58.0
        h = max(18.0, self.height() - y0 - 12.0)
        pts = [v for k in self.hist for _, v in self.hist[k]]
        if not pts:
            p.setFont(mono(8))
            p.setPen(_c(UNKNOWN))
            p.drawText(QRectF(x0, y0, w, h), Qt.AlignCenter,
                       "no data -- not a flat line")
            return
        lo, hi = min(pts), max(pts)
        if self.floor is not None:
            lo, hi = min(lo, self.floor * 0.8), max(hi, self.floor * 1.2)
        if hi - lo < 1e-6:
            lo, hi = lo - 0.01, hi + 0.01
        t1 = max(t for k in self.hist for t, _ in self.hist[k])

        def px(t):
            return x0 + w * (1.0 - min(1.0, (t1 - t) / self.WINDOW_S))

        def py(v):
            return y0 + h * (1.0 - (v - lo) / (hi - lo))

        if self.floor is not None:
            p.setPen(QPen(_c(BAD), 1, Qt.DashLine))
            p.drawLine(QPointF(x0, py(self.floor)), QPointF(x0 + w, py(self.floor)))
        for k, col in (("left", ACCENT), ("right", WARN)):
            hst = self.hist[k]
            if len(hst) < 2:
                continue
            p.setPen(QPen(_c(col), 1.3))
            for i in range(1, len(hst)):
                p.drawLine(QPointF(px(hst[i - 1][0]), py(hst[i - 1][1])),
                           QPointF(px(hst[i][0]), py(hst[i][1])))
        p.setFont(mono(7))
        p.setPen(_c(MUTED))
        p.drawText(QRectF(2, y0 - 5, x0 - 6, 10), Qt.AlignRight, "%.2f" % hi)
        p.drawText(QRectF(2, y0 + h - 6, x0 - 6, 10), Qt.AlignRight, "%.2f" % lo)
        for k, col, dy in (("left", ACCENT, 0), ("right", WARN, 10)):
            if self.hist[k]:
                p.setPen(_c(col))
                p.drawText(QRectF(x0 + w + 4, y0 + dy, 54, 10), Qt.AlignLeft,
                           "%s %.3f" % (k[0].upper(), self.hist[k][-1][1]))


# ====================================================================== #
#  READY TO RUN — the go/no-go indicator                                  #
# ====================================================================== #
class ReadyPanel(Card):
    """One large answer: can a session start now?

    THREE STATES, AND THEY DIFFER IN SHAPE AS WELL AS COLOUR. `ok` is a
    filled pip, `fail` is a filled pip with a glow, `unknown` is a HOLLOW
    RING. That is deliberate: colour alone fails for a colour-blind operator
    and fails again in a greyscale screenshot, and the whole reason this panel
    exists is that "not checked" has repeatedly been mistaken for "healthy" in
    this project. A hollow ring cannot be mistaken for a filled dot.

    GLOW IS RESERVED FOR THE ABNORMAL. A ready system is drawn almost
    entirely in the accent with no glow anywhere; if everything glowed,
    nothing would read as urgent.
    """

    def __init__(self, parent=None):
        super().__init__("READY TO RUN")
        self.data = None
        self.setMinimumHeight(300)

    def set_data(self, d):
        self.data = d
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        self.paint_frame(p)
        x0, y = 14, 42
        w = self.width() - 28
        d = self.data

        if not d:
            p.setFont(sans(12))
            p.setPen(_c(UNKNOWN))
            p.drawText(x0, y + 24, "NO READINESS REPORT")
            p.setFont(sans(9))
            p.setPen(_c(MUTED))
            p.drawText(x0, y + 46,
                       "session_manager is not publishing /session/ready.")
            p.drawText(x0, y + 62,
                       "This is NOT the same as 'ready' — nothing is known.")
            p.end()
            return

        ready = bool(d.get("ready"))
        nf, nu = d.get("n_fail", 0), d.get("n_unknown", 0)

        # ---- the verdict slab ------------------------------------------
        h = 54
        if ready:
            p.fillRect(x0, y, w, h, _c(ACCENT, 34))
            p.setPen(QPen(_c(ACCENT), 1))
        else:
            # a blocked start is the one thing allowed to shout
            p.fillRect(x0, y, w, h, _c(BAD if nf else UNKNOWN, 40))
            p.setPen(QPen(_c(BAD if nf else UNKNOWN), 1))
        p.drawRect(x0, y, w, h)
        p.setFont(sans(20, True))
        p.setPen(_c(ACCENT if ready else (BAD if nf else UNKNOWN)))
        p.drawText(x0 + 14, y + 36, "READY" if ready else "NOT READY")
        p.setFont(mono(10))
        p.setPen(_c(TEXT))
        if not ready:
            bits = []
            if nf:
                bits.append("%d failing" % nf)
            if nu:
                bits.append("%d NOT CHECKED" % nu)
            p.drawText(x0 + 230, y + 36, "   ".join(bits))
        y += h + 14

        # ---- the nine checks -------------------------------------------
        order = ("channels", "cameras", "estop", "homed", "scene", "disk",
                 "one_stack", "recovery", "command_path")
        checks = d.get("checks", {})
        p.setFont(mono(9))
        for name in order:
            c = checks.get(name)
            if not c:
                continue
            st = c.get("state", "unknown")
            col = {"ok": ACCENT, "fail": BAD}.get(st, UNKNOWN)
            cx, cy = x0 + 8, y + 5
            if st == "unknown":
                # HOLLOW RING — distinguishable without colour
                p.setBrush(Qt.NoBrush)
                p.setPen(QPen(_c(col), 1.6))
                p.drawEllipse(cx - 5, cy - 5, 10, 10)
            else:
                if st == "fail":
                    g = QRadialGradient(cx, cy, 12)
                    g.setColorAt(0.0, _c(col, 150))
                    g.setColorAt(1.0, _c(col, 0))
                    p.setBrush(QBrush(g))
                    p.setPen(Qt.NoPen)
                    p.drawEllipse(cx - 12, cy - 12, 24, 24)
                p.setBrush(QBrush(_c(col)))
                p.setPen(Qt.NoPen)
                p.drawEllipse(cx - 4, cy - 4, 8, 8)
            p.setPen(_c(TEXT if st != "unknown" else MUTED))
            p.drawText(x0 + 24, y + 9, c.get("label", name)[:26])
            p.setPen(_c(col if st != "ok" else MUTED))
            p.drawText(x0 + 230, y + 9, str(c.get("detail", ""))[:50])
            y += 19

        p.setFont(sans(8))
        p.setPen(_c(MUTED))
        p.drawText(x0, y + 14,
                   "hollow ring = NOT CHECKED. It blocks exactly as a failure "
                   "does, and never reads as healthy.")
        p.end()
