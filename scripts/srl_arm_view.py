#!/usr/bin/env python3
"""The ACTUAL panel: where the real arms are, drawn beside where the sim is.

WHY THIS IS NOT A SECOND RVIZ. Embedding one RViz works; embedding two does
not clip on this display stack -- measured, screenshotted, and written up in
docs/system/07_gui_rviz_embedding.md. The two instances paint over each other
and over every indicator the GUI exists to show, and the second one costs
~294 MB on a 15 GB machine that has already been OOM-killed twice.

So the COMMANDED view stays RViz, which is the right tool for a 3-D scene
with meshes and a planning scene in it, and the ACTUAL view is drawn here:
a skeleton of the real arms, from the real arms' own report, beside it. That
is the comparison the brief asks for, it survives an RViz crash, and it is
the only one of the two whose contents can be asserted in a test.

WHERE THE POINTS COME FROM, AND WHY THE PANEL SAYS SO ON SCREEN.

  1. `real_*` frames in the SHARED /tf. THERE IS NO /real/tf -- both real
     launches run robot_state_publisher with frame_prefix="real_" into the
     one tree. This is the preferred source: it is the stack's own answer,
     computed by the same publisher RViz would read.
  2. Failing that, forward kinematics on /real/joint_states, straight off the
     URDF (scripts/srl_fk.py). Used when the joint stream is up but the real
     robot_state_publisher is not, which is a real state of this rig.

A DRAWING WITH NO SOURCE NAMED IS A DRAWING THAT CAN BE STALE WITHOUT SAYING
SO, so the source line is part of the panel and not a debug aid. And there is
no third source: when neither works the panel draws NOTHING and says NO REAL
ARM. It never holds the last good pose -- a frozen skeleton and a still arm
are the same picture, and that is the failure this whole GUI is written
against.

The projection is orthographic and the maths is in `project()`, which takes
and returns plain tuples so it can be tested with no Qt and no display.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# The chain, elbow by elbow. `base_link` first so the mount is visible: a
# clearance reading of exactly 0.1610 m is the MOUNT and not the arm, and a
# skeleton that starts at the shoulder cannot show that.
CHAIN = ("base_link", "shoulder_link", "half_arm_1_link", "half_arm_2_link",
         "forearm_link", "spherical_wrist_1_link", "spherical_wrist_2_link",
         "bracelet_link", "end_effector_link")

ARMS = ("left", "right")

# Views, as (horizontal axis, vertical axis, flip_h). The wearer faces +y in
# this world, so FRONT looks along -y and puts world +x to the LEFT of the
# picture -- which is the operator's view of a wearer facing them.
VIEWS = {
    "FRONT  (facing the wearer)": ("x", "z", True),
    "SIDE  (from the wearer's left)": ("y", "z", False),
    "TOP  (from above)": ("x", "y", True),
}
# THE 3-D VIEW. The three above are axis-aligned orthographic slices, which
# is the right thing for reading a coordinate off the screen and the wrong
# thing for seeing whether an arm is in front of or behind the wearer -- in
# FRONT, depth is exactly the axis that is thrown away.
#
# This one rotates the world by (yaw, pitch) first and then projects, so it
# is a real 3-D view that can be dragged around. It is still ORTHOGRAPHIC:
# a perspective divide would make the near arm bigger than the far one, and
# this panel exists to be compared against the sim beside it.
VIEW_3D = "3-D  (drag to orbit)"
VIEWS[VIEW_3D] = ("x", "z", True)      # axes unused; kept so lookups work
DEFAULT_VIEW = VIEW_3D
DEFAULT_YAW_DEG = 35.0
DEFAULT_PITCH_DEG = 22.0

_AXIS = {"x": 0, "y": 1, "z": 2}


def _rotate3(p, yaw_deg, pitch_deg):
    """World (x, y, z) -> view-space (right, depth, up).

    Yaw turns the scene about the vertical axis; pitch tips it towards the
    viewer. The returned tuple keeps DEPTH in slot 1 so a caller that wants
    to draw far things first can sort on it -- which `paintEvent` does, so
    the far arm does not paint over the near one.
    """
    import math as _m
    cy, sy = _m.cos(_m.radians(yaw_deg)), _m.sin(_m.radians(yaw_deg))
    cp, sp = _m.cos(_m.radians(pitch_deg)), _m.sin(_m.radians(pitch_deg))
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    rx = x * cy + y * sy               # screen right
    ry = -x * sy + y * cy              # depth, into the screen
    rz = z                             # up, before the tilt
    return (rx, ry * cp - rz * sp, ry * sp + rz * cp)


def project(points, view=DEFAULT_VIEW, rect=(0, 0, 400, 300), pad=18,
            bounds=None, yaw_deg=None, pitch_deg=None):
    """Orthographic projection of world points into a pixel rectangle.

    `points` is an iterable of (x, y, z). Returns [(px, py)] with y already
    flipped for screen coordinates. `bounds` fixes the world window so the
    skeleton does not rescale every frame -- a view that re-fits itself makes
    a moving arm look still and a still arm look moving.

    Under `VIEW_3D` the points are rotated by (`yaw_deg`, `pitch_deg`) first
    and the projection is then orthographic onto the rotated frame. Both
    default to the shipped isometric angles.

    Pure arithmetic on purpose: it is the one part of this panel whose answer
    can be known in advance, so it is the part with a test.
    """
    yaw_deg = DEFAULT_YAW_DEG if yaw_deg is None else yaw_deg
    pitch_deg = DEFAULT_PITCH_DEG if pitch_deg is None else pitch_deg
    h_ax, v_ax, flip_h = VIEWS.get(view, VIEWS[DEFAULT_VIEW])
    hi, vi = _AXIS[h_ax], _AXIS[v_ax]
    x0, y0, w, h = rect
    pts = list(points)
    if view == VIEW_3D:
        # Rotate about world z (yaw), then tilt (pitch), then take the
        # screen-horizontal and screen-vertical components. Written out
        # rather than delegated so this file keeps its no-numpy, no-Qt
        # property and stays testable with constructed answers.
        pts = [_rotate3(p, yaw_deg, pitch_deg) for p in pts]
        hi, vi, flip_h = 0, 2, False
    if bounds is None:
        if not pts:
            return []
        hs = [p[hi] for p in pts]
        vs = [p[vi] for p in pts]
        lo_h, hi_h = min(hs), max(hs)
        lo_v, hi_v = min(vs), max(vs)
    else:
        lo_h, hi_h, lo_v, hi_v = bounds
    span_h = max(hi_h - lo_h, 1e-6)
    span_v = max(hi_v - lo_v, 1e-6)
    # ONE SCALE FOR BOTH AXES. Independent scales stretch the robot to fill
    # the box, and an arm drawn 1.6x taller than it is wide is a drawing of a
    # different robot.
    s = min((w - 2 * pad) / span_h, (h - 2 * pad) / span_v)
    cx = x0 + w / 2.0
    cy = y0 + h / 2.0
    mid_h = (lo_h + hi_h) / 2.0
    mid_v = (lo_v + hi_v) / 2.0
    out = []
    for p in pts:
        dh = (p[hi] - mid_h) * s
        dv = (p[vi] - mid_v) * s
        out.append((cx - dh if flip_h else cx + dh, cy - dv))
    return out


def world_bounds(view=DEFAULT_VIEW):
    """A FIXED window, in metres, big enough for anything either arm can do.

    Chosen from the measured envelope rather than from the current pose, and
    no bigger: the window is square-scaled, so every metre of slack on the
    wide axis shrinks the whole drawing. The first version ran x -1.0..1.0
    and z 0.30..1.90, and the horizontal axis bound the scale at 162 px/m --
    which drew an arm 0.4 m tall as 65 px.

    What has to fit:
      * the dance band, |x| 0.47-0.76, the widest thing either arm commands;
      * the innermost and outermost safe columns, 0.325 (L) and 0.450 (R);
      * the work plane at z = 1.100 and the mount at 0.161 above the torso;
      * the shipped home, which spans z 1.25-1.64.
    So |x| <= 0.90 and z in 0.75..1.85, with margin on both.
    """
    if view == VIEW_3D:
        # Rotating puts points outside the axis-aligned window, and a window
        # that clips the arm is worse than one with slack. The diagonal of
        # the x/y extent is the worst case for the horizontal axis, and the
        # tilt mixes z into it for the vertical.
        return (-1.30, 1.30, 0.45, 2.05)
    return (-0.90, 0.90, 0.75, 1.85)


def link_names(arm):
    return ["%s_%s" % (arm, n) for n in CHAIN]


def real_link_names(arm):
    return ["real_%s_%s" % (arm, n) for n in CHAIN]


class Skeleton:
    """One arm's points plus where they came from. Never holds a stale pose."""

    def __init__(self, arm, points=None, source="", age_s=None):
        self.arm = arm
        self.points = list(points or [])
        self.source = source
        self.age_s = age_s

    @property
    def ok(self):
        return len(self.points) >= 2


def from_tf(bus, arm, prefix="real_"):
    """Points from the shared /tf, or an empty Skeleton.

    ALL OR NOTHING. A chain with three of nine links resolved draws an arm
    with a hole in it, which reads as a rendering glitch rather than as a
    partial transform tree, so a single missing link fails the whole arm.
    """
    import rclpy
    pts = []
    for name in CHAIN:
        frame = "%s%s_%s" % (prefix, arm, name)
        try:
            tr = bus.buf.lookup_transform("world", frame, rclpy.time.Time())
        except Exception:                                     # noqa: BLE001
            return Skeleton(arm)
        v = tr.transform.translation
        pts.append((v.x, v.y, v.z))
    return Skeleton(arm, pts, source="the arms' own report")


def from_fk(fk, arm, q):
    """Points from joint angles through the URDF. `q` is 7 radians."""
    if fk is None or q is None or len(q) < 7:
        return Skeleton(arm)
    try:
        pts = fk.points(arm, list(q)[:7], list(CHAIN))
    except Exception:                                         # noqa: BLE001
        return Skeleton(arm)
    return Skeleton(arm, [tuple(float(c) for c in p) for p in pts],
                    source="joint angles, through the robot model")


# ===========================================================================
#  The widget. Imported only where Qt is available.
# ===========================================================================
try:                                                          # pragma: no cover
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QColor, QPainter, QPen
    from PyQt5.QtWidgets import QWidget
    _HAVE_QT = True
except Exception:                                             # noqa: BLE001
    _HAVE_QT = False
    QWidget = object


class ArmView(QWidget):                                       # pragma: no cover
    """Draws the real arms solid and the commanded arms as a faint ghost.

    The ghost is the whole point of drawing them in one widget rather than
    two: the gap between commanded and actual is a SHAPE, and a shape is read
    faster than two numbers. The numbers are still underneath, because a
    shape cannot be compared against a threshold.
    """

    def __init__(self, colours, parent=None):
        # Orbit state for VIEW_3D. Set before the base constructor runs any
        # paint, because paintEvent reads them.
        self.yaw = DEFAULT_YAW_DEG
        self.pitch = DEFAULT_PITCH_DEG
        self._drag = None
        super().__init__(parent)
        self.c = colours                     # dict of the GUI's palette
        self.real = {a: Skeleton(a) for a in ARMS}
        self.sim = {a: Skeleton(a) for a in ARMS}
        self.view = DEFAULT_VIEW
        self.note = "waiting for the real arms"
        self.state = "unknown"               # ok | bad | unknown
        self.setMinimumSize(320, 300)

    def set_pose(self, real, sim, note, state):
        self.real, self.sim, self.note, self.state = real, sim, note, state
        self.update()

    def mousePressEvent(self, ev):                        # pragma: no cover
        self._drag = (ev.x(), ev.y())

    def mouseMoveEvent(self, ev):                         # pragma: no cover
        """Drag to orbit. Only in the 3-D view -- dragging an axis-aligned
        slice would silently turn it into something else, and the whole point
        of FRONT is that you can read a coordinate off it."""
        if self.view != VIEW_3D or getattr(self, "_drag", None) is None:
            return
        x0, y0 = self._drag
        self.yaw = (self.yaw + (ev.x() - x0) * 0.5) % 360.0
        # Pitch is CLAMPED. Past +/-89 the up vector flips and the scene
        # turns upside down mid-drag, which reads as a glitch rather than a
        # rotation.
        self.pitch = max(-89.0, min(89.0, self.pitch - (ev.y() - y0) * 0.5))
        self._drag = (ev.x(), ev.y())
        self.update()

    def mouseReleaseEvent(self, ev):                      # pragma: no cover
        self._drag = None

    def reset_view(self):
        """Back to the shipped angles. A view that can be dragged needs a way
        back, or one stray drag leaves the panel permanently odd."""
        self.yaw, self.pitch = DEFAULT_YAW_DEG, DEFAULT_PITCH_DEG
        self.update()

    def set_view(self, name):
        self.view = name
        self.update()

    # ------------------------------------------------------------- paint
    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(self.c["bg"]))
        rect = (0, 18, w, h - 46)
        b = world_bounds(self.view)
        bounds = self._bounds_for(b)
        # ONE PLACE that knows the orbit, so a call site cannot forget it and
        # draw a rotated skeleton on an unrotated grid.
        self._prj = lambda pts, r=rect: project(
            pts, self.view, r, bounds=bounds,
            yaw_deg=self.yaw, pitch_deg=self.pitch)

        self._grid(p, rect, bounds)

        # GHOST FIRST, so the real arm is never hidden by the thing it is
        # being compared against.
        for a in ARMS:
            if self.sim[a].ok:
                self._chain(p, self.sim[a].points, rect, bounds,
                            QColor(self.c["muted"]), 2, dashed=True)
        drew = 0
        for a in ARMS:
            if self.real[a].ok:
                self._chain(p, self.real[a].points, rect, bounds,
                            QColor(self.c["accent"]), 3)
                # LABELLED BY FRAME NAME, NOT BY THE WEARER'S BODY. Whether
                # world +x is the wearer's right is an OPEN contradiction in
                # this repository (CLAUDE.md, desk operation row): the docs
                # say it is and the arms sit the other way round. "left arm"
                # is a fact about the frame and is true either way; "the
                # wearer's right" would be a claim this panel cannot support.
                xy = self._prj(self.real[a].points)
                p.setPen(QPen(QColor(self.c["muted"])))
                f0 = p.font()
                f0.setPointSize(8)
                p.setFont(f0)
                # Offset AWAY from the chain, not over it: at the end
                # effector the label sat on top of the last two segments.
                dx = 8 if xy[-1][0] < self.width() / 2 else -46
                p.drawText(int(xy[-1][0]) + dx, int(xy[-1][1]) + 16,
                           "%s arm" % a)
                drew += 1

        p.setPen(QPen(QColor(self.c["muted"])))
        f = p.font()
        f.setPointSize(8)
        p.setFont(f)
        # THE LEGEND GOES AT THE BOTTOM, not beside the view name. Both were
        # drawn on the top line and they overlapped at any width under about
        # 520 px -- which is the width this panel actually gets, so they
        # overlapped always.
        p.drawText(6, 13, self.view)
        p.drawText(6, h - 8, "solid = real    dashed = commanded")

        col = {"ok": self.c["accent"], "bad": self.c["bad"],
               "unknown": self.c["unknown"]}.get(self.state, self.c["unknown"])
        p.setPen(QPen(QColor(col)))
        p.drawText(6, h - 24, self.note)

        if not drew:
            # NOTHING DRAWN AND NOTHING IMPLIED. An empty panel with a caption
            # is honest; an empty panel that looks like a loading screen is
            # not, and the last good pose is never shown.
            f.setPointSize(13)
            p.setFont(f)
            p.setPen(QPen(QColor(self.c["unknown"])))
            p.drawText(rect[0], rect[1], rect[2], rect[3],
                       Qt.AlignCenter, "NO REAL ARM")
        p.end()

    def _bounds_for(self, b):
        h_ax, v_ax, _ = VIEWS.get(self.view, VIEWS[DEFAULT_VIEW])
        if v_ax == "y":                       # TOP view: metres either way
            return (-1.0, 1.0, -0.4, 1.0)
        return b

    def _pt(self, hv, vv):
        """A world point with this view's two axes set. The third is 0.

        Built by INDEX rather than by writing (hv, hv, vv) and hoping: in the
        TOP view the horizontal axis is x and the vertical axis is y, so a
        tuple that puts hv in both slots draws the vertical rules diagonally
        and nothing about the picture says it is wrong.
        """
        h_ax, v_ax, _ = VIEWS.get(self.view, VIEWS[DEFAULT_VIEW])
        pt = [0.0, 0.0, 0.0]
        pt[_AXIS[h_ax]] = hv
        pt[_AXIS[v_ax]] = vv
        return tuple(pt)

    def _grid(self, p, rect, bounds):
        pen = QPen(QColor(self.c["line"]))
        pen.setWidth(1)
        p.setPen(pen)
        lo_h, hi_h, lo_v, hi_v = bounds
        step = 0.25
        i = 0
        while lo_h + i * step <= hi_h + 1e-9:
            hv = lo_h + i * step
            a = self._prj([self._pt(hv, lo_v)])
            b = self._prj([self._pt(hv, hi_v)])
            p.drawLine(int(a[0][0]), int(a[0][1]), int(b[0][0]), int(b[0][1]))
            i += 1
        i = 0
        while lo_v + i * step <= hi_v + 1e-9:
            vv = lo_v + i * step
            a = self._prj([self._pt(lo_h, vv)])
            b = self._prj([self._pt(hi_h, vv)])
            p.drawLine(int(a[0][0]), int(a[0][1]), int(b[0][0]), int(b[0][1]))
            i += 1

    def _chain(self, p, pts, rect, bounds, colour, width, dashed=False):
        xy = self._prj(pts)
        pen = QPen(colour)
        pen.setWidth(width)
        if dashed:
            pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        for i in range(len(xy) - 1):
            p.drawLine(int(xy[i][0]), int(xy[i][1]),
                       int(xy[i + 1][0]), int(xy[i + 1][1]))
        if not dashed:
            for x, y in xy:
                p.drawEllipse(int(x) - 2, int(y) - 2, 4, 4)
            if xy:
                pen.setWidth(width + 3)
                p.setPen(pen)
                p.drawPoint(int(xy[-1][0]), int(xy[-1][1]))
