#!/usr/bin/env python3
"""Render a fingerprint sweep as RViz markers.

WHY THIS IS A SEPARATE MODULE. The node's job is to decide what changed; this
module's job is to draw it. Keeping them apart means the drawing can be tested
against known verdicts with no ROS graph, no detector and no camera -- which
is the only way the colour/lifetime logic gets checked at all, because looking
at RViz proves a picture exists, not that it is the right picture.

COLOUR IS A SCARCE ALARM CHANNEL, and the console redesign already paid for
this lesson: if every object is saturated, none of them are. So SAME -- the
normal case, usually most of the scene -- is deliberately DESATURATED, and the
verdicts that need the operator's attention carry the colour.

DROPPED OBJECTS ARE SHOWN BEFORE THEY GO. A VANISHED object removed instantly
is indistinguishable from one that was never there: the operator sees a scene
with one fewer object and no reason. It is drawn at its STORED pose (the only
pose it has -- there is no observation, that is what vanished means), hollow
and red, for `drop_hold_s`, and then it stops being published.

EVERY FRAME BEGINS WITH DELETEALL. Ids are assigned per frame, and the topic
is latched in practice, so without it the previous sweep's markers stay on
screen underneath the current one -- two scenes in one picture. That bug has
already been paid for once in this project.
"""
from visualization_msgs.msg import Marker, MarkerArray

SAME = "same"
MOVED = "moved"
MOVED_OR_SWAPPED = "moved_or_swapped"
APPEARED = "appeared"
VANISHED = "vanished"
RECLASSIFIED = "reclassified"

# r, g, b, a
COLOUR = {
    SAME:             (0.55, 0.60, 0.55, 0.45),   # desaturated: nothing to do
    MOVED:            (0.95, 0.65, 0.10, 0.85),   # amber
    MOVED_OR_SWAPPED: (0.95, 0.40, 0.05, 0.90),   # orange, association in doubt
    APPEARED:         (0.20, 0.65, 0.95, 0.85),   # blue: new to the scene
    RECLASSIFIED:     (0.80, 0.30, 0.85, 0.90),   # magenta: identity changed
    VANISHED:         (0.90, 0.15, 0.15, 0.55),   # red, hollow, transient
}
DEFAULT_SIZE = (0.05, 0.05, 0.05)


def _stamp(m, frame, ns, mid, stamp=None):
    m.header.frame_id = frame
    if stamp is not None:
        m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.action = Marker.ADD


def _label(v):
    """One line the operator can read at a glance: what and how sure."""
    parts = [v.get("new_label") or v["label"], v["verdict"].upper()]
    c = v.get("confidence")
    if c is not None:
        parts.append("conf %.2f" % c)
    d = v.get("delta_m")
    if d is not None:
        parts.append("%+.0f mm" % (1000 * d))
    r = v.get("rot_deg")
    if r is not None:
        parts.append("%.0f deg" % r)
    return "  ".join(parts)


def build(verdicts, frame="world", stamp=None, drop_ages=None,
          drop_hold_s=3.0, text_height=0.035):
    """MarkerArray for one sweep result.

    `drop_ages` maps a vanished label to how long it has been gone, so a
    dropped object can be held briefly and then allowed to disappear. A label
    absent from the map is treated as freshly dropped.
    """
    drop_ages = drop_ages or {}
    arr = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    clear.header.frame_id = frame
    arr.markers.append(clear)

    mid = 0
    for v in verdicts:
        verdict = v["verdict"]
        if verdict == VANISHED:
            if drop_ages.get(v["label"], 0.0) > drop_hold_s:
                continue                    # held long enough; let it go
            pos = v.get("stored")
        else:
            pos = v.get("observed") or v.get("stored")
        if pos is None:
            continue
        r, g, b, a = COLOUR.get(verdict, (0.6, 0.6, 0.6, 0.5))
        size = v.get("size") or DEFAULT_SIZE

        box = Marker()
        _stamp(box, frame, "scene_objects", mid, stamp)
        mid += 1
        box.type = Marker.CUBE
        box.pose.position.x, box.pose.position.y, box.pose.position.z = \
            (float(c) for c in pos[:3])
        q = v.get("quat") or (0.0, 0.0, 0.0, 1.0)
        box.pose.orientation.x, box.pose.orientation.y, \
            box.pose.orientation.z, box.pose.orientation.w = \
            (float(c) for c in q)
        box.scale.x, box.scale.y, box.scale.z = (float(c) for c in size)
        box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, a
        arr.markers.append(box)

        txt = Marker()
        _stamp(txt, frame, "scene_labels", mid, stamp)
        mid += 1
        txt.type = Marker.TEXT_VIEW_FACING
        txt.pose.position.x = float(pos[0])
        txt.pose.position.y = float(pos[1])
        txt.pose.position.z = float(pos[2]) + float(size[2]) / 2.0 + 0.03
        txt.pose.orientation.w = 1.0
        txt.scale.z = float(text_height)
        # Text is always fully opaque. A desaturated BOX still reads as an
        # object; desaturated TEXT is simply unreadable, and the label is the
        # part that carries the information.
        txt.color.r, txt.color.g, txt.color.b, txt.color.a = r, g, b, 1.0
        txt.text = _label(v)
        arr.markers.append(txt)

        if verdict in (MOVED, MOVED_OR_SWAPPED) and v.get("stored"):
            # A displacement is only meaningful against where it WAS, so draw
            # the move as an arrow rather than making the operator remember.
            ar = Marker()
            _stamp(ar, frame, "scene_moves", mid, stamp)
            mid += 1
            ar.type = Marker.ARROW
            from geometry_msgs.msg import Point
            s, o = v["stored"], v["observed"]
            ar.points = [Point(x=float(s[0]), y=float(s[1]), z=float(s[2])),
                         Point(x=float(o[0]), y=float(o[1]), z=float(o[2]))]
            ar.scale.x, ar.scale.y, ar.scale.z = 0.006, 0.014, 0.02
            ar.color.r, ar.color.g, ar.color.b, ar.color.a = r, g, b, 0.9
            arr.markers.append(ar)
    return arr
