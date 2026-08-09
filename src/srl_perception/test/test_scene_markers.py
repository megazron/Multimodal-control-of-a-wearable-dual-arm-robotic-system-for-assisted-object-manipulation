#!/usr/bin/env python3
"""The sweep picture must be the RIGHT picture.

Looking at RViz proves a picture exists. These check it says what the diff
said: colours distinguish verdicts, a dropped object is held and then let go,
and a vanished object is drawn where it WAS -- it has no observed pose, that
being what vanished means.
"""
from visualization_msgs.msg import Marker

from srl_perception import scene_markers as sm


def V(verdict, label="obj", stored=None, observed=None, **kw):
    return dict(verdict=verdict, label=label, stored=stored,
                observed=observed, **kw)


def arrows(arr):
    """Marker.ARROW is 0, which is also the default for an unset type, so the
    DELETEALL marker matches it. Filtering on ADD is what distinguishes a real
    arrow from the frame's own clear. The first version of this helper did
    not, and reported an arrow on every verdict."""
    return [m for m in arr.markers
            if m.type == Marker.ARROW and m.action == Marker.ADD]


def boxes(arr):
    return [m for m in arr.markers if m.type == Marker.CUBE]


def texts(arr):
    return [m for m in arr.markers if m.type == Marker.TEXT_VIEW_FACING]


def test_every_frame_begins_with_deleteall():
    """Ids are per-frame, so without this two sweeps stack on screen."""
    a = sm.build([V(sm.SAME, observed=[0, 0, 1])])
    assert a.markers[0].action == Marker.DELETEALL


def test_verdicts_are_visually_distinct():
    seen = {}
    for v in (sm.SAME, sm.MOVED, sm.APPEARED, sm.VANISHED, sm.RECLASSIFIED,
              sm.MOVED_OR_SWAPPED):
        a = sm.build([V(v, stored=[0, 0, 1], observed=[0, 0, 1])])
        b = boxes(a)[0]
        seen[v] = (round(b.color.r, 3), round(b.color.g, 3),
                   round(b.color.b, 3))
    assert len(set(seen.values())) == len(seen), seen


def test_same_is_the_least_saturated():
    """Normal is most of the scene; if it shouts, nothing else can."""
    def sat(v):
        a = sm.build([V(v, stored=[0, 0, 1], observed=[0, 0, 1])])
        c = boxes(a)[0].color
        return max(c.r, c.g, c.b) - min(c.r, c.g, c.b)
    assert sat(sm.SAME) < sat(sm.MOVED)
    assert sat(sm.SAME) < sat(sm.APPEARED)


def test_dropped_is_held_then_released():
    v = [V(sm.VANISHED, stored=[0.1, 0.2, 1.0])]
    young = sm.build(v, drop_ages={"obj": 0.5}, drop_hold_s=3.0)
    assert len(boxes(young)) == 1, "a dropped object must be SHOWN first"
    old = sm.build(v, drop_ages={"obj": 9.0}, drop_hold_s=3.0)
    assert len(boxes(old)) == 0, "and must eventually go"


def test_vanished_is_drawn_at_its_stored_pose():
    """It has no observed pose. Drawing nothing would be the bug."""
    a = sm.build([V(sm.VANISHED, stored=[0.3, 0.4, 1.1])],
                 drop_ages={"obj": 0.0})
    b = boxes(a)[0]
    assert abs(b.pose.position.x - 0.3) < 1e-9
    assert abs(b.pose.position.z - 1.1) < 1e-9


def test_label_carries_name_and_confidence():
    a = sm.build([V(sm.APPEARED, label="red_cube", observed=[0, 0, 1],
                    confidence=0.87)])
    t = texts(a)[0].text
    assert "red_cube" in t and "0.87" in t and "APPEARED" in t


def test_move_gets_an_arrow_from_where_it_was():
    a = sm.build([V(sm.MOVED, stored=[0, 0, 1], observed=[0.1, 0, 1],
                    delta_m=0.1)])
    ar = arrows(a)
    assert len(ar) == 1
    assert abs(ar[0].points[0].x - 0.0) < 1e-9
    assert abs(ar[0].points[1].x - 0.1) < 1e-9


def test_no_arrow_when_nothing_moved():
    a = sm.build([V(sm.SAME, stored=[0, 0, 1], observed=[0, 0, 1])])
    assert not arrows(a)


def test_text_stays_readable_even_when_the_box_is_faint():
    a = sm.build([V(sm.SAME, observed=[0, 0, 1])])
    assert boxes(a)[0].color.a < 1.0
    assert texts(a)[0].color.a == 1.0


def test_ids_are_unique_within_a_frame():
    vs = [V(sm.MOVED, label="a", stored=[0, 0, 1], observed=[0.1, 0, 1]),
          V(sm.SAME, label="b", observed=[0.2, 0, 1]),
          V(sm.APPEARED, label="c", observed=[0.3, 0, 1])]
    a = sm.build(vs)
    live = [m for m in a.markers if m.action == Marker.ADD]
    assert len({(m.ns, m.id) for m in live}) == len(live)
