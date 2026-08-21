"""The homing clearance check must sweep the whole body and the whole arm.

TWO DEFECTS, both found on 2026-08-20 with a real arm powered and a person
near the rig, and both of the same shape: a check that reports a real number
computed over the wrong set, so it reads as protection and is not.

  1. `for part in ("torso", "head", "hips")` -- THREE of TWELVE. The wearer
     model holds torso, head, neck, hips, both thighs, and the wearer's own
     upper arms, forearms and hands. CLAUDE.md records that the wearer's ARMS
     are what bind the right arm inboard: precisely the geometry this loop
     could not see.

  2. `DISTAL_LINKS` starts at the forearm. The shoulder and both half-arm
     tubes -- the segments a SHOULDER-MOUNTED arm sweeps across a person's
     head and chest when it rotates about its base -- were never measured.
     The hand can be a metre clear while the upper tube is inside somebody's
     neck, and the check would report the hand's distance and call it
     clearance.

This is hard constraint 11's SRDF trap one layer down, in the node whose only
job is that check. `avoid_collisions` is not the wearer check; neither was
this.

base_link is deliberately excluded and that is asserted too: it is rigid to
the mount at 0.1610 m from the torso and no joint moves it, so including it
would peg every reading at 0.1610 -- a constant wearing the clothes of a
measurement.
"""
import pathlib

import numpy as np

import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from srl_teleop import wearer_posture                      # noqa: E402
from srl_teleop.clearance import (                         # noqa: E402
    DISTAL_LINKS, PROXIMAL_LINKS, WEARER_CHECK_LINKS, ClearanceModel,
    point_clearance)

SRC = (pathlib.Path(__file__).resolve().parents[1]
       / 'srl_teleop' / 'real_homing_node.py').read_text()


def test_the_check_sweeps_every_part_the_model_has():
    """The literal defect: three names hard-coded against a twelve-part model."""
    assert 'for part in ("torso", "head", "hips")' not in SRC, (
        'the homing clearance check is back to three of twelve body parts')
    assert 'for part in self.clearance.PARTS.keys():' in SRC, (
        'the check must iterate the model, so a part added to the model is '
        'protected without editing this node')


def test_the_model_really_does_hold_more_than_three_parts():
    """Guards the test above from passing vacuously.

    If the model ever collapsed to three parts, the assertion above would be
    satisfied while protecting nothing new.
    """
    parts = [p[0] for p in wearer_posture.wearer_model()]
    assert len(parts) >= 10, parts
    for needed in ('torso', 'head', 'hips', 'neck'):
        assert needed in parts, parts
    arms = [p for p in parts if 'arm' in p or 'hand' in p]
    assert len(arms) >= 6, (
        'the wearer\'s own arms are what bind the right arm inboard; they '
        'must be in the model: %r' % parts)


def test_the_proximal_half_of_the_arm_is_measured():
    for link in ('shoulder_link', 'half_arm_1_link', 'half_arm_2_link'):
        assert link in PROXIMAL_LINKS, link
        assert link in WEARER_CHECK_LINKS, link
    for link in DISTAL_LINKS:
        assert link in WEARER_CHECK_LINKS, link
    assert 'WEARER_CHECK_LINKS' in SRC and 'DISTAL_LINKS' not in SRC, (
        'the homing node must sweep WEARER_CHECK_LINKS, not the distal half')


def test_base_link_is_excluded_and_that_is_deliberate():
    assert 'base_link' not in WEARER_CHECK_LINKS, (
        'base_link is rigid to the mount at 0.1610 m and would peg every '
        'reading, masking the arm entirely')


def test_a_part_that_cannot_be_resolved_is_named_not_skipped():
    """Silence about a hole in the floor is the thing that hurts somebody."""
    assert 'WEARER CLEARANCE IS PARTIAL' in SRC
    assert 'missing.append(part)' in SRC


def test_the_check_can_fail_on_a_body_the_old_one_could_not_see():
    """KNOWN ANSWER, and the whole point of the fix.

    A point placed inside the wearer's own upper arm is a breach. Scored
    against the three parts the old loop swept it is comfortably clear;
    scored against the model it is negative. A check that cannot fail on a
    deliberately broken input is not a check.
    """
    model = ClearanceModel()
    parts = dict(model.PARTS)
    # ClearanceModel names its primitives `human_left_upper_arm`; the
    # posture module names the same limb `L-upperarm`. Two vocabularies for
    # one body -- match on the part of the name they agree about.
    upper = [k for k in parts if 'upper_arm' in k or 'upperarm' in k]
    assert upper, list(parts)
    limb = upper[0]

    # The origin of a part's own frame is inside that part by construction.
    inside = np.array([0.0, 0.0, 0.0])
    breach, _ = model.clearance({limb: [inside]}, 0.0)
    assert breach < 0.0, (
        'a point at the origin of the upper-arm frame must read as a breach, '
        'got %.4f' % breach)

    # WHAT THE OLD LOOP ACTUALLY DID. Points are supplied per part BY THE
    # CALLER, after its own TF lookup -- so a part the loop never named
    # contributed no points at all. The breach was not scored generously; it
    # was not scored. That is why it could read `clear inf m` with the arm
    # inside somebody.
    old_view, who = model.clearance({}, 0.0)
    assert who is None
    assert old_view > breach, (
        'this test proves nothing unless the old sweep is blind to the same '
        'breach the new one reports')
    assert old_view == float('inf'), (
        'a body part nobody looked up reports infinite clearance -- exactly '
        'the `clear inf m` printed on every homing line')


def test_point_clearance_is_signed_so_a_breach_is_visible():
    """If penetration returned 0.0 rather than a negative, every breach would
    tie with 'just touching' and the worst case would stop being findable."""
    model = ClearanceModel()
    prims = model.PARTS['torso']
    deep = point_clearance(np.array([0.0, 0.0, 0.0]), prims, 0.0)
    assert deep < 0.0, deep
