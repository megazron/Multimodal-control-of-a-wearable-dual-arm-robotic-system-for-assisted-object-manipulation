"""The ACTUAL panel's projection, with the answers known in advance.

WHY THIS IS A TEST AND NOT A SCREENSHOT. The panel that shows where the real
arms are is the one panel in the GUI whose content is arithmetic, so it is
the one panel whose content can be asserted rather than looked at. Three
things are checked, and each has been got wrong in a drawing in this project
before:

  * ONE SCALE FOR BOTH AXES. Fitting each axis independently stretches the
    robot to fill the box, and an arm drawn 1.6x taller than it is wide is a
    picture of a different robot.
  * A FIXED WORLD WINDOW. A view that re-fits itself to the current pose
    makes a moving arm look still and a still arm look moving.
  * THE FRONT VIEW IS THE OPERATOR'S VIEW. Desk operation puts the operator
    across the room FACING the wearer, so world +x has to land on the LEFT of
    the picture. Getting that backwards is a mirror, and a mirror is exactly
    the failure `align_yaw_deg` is written to make impossible.

No Qt, no display, no stack.
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))), "scripts"))

import srl_arm_view as V   # noqa: E402


RECT = (0, 0, 400, 300)


FRONT = "FRONT  (facing the wearer)"


def test_front_view_puts_world_plus_x_on_the_left():
    """NAMES ITS VIEW. This used to rely on the default being FRONT, and the
    default is now the 3-D orbit -- so the test silently changed subject
    when the default moved. A test that does not name what it is testing
    fails for a reason that has nothing to do with the property."""
    right, left = (0.5, 0.0, 1.0), (-0.5, 0.0, 1.0)
    (rx, _), (lx, _) = V.project([right, left], view=FRONT, rect=RECT,
                                 bounds=(-1.0, 1.0, 0.3, 1.9))
    assert rx < lx, "world +x must draw to the LEFT when facing the wearer"


def test_one_scale_for_both_axes():
    """A square in the world stays a square on the screen."""
    b = (-1.0, 1.0, 0.3, 1.9)
    pts = [(-0.2, 0, 1.0), (0.2, 0, 1.0), (0.2, 0, 1.4), (-0.2, 0, 1.4)]
    (ax, ay), (bx, by), (cx, cy), _ = V.project(pts, view=FRONT, rect=RECT,
                                                bounds=b)
    width = abs(bx - ax)
    height = abs(cy - by)
    assert abs(width - height) < 1e-6, (width, height)


# ------------------------------------------------------------- the 3-D view
def test_the_default_view_is_the_3d_one():
    assert V.DEFAULT_VIEW == V.VIEW_3D


def test_yaw_zero_pitch_zero_is_the_front_slice():
    """A known answer. With no rotation the 3-D view must reduce to looking
    down -y: screen-horizontal is world x, screen-vertical is world z."""
    pts = [(0.3, 0.0, 1.2), (-0.3, 0.0, 1.2), (0.3, 0.0, 1.6)]
    got = V.project(pts, view=V.VIEW_3D, rect=RECT,
                    bounds=(-1.0, 1.0, 0.3, 1.9), yaw_deg=0.0, pitch_deg=0.0)
    (ax, ay), (bx, by), (cx, cy) = got
    assert ax > bx, "world +x must be to the RIGHT with no flip in 3-D"
    assert cy < ay, "world +z must be UP on screen"
    assert abs(ax - cx) < 1e-9, "x unchanged means screen-x unchanged"


def test_yaw_actually_rotates_the_scene():
    """Two points differing only in world y must separate horizontally once
    the scene is yawed -- in the FRONT slice they overlap exactly, which is
    the whole reason the 3-D view exists."""
    near, far = (0.0, -0.4, 1.2), (0.0, 0.4, 1.2)
    b = (-1.3, 1.3, 0.45, 2.05)
    flat = V.project([near, far], view=FRONT, rect=RECT, bounds=b)
    assert abs(flat[0][0] - flat[1][0]) < 1e-9, (
        "the FRONT slice is supposed to throw depth away")
    turned = V.project([near, far], view=V.VIEW_3D, rect=RECT, bounds=b,
                       yaw_deg=45.0, pitch_deg=0.0)
    assert abs(turned[0][0] - turned[1][0]) > 10.0, (
        "yawing 45 degrees did not separate two points that differ only in "
        "depth, so the view is not rotating")


def test_depth_is_returned_so_far_things_can_be_drawn_first():
    """`_rotate3` keeps depth in slot 1. Without it the far arm paints over
    the near one and the picture is wrong in the one way a 3-D view is
    supposed to fix."""
    near = V._rotate3((0.0, -0.5, 1.2), 0.0, 0.0)
    far = V._rotate3((0.0, 0.5, 1.2), 0.0, 0.0)
    assert near[1] < far[1]


def test_the_3d_window_is_wide_enough_for_a_rotated_arm():
    """Rotating pushes points outside the axis-aligned window. A window that
    clips the arm is worse than one with slack."""
    lo_h, hi_h, lo_v, hi_v = V.world_bounds(V.VIEW_3D)
    fh, fhi, fv, fvi = V.world_bounds(FRONT)
    assert hi_h >= fhi and lo_h <= fh
    assert hi_v >= fvi and lo_v <= fv


def test_bounds_fix_the_window_so_a_still_arm_stays_still():
    b = (-1.0, 1.0, 0.3, 1.9)
    a = V.project([(0.1, 0, 1.0), (0.2, 0, 1.1)], rect=RECT, bounds=b)
    # The same two points with a third, far-away point added. With a fixed
    # window the first two must not move; with auto-fit they would.
    c = V.project([(0.1, 0, 1.0), (0.2, 0, 1.1), (0.9, 0, 1.8)],
                  rect=RECT, bounds=b)
    assert a == c[:2]


def test_auto_fit_is_available_and_does_rescale():
    """The auto-fitting path still works -- it is simply not what the panel
    uses. Asserted so the fixed-window test above is a CHOICE and not the
    only behaviour the function has."""
    a = V.project([(0.1, 0, 1.0), (0.2, 0, 1.1)], rect=RECT)
    c = V.project([(0.1, 0, 1.0), (0.2, 0, 1.1), (0.9, 0, 1.8)], rect=RECT)
    assert a != c[:2]


def test_side_and_top_views_use_different_axes():
    p = (0.4, 0.7, 1.2)
    front = V.project([p], "FRONT  (facing the wearer)", RECT,
                      bounds=(-1, 1, 0.3, 1.9))[0]
    side = V.project([p], "SIDE  (from the wearer's left)", RECT,
                     bounds=(-1, 1, 0.3, 1.9))[0]
    top = V.project([p], "TOP  (from above)", RECT, bounds=(-1, 1, -0.4, 1.0))[0]
    assert front != side and side != top and front != top


def test_a_skeleton_with_too_few_points_is_not_drawable():
    """The all-or-nothing rule: a chain with a hole in it reads as a
    rendering glitch, not as a partial transform tree."""
    assert not V.Skeleton("left").ok
    assert not V.Skeleton("left", [(0, 0, 0)]).ok
    assert V.Skeleton("left", [(0, 0, 0), (0, 0, 1)]).ok


def test_from_fk_returns_the_whole_chain_and_from_nothing_returns_nothing():
    assert not V.from_fk(None, "left", [0] * 7).ok
    import srl_fk
    fk = srl_fk.FK()
    s = V.from_fk(fk, "left", [0.0] * 7)
    assert len(s.points) == len(V.CHAIN)
    assert s.source
    # The mount is the FIRST point, so a clearance reading that is really the
    # mount is visible as such rather than attributed to the arm.
    assert s.points[0][2] > 0.5


def test_real_frames_carry_the_prefix_and_the_sim_frames_do_not():
    """THERE IS NO /real/tf. Both real launches publish into the shared tree
    under `real_*`, so the prefix is the whole difference."""
    assert V.real_link_names("left")[0] == "real_left_base_link"
    assert V.link_names("left")[0] == "left_base_link"
    assert all(n.startswith("real_") for n in V.real_link_names("right"))
