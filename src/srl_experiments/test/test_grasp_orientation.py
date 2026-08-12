"""A grasp must match the object's ORIENTATION, not only its position."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), "scripts"))

import clip_scene as CS  # noqa: E402


def test_square_object_square_gripper_is_zero_error():
    assert CS.yaw_error_deg((1, 0, 0), 0.0) == 0.0


def test_error_grows_with_the_object_angle():
    assert CS.yaw_error_deg((1, 0, 0), 15.0) == 15.0
    assert CS.yaw_error_deg((1, 0, 0), 30.0) == 30.0


def test_a_cube_is_symmetric_under_ninety_degrees():
    """A gripper at 90 deg to a square cube grasps it perfectly well."""
    assert CS.yaw_error_deg((0, 1, 0), 0.0) == 0.0
    assert CS.yaw_error_deg((1, 0, 0), 90.0) == 0.0


def test_thirty_and_fortyfive_degrees_EXCEED_the_tolerance():
    """The whole point: these must be refused, not quietly accepted."""
    for ang in (30.0, 45.0):
        assert CS.yaw_error_deg((1, 0, 0), ang) > CS.GRASP_YAW_TOL_DEG


def test_fifteen_degrees_is_within_tolerance():
    assert CS.yaw_error_deg((1, 0, 0), 15.0) <= CS.GRASP_YAW_TOL_DEG


def test_a_missing_axis_is_NOT_a_pass():
    """None means 'not measured'. It must never read as aligned."""
    assert CS.yaw_error_deg(None, 30.0) is None
    assert CS.yaw_error_deg((0, 0, 1), 30.0) is None


def test_the_check_can_fail_on_a_deliberately_broken_input():
    assert CS.yaw_error_deg((1, 0, 0), 44.0) > CS.GRASP_YAW_TOL_DEG
