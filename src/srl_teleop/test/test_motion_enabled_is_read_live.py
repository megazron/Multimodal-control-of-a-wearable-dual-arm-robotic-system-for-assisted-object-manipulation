"""`motion_enabled` is honoured LIVE, and for three sessions it was not.

WHAT WAS WRONG. `ik_follower_node` read the parameter once at construction

    self.motion_enabled = bool(self.get_parameter("motion_enabled").value)

into an instance attribute, and registered no parameter callback. So
`set_parameters` changed the parameter server's copy and the follower carried
on publishing. Every caller that "paused the follower" by lowering it paused
nothing, and the node's own blocker recovery text -- `ros2 param set
motion_enabled true` -- could not work either.

MEASURED ON A LIVE STACK, both controls correct
(`scripts/verify_follower_pause.py`):

    before   ARMED  commanded +60 mm -> the arm moved 0.0600 m
             PAUSED commanded -60 mm -> the arm moved 0.0600 m
    after    ARMED  commanded +60 mm -> the arm moved 0.0600 m
             PAUSED commanded -60 mm -> the arm moved 0.0000 m

WHAT IT COST. `stage_presentation_pose.py` has documented this pause since
2026-08-15 as its answer to "the follower wins", and T1's look was given the
same treatment. Neither worked, so the arm settled at the edge of
`Vision.stage()`'s 0.02 rad tolerance in a tug of war with the follower and
never stopped moving -- which is why T1's cubes deprojected 31.7-35.7 mm off
inside the recording sweep and 1.3-3.0 mm standalone on a fresh stack, where
the follower has no target yet and there is nothing to fight.

WHAT IS CHECKED HERE, without a ROS graph:

  * the callback is registered, and registered AFTER `motion_enabled` is
    declared -- registering it first would fire it on the declaration itself;
  * it changes `self.motion_enabled`, which is the attribute the gate in
    `on_pose` actually reads. A callback that validated the value and did not
    store it would pass a "does the callback exist" test and change nothing;
  * every OTHER parameter is reported as ignored rather than accepted and
    silently dropped. `max_vel` is clamped by real_robot mode one line after
    it is read, and re-applying the raw value would discard that clamp -- a
    bug this file has already had once, and the reason the callback is
    deliberately narrow;
  * HARD CONSTRAINT 8 still holds: the parameter still DEFAULTS to false in
    real_robot mode, so honouring a live set is the by-hand arming path being
    made real, not an automatic re-arm.
"""
import ast
import os

SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "srl_teleop", "ik_follower_node.py")


def _src():
    return open(SRC).read()


def test_a_parameter_callback_is_registered():
    s = _src()
    assert "self.add_on_set_parameters_callback(" in s, (
        "without a parameter callback, motion_enabled is a snapshot and every "
        "pause in this repository is decorative")


def test_it_is_registered_AFTER_motion_enabled_is_declared():
    """Registering first would fire the callback on the declaration itself,
    which is a different code path and a needless one."""
    s = _src()
    assert (s.index('self.declare_parameter("motion_enabled"')
            < s.index("self.add_on_set_parameters_callback("))


def test_the_callback_actually_assigns_the_attribute_the_gate_reads():
    """The gate is `if not self.motion_enabled`. A callback that validated the
    request and returned successful without storing it would look correct and
    do nothing."""
    tree = ast.parse(_src())
    fn = next((n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef)
               and n.name == "_on_set_parameters"), None)
    assert fn is not None, "the callback must be a named method, not a lambda"
    assigns = [n for n in ast.walk(fn)
               if isinstance(n, ast.Assign)
               for t in n.targets
               if isinstance(t, ast.Attribute) and t.attr == "motion_enabled"]
    assert assigns, "the callback never assigns self.motion_enabled"


def test_other_parameters_are_reported_as_ignored_not_silently_dropped():
    s = _src()
    fn_at = s.index("def _on_set_parameters")
    body = s[fn_at:s.index("\n    # ---------------- startup unwind", fn_at)]
    assert 'p.name != "motion_enabled"' in body
    assert "NO EFFECT on this node" in body, (
        "a parameter this node cannot apply must say so; accepting it and "
        "dropping it is the silent-substitution failure this repository keeps "
        "finding")


def test_the_gate_still_reads_the_attribute():
    """If the gate were ever changed to read the parameter directly, the
    callback would be dead code and this test would be about nothing."""
    s = _src()
    assert "if not self.motion_enabled:" in s


def test_real_robot_still_defaults_to_disarmed():
    """HARD CONSTRAINT 8. Honouring a live set is the by-hand arming path
    working; it must not become an automatic one."""
    s = _src()
    assert 'self.declare_parameter("motion_enabled", not self.real_robot)' in s


def test_every_transition_is_logged():
    """An arm disarmed by a script that then failed to restore is the failure
    mode with the longest tail. It must be visible in the node's own log."""
    s = _src()
    fn_at = s.index("def _on_set_parameters")
    body = s[fn_at:s.index("\n    # ---------------- startup unwind", fn_at)]
    assert "[MOTION]" in body and "get_logger().warn" in body
