#!/usr/bin/env python3
"""THE WORK SURFACE HEIGHT. One owner, one value, every consumer reads it here.

    from srl_experiments.work_surface import table_top, set_measured, check

WHY THIS FILE EXISTS. The table height was a literal in the scene definition
and every consumer either imported that literal or carried its own. Nothing
measured it, so a real table 20 mm off the declared height was absorbed
silently into every grasp: no check compared the declared value to anything,
because there was nothing to compare it to. On lab day that is a day spent
wondering why grasps miss by a consistent amount.

THE SPLIT THAT MATTERS is between DECLARED and MEASURED.

  * DECLARED is what the sim builds and what every task coordinate was
    verified against. It is a constant and it should be.
  * MEASURED is what the depth camera says the real surface is. It does not
    exist until something measures it, and `table_top()` returns the declared
    value until then -- which is correct for sim and is exactly the state that
    must be VISIBLE rather than assumed on real hardware.

`source()` says which you are getting. A consumer that cannot tell the two
apart is the bug this module replaces.

THE TOLERANCE IS NOT A STYLE CHOICE. `check()` fails past ``TOL_M``. The pad
measurement (recordings/baselines/pad_clearance.json) puts the pads 142 mm
above the surface at the worst grasp, so a 20 mm surface error does not drive
anything into the table -- but it does move the object 20 mm relative to a
grasp computed for the declared height, and the cube is 40 mm. Half a cube of
error is the difference between a grasp and a miss, so 20 mm has to be loud.
"""

# The height the sim builds and every task coordinate was verified against.
#
# STAYS AT 0.950, AND A 2026-08-16 ATTEMPT TO RAISE IT IS RECORDED BELOW.
# TASK_SPEC T1-1 asks for the cubes to REST on the table. They cannot: with
# the cubes at z = 1.120, `measure_objects_on_the_table.py` walks T1's six pick
# paths at N=5 with both controls correct and finds
#
#     slab top 0.950 / 1.000 / 1.020    0 of 54 waypoints lost
#     slab top 1.040                   12 of 54
#     slab top 1.100 (cubes RESTING)   38 of 54
#
# reading 1.020 as the highest surface that costs nothing. IT WAS BUILT AND IT
# BROKE T1, and the reason is an instrument fault worth recording: the sweep
# solves through `measure_what_binds.Rig`, whose `self.quat` is the LIVE
# end-effector orientation read off TF at construction -- the HOME wrist -- and
# NOT `WORKSPACE_ORIENT`, the pinned 30.7 deg near-side anchor that
# `run_abc.send()` writes into every waypoint. Measured at the anchor instead,
# a table at 1.020 puts its top 43 mm under the pads' place-down wrist
# (z = 1.0628 = the pad at 1.120 minus the 0.0572 m pad offset) and T1 loses
# 14 waypoints, all of them at the two pad slots.
#
# So the surface stays where every verified coordinate was measured against
# it. The cubes stand 170 mm clear of it and T1-1 remains BLOCKED, which is
# what TASK_SPEC section 9 already records. What blocks it is the pinned
# wrist: the anchor sits 30.7 deg above horizontal, so the hand arrives from
# the near side and BELOW, through the volume a table top occupies.
DECLARED_M = 0.950

# Past this, a measured surface disagrees with the declared one loudly enough
# to stop a session rather than be absorbed. See the module docstring.
TOL_M = 0.010

_measured = None
_note = ""


def set_measured(z_m, note=""):
    """Record the surface height MEASURED from depth data.

    `note` should say how: which camera, how many samples, what spread. A
    measured value with no provenance is a second declared value wearing a
    different hat.
    """
    global _measured, _note
    _measured = None if z_m is None else float(z_m)
    _note = note


def clear_measured():
    set_measured(None)


def table_top():
    """The height to build and grasp against: measured if known, else declared."""
    return DECLARED_M if _measured is None else _measured


def source():
    """'declared' or 'measured', so a caller can tell which it has."""
    return "declared" if _measured is None else "measured"


def provenance():
    return _note


def delta_m():
    """Measured minus declared, or None if nothing has been measured."""
    return None if _measured is None else _measured - DECLARED_M


def subscribe(node, topic="/perception/work_surface_z"):
    """Keep THIS process's measured value in step with the estimator's.

    THE PRODUCER IS IN ANOTHER PROCESS AND MODULE STATE DOES NOT TRAVEL.
    `srl_perception.work_surface_node` measures the surface from depth and
    calls `set_measured()` in its own interpreter, which does nothing at all
    for the follower, the clip scene or the task layer. Every consumer that
    grasps against `table_top()` has to subscribe, and this is the one line
    that does it.

    Returns the subscription so the caller can hold it; dropping it on the
    floor lets rclpy garbage-collect the callback and the value silently
    stops updating, which is the same class of quiet failure this module
    exists to remove.
    """
    from std_msgs.msg import Float64

    def _on(msg):
        set_measured(float(msg.data), "from %s" % topic)

    return node.create_subscription(Float64, topic, _on, 1)


def check(require_measured=False):
    """(ok, message). NEVER returns a bare boolean.

    A surface check that answers True/False gives an operator nothing to act
    on. Every path here names the number and what it means.
    """
    if _measured is None:
        if require_measured:
            return (False,
                    "WORK SURFACE NOT MEASURED. Running on the declared "
                    "%.3f m. On real hardware this is a guess: a surface "
                    "20 mm out moves every object half a cube relative to "
                    "the grasp computed for it." % DECLARED_M)
        return (True,
                "work surface %.3f m (DECLARED -- nothing has measured the "
                "real one)" % DECLARED_M)
    d = _measured - DECLARED_M
    if abs(d) > TOL_M:
        return (False,
                "WORK SURFACE %+.1f mm FROM DECLARED: measured %.3f m against "
                "%.3f m. Every grasp was verified against the declared "
                "height, so each one is now %+.1f mm off the object. The cube "
                "is 40 mm. Re-measure, or re-verify the task coordinates "
                "against %.3f m before running. %s"
                % (d * 1000, _measured, DECLARED_M, -d * 1000, _measured,
                   _note))
    return (True,
            "work surface %.3f m (MEASURED, %+.1f mm from declared, inside "
            "the %.0f mm tolerance) %s"
            % (_measured, d * 1000, TOL_M * 1000, _note))
