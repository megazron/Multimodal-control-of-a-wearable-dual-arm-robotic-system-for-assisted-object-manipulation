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
DECLARED_M = 0.95

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
