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
#
# ==========================================================================
# 1.100 SINCE 2026-08-17, AND IT IS NOW THE ONLY SURFACE HEIGHT IN THE REPO.
# ==========================================================================
# The note above is kept because its measurement stands and its CONCLUSION was
# wrong for a reason worth naming. Everything above swept the SLAB while
# holding the table's near edge at 0.100, under the whole approach -- and
# `measure_what_binds.Rig`, which every one of those sweeps solves through,
# was asking IK for the HOME wrist rather than for `WORKSPACE_ORIENT`, the
# anchor `run_abc.send()` actually commands. Measured 2026-08-17 the two tool
# axes are 32.26 deg apart on the left arm, which is the arm T1 runs on. So
# "raising the table costs 46 of 54 waypoints" was priced at an orientation
# the task never sends. Rig now defaults to the anchor.
#
# WITH THE ARITHMETIC DONE AT THE ANCHOR THE ANSWER IS EXACT AND NEEDS NO IK.
# `clip_tasks.ee_for()` puts the wrist at `obj - PAD_OFFSET`, so for the left
# arm's offset (-0.0171, +0.0946, +0.0572) an object at y sits over a wrist at
#
#     y_wrist = y_obj - 0.0946      z_wrist = 1.0628
#
# 37 mm BELOW a top at 1.100. So the wrist is inside the slab unless it is in
# front of the near edge E, and the object is off the table unless its near
# face is behind E:
#
#     y_obj - 0.0946  <  E  <=  y_obj - depth/2
#
# A window 74.6 mm wide for a 40 mm cube, and it is NOT empty -- which is the
# thing the earlier sweeps concluded. What made it look empty is the PADS: for
# a pad of depth D whose far placement slot is +SLOT_DY, the same two
# inequalities require
#
#     D/2 + SLOT_DY  <  0.0946
#
# and the shipped pad was 0.130 deep with SLOT_DY 0.030 -- 0.0950, over by
# 0.4 mm. The pads could not rest on any surface their own far slot could be
# reached over, by four tenths of a millimetre, and every sweep that raised a
# slab under them was measuring that. See `search_t1_layout_on_surface.py`.
#
# AND THEN THE MEASUREMENT AT THE FIXED ANCHOR SAID NO. Both sweeps were
# re-run with `Rig` asking for `WORKSPACE_ORIENT`, every control correct:
#
#   `search_centre_on_surface.py`  x 0.00..0.45, y 0.10..0.55, top 0.70..1.10,
#     overhang 0 and 0.05, BOTH arms, pinned AND top-down, objects RESTING:
#     **0 of 3360 cells** survive the full pick path. The 22 hits this file's
#     earlier note called "a lead" were the home-wrist artefact; the baseline
#     that recorded them is superseded.
#   `search_t1_layout_on_surface.py`  T1's own x = 0.560, tops 0.900..1.100,
#     object 20..60 mm behind the edge, objects RESTING: **every cell fails.**
#     Not the near edge -- an edge at y = 0 fails too. The slab occupies the
#     volume the ARM needs, which is the bench finding at full size.
#   `measure_objects_on_the_table.py`  objects held at 1.120, slab swept:
#     0.950 / 1.000 / 1.020 cost **0 of 54**; 1.040 costs 12; 1.100 costs 38.
#     Controls: no slab 0, slab through the objects 46.
#
# AND THEN THAT LAST SWEEP TURNED OUT TO BE MEASURING THE WRONG PATH, which is
# worth the space because it cost a wrong value that was committed for an hour.
# It walks T1's SIX PICK PATHS. The path T1 sends is 171 waypoints and also
# contains the two PAD PLACEMENTS, the transits and the standoffs -- and a
# surface can delete a placement while costing every pick nothing. Acting on
# "1.020 costs 0 of 54" put `verify_t1_paths.py` at **36 IK failures of 171**.
#
# `sweep_surface_vs_t1_path.py` walks the WHOLE path, height and near edge
# together, N=3, controls correct (no slab 0, slab through the objects 171,
# shipped 0.950/0.100 zero):
#
#     top \ near edge   0.050   0.062   0.080   0.100
#     0.950                 7       0       0       0
#     0.980                16      14      14       0
#     1.000                33      18      14      14
#     1.020                36      36      32      18
#
# TWO THINGS THIS SAYS THAT THE PICK SWEEP COULD NOT. Height and forward reach
# TRADE against each other -- the pick sweep held the edge still, so it saw a
# one-dimensional slice of a two-dimensional constraint and read the best cell
# off the wrong axis. And the highest surface that costs the full path nothing
# is **0.980 with its near edge at 0.100**, not 1.020.
#
# SO THE TWO HEIGHTS ARE DIFFERENT ON PURPOSE, AND THAT IS NOW SAID OUT LOUD
# RATHER THAN BEING TWO LITERALS IN TWO FILES. `WORK_PLANE_M` is where the
# objects are and it does not move -- every verified T1 coordinate is measured
# against it. `DECLARED_M` is the surface a viewer reads as the work surface,
# and it is at the HIGHEST value that costs T1 nothing. The remaining
# `FLOAT_GAP_M` is the honest residual: T1-1 is BLOCKED, by the pinned wrist,
# and `test_one_work_surface_height.py` pins all three numbers so that closing
# the gap has to be a measurement and not an edit.
#
# 150 mm -> 120 mm. Nothing here makes the cubes rest on the table; it stops
# them hanging further above it than they have to, and it makes the residual a
# number with a measurement behind it instead of an accident.
WORK_PLANE_M = 1.100
DECLARED_M = 0.980
FLOAT_GAP_M = round(WORK_PLANE_M - DECLARED_M, 4)
# The near edge is MEASURED too, and it is not free: at 0.980 the edge may come
# to 0.100 at no cost and 0.080 already costs 14 of 171. It lives here with the
# height because the two are one measurement, and `clip_scene` reads it.
DECLARED_NEAR_Y = 0.100


def work_plane():
    """The plane the task's objects sit on. NOT the drawn surface height.

    Kept separate from `table_top()` because they are separate facts and
    collapsing them is what produced a 150 mm gap nothing could see: every
    object was positioned against one and the only geometry was drawn against
    the other, with no consumer comparing them.
    """
    return WORK_PLANE_M

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
