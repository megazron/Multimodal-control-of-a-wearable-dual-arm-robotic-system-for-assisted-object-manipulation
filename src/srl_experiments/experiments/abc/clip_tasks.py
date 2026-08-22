#!/usr/bin/env python3
"""The three CLIP tasks: A pick and place, B bimanual hold and place, C multimeter.

THESE ARE NOT THE STUDY TASKS. `tasks.py` holds the participant spec (A
positioning, B coordinated carry, C dual pursuit), verified N=10 over the
densified full path. This module holds the three tasks the RECORDING set
demonstrates, which is a different question: what the rig can be shown doing.

EVERY COORDINATE BELOW IS TAKEN FROM SOMETHING ALREADY VERIFIED, and where a
task needs a shape the study spec does not have, it is built from verified
points rather than chosen freshly. Inventing a nearby coordinate is how a
protocol acquires a pose that fails IK on the day, and this project has paid
for that once already.

  A  pick and place    from OPTIONAL_PICK_PLACE -- picks at |x| >= 0.30, which
                       is where a TOP-DOWN grasp is kinematically feasible
                       (measured 0/9 at x = 0.25, 9/9 at x = 0.30), with the
                       0.10 m standoff and the bin, 3/3 approaches and 3/3
                       places verified.
  B  hold and place    both arms, on the 500 mm span of TASK_B, whose whole
                       path is verified at N=10. LEFT HOLDS STILL while RIGHT
                       traverses -- that asymmetry is the task, and it is
                       drawn from the same verified band.
  C  multimeter        a two-handed instrument task on the same verified band:
                       left presents the body, right brings the probe to it
                       and holds contact.

WHAT "MULTIMETER" MEANS HERE, STATED PLAINLY. There is no multimeter in the
scene and no multimeter model in this repository. Task C is the two-handed
PRESENT-AND-PROBE MOTION that such a task consists of, executed on verified
coordinates. The clip shows the motion; it does not show an instrument, and
the caption on every C clip says so. Filming an empty gripper and captioning
it "multimeter" without that sentence would be the kind of claim this project
exists to avoid.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tasks as T                                            # noqa: E402
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "srl_teleop"))
from srl_teleop import gripper_state as _gs                  # noqa: E402

Y = T.Y                       # 0.35, the only fore/aft band both arms work in
SEP = T.TRAY_SEP              # 0.500, verified clear of the dead band


def _dense(path, step=0.03):
    out = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        n = max(1, int(math.ceil(math.dist(a, b) / step)))
        for k in range(n):
            f = k / float(n)
            out.append([a[j] + f * (b[j] - a[j]) for j in range(3)])
    out.append(list(path[-1]))
    return out


def _hold(pt, n):
    return [list(pt) for _ in range(n)]



# ---------------------------------------------------------- THE WORK SURFACE
# OBJECTS MUST REST ON SOMETHING, AND THAT SOMETHING MUST BE IN THE PLANNING
# SCENE.
#
# Two separate defects, both measured:
#
#  1. Everything floated. The bench top was at z = 0.935 while the objects sat
#     at their EE task coordinates, so measured base heights were 1.042 to
#     1.292 -- between 0.107 m and 0.357 m of clear air under every object.
#     They were markers at coordinates, not things on a surface.
#
#  2. The bench was DECORATION. clip_scene published a MarkerArray to
#     /task_objects and nothing at all to /planning_scene, so `avoid_collisions`
#     could not see it and the arm swept straight through a bench that looked
#     solid on screen. CLAUDE.md already records this exact failure for the
#     earlier task_scene work -- "rehearsing against decoration teaches a
#     motion that will collide on the real rig" -- and the clip scene had
#     regressed it.
#
# THE BENCH IS AT CHEST HEIGHT, NOT TABLE HEIGHT, and that is not a choice.
# These arms cannot work a table: measured 16.7% IK at z = 0.75 against 77.8%
# at chest height, and the log's own conclusion is "the tray needs a stand,
# not a table". The surface is therefore placed under the objects rather than
# the objects dropped onto a table they cannot reach.
# READ FROM THE ONE OWNER, NOT DECLARED HERE. This was the literal `1.10` and
# `clip_scene.TABLE_TOP` was the literal `0.950`, and the two drifted 150 mm
# apart without anything noticing, because nothing compared them: every cube,
# pad and marking tile is positioned against THIS constant and the only surface
# with geometry is drawn against the other. The raised bench that used to close
# the gap was deleted as scenery ("THE BENCH IS GONE. ONE TABLE.") and the
# objects were left in the air. `srl_experiments.work_surface` is the owner and
# both now read it, so they cannot diverge again --
# `test_one_work_surface_height.py` asserts they are the same number.
try:
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.join(
        _os.path.dirname(_os.path.dirname(_os.path.dirname(
            _os.path.dirname(_os.path.abspath(__file__)))))), )
    from srl_experiments.work_surface import work_plane as _work_plane
    BENCH_TOP = _work_plane()
except Exception as _e:                                  # pragma: no cover
    # LOUD. A silent fallback here re-creates the exact defect: a second
    # surface height that nothing compares to the first.
    import warnings as _w
    _w.warn("work_surface unavailable (%s); BENCH_TOP is falling back to a "
            "HARDCODED height. This is the two-surfaces defect returning."
            % _e, RuntimeWarning)
    BENCH_TOP = 1.10
# 0.245, MEASURED. The gripper reaches up into the object from in front, so
# its body ends up BELOW the bench top -- free space only ahead of the edge.
# Swept against the real collision bench: at 0.275 the bin release and both
# box poses are blocked, at 0.245 only the box poses are, and those turned
# out to be an x problem rather than an edge problem (below).
BENCH_NEAR_Y = 0.245         # the edge the hand approaches from
BENCH_FAR_Y = 0.63
BENCH_HALF_X = 0.85
BENCH_THICK = 0.04

# HOW FAR EVERY OBJECT PROJECTS PAST THE EDGE. This is the parameter that
# actually varies gripper-to-bench clearance, and finding that out took a
# broken sweep: the first version swept BENCH_NEAR_Y, but on_bench() places
# objects at BENCH_NEAR_Y + depth/2, so moving the edge moved every object
# with it and the wrist-to-edge geometry was INVARIANT. The score sat at
# exactly 40 failures for every edge from 0.10 to 0.26 m -- the parameter had
# been cancelled out of the quantity being scored. Swept properly, failures
# fall 40 -> 31 and plateau at 0.06; 0.08 sits past the knee.
OVERHANG = 0.08

# WHERE AN IDLE ARM WAITS. In front of the bench edge and above the bench
# top, so an arm that is not working is not standing in the furniture. The
# idle holds used to sit at the study band y = 0.35, which is behind the edge,
# and each one failed on its own every run.
def park(x):
    return [x, round(BENCH_NEAR_Y - 0.12, 4), round(BENCH_TOP + 0.18, 4)]


# Clearance the transit keeps above the bench top. The hand must dip below
# the top to reach an object resting on it -- that is what the overhang is
# for -- but it has no business down there while merely travelling.
TRANSIT_Z = 0.10

# Wrist -> finger-pad offset along the tool axis, in world, at the anchor
# orientation. `orientation_mode` is `fixed` so this is constant for a run.
# The LEFT arm's offset, kept as a module-level name because the legacy A/B/C
# coordinates default to it. Derived from the same one source below; it is
# assigned after PAD_OFFSET_BY_ARM is built.
PAD_OFFSET = None

# ONE PAD OFFSET PER ARM, AND IT IS DERIVED, NOT RECORDED.
#
# WHAT IT IS. The wrist-to-pad vector in WORLD coordinates, per arm, at the
# pinned anchor -- which is the only orientation these tasks ever command,
# because `run_abc.send()` writes `master_calibration.WORKSPACE_ORIENT` into
# every waypoint. `ee_for` subtracts it, so declaring where an OBJECT is gives
# the wrist pose that puts the pads on it.
#
# WHY IT IS DERIVED AS OF 2026-08-23, AND WHAT IT COST NOT TO BE.
#
# It used to be two hand-recorded world vectors of magnitude 0.1118 m, sampled
# off TF at the home pose. `grasp_frames.PAD_MID_EE` is the same quantity in
# the END EFFECTOR frame, measured on 2026-08-18 by `/compute_fk` on the two
# finger-tip links -- 0.09833 m, both arms agreeing to 0.000 mm. The recorded
# world vectors were **13.45 mm (left) / 13.51 mm (right) LONGER** than that
# measurement, purely along the tool axis.
#
# The consequence was not a broken grasp, which is why it survived: the scene
# DRAWS each object at `ee_for(obj) + this same offset`, so the drawn object
# and the declared coordinate coincided and every picture looked right. What
# was wrong is where the FINGERS went. The pads sit at
# `wrist + R(anchor) . PAD_MID_EE`, so with a 13.45 mm-too-long offset they
# landed **13.45 mm SHORT of the declared object**, on every object of T0, T2
# and T3, identically -- the signature of a constant rather than of a path.
# It was the single largest term in `scripts/measure_control_budget.py`'s
# error budget, against a 30 mm grasp capture gate.
#
# T1 has been correct since 2026-08-18 because it declares its own approach
# and goes through `grasp_frames.wrist_for`, i.e. through the measurement.
#
# DERIVING IT KEEPS TASK AND PICTURE IN AGREEMENT. Both sides use this one
# constant, so the object is still drawn exactly at its declared coordinate;
# what moves is the commanded WRIST, 13.45 mm along the tool axis toward the
# object. Every T0/T2/T3 pose was re-verified after the change -- reachability
# at N=10 over the densified path, the 0.15 m wearer floor, and the finger
# tips against the table -- and `recordings/baselines/pad_offset_change.json`
# records the before and after.
#
# THE TWO ARMS DIFFER BY 48.3 mm AND THAT IS ENTIRELY THEIR ANCHORS. Rotated
# into each arm's own end-effector frame the two vectors are identical to
# 0.0002 mm, because they are now the SAME constant rotated twice. The
# hardware is symmetric; only the parked orientations differ.
def _pad_offset_by_arm():
    """`grasp_frames.PAD_MID_EE` rotated into world by each arm's anchor."""
    import numpy as _np
    import grasp_frames as _gf
    from srl_teleop.master_calibration import WORKSPACE_ORIENT as _W
    out = {}
    for _arm in ("left", "right"):
        _q = _np.asarray(_W[_arm], float)
        # NORMALISED ON THE WAY IN. WORKSPACE_ORIENT is not stored unit
        # (norms 0.99995844 and 1.00000561), so a rotation built from it is
        # not orthonormal -- the same defect `orientation_policy` found and
        # fixed on its own side. The stored constant is NOT rewritten;
        # HARD CONSTRAINT 1 is untouched.
        _q = _q / _np.linalg.norm(_q)
        out[_arm] = [round(float(v), 6)
                     for v in _gf.q_matrix(_q) @ _np.asarray(_gf.PAD_MID_EE)]
    return out


PAD_OFFSET_BY_ARM = _pad_offset_by_arm()
PAD_OFFSET = PAD_OFFSET_BY_ARM["left"]

# The legacy hand-recorded vectors, kept so the change is auditable and so
# `recordings/baselines/pad_offset_change.json` can be reproduced. Read by the
# test that asserts the size and direction of what was corrected. NOT used to
# command anything.
LEGACY_PAD_OFFSET_BY_ARM = {
    "left": [-0.0171, 0.0945, 0.0572],
    "right": [0.0289, 0.0995, 0.0421],
}


def ee_for(obj_xyz, arm="left"):
    """WRIST pose that puts the finger pads -- and so the object -- HERE.

    DECLARE WHERE THE OBJECT IS, DERIVE WHERE THE WRIST GOES. Getting this
    backwards is what put the wrist inside the bench: the task coordinates
    are EE poses, the pads are +0.095 m forward and +0.057 m up of the wrist,
    so a wrist at the object's own y sits 95 mm too far into the bench and
    37 mm below its top. With the bench a real collision object, the pick
    pose was correctly REFUSED -- the geometry had been wrong all along and
    a hollow bench had been hiding it.

    `arm` DEFAULTS TO LEFT so the legacy A/B/C coordinates do not move. Every
    MSc task passes its own arm; see PAD_OFFSET_BY_ARM for why one number for
    two arms was wrong by 48 mm.
    """
    off = PAD_OFFSET_BY_ARM.get(arm, PAD_OFFSET)
    return [round(obj_xyz[i] - off[i], 4) for i in range(3)]


def on_bench(x, depth_m, height_m, surface_z=None):
    """Object centre resting on a surface with its NEAR FACE at the edge.

    THE EDGE IS WHAT MAKES THE APPROACH POSSIBLE. The pinned wrist's tool
    axis is (-0.153, +0.846, +0.511) in world -- measured, not assumed --
    120.8 deg from straight down and 30.7 deg ABOVE horizontal. So the hand
    comes in from the near side, BELOW the object, and reaches up into it.
    The wrist therefore ends up in front of the bench edge and below the
    bench top, which is free space only at the edge. Anywhere further back
    the bench is exactly where the wrist needs to be.
    """
    z = (BENCH_TOP if surface_z is None else surface_z) + height_m / 2.0
    return [x, round(BENCH_NEAR_Y + depth_m / 2.0 - OVERHANG, 4), round(z, 4)]


# ---------------------------------------------------------------- TASK A
# Single arm. The other arm holds its start pose so the clip shows ONE arm
# working, which is the point of the task.
# The 40 mm block is a SMALL CUBE and stays where it is in x/y -- the brief
# keeps the cubes put. Only its height is derived, so it rests on the bench.
A_BLOCK_MM = 40
A_BLOCK = 0.040
# The block is a small cube: it keeps its x, and rests on the bench with its
# near face at the edge so the hand can come in under it.
A_BLOCK_OBJ = on_bench(0.32, A_BLOCK, A_BLOCK)
A_PICK = ee_for(A_BLOCK_OBJ)
# THE BIN IS 0.30 m LATERALLY FROM THE PICK, and that is the whole point.
#
# It used to be [0.30, Y, 1.02] -- 0.132 m from the pick of which 0.130 m was
# VERTICAL, so the block started hovering directly above the bin and the
# "place" lowered it 130 mm straight down. That is a gripper descending, not a
# pick-and-place, and no viewer would read it as one.
#
# 0.62 comes from `scripts/sweep_task_a_bin.py`, which walks the bin outward
# and verifies the WHOLE densified path at N=10 for each candidate:
#
#     x = 0.35 .. 0.68   PASS      (lateral 0.030 .. 0.360 m)
#     x = 0.71            fail     <- the IK boundary, 3863 calls
#
# So IK is NOT the binding constraint here: the BENCH is. At 1.30 m wide it
# spanned x = +/-0.65, and a 0.16 m bin centred at 0.62 hangs over the edge.
# The bench is scenery and carries no verified coordinate, so it was widened
# to 1.70 m rather than compressing the task to x = 0.57, where the lateral
# travel would land exactly on the 0.25 m requirement with no margin.
#
# 0.62 leaves 0.06 m of IK margin against the last passing candidate and
# 0.23 m of bench. Feasibility on this rig falls off a cliff rather than
# degrading, so the margin is the defence and N only located the edge.
# The bin is not a small cube, so it goes to the bench EDGE, and its base
# rests on the bench. The block is released just above the rim.
# 0.26 m across, not 0.16. The gripper comes in tilted 30.7 deg from
# horizontal, so its body sweeps a wider footprint than the jaws; at 0.16 m
# it clipped the rim on every release once the bin walls were real collision
# objects rather than markers.
A_BIN_H, A_BIN_D = 0.07, 0.26
A_BIN_OBJ = on_bench(0.62, A_BIN_D, A_BIN_H)        # the bin itself
# Release the block just above the rim, over the bin's centre.
# Release ABOVE the rim, not inside the bin. The block drops the last few
# centimetres, which is what happens physically anyway, and the gripper never
# has to fit between the walls.
# RELEASE OVER THE BIN'S NEAR EDGE, NOT ITS CENTRE. Over the centre the
# wrist lands at y = 0.20 against a near wall at 0.165 -- inside the bin's
# own footprint -- so the gripper body clipped the rim for the whole final
# approach. All twelve of task A's remaining failures were exactly those
# waypoints, from x = 0.487 to 0.637 at z = 1.202. Dropping over the near
# edge puts the wrist ahead of the wall in free space.
A_BIN = ee_for([A_BIN_OBJ[0],
                round(A_BIN_OBJ[1] - A_BIN_D / 2.0 + 0.03, 4),
                BENCH_TOP + A_BIN_H + A_BLOCK / 2.0 + 0.07])
A_STANDOFF = 0.10


def task_a():
    pre = [A_PICK[0], A_PICK[1], A_PICK[2] + A_STANDOFF]
    # Lift clear of the bench BEFORE travelling, and cross at that height.
    hi = round(BENCH_TOP + TRANSIT_Z, 4)
    lift = [A_PICK[0], A_PICK[1], hi]
    over = [A_BIN[0], A_BIN[1], max(A_BIN[2], hi)]
    # 30 held waypoints at the bin, not 4. The gripper opens on a WAYPOINT
    # INDEX while the arm arrives on its own schedule, so a mode that lags the
    # waypoint stream releases before it gets there. Measured: under VR teleop
    # the block was let go at (0.453, 0.445, 1.226) against a bin at
    # (0.603, 0.445, 1.077) -- 0.15 m short and 0.15 m high, dropped in mid
    # air -- while every other mode placed the block at 0 mm from target.
    # 14 waypoints (2.1 s) was not enough: VR still missed by 0.189 m, and its
    # clips run 27-35 s against 13-16 s for every other mode, so the lag is a
    # property of the extra hop through vr_pose_mapper rather than noise.
    # 30 waypoints is 4.5 s of dwell at the bin.
    left = _dense([pre, A_PICK]) + _hold(A_PICK, 6) + \
        _dense([A_PICK, lift, over, A_BIN]) + _hold(A_BIN, 30)
    return {"left": left, "right": _hold(park(-0.32), len(left))}


# ---------------------------------------------------------------- TASK B
# Bimanual. LEFT holds the work still; RIGHT brings a part to it and places.
B_PART_MM = 45
B_PART = 0.050
BOX_H, BOX_D = 0.05, 0.11
BOX_OBJ = None                                      # set below
# SURFACE TO SURFACE. The part STARTS resting on the bench and is placed on
# top of the circuit box, so it is supported at both ends of the motion
# instead of beginning in mid-air.
# THE CIRCUIT BOX IS AT x = -0.35, NOT -0.25, AND THAT IS THE WEARER'S DOING.
# Sweeping x against the real scene: -0.20 and -0.25 are BLOCKED (the arm
# crosses toward the centreline and meets the torso), -0.30 and outboard are
# clear. Raising the box did nothing at any height from 0.05 to 0.20 m, which
# is what ruled out the box's own top surface as the blocker. -0.35 keeps
# 50 mm of margin past the boundary.
BOX_X = -0.35
B_HOLD = ee_for(on_bench(SEP / 2.0, B_PART, B_PART))
B_START = ee_for(on_bench(-0.60, B_PART, B_PART))
BOX_OBJ = on_bench(BOX_X, BOX_D, BOX_H)             # the circuit box
# SAME NEAR-EDGE RULE AS THE BIN. Targeting an object's CENTRE puts the wrist
# inside that object's own footprint -- the wrist trails the pads by 95 mm --
# so the gripper body clips the near face on the way in. Both of task B's and
# both of task C's remaining failures were exactly that.
BOX_NEAR_Y = round(BOX_OBJ[1] - BOX_D / 2.0 + 0.02, 4)
# Released just clear of the box top for the same reason as the bin.
# 0.08 m of standoff above the box, not 0.04. MEASURED: the two right-arm
# box poses were the only ones under the project's 20 mm margin rule, both at
# 10 mm, tight in +y and -z. Sweeping the two available levers:
#
#     overhang 0.08 standoff 0.04 -> 10 mm     overhang 0.14 standoff 0.04 -> 10 mm
#     overhang 0.08 standoff 0.08 -> 50 mm     overhang 0.14 standoff 0.08 -> 20 mm
#     overhang 0.20 -> the pose FAILS at any standoff
#
# So the standoff is the lever and the overhang is not; more overhang makes it
# worse. 0.08 takes the right arm from 10 mm to 50 mm, clearing the bar with
# room, and no task has to move arms or be cut.
BOX_STANDOFF = 0.08
B_PLACE = ee_for([BOX_OBJ[0], BOX_NEAR_Y,
                  BENCH_TOP + BOX_H + B_PART / 2.0 + BOX_STANDOFF])


def task_b():
    hi = round(BENCH_TOP + TRANSIT_Z, 4)
    up = [B_START[0], B_START[1], hi]
    over = [B_PLACE[0], B_PLACE[1], hi]
    right = _hold(B_START, 4) + _dense([B_START, up, over, B_PLACE]) + \
        _hold(B_PLACE, 8)
    return {"left": _hold(park(0.32), len(right)), "right": right}


# ---------------------------------------------------------------- TASK C
# Two-handed instrument motion: left presents, right probes and holds contact.
# RESPEC'D 2026-08-10: 50 x 90 x 130 -> 30 x 70 x 45 mm.
#
# The multimeter bound the object set THREE ways at once -- 8.0 mm pose error,
# a 17.5 mm capture half-window, and the only object failing the scene
# fingerprint's sigma <= 2 mm. Sweeping both available dimensions shows the
# three constraints pull on DIFFERENT ones, so only changing both fixes it:
#
#   candidate            pose err   capture   sigma<=2
#   50 x 90 x 130 (old)    8.0 mm    17.5 mm     no
#   35 x 90 x 130          7.9 mm    25.0 mm     no      <- narrow only: pose
#                                                           is unchanged
#   50 x 90 x  60          2.2 mm    17.5 mm     no      <- shallow only:
#                                                           capture unchanged
#   30 x 70 x  45          1.7 mm    27.5 mm     YES     <- all three
#
# WIDTH sets the capture window ((85 - w)/2); DEPTH conditions the
# known-dimension fit, because a deep object is seen from one face and the
# hidden extent has to be inferred. 30 x 70 x 45 is a compact pocket DMM and
# costs nothing but a reprint.
C_MM_MM = 30
C_MM_SIZE = (0.03, 0.07, 0.045)
C_MM_OBJ = on_bench(0.35, C_MM_SIZE[1], C_MM_SIZE[2])
C_PRESENT = ee_for(C_MM_OBJ)
# The probe touches the top of the circuit box.
C_PROBE_ON = ee_for([BOX_OBJ[0], BOX_NEAR_Y,
                     BENCH_TOP + BOX_H + BOX_STANDOFF])
C_PROBE_UP = [C_PROBE_ON[0], C_PROBE_ON[1], round(C_PROBE_ON[2] + 0.11, 4)]


def task_c():
    hi = round(BENCH_TOP + TRANSIT_Z, 4)
    up = [C_PROBE_ON[0], C_PROBE_ON[1], max(C_PROBE_UP[2], hi)]
    right = _dense([up, C_PROBE_ON]) + _hold(C_PROBE_ON, 10) + \
        _dense([C_PROBE_ON, up])
    left = _hold(C_PRESENT, len(right))
    return {"left": left, "right": right}


# THE ONE DEFINITION, imported. This was a third copy of the same map -- kept
# "so this module has no dependency on the recorder", which was the right
# instinct pointed at the wrong owner: the dependency belongs on srl_teleop,
# which depends on nothing in-repo, not on a script under scripts/. Three
# copies of the threshold that decides whether a grasp happened is three
# chances for a study's trials to be reclassified by a typo.
grip_for = _gs.grip_for
OPEN = _gs.CMD_OPEN_RAD


def _sched(n, arm, close_at, open_at, width_mm):
    """A per-waypoint gripper schedule: OPEN, close to the object's WIDTH,
    hold, open again.

    Closing to the WIDTH rather than fully is what makes the grasp legible and
    what makes the attachment honest -- the scene attaches only once the
    fingers reach 0.90 of this value, so a gripper that merely left the open
    position never picks anything up.
    """
    g = grip_for(width_mm)
    out = []
    for k in range(n):
        if k < close_at:
            out.append(OPEN)
        elif k < open_at:
            out.append(g)
        else:
            out.append(OPEN)
    return {arm: out, ("right" if arm == "left" else "left"): [OPEN] * n}


TASKS = {
    "a": dict(name="pick and place",
              scenario="S1_single_arm",
              build=task_a,
              # descend (open) -> close on the 40 mm block at the pick ->
              # carry -> open over the bin
              grip=lambda n: _sched(n, "left", 5, n - 4, 40),
              width_mm=40,
              # WHERE THE FINGERS MUST BE FOR THE CLOSE TO BE A GRASP, in
              # OBJECT coordinates. The runner gates the close on the PADS
              # reaching this, not on a waypoint index -- see run_abc. A
              # waypoint index is only a proxy for "the arm is there", and it
              # is a false one on any transport whose commands are not in the
              # world frame (VR sends controller-frame poses).
              grip_obj=A_BLOCK_OBJ,
              # Where the object must END UP, as an EE-frame coordinate. The
              # scene shifts objects to the finger pads, so the check adds the
              # same offset -- see record_abc_sweep.
              place_target=A_BIN,
              expect="LEFT arm descends 0.10 m to the pick, closes, lifts "
                     "0.14 m, carries to the bin and releases. RIGHT arm "
                     "holds still throughout.",
              caveat=""),
    "b": dict(name="hold and place",
              scenario="S1_bimanual",
              build=task_b,
              # RIGHT already holds the part, carries it down, releases at the
              # placement; LEFT stays open, holding the work steady.
              grip=lambda n: _sched(n, "right", 1, n - 5, 45),
              width_mm=45,
              grip_obj=on_bench(-0.60, B_PART, B_PART),
              expect="LEFT arm HOLDS the work still at x=+0.25. RIGHT arm "
                     "brings the part down 0.15 m and places it. Both arms "
                     "engaged at once.",
              caveat=""),
    "c": dict(name="multimeter",
              scenario="S1_present_probe",
              build=task_c,
              # LEFT grips the multimeter for the whole task and never lets
              # go -- presenting it IS the task. RIGHT stays open: the probe
              # is the gripper itself.
              grip=lambda n: _sched(n, "left", 0, n, C_MM_MM),
              width_mm=C_MM_MM,
              grip_obj=C_MM_OBJ,
              expect="LEFT presents the body and holds it steady. RIGHT "
                     "brings the probe down 0.11 m, holds contact, retracts.",
              caveat="The multimeter is a DUMMY BODY -- a coloured box of "
                     "the right size, not an instrument model. It is grasped, "
                     "carried and presented for real. Respec'd to "
                     "30 x 70 x 45 mm: it bound the object set three ways at "
                     "once and both dimensions had to change."),
}
