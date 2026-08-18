#!/usr/bin/env python3
"""T1 -- COLOUR-MATCHED PICK AND PLACE, REBUILT 2026-08-17 FOR MODE 06 ALONE.

    import t1_task as T1
    path = T1.build()                  # {"left": [...], "right": [...]}
    grip = T1.grip(len(path["left"]))

WHAT WAS DELETED AND WHY. The T1 that stood until 2026-08-17 put four cubes
and two pads on the LEFT side of the table, 605 mm off the centreline, all six
of them floating 120 mm above the surface, and ran in all five modes. Both of
those properties were consequences of ONE constraint: every mode commands
`master_calibration.WORKSPACE_ORIENT`, the pinned near-side anchor, which
points 30.8 deg ABOVE horizontal, so the hand arrives from the near side and
from BELOW -- through the volume a table top occupies, and never far enough
inboard to work in front of the person. `archive/recordings/
t1_20260817_deleted_and_rebuilt/NOTE.md` holds the whole account.

THIS T1 RUNS UNDER `06_full_autonomy` AND NOTHING ELSE, and that is what makes
it possible. Under 06 the system supplies the pose; there is no operator whose
wrist is being pinned, so the approach is a variable of the task. Measured
(`recordings/baselines/t1_centre.json`), of 181 candidate approach
orientations exactly 14 can touch an object RESTING on a table at all, every
one of them within 10 deg of level with the hand pointing INBOARD, and the
pinned anchor is not among them.

THE ANCHOR IS NOT TOUCHED. HARD CONSTRAINT 1 forbids re-deriving
`WORKSPACE_ORIENT`, and re-deriving it costs T2 its right arm. What this
module carries is a per-task approach; `run_abc.set_orient()` sends it and
rebuilds the finger-pad offset from it, and every other task still sends the
anchor.

TWO ARMS, BECAUSE THE PADS ARE IN THE CENTRE. One arm cannot work on both
sides of a centreline, so a one-armed T1 can only ever put its pads to one
side. The blue pad sits at the LEFT arm's innermost workable column and the
green pad at the RIGHT arm's, straddling the centreline: the PAIR is directly
in front of the person, and each pad's own offset is stated in `OFFCENTRE_MM`
rather than hidden.

WHICH ARM PICKS WHICH CUBE FOLLOWS FROM THE COLOUR, NOT FROM THE SIDE. A cube
goes to the pad of the colour the CAMERA saw; the arm that runs it is the arm
that can reach that pad. So the blue cubes start on the left and the green on
the right -- and `scripts/verify_vision_drives_grasp.py` swaps the rendered
colours to prove the destination is chosen by the pixels and not by the side
the cube happens to be on.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import clip_tasks as CT                                       # noqa: E402
import grasp_frames as GF                                     # noqa: E402

# ==========================================================================
# THE LAYOUT. Every number here was produced by `scripts/solve_t1_layout.py`
# and re-verified by `scripts/verify_t1.py`; the baseline it came from is
# `recordings/baselines/t1_layout.json` and `test_t1_layout_is_verified.py`
# fails if the two drift apart.
# ==========================================================================
CUBE_M = 0.040
PLANE_COLOURS = ("blue", "green")

# The approach, in degrees. Heading is INBOARD and is mirrored per arm, so
# "50 deg inboard" means the same thing on both.
# MEASURED, not chosen. Of 181 candidate orientations swept in
# `t1_centre.json`, 14 could touch an object resting on a table at all and
# this is the one the layout was solved and verified at. It must not be edited
# without re-running `scripts/solve_t1_layout.py`: the layout's coordinates
# ARE this orientation's reachable cells, and a 5 deg change cost every cube
# its grasp pose the first time these two drifted apart.
APPROACH_ELEV_DEG = -10.0
# -50, NOT -65, SINCE 2026-08-18. The heading was re-measured after the pad
# offset was corrected, because the old one no longer reached: at -65 deg the
# gripper base fouls the table 5.2 mm and 0 of 40 grasp attempts solve on
# either arm. Swept at N=10 per cell over the four cubes, elevation -10 solves
# 40 of 40 from heading -45 to -57.5 and 0 of 40 outside it; -50 is the middle
# of that window and is the only heading that also solves at elevation -7.5,
# so it is the one with margin in both directions rather than the one that
# merely works.
APPROACH_HEAD_DEG = -50.0
APPROACH_ROLL_DEG = 0.0

APPROACH = {
    "left": GF.q_from_axis(GF.axis_for(APPROACH_ELEV_DEG, APPROACH_HEAD_DEG),
                           APPROACH_ROLL_DEG * 3.141592653589793 / 180.0),
    "right": GF.q_from_axis(GF.axis_for(APPROACH_ELEV_DEG, -APPROACH_HEAD_DEG),
                            APPROACH_ROLL_DEG * 3.141592653589793 / 180.0),
}

# ==========================================================================
# THE LAYOUT, proposed by `scripts/solve_t1_layout.py` and verified by
# `scripts/verify_t1.py` over the WHOLE composed task path with the wearer
# measured geometrically. `recordings/baselines/t1_paths.json` is the record,
# and `test_t1_layout_is_verified.py` fails if these constants and that record
# ever describe different layouts.
#
# THERE IS NO `t1_layout.json` ANY MORE, DELIBERATELY. The solver wrote one,
# the pads then moved twice on 2026-08-18, and it sat recording the first of
# those -- 105 mm from where the task actually sends the arm -- while every
# check that reads the CODE went on passing. One record, regenerated by the
# thing that walks the paths, is the only arrangement that cannot go stale
# quietly.
#
# VERIFIED, 203 waypoints per arm, three consecutive runs each:
#
#   --mode sequential   (what `ik_follower_node` tracks: each waypoint solved
#                       from the previous solution)
#                       0 IK failures, 0 inside the 150 mm floor, 3 of 3 runs,
#                       worst clearance 0.1503 m
#   --mode independent  (every waypoint solved afresh from home, N=10)
#                       0 IK failures, and the LEFT arm's carry admits
#                       branches reaching 0.0662 to 0.1489 m on one to seven
#                       waypoints of 203, varying run to run. The worst single
#                       reading seen is 0.0662 m -- 84 mm inside the floor
#
# THE SECOND NUMBER IS NOT NOISE AND IT IS NOT HIDDEN, and it is the biggest
# open risk in this task. It is what happens if the solver lands in another
# null-space branch mid-path: the left arm's elbow swings inboard and the
# forearm ends up 66 to 149 mm from the wearer against a 150 mm floor, with
# `/compute_ik` returning `valid` every time, because the SRDF excludes
# exactly those pairs.
#
# `ik_follower_node` has two guards and NEITHER is sufficient here. It rejects
# a solution that JUMPS -- which catches a branch change between consecutive
# waypoints -- and it refuses to publish below `min_clearance_m`, which is
# 0.05 m in sim. A 0.09 m branch passes both. Raising that floor to 0.150 for
# this task would close it, and that is a change to a safety parameter which
# should be made deliberately and measured, not slipped in here.
#
# So: run it, and watch `min_clearance` in the follower's own telemetry.
# Recorded in `t1_paths_independent.json` and in findings.
#
# EVERYTHING STANDS IN ONE ROW ON THE TABLE'S FRONT EDGE. That is measured,
# not arranged: of the two table overhangs swept in `t1_centre.json` -- 0 mm
# and 60 mm behind the object -- not one cell at 60 mm survived, on either arm,
# at any of five table heights. A level hand trails its forearm outboard and
# NEARER at the object's own height, 20 mm above the table, and 60 mm of table
# in that volume is 60 mm of collision. The pads extend FORWARD from the row,
# away from the wearer, which is free: nothing ever approaches from there.
#
# TWO CUBES SHARE A PAD AND THEIR LANDING SLOTS ARE SEPARATED IN X, along the
# row, for the same reason. A slot 35 mm further forward is a slot with 35 mm
# of table behind it.
# ==========================================================================
TABLE_TOP = 1.250
TABLE_NEAR_Y = 0.430
T1_Z = round(TABLE_TOP + CUBE_M / 2.0, 4)
# Every object's centre sits on this line: near face on the table edge.
ROW_Y = round(TABLE_NEAR_Y + CUBE_M / 2.0, 4)
# The pad's depth, declared before PAD_Y needs it.
PAD_D_DECL = 0.140

# Pad centres. Index 0 is BLUE and belongs to the LEFT arm, 1 is GREEN and
# belongs to the RIGHT arm. They straddle the centreline.
#
# THE PAD CENTRE IS NOT THE CLOSEST REACHABLE POINT AND THE DIFFERENCE IS
# STATED RATHER THAN HIDDEN, per T1-10. A pad is not a point: two cubes land
# on it, at +/- SLOT_DX along the row, and it is the INNER SLOT that has to be
# reachable. The measured innermost workable columns are 0.375 (left) and
# 0.275 (right), so the pad centres sit one slot outboard of those.
#
# OFF THE CENTRELINE: 425 mm (blue, left) and 325 mm (green, right). The PAIR
# is centred 50 mm to the left of the centreline and its two inner edges are
# 550 mm apart. Against the T1 this replaced -- both pads on one side at 595
# and 825 mm -- the near pad has come in by 500 mm and the work now straddles
# the person instead of sitting beside them. The two arms are not symmetric
# and that is structural: `home_render.json` records the mounts mirroring in
# position while differing by 168 deg of roll, and the left arm pays 100 mm
# for it here.
# THE LEFT PAD SITS WHERE ITS PLACE POSES ARE CLEAN ON EVERY DRAW, and that
# is a distribution rather than a single reading.
# `scripts/sample_place_branches.py` took 40 independent IK solutions at each
# candidate slot: at |x| = 0.425 both slots came back 40 of 40 at 0.1610 m,
# and at 0.450 the inner slot returned one branch of forty at 0.1258 m --
# 25 mm inside the floor, with `/compute_ik` calling all forty valid.
#
# READ THAT WITH THE `independent` CAVEAT BELOW. A 7-DOF arm has a continuum
# of elbow configurations for one hand pose, and this layout is clean for the
# branch the follower tracks, not for every branch that exists.
# THE PAD'S DRAWN CENTRE IS NOT WHERE A CUBE LANDS ON IT, and the two have to
# be separate numbers.
#
# A cube lands in the ROW, at y = ROW_Y, because the row is the only line the
# arm can work on -- everything stands on the table's front edge and 60 mm of
# table nearer than the object is 60 mm of collision. But the pad is 140 mm
# deep, so a pad CENTRED on the row would hang 50 mm off the front edge, which
# `verify_objects_on_table` correctly calls OVERHANGING: 64% of its base over
# the table. Measured that way before this split existed.
#
# So the pad's near face sits on the table edge and it extends FORWARD, away
# from the wearer, where nothing ever approaches from. The landing slots stay
# in the row, inside the pad's near strip.
PAD_Y = round(TABLE_NEAR_Y + PAD_D_DECL / 2.0, 4)
# MOVED OUT 2026-08-18, AND THE REASON IS A MEASUREMENT NOT A PREFERENCE.
#
# `measure_centre_gap.py` walked every column from the centreline outward with
# each waypoint solved AFRESH -- the pessimistic seeding -- and named the
# blocker per column. On the LEFT arm, |x| = 0.300 to 0.475 all come back
# FLOOR: the robot's `half_arm_2_link -> forearm_link` passes 0.076 to
# 0.122 m from the WEARER'S OWN FOREARM against a 150 mm floor. The first
# reliably clear column is 0.500, where the closest thing to the wearer is
# `base_link -> shoulder_link` at 0.1610 m -- the mount, which no joint moves.
#
# The pads were at 0.425 and 0.325, whose SLOTS at 0.375 and 0.275 sit inside
# that band. They verified clean under sequential seeding, which is what the
# follower tracks, and that is exactly the residual risk `t1_paths_
# independent.json` records. Moving both pads out until every slot is in the
# reliably-clear band removes the risk instead of documenting it.
#
# THE COST IS 105 mm ON THE LEFT AND 25 mm ON THE RIGHT, and it is stated
# rather than absorbed: the pair now sits at 530 and 350 mm off the
# centreline instead of 425 and 325.
# SYMMETRIC, AND RE-SOLVED 2026-08-18 AFTER THE PAD OFFSET WAS MEASURED.
#
# The pair was [[0.320, PAD_Y], [-0.165, PAD_Y]] -- 320 mm left of centre and
# 165 mm right of it, chosen on the centre search. Two things moved it:
#
# 1. `grasp_frames.PAD_MID_EE` was 13.47 mm too long (measured from FK,
#    `recordings/baselines/pad_mid_ee.json`), so every wrist pose in this task
#    sat 13 mm back along its own approach axis. Correcting it put the fingers
#    on the cube and the GRIPPER BASE 5.2 mm into the table, and the shipped
#    heading of -65 deg stopped solving at all: 0 of 40 grasp attempts.
# 2. The right pad's outer slot was in the wearer's own UPPER ARM. Walked
#    three times at N=10, the right arm read 0.1611, 0.1787 and then 0.1483 m
#    at the same waypoint against a 0.150 m floor -- one run in three inside
#    it. That is the null-space branch scatter this file already records for
#    the left arm, landing on a column with no margin to absorb it.
#
# Swept per column with the place pose solved afresh at N=10
# (`recordings/baselines/t1_pad_columns.json`): the right arm's clearance
# climbs 0.1398 -> 0.1902 -> 0.2111 m from |x| = 0.120 to 0.225 and saturates
# at the MOUNT's own 0.2202 m from 0.255 outward; the left arm is mount-limited
# at every column from 0.260 to 0.440. So there is a band where both arms are
# limited by the mount rather than by the person, and the pads are now in it --
# which also makes them SYMMETRIC, 260 mm either side of the centreline, with
# their inner edges 420 mm apart and the pair centred on the person.
T1_PLANES = [[0.260, PAD_Y], [-0.260, PAD_Y]]
# PAD_T is the mat's THICKNESS. `PLANE_T` is the name clip_scene has always
# used for the same quantity, kept as an alias so the scene and the task
# cannot end up drawing and placing at two different heights.
# 100 mm WIDE, AND THE PAD IS WHAT GAVE WAY. Moving the pads out to the
# reliably-clear band pushed the cubes out with them -- they have to start
# clear of the pad footprint -- and the outer left cube's PRE-GRASP then
# landed at x = 0.870, which the left arm cannot reach: measured, 1 IK failure
# at waypoint 43, reproducibly, in all six verification runs.
#
# So the pad narrows instead of the cubes moving. At 100 mm it is still two
# and a half cubes wide and its two landing slots are 60 mm apart inside it,
# and the cubes go back to 0.625 and 0.685 -- columns that verified clean
# before any of this moved.
PAD_W, PAD_D, PAD_T = 0.100, PAD_D_DECL, 0.010
PLANE_T = PAD_T
# 30 mm, NOT 50. The two slots have to be 60 mm apart so a 40 mm cube leaves a
# 20 mm gap and reads as two objects, and they BOTH have to land in the
# reliably-clear band. At +/-50 mm the inner slot of the left pad would sit at
# 0.480, back inside the wearer-forearm band the pad was just moved out of.
SLOT_DX = 0.030
# Kept for the consumers that still name it; the slots are in X now.
SLOT_DY = 0.0

# Cube positions, in the order the task runs them. 60 mm pitch, which is T0's
# measured minimum separation and what makes four cubes read as four objects
# rather than a heap -- the first run of the solver put two of them 25 mm
# apart, which for a 40 mm cube is one inside the other.
# SYMMETRIC TOO, and for the pads' sake rather than for tidiness: a pad now
# occupies |x| = 0.210 to 0.310 on each side, so a cube at 0.300 would start ON
# the pad it is meant to be delivered to, which is T1-2. 60 mm pitch as before.
T1_CUBES = [[0.400, ROW_Y], [0.460, ROW_Y],
            [-0.400, ROW_Y], [-0.460, ROW_Y]]
# Cube index -> pad index, and so colour. DECLARED, and deliberately NOT read
# by the planner: `t1_instruction` grounds on the camera's colour and this
# table exists so that a wrong-colour placement is scoreable and so that the
# mislabel control has something to flip.
T1_PAIR = {0: 0, 1: 0, 2: 1, 3: 1}

# WHAT THE CUBES ACTUALLY LOOK LIKE, kept SEPARATE from what the task believes
# about them. This is the whole mislabel control and it only works if the two
# are independent.
#
# `T1_PAIR` is a DECLARATION: it is what the task file says each cube is, and
# `scripts/verify_vision_drives_grasp.py` flips it to see whether the plan
# moves. `T1_RENDERED` is what `mock_rgbd_camera` paints and what `clip_scene`
# draws -- the PIXELS. Flipping the declaration must change nothing the camera
# sees, or the experiment is comparing a file with itself.
#
# They agree today, and that is the point: the plan built from the declaration
# and the plan built from the camera are the same plan until someone makes
# them disagree on purpose.
T1_RENDERED = ("blue", "blue", "green", "green")


def rendered_pad_index(i):
    """The pad index implied by what cube `i` LOOKS like. Never T1_PAIR."""
    return PLANE_COLOURS.index(T1_RENDERED[i])

STANDOFF, LIFT = 0.10, 0.08
PARK_X = 0.60

# WHICH ARM TAKES THE FRAME. One look serves the whole task -- the observe
# pose is solved against all four cubes AND both pads at once, so both arms
# then work from one set of detections. It is a property of the CAMERA, not of
# the work: the looking arm need not be the arm that picks anything. Set from
# `scripts/solve_observe_pose.py`, which refuses a pose that cannot see every
# point.
LOOK_ARM = "left"

_dense = CT._dense
_hold = CT._hold


def arm_for_pad(pad_index):
    return "left" if int(pad_index) == 0 else "right"


def ee_for(obj_xyz, arm):
    """WRIST pose putting the pads on `obj_xyz`, AT THIS TASK'S APPROACH.

    NOT `clip_tasks.ee_for`, which bakes in the anchor. Same construction,
    different orientation; `test_pad_offset_has_one_source.py` pins that the
    two agree wherever the anchor is what is being sent.
    """
    return GF.wrist_for(obj_xyz, APPROACH[arm])


_GRIP = {}
_GRIP_AT = {}


def plan_for(cubes=None):
    """[(x, y, pad_index, arm)] -- WHICH CUBE GOES WHERE, before any waypoint.

    The same resolution `build()` performs, exposed on its own so a caller can
    ask "which pad did cube 0 go to" without reverse-engineering it out of a
    waypoint list. `verify_vision_drives_grasp` needs exactly that, and the
    old way of getting it -- slicing the path into equal shares per cube --
    stopped working when T1 became two-armed: the shares are no longer equal,
    because a cube's arm follows its colour.
    """
    items = ([(cx, cy, T1_PAIR[i]) for i, (cx, cy) in enumerate(T1_CUBES)]
             if cubes is None else
             [(float(c[0]), float(c[1]), int(c[2])) for c in cubes])
    return [(x, y, p, arm_for_pad(p)) for x, y, p in items]


def build(cubes=None):
    """Pick each cube, place it on the pad of its own colour, both arms.

    `cubes` is [(x, y, pad_index)] as `vision_grasp.observe_and_detect`
    returns it -- what the CAMERA saw. `None` uses the declared layout, which
    is what the layout verification and the unit tests walk.
    """
    items = ([(cx, cy, T1_PAIR[i]) for i, (cx, cy) in enumerate(T1_CUBES)]
             if cubes is None else
             [(float(c[0]), float(c[1]), int(c[2])) for c in cubes])

    seq = {"left": [], "right": []}
    grip = {"left": [], "right": []}
    at = {"left": [], "right": []}
    g = CT.grip_for(int(CUBE_M * 1000))

    def add(arm, pts, state, obj):
        seq[arm].extend([list(p) for p in pts])
        grip[arm].extend([state] * len(pts))
        at[arm].extend([list(obj)] * len(pts))

    used = {}
    for cx, cy, pad_i in items:
        arm = arm_for_pad(pad_i)
        k = used.get(pad_i, 0)
        used[pad_i] = k + 1
        cube_obj = [float(cx), float(cy), T1_Z]
        px, _pad_y = T1_PLANES[int(pad_i)]
        px = round(px + (-SLOT_DX if k == 0 else +SLOT_DX), 4)
        # THE CUBE LANDS IN THE ROW, not at the pad's drawn centre. See the
        # note on PAD_Y: the pad extends forward for the picture, the row is
        # where the arm can work.
        py = ROW_Y
        # THE CUBE LANDS ON TOP OF THE PAD, NOT INSIDE IT. The pad is a mat
        # PAD_T thick lying on the table, so a cube placed on it sits that
        # much higher. Placing at T1_Z would bury the cube's lower PAD_T in
        # the mat, which is the "not inside the table" half of T1-5 wearing
        # different clothes.
        pad_obj = [px, py, round(T1_Z + PAD_T, 4)]

        pre_pick, pick = GF.approach_path(cube_obj, APPROACH[arm], STANDOFF)
        lift = [pick[0], pick[1], pick[2] + LIFT]
        place = ee_for(pad_obj, arm)
        # THE PLACE IS A VERTICAL DESCENT, AND THE PICK IS NOT. That asymmetry
        # is measured, not stylistic.
        #
        # A pick has to come in ALONG the approach axis: the cube is standing
        # on a table and a vertical descent would drive the trailing fingers
        # through the surface it rests on. A place has nothing above the pad,
        # so it does not need the axis -- and taking it anyway costs dearly.
        # The pre-place standoff sits 100 mm back along an axis that points
        # inboard and forward, which puts the WRIST at y = 0.166 against a row
        # at y = 0.300: 134 mm nearer the wearer than anything the layout was
        # verified at. Measured over three runs of `verify_t1.py`, one waypoint
        # of the left arm's carry landed 0.1436 to 0.1489 m from the wearer's
        # own forearm against the 150 mm floor, with 0 IK failures -- invisible
        # to every check that asks `/compute_ik`, because the SRDF excludes
        # exactly those pairs.
        #
        # Moving the whole row forward was tried first and is much worse: at
        # y = 0.360 the same three runs gave 29 to 35 breaches and took the
        # RIGHT arm below the floor as well.
        over = [place[0], place[1], place[2] + LIFT]

        add(arm, _dense([pre_pick, pick]), CT.OPEN, cube_obj)
        # FOURTEEN HELD WAYPOINTS AT THE CLOSE. Carried over unchanged from
        # the deleted T1, where it was measured: the close is gated on
        # ARRIVAL and the arm's lag accumulates along the sequence, so a
        # shorter dwell left the last cube ungrasped at 33.7 mm against a
        # 30 mm window. It is in the TASK, identically for every cube.
        add(arm, _hold(pick, 14), g, cube_obj)
        add(arm, _dense([pick, lift]), g, cube_obj)
        add(arm, _dense([lift, over]), g, pad_obj)      # traverse, high
        add(arm, _dense([over, place]), g, pad_obj)     # straight down
        add(arm, _hold(place, 2), g, pad_obj)           # arrive still holding
        add(arm, _hold(place, 3), CT.OPEN, pad_obj)     # release, ON the pad
        add(arm, _dense([place, over]), CT.OPEN, pad_obj)

    # BOTH ARMS RUN, AND THEY RUN IN TURN, LEFT THEN RIGHT.
    #
    # Not at the same time, and that is a measurement gap being respected
    # rather than a preference. The two pads are 2 x OFFCENTRE apart across
    # the centreline and each arm reaches INBOARD to its own, so running them
    # together puts two 85 mm hands into one narrow strip directly in front of
    # the wearer. Whether those two paths collide with each other is a
    # dual-arm question nothing in this repository has measured for these
    # cells -- `avoid_collisions` sees the other arm, but the sweep that built
    # this layout solved one arm at a time. T1 stage 2 is the task that
    # deliberately runs both at once, in cells surveyed for exactly that.
    #
    # Each arm ends by returning to its park pose, so the clip does not end
    # with a hand sitting over the work.
    for arm in ("left", "right"):
        if not seq[arm]:
            continue
        back = _dense([seq[arm][-1], park_for(arm)])
        add(arm, back, CT.OPEN, at[arm][-1])

    nl, nr = len(seq["left"]), len(seq["right"])
    out, out_g, out_at = {}, {}, {}
    for arm, before, after in (("left", 0, nr), ("right", nl, 0)):
        p = park_for(arm)
        obj0 = at[arm][0] if at[arm] else [0.0, 0.0, T1_Z]
        objN = at[arm][-1] if at[arm] else obj0
        out[arm] = ([list(p)] * before + seq[arm] + [list(p)] * after)
        out_g[arm] = [CT.OPEN] * before + grip[arm] + [CT.OPEN] * after
        out_at[arm] = ([list(obj0)] * before + at[arm]
                       + [list(objN)] * after)
    _GRIP.clear()
    _GRIP.update(out_g)
    _GRIP_AT.clear()
    _GRIP_AT.update(out_at)
    return out


def park_for(arm):
    return CT.park(PARK_X if arm == "left" else -PARK_X)


def _resample(full, n):
    if n == len(full):
        return list(full)
    if n < len(full):
        step = len(full) / float(n)
        return [full[min(len(full) - 1, int(i * step))] for i in range(n)]
    raise RuntimeError(
        "T1's schedule is %d long and %d waypoints were asked for -- "
        "refusing to pad it. A schedule longer than the path it was built "
        "for opens the hand somewhere nobody chose." % (len(full), n))


def grip(n):
    if not _GRIP:
        build()
    return {a: _resample(_GRIP[a], n) for a in ("left", "right")}


def grip_at(n):
    if not _GRIP_AT:
        build()
    return {a: _resample(_GRIP_AT[a], n) for a in ("left", "right")}


def offcentre_mm():
    return {"left": abs(T1_PLANES[0][0]) * 1000.0,
            "right": abs(T1_PLANES[1][0]) * 1000.0}
