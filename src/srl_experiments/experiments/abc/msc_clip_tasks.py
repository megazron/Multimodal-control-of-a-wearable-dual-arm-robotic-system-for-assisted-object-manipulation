#!/usr/bin/env python3
"""Clip-recording paths for the FOUR MSc tasks: T0, T1, T2, T3.

Same shape as `clip_tasks.TASKS` so `record_abc_sweep.py` can drive these
without a second sweep -- the registry key, `build`, `grip`, `grip_obj`,
`place_target`, `expect` and `caveat` mean exactly what they mean there.

EVERY COORDINATE HERE IS ALREADY VERIFIED, and nothing new is invented:

    T0  task0.TARGETS_LEFT / TARGETS_RIGHT      0 failures, N=10, bench in
    T1  the option-4 layout                     0 failures, N=10, bench in
    T2  tasks.TASK_B["paths"], band 1.32-1.40   0 failures, N=10, both
                                                arm assignments
    T3  task3 BOX/METER and their present poses 0 failures, N=10, bench in

    -> recordings/baselines/msc_verification.json, 4234 IK calls, 0 failures

THE PATHS ARE EE POSES, not object poses.  `ee_for()` derives the wrist from
the object, because doing it the other way round is what once put the wrist
95 mm inside the bench.  T2's carry path is the exception and is already
declared in EE coordinates, because the tray is held rather than approached.

BOTH ARMS ALWAYS GET A LIST OF THE SAME LENGTH.  The recorder steps them
together, so an arm that is idle HOLDS its start pose rather than being
absent -- an absent arm reads as a crashed one in the clip.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import clip_tasks as CT                                      # noqa: E402
import task0 as T0                                           # noqa: E402
import task3 as T3M                                          # noqa: E402
import tasks as TSK                                          # noqa: E402

_dense = CT._dense
_hold = CT._hold
_sched = CT._sched
ee_for = CT.ee_for
park = CT.park

CUBE_M = 0.040
# T1's layout, from the option-4 search: 4 cubes + 2 planes, left arm, all
# verified over the full pick path at N=10 with the bench in the scene.
# THE ONE-TABLE LAYOUT. The bench is deleted -- it cost 0.250 m of forward
# reach and its top surface overlapped the reachable band by EXACTLY ZERO, so
# no object could ever sit on it. The objects now stand on slim pedestals on
# the table and the work moves OUT IN FRONT instead of against the wearer.
#
# Reach at object height with the table alone is 0.425 m (measured,
# scripts/measure_forward_reach.py). Cubes sit at y 0.19 and planes at 0.32,
# so the furthest the gripper visits is 0.37 -- 55 mm inside the limit, per
# the rule of staying at least 20 mm inside the last pose that passed N/N.
#
# CUBES CLEAR OF PLANES AT THE START. The old rows were 85 mm apart and
# cube_2 straddled BOTH planes (10 x 40 mm each), so the cubes began heaped
# on their own targets and "placed on the plane of its colour" had no visible
# before and after. The rows are now 130 mm apart, which no footprint spans.
# INSIDE THE SURVEYED BAND, which is |x| 0.30..0.70. The row used to start at
# |x| 0.26, which is 40 mm INBOARD of it, and the approach column for that
# cube failed 10 of T1's waypoints -- the grasp itself was fine, so a check
# that only tested grasp points passed it. Shifted outboard by 60 mm.
#
# 2026-08-15: T1 MOVES BACK TO THE LEFT ARM AND THE CUBES TO THE LEFT SIDE,
# and the reason the earlier move to the right was made no longer holds.
#
# That move was made on `t1_layout_options.json`: left arm 2/4 cubes and 1/2
# planes, right arm 4/4 and 2/2. Both figures were true of the layout they
# tested -- cubes at |x| 0.32..0.50, y 0.19 -- and both were taken with a
# survey that had the LEFT arm's wrist-to-pad offset applied to BOTH arms
# (survey_work_surface.py passed no `arm` to `ee_for`, fixed 2026-08-15), so
# the right arm's map was drawn 48.3 mm from where the right hand closes.
#
# Re-measured on the left arm with the clearance floor applied, the left arm
# has 193 cells that are reachable over the whole pick path AND keep 150 mm
# from the wearer, spanning x 0.425..1.000. The layout below sits inside them
# with margin. See recordings/baselines/work_surface_region.json.
#
# THE CUBES MOVED OUTBOARD, AND THAT IS THE CLEARANCE FLOOR TALKING. At the
# old |x| 0.32..0.50 the first two cubes put the arm 58.7 mm and 116.3 mm
# from the wearer's upper arm against a 150 mm floor -- measured over the
# shipped right-arm path, 70 of its 143 waypoints inside the floor, with
# zero IK failures. Every check this project had said that layout was clean,
# because they all ask /compute_ik and the SRDF excludes the wearer pairs a
# shoulder-mounted arm actually threatens.
#
# 60 mm pitch, unchanged: it is T0's measured minimum separation and it is
# what makes four cubes read as four objects.
T1_CUBES = [[0.560, 0.120], [0.620, 0.120], [0.680, 0.120],
            [0.740, 0.120]]
# MOVED 15 mm OUTBOARD IN y, 0.130 -> 0.145, TO MAKE ROOM FOR THE SLOTS.
# Two cubes share each plane and each now gets its own slot at +/-SLOT_DY in
# y. At the old centre those slots fell at y = 0.100 and 0.160, and the near
# one cost 5 clip-path waypoints at N=10 -- it sits on the front edge of the
# measured reachable band (y 0.05..0.20 at the grasp pose only, and the place
# path also has to clear a 0.10 m standoff above it). At 0.145 the slots are
# 0.115 and 0.175, both well inside.
# 0.300, not 0.320. At 0.320 the place-down point sits at EE
# (x, 0.255, 1.063) and BOTH planes failed there -- the lowest
# point of the descent, after the grasp had already succeeded.
# plane_1 pulled in from -0.500: its PLACE-DOWN point, EE
# (-0.483, 0.235, 1.063), was the single remaining failure of 80.
#
# THE PLANES ARE AS NEAR THE CENTRE AS THE LEFT ARM CAN WORK, AND THAT IS
# NOT THE CENTRE. The brief asks for them in the centre of the table,
# directly in front of the person. Measured, and the number is not close:
#
#   the left arm's innermost workable column, any forward distance   x = 0.250
#   ... that ALSO keeps the 150 mm wearer clearance floor            x = 0.425
#   what stops it going further in                    THE WEARER'S OWN ARM
#                                                     (23 mm inside, at 0.225)
#   x = 0.00 and x = +/-0.10, asked directly          NO y from 0.10 to 0.55
#
# So the centreline is not reachable at all, by either arm, at any forward
# distance, before the clearance floor is even applied. The planes sit at the
# innermost clearance-safe column plus the project's own 20 mm margin. They
# are 450 mm off centre, and that is a measurement about a shoulder-mounted
# arm rather than a layout preference. See recordings/baselines/
# centre_reach.json and section 2 of docs/TASK_SPEC.md.
#
# 160 mm apart, against a 140 mm plane, so a cube released over one cannot
# land on the other. Both slots (+/-SLOT_DY in y) stay inside rows the survey
# measured clear: y 0.210 and 0.270 against clear rows at 0.200 and 0.275.
T1_PLANES = [[0.450, 0.240], [0.610, 0.240]]
# THE SAME TWO PADS, ONE PAIR PER SIDE, AND WHY THERE HAS TO BE A PAIR.
#
# Stage 2 used to place onto coordinates drawn from the arm's own surveyed
# cells: arbitrary points, no colour, and nothing drawn on screen. So the two
# stages of one task had different targets and stage 2 had no colour rule at
# all, which is the thing T1 is about.
#
# They cannot literally share the two pads. Both sit at POSITIVE x, and the
# right arm's innermost reachable-and-clear column is |x| = 0.450 -- it cannot
# get to +0.450 at all, because that is 900 mm across the far side of a person.
# So "the same pads" is honoured the only way the geometry permits: the SAME
# pair, same colours, same spacing, same y, mirrored for the arm that has to
# reach them. A cube goes to the pad of ITS OWN COLOUR on the side it landed
# on, which is stage 1's rule with the side made random.
T1_PLANES_BY_ARM = {
    "left": [list(p) for p in T1_PLANES],
    "right": [[-p[0], p[1]] for p in T1_PLANES],
}
# Cube index -> pad index, and therefore colour. Stage 1 declares this in
# T1_PAIR below; stage 2 uses the same rule so the two stages agree about
# which cube is blue.
PLANE_COLOURS = ("blue", "green")
T1_Z = CT.BENCH_TOP + CUBE_M / 2.0
# T1 STAGE 1 RUNS ON THE LEFT ARM, on cubes on the LEFT side. See the note on
# T1_CUBES for why the earlier move to the right arm no longer holds: it was
# decided on a layout inside the clearance floor, using a survey that applied
# the left arm's pad offset to the right arm.
#
# The two arms are still NOT mirror images -- CLAUDE.md records
# |v_R - M v_L| = 1.3837 m, a parking asymmetry proven independent of the
# mount -- so this is not "the same layout, flipped". It is a layout derived
# for THIS arm from THIS arm's measured cells, and it is verified as such.
T1_ARM = "left"
STANDOFF, LIFT = 0.10, 0.08
# Where an arm with nothing to do waits, in |x|. Outboard of the wearer, in
# the measured clearance-safe region for either arm.
PARK_X = 0.60
# Fixed so the T0 clip is reproducible; recorded in the clip metadata.
T0_CLIP_SEED = 0


def _pad(short, n):
    """Hold the last pose until the list is n long."""
    return list(short) + [list(short[-1])] * (n - len(short))


# --------------------------------------------------------------------------
# T0 -- reaching. No objects, no grasp, so it runs in every mode.
# --------------------------------------------------------------------------
def t0():
    """Touch L1, L2, L3 then R1, R2, R3.

    The arms move in TURN, not together: T0's difficulty factor is how many
    targets are VISIBLE, not how many arms are moving, and a clip showing both
    arms sweeping at once would misrepresent the task.
    """
    # A SAMPLED trial, at a FIXED seed. TARGETS_LEFT/RIGHT are the A/B/C/D
    # Fitts CALIBRATION set; the study instrument is the randomised L1-L3 /
    # R1-R3 set, and the clip must show what a participant sees. Seed fixed so
    # the clip is reproducible -- the same seed gives the same spheres for
    # ever, which is the contract that makes any trial replayable.
    tgt, _ = T0.sample_trial(T0_CLIP_SEED)
    L = [tgt[k] for k in ("L1", "L2", "L3")]
    R = [tgt[k] for k in ("R1", "R2", "R3")]
    lp = _dense([L[0], L[1], L[2]]) + _hold(L[2], 4)
    rp = _dense([R[0], R[1], R[2]]) + _hold(R[2], 4)
    n = len(lp) + len(rp)
    left = lp + _hold(lp[-1], len(rp))
    right = _hold(R[0], len(lp)) + rp
    return {"left": left[:n], "right": right[:n]}


# --------------------------------------------------------------------------
# T1 -- pick and place, ONE ARM, and that arm is T1_ARM. This comment used to
# read "left only" long after the layout moved to the right arm, which is the
# same drift that had clip_scene watching the left gripper while the right one
# did the work. Nothing here names a side except through T1_ARM.
# --------------------------------------------------------------------------
T1_PAIR = {0: 0, 1: 1, 2: 0, 3: 1}
# The per-waypoint gripper schedule t1() builds alongside its path. It cannot
# be derived from the path length afterwards, which is what the old
# `_sched(n, "left", 5, n - 6, 40)` tried to do.
_T1_GRIP = []
# The OBJECT the arrival gate should measure against, per waypoint. A
# single-grasp task can name one; a four-cube task cannot.
_T1_GRIP_AT = []
# Half the separation between the two cubes that share a plane. 30 mm against
# a 40 mm cube leaves a 20 mm gap: two distinct objects, not a stack.
SLOT_DY = 0.030


def t1():
    """Pick each cube, place it on the plane of its own colour.

    Colour pairing: cubes 0 and 2 are blue -> plane 0; cubes 1 and 3 are
    green -> plane 1.  The pairing is declared here rather than inferred, so
    a WRONG-COLOUR placement is a scoreable event and not an ambiguity.

    THE GRIPPER SCHEDULE IS BUILT HERE, WITH THE PATH, AND THAT IS A FIX.
    It used to be `_sched(n, "left", 5, n - 6, 40)` -- close at waypoint 5,
    open six from the end -- which is ONE close and ONE open across all four
    pick-and-places. Measured on the first mode-06 re-record: the hand closed
    at the first cube, stayed shut through every carry, and let go once, 0.10 m
    ABOVE the last plane. All four cubes ended at (0.46, 0.130, 1.22) and the
    sweep reported "PLACED 0.189 m FROM TARGET" for cube_0, which was true and
    named the wrong cause. A four-cube task needs four closes and four opens,
    and only the code that lays out the path knows where they fall.

    The open happens AT the placement and before the withdrawal, held for
    three waypoints so the mock gripper has time to actually reach the open
    position -- a single waypoint is a command, not a release.
    """
    seq, grip, at = [], [], []
    g = CT.grip_for(40)

    def add(pts, state, obj):
        seq.extend(pts)
        grip.extend([state] * len(pts))
        at.extend([list(obj)] * len(pts))

    # A SLOT PER CUBE, SO TWO CUBES DO NOT LAND IN ONE PLACE.
    #
    # The pairing sends cubes 0 and 2 to the SAME plane and 1 and 3 to the
    # other, and both were placed at the plane's centre -- so the second cube
    # of each pair was delivered INSIDE the first. Each cube gets its own slot
    # on its plane, separated in y because the region is only 0.40 m wide in x
    # and the planes already sit at its inboard edge: measured, the reachable
    # band is x 0.30..0.70 and y 0.05..0.20, so +/-0.03 in y stays well inside
    # it while +/-0.035 in x would put the inboard slot at 0.265, outside.
    slot = {0: -SLOT_DY, 2: +SLOT_DY, 1: -SLOT_DY, 3: +SLOT_DY}
    for i, (cx, cy) in enumerate(T1_CUBES):
        pick = ee_for([cx, cy, T1_Z], T1_ARM)
        px, py = T1_PLANES[T1_PAIR[i]]
        py = round(py + slot[i], 4)
        place = ee_for([px, py, T1_Z], T1_ARM)
        cube_obj = [cx, cy, T1_Z]
        plane_obj = [px, py, T1_Z]
        add(_dense([[pick[0], pick[1], pick[2] + STANDOFF], pick]),
            CT.OPEN, cube_obj)
        # EIGHT, NOT FOUR. The close is gated on ARRIVAL, and the arm's lag
        # accumulates along the sequence: measured under MASTER_TELEOP, cubes
        # 0-2 closed with the pads 8.5-8.8 mm from the cube and the FOURTH
        # closed at 33.7 mm -- 3.7 mm outside the 30 mm capture window, so the
        # last cube was never picked up and the clip showed three of four
        # placements. The gate was right; the schedule ran out of waypoints
        # before the arm got there. This is the same lengthening T3 needed and
        # for the same reason, and it has to be in the TASK, identical across
        # modes, or the mode comparison is contaminated by the clip.
        # FOURTEEN, NOT EIGHT, AND THE MODE THAT NEEDED IT IS 02_vr_teleop.
        # Measured on the 2026-08-15 set, identical layout, all five modes:
        # 01, 03, 04 and 06 close on all four cubes at 0.0-0.1 mm, and 02
        # closes on three and misses cube_3 at 44.0 mm against a 30 mm gate,
        # having carried it 0.000 m. VR's whole run is ~35% longer -- grasp at
        # +17.0 s against +5.0 s, clip 39.7 s against 29.3 -- because of the
        # extra hop through vr_pose_mapper, and the lag is positional: the arm
        # is still catching up when the schedule opens the hand.
        #
        # IT GOES IN THE TASK, NOT IN THE MODE. A longer dwell for VR alone
        # would put the mode comparison at the mercy of the clip, which is the
        # one thing the mode-independence property exists to prevent. Same
        # reasoning as the 8 that replaced 4, and as T3's long holds.
        add(_hold(pick, 14), g, cube_obj)           # close ON the cube
        add(_dense([pick, [pick[0], pick[1], pick[2] + LIFT]]), g, cube_obj)
        add(_dense([[pick[0], pick[1], pick[2] + LIFT],
                    [place[0], place[1], place[2] + LIFT], place]),
            g, plane_obj)
        add(_hold(place, 2), g, plane_obj)          # arrive still holding
        add(_hold(place, 3), CT.OPEN, plane_obj)    # release, at the plane
        add(_dense([place, [place[0], place[1], place[2] + STANDOFF]]),
            CT.OPEN, plane_obj)
    _T1_GRIP[:] = grip
    _T1_GRIP_AT[:] = at
    other = "left" if T1_ARM == "right" else "right"
    # THE IDLE ARM PARKS OUTBOARD, AND IT DID NOT USED TO. park(+/-0.32) sits
    # 30.6 mm from the wearer's upper arm -- measured -- against the 150 mm
    # floor, so the arm that is doing NOTHING in this task spent the whole
    # clip inside the floor. An idle pose is the easiest thing in the task to
    # move and it had never been checked, because nothing checked clearance
    # at all. 0.60 is inside the measured clear region for both arms.
    sx = PARK_X if other == "left" else -PARK_X
    return {T1_ARM: seq, other: _hold(park(sx), len(seq))}


def t1_grip(n):
    """T1's schedule, built by t1() alongside the path it belongs to.

    TWO CALLERS, ONE OF WHICH IS ASKING A DIFFERENT QUESTION, and conflating
    them cost a recording. `record_abc_sweep` calls `grip(8)` before it drives
    anything, to work out WHICH ARM the close-up camera should follow -- it
    compares how many distinct values each arm's schedule has and picks the
    busier one. That is a probe, not a schedule request, and the first version
    of this function refused it: "T1 grip schedule is 125 long for a
    8-waypoint path", which is true, correct in spirit, and killed the run.

    So: at the real length, the real schedule. At a shorter length, the real
    schedule SAMPLED -- which preserves the only property the probe reads,
    that the left arm's schedule contains both states and the right arm's does
    not. At a longer length, still a refusal, because there is no honest way
    to invent gripper commands the path never asked for.
    """
    if not _T1_GRIP:
        t1()
    full = list(_T1_GRIP)
    if n == len(full):
        return {T1_ARM: full,
                ("left" if T1_ARM == "right" else "right"):
                    [CT.OPEN] * n}
    if n < len(full):
        step = len(full) / float(n)
        return {T1_ARM: [full[min(len(full) - 1, int(i * step))]
                         for i in range(n)],
                ("left" if T1_ARM == "right" else "right"):
                    [CT.OPEN] * n}
    raise RuntimeError(
        "T1 grip schedule is %d long and %d waypoints were asked for -- "
        "refusing to pad it. A schedule longer than the path it was built "
        "for opens the hand somewhere nobody chose." % (len(full), n))


def t1_grip_at(n):
    """Which OBJECT the arrival gate measures against, waypoint by waypoint.

    THE GATE WAS BUILT FOR A TASK WITH ONE OBJECT. `run_abc` holds a grip
    change pending until the finger pads reach `grip_obj` -- a single declared
    point -- which is right for A, B, C and T3 and wrong for four cubes and two
    planes. Measured: the first close fired at cube_0 correctly, and the first
    OPEN then waited for the pads to return to cube_0, which never happens
    because the arm has gone to the plane. So the hand never opened, every
    later cube was collected on the way past, and all four ended in the same
    place, still held. Four GRASPED events and no RELEASED.

    Closing is gated on the CUBE and opening on the PLANE, which is what the
    task means by "arrived".
    """
    if not _T1_GRIP_AT:
        t1()
    full = list(_T1_GRIP_AT)
    if n == len(full):
        return full
    if n < len(full):
        step = len(full) / float(n)
        return [full[min(len(full) - 1, int(i * step))] for i in range(n)]
    raise RuntimeError("T1 grip_at is %d long, %d asked for" % (len(full), n))


# --------------------------------------------------------------------------
# T2 -- coordinated carry. The COUPLING task.
# --------------------------------------------------------------------------
def t2(scenario="S2_full_lift"):
    """Both grippers on one rigid tray, 500 mm apart, lifted together.

    The two arms are driven from the SAME waypoint list with a fixed
    separation, so the clip shows what the task is: neither arm's pose is free
    given the other's.
    """
    path = TSK.TASK_B["paths"][scenario]
    dense = _dense(path)
    half = TSK.TRAY_SEP / 2.0
    lead = _hold([dense[0][0] + half, dense[0][1], dense[0][2]], 4)
    left = lead + [[p[0] + half, p[1], p[2]] for p in dense]
    right = _hold([dense[0][0] - half, dense[0][1], dense[0][2]], 4) + \
        [[p[0] - half, p[1], p[2]] for p in dense]
    left += _hold(left[-1], 6)
    right += _hold(right[-1], 6)
    return {"left": left, "right": right}


# --------------------------------------------------------------------------
# T3 -- circuit box and multimeter. Bimanual BY ROLE.
# --------------------------------------------------------------------------
def t3():
    """Right holds the box, left presents the meter, both return.

    The HOLD in the middle is where the task's own measurement lives -- arm
    drift while the wearer works -- so it is long in the clip on purpose.
    """
    box, boxp = (ee_for(T3M.BOX_OBJ, T3M.BOX_ARM),
                 ee_for(T3M.BOX_PRESENT, T3M.BOX_ARM))
    met, metp = (ee_for(T3M.METER_OBJ, T3M.METER_ARM),
                 ee_for(T3M.METER_PRESENT, T3M.METER_ARM))
    # THE HOLDS ARE LONG BECAUSE THE ARM HAS TO ARRIVE, not for effect.
    #
    # At 3 waypoints per hold the whole cycle was 38 waypoints -- about six
    # seconds -- and the follower is asynchronous, so the schedule opened the
    # hand while the arm was still climbing: measured, the circuit box was
    # carried 0.025 m of an 80 mm lift and released before it ever reached the
    # presented pose. The task's own distinctive measurement is ARM DRIFT
    # DURING THE HOLD, so a hold the arm never settles into measures nothing.
    r = _dense([[box[0], box[1], box[2] + STANDOFF], box]) + _hold(box, 8) + \
        _dense([box, boxp]) + _hold(boxp, 30) + _dense([boxp, box]) + \
        _hold(box, 8) + _dense([box, [box[0], box[1], box[2] + STANDOFF]])
    ll = _hold(met, 6) + \
        _dense([[met[0], met[1], met[2] + STANDOFF], met]) + _hold(met, 8) + \
        _dense([met, metp]) + _hold(metp, 24) + _dense([metp, met]) + \
        _hold(met, 8)
    n = max(len(r), len(ll))
    return {"left": _pad(ll, n), "right": _pad(r, n)}



# --------------------------------------------------------------------------
# T1 STAGE 2 -- BOTH ARMS AT ONCE, RANDOM POSITIONS. A DIFFERENT KIND.
# --------------------------------------------------------------------------
# STAGE 1 (above) is one arm, fixed layout. Stage 2 is both arms working
# SEPARATE cubes SIMULTANEOUSLY, at positions drawn from the surveyed region.
#
# THIS IS A THIRD KIND OF BIMANUAL AND THE WRITE-UP MUST NOT MERGE IT WITH THE
# OTHER TWO. The project already distinguishes:
#
#   T2   PHYSICAL COUPLING. One rigid body held at two points 500 mm apart.
#        Neither arm's pose is free given the other's -- a height difference
#        at one grip IS a tilt at the other. Remove an arm and the task is
#        IMPOSSIBLE, not slower. Tilt and separation are JOINT metrics: no
#        single arm can produce them.
#   T3   BIMANUAL BY ROLE. Two objects, two places, two jobs. Neither arm's
#        pose constrains the other's. One arm could do both sequentially --
#        slower and more awkward, but possible.
#
# Stage 2 is SIMULTANEITY: the same job, twice, at the same time. Neither arm
# constrains the other geometrically -- their reachable sets are disjoint, so
# they physically cannot interfere -- and one arm COULD do all of it
# sequentially. What it costs is ATTENTION, and that is the whole measurement:
# it is T3's independence with the roles made identical, so any difference
# from two single-arm stages is divided attention and not task difficulty.
#
# Removing an arm here HALVES THE WORK RATE. It does not make the task
# impossible (T2) and does not merely make it awkward (T3).
#
# POSITIONS COME ONLY FROM THE SURVEYED CELLS, and only from their CENTRES.
# recordings/baselines/work_surface_region.json holds every (x, y) that
# actually solved, and a cell centre is a point that was TESTED -- jittering
# inside a cell would be sampling between grid points that were not. The
# survey is the only thing standing between "random" and "unreachable", so it
# is used exactly as measured.
STAGE2_MIN_SEP_M = 0.12          # two cubes closer than this read as one pile


class RegionUnavailable(RuntimeError):
    """The surveyed region file is missing or empty.

    RAISES rather than falling back to a hardcoded box. A fallback would be a
    guess wearing the survey's name, and the entire point of stage 2 is that
    the positions are drawn from something measured.
    """


# THE INNERMOST COLUMN EACH ARM CAN ACTUALLY WORK, AT THE CURRENT HOME.
#
# `work_surface_region.json` was surveyed BEFORE the 2026-08-15 home change and
# its right-arm cells reach |x| = 0.400. Re-measured at the current home over
# the full path at N=10, the right arm's innermost column that is reachable AND
# clear of the 150 mm floor is **0.450**; the left's is 0.325, which is inboard
# of the survey's own 0.425 and so costs nothing. See
# recordings/baselines/centre_posture_down.json and the reconciliation in
# docs/system/findings.md, 2026-08-15.
#
# THIS IS NOT A MARGIN, IT IS A CORRECTION. Stage 2 seed 0 drew a right-arm
# cell from the stale part of the pool and its path put **26 waypoints inside
# the wearer floor, worst 0.1135 m**, with zero IK failures -- the exact
# signature of a region built on a boundary that has moved. Seeds 1 and 2 were
# clean, which is what made it look like one bad draw rather than a pool that
# includes cells the arm can no longer work.
INNERMOST_SAFE_X = {"left": 0.325, "right": 0.450}


def _region(path=None):
    """The surveyed cells, per arm, as [(x, y)]."""
    import json
    import os as _os
    p = path or _os.path.join(
        _os.path.dirname(_os.path.abspath(__file__)),
        "..", "..", "..", "..", "recordings", "baselines",
        "work_surface_region.json")
    p = _os.path.normpath(p)
    if not _os.path.exists(p):
        raise RegionUnavailable(
            "no surveyed region at %s -- run scripts/survey_work_surface.py. "
            "Stage 2 will not invent a region: 'random positions' that were "
            "never solved for is how a trial fails on the day." % p)
    d = json.load(open(p))
    # THE REGION MUST BE A FULL-PATH REGION, and this refusal is the whole
    # lesson of the first stage-2 verification. Sampling from a region
    # surveyed at the GRASP POSE ALONE looked right and was not: seed 1 gave
    # 5 waypoint failures out of 51 while four other seeds gave 0, because the
    # pick path also climbs to a 0.10 m standoff above the cell and descends
    # from it, and those poses were never tested. A cell that can be reached
    # is not a cell that can be worked.
    if not d.get("full_path"):
        raise RegionUnavailable(
            "%s was surveyed with the GRASP POSE ONLY (full_path=false). "
            "Stage 2 samples positions that must survive the whole pick path "
            "-- standoff, descend, lift -- and a grasp-pose region silently "
            "includes cells that fail it. Re-run: "
            "scripts/survey_work_surface.py --full-path" % p)
    # THE CLEARANCE FLOOR IS PART OF THE REGION NOW, and this refusal is the
    # second half of the same lesson as the full-path one above.
    #
    # `cells` is the IK-reachable set, and IK is not the whole constraint: the
    # SRDF permanently excludes torso/harness/backpack against each arm's
    # base, shoulder and half_arm_1 -- the pairs a shoulder-mounted arm
    # actually threatens -- so collision-aware IK returns poses with the tube
    # inside the wearer and a survey built on it inherits that silence.
    # Measured: 72 of the left arm's 265 IK-reachable cells and 68 of the
    # right's 314 are inside the 150 mm floor, one of them at -2.7 mm.
    #
    # Stage 2 draws the positions a participant will be sent to. It draws
    # from `clear_cells` or it does not draw at all.
    if not d.get("clearance_floor_m"):
        raise RegionUnavailable(
            "%s carries no clearance_floor_m, so its cells were chosen by IK "
            "alone. HARD CONSTRAINT 11: the clearance floor is the last thing "
            "between the arms and a person's chest, and a random position "
            "drawn from an IK-only region can put the arm inside it with "
            "every check passing. Re-run: scripts/measure_clearance_region.py "
            "then scripts/merge_work_surface_region.py" % p)
    cells = {a: [tuple(c) for c in d["clear_cells"].get(a, [])]
             for a in ("left", "right")}
    # DROP THE CELLS THE SURVEY STILL BELIEVES IN AND THE ARM CANNOT WORK.
    # See INNERMOST_SAFE_X: the file predates the home change and its right-arm
    # cells reach 0.400 against a re-measured 0.450.
    dropped = {}
    for a in ("left", "right"):
        lim = INNERMOST_SAFE_X[a]
        keep = [c for c in cells[a] if abs(c[0]) >= lim - 1e-9]
        dropped[a] = len(cells[a]) - len(keep)
        cells[a] = keep
    d = dict(d, inboard_limit_applied=INNERMOST_SAFE_X,
             cells_dropped_as_stale=dropped)
    for a, c in cells.items():
        if not c:
            raise RegionUnavailable(
                "the survey has NO cell for the %s arm that is both "
                "reachable and clear of the wearer, after the inboard limit "
                "of %.3f m is applied" % (a, INNERMOST_SAFE_X[a]))
    return cells, d


STAGE2_N_CUBES = 4               # across BOTH sides, not per side


def stage2_targets(seed, n_cubes=STAGE2_N_CUBES, cells=None,
                   min_sep=STAGE2_MIN_SEP_M, max_draws=500, n_per_arm=None):
    """Cubes drawn at random from the surveyed cells of BOTH sides.

    THE SIDE IS PART OF THE DRAW. Stage 2's brief is "cubes placed randomly
    on either the left or the right, both arms working", and the first
    version drew a FIXED two per arm -- so the side of every cube was decided
    in the source and only its position was random. A participant who learns
    "two on each side, always" is not doing the task stage 2 is for: the
    thing being measured is divided attention, and attention is only divided
    if you do not know in advance where the work will be.

    So the pool is the UNION of both arms' cells, each cube is drawn from it,
    and the arm that works a cube is the arm whose region it landed in. The
    counts per side therefore vary from trial to trial.

    BOTH ARMS MUST GET AT LEAST ONE, and that constraint is not a fudge of
    the randomness -- it is the task definition. "Both arms working" is what
    separates stage 2 from two consecutive stage 1s, and a draw that puts all
    four cubes on one side is a stage 1 with extra steps. The draw is
    REJECTED and retaken rather than nudged, so every layout that is returned
    is a uniform draw from the set of layouts that satisfy the task.

    It RAISES rather than returning fewer or quietly relaxing the separation.
    `n_per_arm` is accepted and refused by name: it is the old signature and
    a caller still passing it is asking for the fixed-side behaviour.
    """
    import random
    if n_per_arm is not None:
        raise TypeError(
            "stage2_targets() no longer takes n_per_arm -- the SIDE is part "
            "of the random draw now, so a per-arm count is not a thing that "
            "can be asked for. Pass n_cubes (total, across both sides).")
    if cells is None:
        cells, _ = _region()
    rng = random.Random("t1s2|%s|%d" % (seed, n_cubes))
    pool = [("left", x, y) for x, y in cells["left"]] + \
           [("right", x, y) for x, y in cells["right"]]
    for _attempt in range(max_draws):
        got, draws = [], 0
        while len(got) < n_cubes and draws <= max_draws:
            draws += 1
            arm, x, y = pool[rng.randrange(len(pool))]
            p = [round(x, 4), round(y, 4), T1_Z]
            if any(math.dist(p, q[1]) < min_sep for q in got):
                continue
            got.append((arm, p))
        if len(got) < n_cubes:
            break
        out = {"left": [p for a, p in got if a == "left"],
               "right": [p for a, p in got if a == "right"]}
        if not (out["left"] and out["right"]):
            continue
        # A PLACE TARGET PER CUBE, DRAWN FROM THE SAME MEASURED CELLS.
        #
        # This was one point per ARM -- the outermost x at the nearest y --
        # so both of an arm's cubes were delivered to the same coordinate,
        # one inside the other. Watched on the first stage-2 clips of the
        # 2026-08-15 set: `cube_left_0` and `cube_left_1` both end at
        # (0.675, 0.225) and both right-arm cubes at (-0.775, 0.200). It is
        # exactly the defect stage 1 fixed with SLOT_DY, which stage 2 never
        # inherited.
        #
        # And it made one cube per arm unmoveable: the target was the
        # outermost cube's OWN cell, so that cube was "placed" 28 mm from
        # where it started. A placement that short is not visible in a clip
        # and is not a placement in the data either.
        #
        # The targets are drawn the same way the cubes are -- from the arm's
        # own clearance-safe cells, at the same minimum separation from every
        # cube and from each other -- so a destination is a MEASURED cell and
        # not a formula over the draw.
        places, ok = {}, True
        for arm in ("left", "right"):
            pool_a = [c for c in cells[arm]]
            chosen, tries = [], 0
            while len(chosen) < len(out[arm]):
                tries += 1
                if tries > max_draws:
                    ok = False
                    break
                x, y = pool_a[rng.randrange(len(pool_a))]
                p = [round(x, 4), round(y, 4), T1_Z]
                if any(math.dist(p, q) < min_sep
                       for q in out[arm] + chosen):
                    continue
                chosen.append(p)
            if not ok:
                break
            places[arm] = chosen
        if ok:
            return {"cubes": out, "places": places}
    raise RegionUnavailable(
        "could not draw %d cubes at %.0f mm separation with at least one on "
        "each side, from %d surveyed cells, in %d attempts. Do not relax the "
        "separation or the both-sides rule silently -- either is a change to "
        "what stage 2 measures." % (n_cubes, min_sep * 1000, len(pool),
                                    max_draws))


def _stage2_pad_index(arm, i, tgt):
    """Which coloured pad this cube belongs on: 0 = blue, 1 = green.

    The colour follows the cube's position in the WHOLE draw, not its position
    within one arm's share, so a four-cube draw is always two blue and two
    green however the sides fall. Anything keyed on the per-arm index would
    give three blue and one green whenever the split is 3/1, which is a
    different task from the one T1 declares.
    """
    order = [(a, k) for a in ("left", "right") for k in range(len(tgt[a]))]
    try:
        n = order.index((arm, i))
    except ValueError:                                          # pragma: no cover
        n = i
    return T1_PAIR[n % len(T1_PAIR)]


def t1_stage2(seed=0, n_cubes=STAGE2_N_CUBES):
    """Both arms pick and place their own cubes AT THE SAME TIME.

    The two arms are stepped together from one waypoint list each, so the
    clip shows genuine simultaneity rather than one arm waiting.

    THE SEED IS AN ARGUMENT AND SOMETHING HAS TO PASS IT. `run_abc` has had a
    `--seed` flag all along and wrote it into the manifest, and the task
    layout was built by `spec["build"]()` with no arguments -- so the seed was
    RECORDED and never READ, and every trial of stage 2 ran the same layout
    under a different seed number in its own metadata. That is the "feature
    present but does nothing" row of CLAUDE.md's table, and it is worse than
    absent here: the manifest asserted a randomisation that had not happened.
    """
    draw = stage2_targets(seed, n_cubes)
    tgt, plc = draw["cubes"], draw["places"]
    paths, grips, ats = {}, {}, {}
    g = CT.grip_for(40)
    for ai, arm in enumerate(("left", "right")):
        seq, grip, at = [], [], []

        def add(pts, state, obj, _s=seq, _g=grip, _a=at):
            _s.extend(pts)
            _g.extend([state] * len(pts))
            _a.extend([list(obj)] * len(pts))

        for i, cube in enumerate(tgt[arm]):
            pick = ee_for(cube, arm)
            # THE PLACE IS THE COLOURED PAD, THE SAME ONE STAGE 1 USES.
            #
            # It used to be `plc[arm][i]` -- a coordinate drawn from the arm's
            # own surveyed cells. That made stage 2 a different task from
            # stage 1: arbitrary targets, no colour rule, and nothing on
            # screen to place ONTO. T1 is "blue cube to blue pad, green cube
            # to green pad" and stage 2 is the same task with the SIDE
            # randomised, so the pads travel with it. The pad pair is mirrored
            # for the right arm because the right arm cannot reach +0.450; see
            # T1_PLANES_BY_ARM.
            #
            # The colour of a cube is its index parity, exactly as T1_PAIR
            # declares for stage 1, so the two stages cannot disagree about
            # which cube is blue.
            pad_i = _stage2_pad_index(arm, i, tgt)
            px, py = T1_PLANES_BY_ARM[arm][pad_i]
            place_obj = [px, py, T1_Z]
            _ = plc
            place = ee_for(place_obj, arm)
            add(_dense([[pick[0], pick[1], pick[2] + STANDOFF], pick]),
                CT.OPEN, cube)
            add(_hold(pick, 14), g, cube)   # same dwell as stage 1; see t1()
            add(_dense([pick, [pick[0], pick[1], pick[2] + LIFT]]), g, cube)
            add(_dense([[pick[0], pick[1], pick[2] + LIFT],
                        [place[0], place[1], place[2] + LIFT], place]),
                g, place_obj)
            add(_hold(place, 2), g, place_obj)
            add(_hold(place, 3), CT.OPEN, place_obj)
            add(_dense([place, [place[0], place[1], place[2] + STANDOFF]]),
                CT.OPEN, place_obj)
        paths[arm], grips[arm], ats[arm] = seq, grip, at

    # BOTH ARMS GET A LIST OF THE SAME LENGTH -- the recorder steps them
    # together, and an arm that runs out is an arm that looks crashed.
    n = max(len(paths["left"]), len(paths["right"]))
    for arm in ("left", "right"):
        paths[arm] = _pad(paths[arm], n)
        grips[arm] = grips[arm] + [CT.OPEN] * (n - len(grips[arm]))
        ats[arm] = ats[arm] + [ats[arm][-1]] * (n - len(ats[arm]))
    # THE SEED TRAVELS WITH THE LAYOUT. `clip_scene` draws the cubes and
    # `run_abc` writes the manifest, and if they build the layout separately
    # they must agree about which draw they are talking about. Storing the
    # seed here lets both read it back from the module rather than each
    # remembering a default -- which is how the picture and the data came to
    # be able to disagree about where the cubes were.
    _T1S2["grip"], _T1S2["at"], _T1S2["targets"] = grips, ats, tgt
    _T1S2["places"] = {
        a: [[T1_PLANES_BY_ARM[a][_stage2_pad_index(a, k, tgt)][0],
             T1_PLANES_BY_ARM[a][_stage2_pad_index(a, k, tgt)][1], T1_Z]
            for k in range(len(tgt[a]))] for a in ("left", "right")}
    _T1S2["pads"] = {a: [list(p) for p in T1_PLANES_BY_ARM[a]]
                     for a in ("left", "right")}
    _T1S2["seed"], _T1S2["n_cubes"] = seed, n_cubes
    _T1S2["per_side"] = {a: len(v) for a, v in tgt.items()}
    return paths


_T1S2 = {}


def t1s2_grip(n):
    if not _T1S2:
        t1_stage2()
    g = _T1S2["grip"]
    if len(g["left"]) == n:
        return {a: list(v) for a, v in g.items()}
    step = len(g["left"]) / float(n)
    return {a: [v[min(len(v) - 1, int(i * step))] for i in range(n)]
            for a, v in g.items()}


def t1s2_grip_at(n):
    if not _T1S2:
        t1_stage2()
    a = _T1S2["at"]
    if len(a["left"]) == n:
        return {k: [list(p) for p in v] for k, v in a.items()}
    step = len(a["left"]) / float(n)
    return {k: [list(v[min(len(v) - 1, int(i * step))]) for i in range(n)]
            for k, v in a.items()}


TASKS = {
    "t0": dict(
        name="target reaching",
        scenario="D3_three_targets",
        build=t0,
        # No grasp anywhere in T0, but the SAME dict shape as every other
        # task -- a bare list would be a second schedule format and the
        # recorder would have to branch on the task.
        grip=lambda n: {"left": [CT.OPEN] * n, "right": [CT.OPEN] * n},
        width_mm=0,
        grip_obj=None,
        place_target=None,
        expect="LEFT arm touches L1, L2, L3 in turn while RIGHT holds still, "
               "then RIGHT touches R1, R2, R3 while LEFT holds. No object, "
               "no gripper motion at any point.",
        caveat="T0 is the CONTROL for the pinned-wrist result as well as the "
               "baseline: it has no grasp, so the VR-vs-master gap predicted "
               "for grasping must be ABSENT here."),
    "t1": dict(
        name="pick and place, colour matched",
        # DERIVED FROM T1_ARM, for the same reason `expect` is. This was the
        # literal "S1_right_arm" and it survived the move back to the left
        # arm by two minutes of recording: the scenario names the output
        # DIRECTORY and the caption, so a clip of the left arm working was
        # being filed, captioned and counted as a right-arm scenario.
        scenario="S1_%s_arm" % T1_ARM,
        build=t1,
        grip=lambda n: t1_grip(n),
        width_mm=40,
        grip_obj=[T1_CUBES[0][0], T1_CUBES[0][1], T1_Z],
        # PER WAYPOINT, because this task has six objects and the gate was
        # written for one. See t1_grip_at().
        grip_at=lambda n: t1_grip_at(n),
        place_target=ee_for([T1_PLANES[0][0], T1_PLANES[0][1], T1_Z],
                            T1_ARM),
        # DERIVED FROM T1_ARM, never written out. The literal used to say
        # "LEFT arm descends to each cube" while T1_ARM was "right", so the
        # sweep printed and the write-up would have quoted the wrong arm for
        # the whole task.
        expect="%s arm descends to each cube, closes on 40 mm, lifts, "
               "carries to the plane of the SAME COLOUR and opens. %s arm "
               "parked throughout. Four cubes, two planes."
               % (T1_ARM.upper(),
                  ("LEFT" if T1_ARM == "right" else "RIGHT")),
        caveat="Objects are FIXTURED, not resting (option 4): a cube that "
               "cannot fall cannot be dropped, so `drops` is not a "
               "measurable outcome in this task."),
    "t1s2": dict(
        name="pick and place, both arms at once",
        scenario="S2_both_arms_random",
        build=t1_stage2,
        # DECLARED, not inferred. `run_abc` passes --seed only to a task that
        # says it is seeded, so the one task whose layout is random is the
        # one task whose seed is read.
        seeded=True,
        grip=lambda n: t1s2_grip(n),
        grip_at=lambda n: t1s2_grip_at(n),
        width_mm=40,
        grip_obj=None,
        place_target=None,
        expect="BOTH arms pick and place their own cubes AT THE SAME TIME, "
               "from positions drawn at random from the surveyed reachable "
               "region. Two cubes per arm, never in the other arm's cells.",
        caveat="SIMULTANEITY, and a third kind of bimanual: the same job "
               "twice at once. T2 is physical COUPLING (remove an arm and it "
               "is impossible); T3 is bimanual BY ROLE (remove an arm and it "
               "is slower); here removing an arm HALVES THE WORK RATE. The "
               "cost measured is divided attention, not task difficulty."),
    "t2": dict(
        name="coordinated carry",
        scenario="S2_full_lift",
        build=t2,
        # Both grippers are already closed on the tray and stay closed: the
        # task is the CARRY, not the grasp.
        # Both grippers are already closed on the tray and STAY closed: the
        # task is the carry, not the grasp. Closed to the tray's grip block
        # width, not fully, so the clip shows fingers on an object.
        grip=lambda n: {"left": [CT.grip_for(30)] * n,
                        "right": [CT.grip_for(30)] * n},
        width_mm=0,
        grip_obj=None,
        place_target=None,
        # THE HEIGHTS ARE READ FROM THE PATH, NOT WRITTEN OUT. This said
        # "from z=1.32 to z=1.40" long after the band was re-measured and the
        # path taken up to 1.60 -- every 20 mm step from 1.32 to 1.70 passes,
        # and the path stops 100 mm inside the last one that did. A caption
        # that describes a lift 200 mm shorter than the one on screen is the
        # same class of error as T1's `expect` naming the wrong arm.
        expect="BOTH arms hold a rigid tray %.0f mm apart and lift together "
               "from z=%.2f to z=%.2f, staying level. The ball stays on the "
               "tray. Tilt past %.1f deg would drop it."
               % (TSK.TRAY_SEP * 1000,
                  min(p[2] for p in TSK.TASK_B["paths"]["S2_full_lift"]),
                  max(p[2] for p in TSK.TASK_B["paths"]["S2_full_lift"]),
                  TSK.TASK_B["fail_tilt_deg"]),
        caveat="PHYSICAL COUPLING -- remove one arm and the task is "
               "impossible, not slower. Band re-spec'd 2026-08-11: the old "
               "1.10-1.30 was inside the bench slab."),
    "t3": dict(
        name="circuit box and multimeter",
        scenario="S1_measure_cycle",
        build=t3,
        # BOTH arms grip in T3 -- right on the box, left on the meter -- and
        # _sched only drives one, so the two schedules are composed. A
        # single-arm schedule here would show the meter being carried by an
        # open hand.
        grip=lambda n: {
            # 50 mm, not 110: the box is 110 mm deep and the 2F-85 spans 85,
            # so 110 is not a grip this hand can make. It takes the box across
            # its 50 mm height.
            "right": _sched(n, "right", 4, n - 5, 50)["right"],
            "left": _sched(n, "left", 12, n - 8, 30)["left"]},
        width_mm=50,
        grip_obj=T3M.BOX_OBJ,
        # PER ARM, and this is the same defect T1 had wearing different
        # clothes. The arrival gate holds a grip change until the pads reach
        # `grip_obj` -- ONE point, the circuit box -- and T3's LEFT arm never
        # goes near the box, so the meter's close never fired: measured,
        # multimeter carried 0.000 m with no GRASPED event, in the task whose
        # left arm exists to present it. Each arm is gated on its own object.
        grip_at=lambda n: {"left": [list(T3M.METER_OBJ)] * n,
                           "right": [list(T3M.BOX_OBJ)] * n},
        # WHERE THE OBJECT MUST END UP IS WHERE IT STARTED. T3 RETURNS both
        # objects to the bench -- that is a declared verb in task3.commands()
        # and the last leg of the path. Comparing the final position against
        # BOX_PRESENT scored the box 80 mm out for having been put back
        # correctly, which is PRESENT_LIFT_M to the millimetre.
        place_target=ee_for(T3M.BOX_OBJ, T3M.BOX_ARM),
        expect="RIGHT arm picks up the circuit box and holds it raised while "
               "LEFT presents the multimeter; both hold still for the "
               "measurement; both return their object to the bench.",
        caveat="Bimanual BY ROLE, not coupling -- two objects, two places, "
               "two arms. T2 is the coupling task."),
}

ORDER = ("t0", "t1", "t1s2", "t2", "t3")


if __name__ == "__main__":
    for k in ORDER:
        d = TASKS[k]
        p = d["build"]()
        g = d["grip"](len(p["left"]))
        print("%-4s %-32s left %3d  right %3d  grip L%3d R%3d"
              % (k, d["name"], len(p["left"]), len(p["right"]),
                 len(g["left"]), len(g["right"])))
        assert len(p["left"]) == len(p["right"]), k
        assert len(g["left"]) == len(p["left"]), k
        assert len(g["right"]) == len(p["right"]), k
    print("all four build, both arms equal length, grip schedules aligned")
