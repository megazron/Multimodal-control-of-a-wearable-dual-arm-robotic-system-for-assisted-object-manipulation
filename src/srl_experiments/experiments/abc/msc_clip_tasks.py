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

# ==========================================================================
# T1 WAS DELETED AND REBUILT ON 2026-08-17. Its layout, its approach and its
# path now live in `t1_task.py`, and the names below are RE-EXPORTS so the
# consumers that read them -- `clip_scene`, `mock_rgbd_camera`,
# `solve_observe_pose`, `vision_grasp`, `t1_instruction` -- keep one source.
#
# What changed, and why it could not be done by moving coordinates:
#
#   * the two pads STRADDLE THE CENTRELINE, one per arm, so the pair is
#     directly in front of the person instead of both being on one side;
#   * every object RESTS ON THE TABLE. The float gap is 0, not 120 mm;
#   * the task runs under `06_full_autonomy` ALONE, which is what makes the
#     first two possible: 06 does not pin the wrist, and the pinned wrist is
#     what forced the old layout to the table's edge and into the air.
#
# `archive/recordings/t1_20260817_deleted_and_rebuilt/NOTE.md` is the account.
# ==========================================================================
import t1_task as T1M                                        # noqa: E402

CUBE_M = T1M.CUBE_M
PLANE_COLOURS = T1M.PLANE_COLOURS
T1_CUBES = T1M.T1_CUBES
T1_PLANES = T1M.T1_PLANES
T1_Z = T1M.T1_Z
T1_PAIR = T1M.T1_PAIR
SLOT_DY = T1M.SLOT_DY
STANDOFF, LIFT = T1M.STANDOFF, T1M.LIFT
PARK_X = T1M.PARK_X

# STAGE 2 IS STAGE 1'S GEOMETRY, SINCE 2026-08-18. THE SIDE IS WHAT VARIES.
#
# It kept its own until now -- objects on the work plane 120 mm above the
# table, a PAIR of pads per side, the wrist pinned at the anchor -- because
# none of that had survived stage 1's rebuild and pulling it across would have
# meant placing cubes onto pads whose paths had never been walked. They have
# now been walked, per seed, so the divergence is closed rather than asserted:
# same table, same two pads at +/- 0.290, same row, same approach, same
# builder, and `test_t1_stages_agree` says so instead of the reverse.
#
# WHAT STAGE 2 STILL IS: the SIDE of each cube is part of the trial's draw, so
# the split between the arms varies 1/3, 2/2 or 3/1 and both arms always get
# work. The colour follows the side, because neither arm crosses the
# centreline -- 0 of 10 IK solutions at every cross-side pose, measured -- so
# a cube can only be delivered to the pad on its own side. Drawing colour and
# side independently would be drawing trials the rig cannot perform.
T1S2_Z = T1M.T1_Z
STAGE2_N_CUBES = T1M.STAGE2_N_CUBES
# ONE PAD PER ARM NOW, and the table is kept in this shape because several
# scripts and the scene read it. The left arm owns the blue pad and the right
# the green, which is the same statement as `t1_task.arm_for_pad`.
T1_PLANES_BY_ARM = {"left": [list(T1M.T1_PLANES[0])],
                    "right": [list(T1M.T1_PLANES[1])]}
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
# T1 -- pick and place, colour matched. THE BUILDER LIVES IN `t1_task.py`.
#
# It was 170 lines here and it is gone: the rebuilt task is two-armed, rests
# its objects on the table and commands its own approach orientation, none of
# which the old builder could express. `t1_task.build/grip/grip_at` are the
# whole of it and the TASKS entry below is the only thing that reaches them.
# --------------------------------------------------------------------------


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


def stage2_layout(seed=0, n_cubes=STAGE2_N_CUBES):
    """The drawn stage 2 scene: [(x, y, pad_index)]. ONE source, `t1_task`."""
    return T1M.stage2_layout(seed, n_cubes)


def _slot_for(pad_i, k, n_on_pad):
    """Where the k-th of n cubes lands on a pad, in the ROW."""
    px, _py = T1M.T1_PLANES[int(pad_i)]
    return [round(px + T1M.SLOT_OFFSETS[n_on_pad][k], 4), T1M.ROW_Y,
            round(T1M.T1_Z + T1M.PAD_T, 4)]


def stage2_targets(seed=0, n_cubes=STAGE2_N_CUBES, cells=None, n_per_arm=None):
    """{'cubes': {arm: [...]}, 'places': {arm: [...]}} for a seed.

    KEPT IN THIS SHAPE ON PURPOSE. `clip_scene`, `verify_objects_on_table`,
    `audit_task_spec` and `check_placement` all read it, and the point of this
    change is that stage 2 moves onto stage 1's geometry, not that every
    consumer of stage 2 is rewritten on the same day.
    """
    if cells is not None or n_per_arm is not None:
        raise TypeError(
            "stage2_targets() no longer takes cells or n_per_arm -- the SIDE "
            "is part of the draw and the columns are the measured ones in "
            "t1_task.CUBE_COLUMNS")
    lay = T1M.stage2_layout(seed, n_cubes)
    cubes = {"left": [], "right": []}
    for x, y, pad in lay:
        cubes[T1M.arm_for_pad(pad)].append([x, y, T1M.T1_Z])
    places = {}
    for arm in ("left", "right"):
        pad_i = 0 if arm == "left" else 1
        n = len(cubes[arm])
        places[arm] = [_slot_for(pad_i, k, n) for k in range(n)] if n else []
    return dict(cubes=cubes, places=places, seed=seed, n_cubes=n_cubes)


def _stage2_pad_index(arm, i, tgt=None):
    """Which pad the i-th cube of `arm` goes to.

    IT IS THE ARM'S OWN PAD, ALWAYS, and that is the measurement rather than a
    rule: the blue pad is left of the centreline, the green pad is right of
    it, and neither arm can cross. The old alternating rule belonged to a
    stage 2 that gave each arm a pad of each colour.
    """
    return 0 if arm == "left" else 1


# THE NAME `t1` STILL RESOLVES, and it is one line rather than a second
# implementation.
#
# T1's builder moved to `t1_task` in the 2026-08-17 rebuild and the alias went
# with it, so five scripts that had called `msc_clip_tasks.t1()` for months --
# including `audit_task_spec`, which is the pre-recording pass TASK_SPEC
# section 8 requires -- died with AttributeError the first time they were run
# afterwards. A module that owns a task's SPEC should still answer to the
# task's name.
t1 = T1M.build


def t1_stage2(seed=0, n_cubes=STAGE2_N_CUBES):
    """Both arms pick and place their own cubes, sides drawn from the seed.

    ONE BUILDER. This is `t1_task.build()` with a drawn layout instead of the
    fixed one, so stage 2 cannot acquire a dwell, a standoff or a slot spacing
    that stage 1 does not have.

    THE SEED IS AN ARGUMENT AND SOMETHING HAS TO PASS IT. `run_abc` has had a
    `--seed` flag all along and wrote it into the manifest, and the task
    layout was once built by `spec["build"]()` with no arguments -- so the
    seed was RECORDED and never READ, and every trial ran the same layout
    under a different seed number in its own metadata.
    """
    paths = T1M.build_stage2(seed, n_cubes=n_cubes)
    draw = stage2_targets(seed, n_cubes)
    _T1S2["grip"] = T1M._GRIP
    _T1S2["at"] = T1M._GRIP_AT
    _T1S2["targets"] = draw["cubes"]
    _T1S2["places"] = draw["places"]
    _T1S2["pads"] = {a: [list(p) for p in T1_PLANES_BY_ARM[a]]
                     for a in ("left", "right")}
    _T1S2["seed"], _T1S2["n_cubes"] = seed, n_cubes
    _T1S2["per_side"] = {a: len(v) for a, v in draw["cubes"].items()}
    _T1S2["layout"] = [list(c) for c in T1M.stage2_layout(seed, n_cubes)]
    return paths


_T1S2 = {}


def t1s2_grip(n):
    """The gripper schedule belonging to the LAST stage 2 path built."""
    if not T1M._GRIP:
        t1_stage2()
    return T1M.grip(n)


def t1s2_grip_at(n):
    if not T1M._GRIP_AT:
        t1_stage2()
    return T1M.grip_at(n)


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
        # BOTH ARMS, AND THE NAME SAYS SO. The scenario names the output
        # DIRECTORY and the caption, and a one-arm name on a two-arm task is
        # the same defect that once filed a left-arm clip as "S1_right_arm".
        scenario="S1_both_arms_centre",
        build=T1M.build,
        grip=lambda n: T1M.grip(n),
        # THE ORIENTATION THIS TASK SENDS. Read by `run_abc.set_orient()`,
        # which also rebuilds the finger-pad offset from it. No other task
        # declares one, so every other task still sends the pinned anchor --
        # HARD CONSTRAINT 1 is about the global constant and this is not it.
        orient=T1M.APPROACH,
        # 06 ONLY, AND IT IS ENFORCED RATHER THAN WRITTEN DOWN. The approach
        # this task commands is not the pinned anchor, so running it under a
        # teleop mode would compare an operator's pinned wrist against a
        # different geometry and call the difference a mode effect.
        modes=("06_full_autonomy",),
        width_mm=int(T1M.CUBE_M * 1000),
        grip_obj=[T1M.T1_CUBES[0][0], T1M.T1_CUBES[0][1], T1M.T1_Z],
        # PER WAYPOINT AND PER ARM: six objects and two arms, against a gate
        # written for one of each.
        grip_at=lambda n: T1M.grip_at(n),
        place_target=T1M.ee_for(
            [T1M.T1_PLANES[0][0], T1M.T1_PLANES[0][1],
             round(T1M.T1_Z + T1M.PLANE_T, 4)], "left"),
        expect="LEFT arm carries the two BLUE cubes to the blue pad and RIGHT "
               "carries the two GREEN cubes to the green pad, in turn. The "
               "pads sit either side of the centreline, directly in front of "
               "the person. Every object rests ON the table.",
        caveat="06_full_autonomy only. The cubes and their colours come from "
               "the CAMERA, not from this file; the destination pad is chosen "
               "by the colour the camera saw. Objects rest on the surface, so "
               "unlike every earlier T1 a drop IS a measurable outcome."),
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
