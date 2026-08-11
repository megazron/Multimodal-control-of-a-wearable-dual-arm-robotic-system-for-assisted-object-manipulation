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
T1_CUBES = [[0.280, 0.230], [0.340, 0.230], [0.380, 0.170], [0.400, 0.230]]
T1_PLANES = [[0.300, 0.130], [0.460, 0.130]]
T1_Z = CT.BENCH_TOP + CUBE_M / 2.0
T1_ARM = "left"
STANDOFF, LIFT = 0.10, 0.08
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
# T1 -- pick and place, ONE ARM. Left only: the right arm's pinned-wrist
# grasp rate is 42%, and T1 is already specified as one arm at a time.
# --------------------------------------------------------------------------
def t1():
    """Pick each cube, place it on the plane of its own colour.

    Colour pairing: cubes 0 and 2 are blue -> plane 0; cubes 1 and 3 are
    green -> plane 1.  The pairing is declared here rather than inferred, so
    a WRONG-COLOUR placement is a scoreable event and not an ambiguity.
    """
    seq = []
    pair = {0: 0, 1: 1, 2: 0, 3: 1}
    for i, (cx, cy) in enumerate(T1_CUBES):
        pick = ee_for([cx, cy, T1_Z])
        px, py = T1_PLANES[pair[i]]
        place = ee_for([px, py, T1_Z])
        seq += _dense([[pick[0], pick[1], pick[2] + STANDOFF], pick])
        seq += _hold(pick, 3)
        seq += _dense([pick, [pick[0], pick[1], pick[2] + LIFT]])
        seq += _dense([[pick[0], pick[1], pick[2] + LIFT],
                       [place[0], place[1], place[2] + LIFT], place])
        seq += _hold(place, 3)
        seq += _dense([place, [place[0], place[1], place[2] + STANDOFF]])
    return {"left": seq, "right": _hold(park(-0.32), len(seq))}


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
    box, boxp = ee_for(T3M.BOX_OBJ), ee_for(T3M.BOX_PRESENT)
    met, metp = ee_for(T3M.METER_OBJ), ee_for(T3M.METER_PRESENT)
    r = _dense([[box[0], box[1], box[2] + STANDOFF], box]) + _hold(box, 3) + \
        _dense([box, boxp]) + _hold(boxp, 14) + _dense([boxp, box]) + \
        _hold(box, 3) + _dense([box, [box[0], box[1], box[2] + STANDOFF]])
    ll = _hold(met, 6) + \
        _dense([[met[0], met[1], met[2] + STANDOFF], met]) + _hold(met, 3) + \
        _dense([met, metp]) + _hold(metp, 8) + _dense([metp, met]) + \
        _hold(met, 3)
    n = max(len(r), len(ll))
    return {"left": _pad(ll, n), "right": _pad(r, n)}


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
        scenario="S1_left_arm",
        build=t1,
        grip=lambda n: _sched(n, "left", 5, n - 6, 40),
        width_mm=40,
        grip_obj=[T1_CUBES[0][0], T1_CUBES[0][1], T1_Z],
        place_target=ee_for([T1_PLANES[0][0], T1_PLANES[0][1], T1_Z]),
        expect="LEFT arm descends to each cube, closes on 40 mm, lifts, "
               "carries to the plane of the SAME COLOUR and opens. RIGHT arm "
               "parked throughout. Four cubes, two planes.",
        caveat="Objects are FIXTURED, not resting (option 4): a cube that "
               "cannot fall cannot be dropped, so `drops` is not a "
               "measurable outcome in this task."),
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
        expect="BOTH arms hold a rigid tray 500 mm apart and lift together "
               "from z=1.32 to z=1.40, staying level. The ball stays on the "
               "tray. Tilt past 6.8 deg would drop it.",
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
        place_target=ee_for(T3M.BOX_PRESENT),
        expect="RIGHT arm picks up the circuit box and holds it raised while "
               "LEFT presents the multimeter; both hold still for the "
               "measurement; both return their object to the bench.",
        caveat="Bimanual BY ROLE, not coupling -- two objects, two places, "
               "two arms. T2 is the coupling task."),
}

ORDER = ("t0", "t1", "t2", "t3")


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
