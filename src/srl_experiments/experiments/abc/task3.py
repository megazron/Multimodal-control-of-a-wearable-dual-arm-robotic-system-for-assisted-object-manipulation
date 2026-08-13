#!/usr/bin/env python3
"""T3 -- CIRCUIT BOX AND MULTIMETER. The realistic supernumerary scenario.

The only task in the set where the wearer does SKILLED WORK WITH THEIR OWN
HANDS OCCUPIED, which is the actual premise of wearable extra arms. The robot
holds the circuit box steady with one arm and presents the multimeter where it
can be read with the other; the wearer's hands hold the probes and take the
measurement; the robot then returns both objects.

WHY THE BOX MOVED TO |x| = 0.54, AND IT IS NOT A PREFERENCE
-----------------------------------------------------------
The task needs the box HELD -- grasped and kept still -- which the previous
scenes never did: `task_b()` places a part ON TOP of the box and `task_c()`
probes its top, and neither grasps it. So "can an arm hold the circuit box"
had never been asked.

Asked properly, at the near-edge pose the file's own rule requires (targeting
an object's CENTRE puts the wrist inside its footprint, because the wrist
trails the pads by 95 mm), with both arm assignments and N=10, bench in scene:

    pose                              pinned wrist    top-down
    box centre  (the WRONG pose)      NEITHER         right
    near-edge, box mid-height         NEITHER         right
    near-edge, box top                NEITHER         right
    B_PLACE     (what task_b commands)   right        right
    C_PROBE_ON  (what task_c commands)   right        right

So placing onto the box and probing it work with a pinned wrist at x = -0.35;
HOLDING it does not. A sweep over 156 candidate (x, y, z) hold poses found 37
that a pinned wrist can reach -- **all of them right-arm, all at |x| >= 0.51**:

    x in [-0.60, -0.51],  y in [0.13, 0.23],  z = bench top + h/2 or + h

That is 190 mm further outboard than the old x = -0.35, and it is the pinned
wrist's own graspable strip for the right arm (|x| 0.48-0.57), measured
independently for T1. The box is therefore where the RIGHT ARM can hold it,
not where it used to sit.

THE WEARER STILL HAS TO REACH IT, and that part is geometry plus a judgement.
Geometry: the wearer's shoulder sits at |x| ~ 0.21, so a box at
(-0.54, 0.17, 1.13) is about 0.41 m from that shoulder -- well inside a
seated arm's reach. Judgement, and it belongs to the experimenter, not to
this file: the box is at the wearer's SIDE rather than in front of them, so
the working posture must be checked with a person before the first session.
If it is not comfortable, the finding is that no position satisfies both the
robot's pinned-wrist reach and a natural working posture -- and that is a
result about the platform, not a layout failure.

FORCING THE COORDINATION TO SHOW
--------------------------------
"If the two people work silently the coordination measure has no data." The
measurement points are therefore chosen so that ONE presentation cannot serve
all of them:

  * P1 and P2 sit on the near face, reachable at the presented angle;
  * P3 sits on the FAR face, which the wearer cannot reach without the box
    being rotated or moved;
  * P4 requires the meter display turned toward the wearer, which the initial
    presentation deliberately does not do.

So at least two repositioning requests are structurally required. A trial with
zero requests is evidence the participants ignored the protocol, not evidence
of good coordination, and `repositioning_requests` is logged with timestamps
and a satisfied-first-attempt flag so the two can be told apart.
"""

import math

# --------------------------------------------------------------------------
# Geometry. Every figure below is measured; the provenance is in the docstring
# and in recordings/baselines/t3_box_hold_sweep.json.
# --------------------------------------------------------------------------
BENCH_TOP = 1.100

BOX_SIZE = (0.170, 0.110, 0.050)        # w, d, h -- the circuit box
# 30 x 70 x 45 mm. RESPEC'D 2026-08-10 from 50 x 90 x 130, which bound three
# ways at once: 8.0 mm pose error, a 17.5 mm capture half-window, and it was
# the only object missing the sigma <= 2 mm the scene fingerprint needs. The
# new one gives 1.7 mm, 27.5 mm and passes.
METER_SIZE = (0.030, 0.070, 0.045)

# HOLD pose for the box: right arm, inside the pinned wrist's own strip.
BOX_XY = (-0.540, 0.170)
BOX_Z = BENCH_TOP + BOX_SIZE[2] / 2.0                    # mid-height grip
BOX_OBJ = [BOX_XY[0], BOX_XY[1], round(BOX_Z, 4)]
BOX_ARM = "right"

# PRESENT pose for the meter: left arm, inside ITS pinned strip
# (|x| 0.30-0.39, y 0.15-0.23, measured for T1).
METER_XY = (0.340, 0.190)
METER_Z = BENCH_TOP + METER_SIZE[2] / 2.0
METER_OBJ = [METER_XY[0], METER_XY[1], round(METER_Z, 4)]
METER_ARM = "left"

# Where each object is presented once picked up: lifted clear of the bench so
# the wearer can get a hand and a probe in. Same x/y, raised.
PRESENT_LIFT_M = 0.080
BOX_PRESENT = [BOX_OBJ[0], BOX_OBJ[1], round(BOX_OBJ[2] + PRESENT_LIFT_M, 4)]
METER_PRESENT = [METER_OBJ[0], METER_OBJ[1],
                 round(METER_OBJ[2] + PRESENT_LIFT_M, 4)]

# --------------------------------------------------------------------------
# The measurement points, designed to FORCE repositioning
# --------------------------------------------------------------------------
MEASUREMENT_POINTS = [
    dict(id="P1", face="near", offset_mm=(-40, 0),
         served_by_initial_presentation=True),
    dict(id="P2", face="near", offset_mm=(+40, 0),
         served_by_initial_presentation=True),
    dict(id="P3", face="far", offset_mm=(0, +55),
         served_by_initial_presentation=False,
         needs="the box rotated or moved -- the far face is not reachable "
               "past the box at the presented angle"),
    # ON THE BOX, like the other three. The clip drew this one on the
    # MULTIMETER for a while, which contradicts the spec -- "one circuit box
    # with FOUR measurement points, one multimeter" -- and misread the line
    # below: the meter's DISPLAY has to face the wearer to READ this point,
    # which is a fact about the meter, not about where the point is. A test
    # point is on the board being probed.
    dict(id="P4", face="near", offset_mm=(0, -55),
         served_by_initial_presentation=False,
         needs="the METER display turned toward the wearer; the initial "
               "presentation deliberately does not"),
]
MIN_EXPECTED_REQUESTS = sum(
    0 if p["served_by_initial_presentation"] else 1 for p in MEASUREMENT_POINTS)


def commands(seed=None):
    """The TaskCommands for one T3 cycle.

    Built before any mode is chosen and naming none -- the architecture
    principle. RETURN is a separate verb from PLACE_OBJECT because the return
    leg is scored on its own and its destination is not the participant's
    choice.
    """
    from srl_experiments.task_actions import TaskCommand, Verb
    return [
        TaskCommand(Verb.PICK_OBJECT, BOX_ARM, "circuit_box", pose=BOX_OBJ),
        TaskCommand(Verb.PLACE_OBJECT, BOX_ARM, "circuit_box", pose=BOX_OBJ,
                    to_pose=BOX_PRESENT),
        TaskCommand(Verb.PICK_OBJECT, METER_ARM, "multimeter",
                    pose=METER_OBJ),
        TaskCommand(Verb.PLACE_OBJECT, METER_ARM, "multimeter",
                    pose=METER_OBJ, to_pose=METER_PRESENT),
        # the wearer measures here; the robot holds both, and ARM DRIFT over
        # this hold is one of the task's own metrics
        TaskCommand(Verb.RETURN, METER_ARM, "multimeter", pose=METER_PRESENT,
                    to_pose=METER_OBJ),
        TaskCommand(Verb.RETURN, BOX_ARM, "circuit_box", pose=BOX_PRESENT,
                    to_pose=BOX_OBJ),
    ]


def poses_to_verify():
    """Every declared pose, as (arm, xyz), for the verifier."""
    return [(BOX_ARM, BOX_OBJ), (BOX_ARM, BOX_PRESENT),
            (METER_ARM, METER_OBJ), (METER_ARM, METER_PRESENT)]


TASK_3 = dict(
    key="3",
    name="circuit_box_and_multimeter",
    role="the realistic supernumerary scenario; the only task where the "
         "wearer does skilled work with their own hands occupied",
    bimanual="YES, but by ROLE not by coupling: the two arms hold two "
             "different objects at two places. Neither arm's pose constrains "
             "the other's, so this is not physical coupling -- T2 is.",
    # SAID PLAINLY, because the two claims are easy to merge and only one of
    # them is strong. T3 uses both arms because the TASK gives them different
    # jobs at the same time, not because the physics ties them together: one
    # arm could hold the box, put it down, pick up the meter and present it,
    # and the task would be slower and more awkward but still possible.
    # T2 could not be done that way at all -- a single arm cannot hold a rigid
    # body at two points 500 mm apart, and no amount of time changes that.
    #
    # So: T2 supports "removing an arm makes it impossible"; T3 supports
    # "removing an arm makes it worse". Both are worth measuring. Reporting
    # them under one word is not.
    bimanual_kind="by ROLE -- the weaker sense. T2 is the coupling task.",
    arms_used="right holds the box, left presents the meter",
    objects=dict(
        circuit_box=dict(size=BOX_SIZE, pose=BOX_OBJ, arm=BOX_ARM,
                         low_voltage_demonstration_circuit=True),
        multimeter=dict(size=METER_SIZE, pose=METER_OBJ, arm=METER_ARM),
    ),
    measurement_points=MEASUREMENT_POINTS,
    min_expected_repositioning_requests=MIN_EXPECTED_REQUESTS,
    repeats=2,
    metrics=("pickup_success", "placement_success", "completion_time_s",
             "human_interruption_time_s", "times_human_stopped_measuring",
             "repositioning_requests", "requests_satisfied_first_attempt",
             "arm_drift_mm_left", "arm_drift_mm_right",
             "drops", "corrective_actions", "trajectory_length_m",
             "collisions", "near_collisions", "overall_success",
             "subjective_usefulness"),
    # The hold is where this task's distinctive measurement lives: the wearer
    # is working against the robot, so an arm that creeps is a fault the
    # participant feels directly. Sampled continuously, like T2's tilt.
    log_continuously=("ee_pose_left", "ee_pose_right", "drift_mm"),
    known_limitation="Holding the box needs a grasp the PINNED wrist can only "
                     "make at |x| >= 0.51. MASTER_TELEOP is therefore the "
                     "binding mode here as it is in T1; VR_TELEOP commands "
                     "6-DOF and is not affected.",
)


if __name__ == "__main__":
    import json
    print(json.dumps(dict(box=BOX_OBJ, box_present=BOX_PRESENT,
                          meter=METER_OBJ, meter_present=METER_PRESENT,
                          min_requests=MIN_EXPECTED_REQUESTS), indent=2))
    print("\ncommands:")
    for c in commands():
        print("  %-13s %-5s %-12s %s" % (c.verb.value, c.arm, c.subject,
                                         c.pose))
