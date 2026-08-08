#!/usr/bin/env python3
"""Shared metrics for the bimanual tasks. One definition, five tasks.

Kept in one place so T1-T5 cannot drift apart, exactly as
`srl_experiments/validity.py` does for the single-arm set.

Everything here is computed from `/joint_states` and TF only. There are no
AprilTags in the bimanual set: object poses come from a fixed calibrated
layout measured once and stored in each task's `layout.yaml`.
"""
import math

import numpy as np

# Robotiq 2F-85 driven knuckle, radians. 0 = open, ~0.79 = closed on nothing.
GRIPPER_OPEN_RAD = 0.10
GRIPPER_FREE_AIR_RAD = 0.74
TRAY_SEPARATION_M = 0.300


def gripper_state(knuckle_rad):
    """open | holding | free_air  -- the three states that matter.

    `free_air` is the one that earns its keep: a gripper that closes all the
    way has nothing in it. Without this, a missing or knocked object is
    indistinguishable from a grasp the operator fumbled, and the experiment
    silently blames the participant for a scene error.
    """
    if knuckle_rad is None or not math.isfinite(knuckle_rad):
        return "unknown"
    if knuckle_rad < GRIPPER_OPEN_RAD:
        return "open"
    if knuckle_rad >= GRIPPER_FREE_AIR_RAD:
        return "free_air"
    return "holding"


def object_present(knuckle_rad):
    return gripper_state(knuckle_rad) == "holding"


def tilt_deg(z_left, z_right, separation_m=TRAY_SEPARATION_M):
    """Tray tilt from the two EE heights. THE T3 headline metric.

    20 mm of height difference over a 300 mm tray is 3.8 deg -- visible
    wobble. 60 mm is 11.3 deg and the ball leaves. Computed per sample and
    logged continuously, never as a pass/fail at the end: a coordination
    failure has a SHAPE, and a single number cannot distinguish a step from a
    drift from a wobble.
    """
    if z_left is None or z_right is None:
        return float("nan")
    return math.degrees(math.atan2(abs(z_left - z_right), separation_m))


def separation_mm(p_left, p_right):
    if p_left is None or p_right is None:
        return float("nan")
    return 1000.0 * float(np.linalg.norm(np.asarray(p_left) -
                                         np.asarray(p_right)))


def hold_disturbance_mm(p_now, p_origin):
    """T2: how far the HOLDING arm drifted. The failure mode of that task."""
    if p_now is None or p_origin is None:
        return float("nan")
    return 1000.0 * float(np.linalg.norm(np.asarray(p_now) -
                                         np.asarray(p_origin)))


def summarise_tilt(tilt_series, dt=0.02):
    """Per-trial roll-up of the continuous tilt trace."""
    t = np.asarray([x for x in tilt_series if x == x], float)
    if t.size == 0:
        return dict(tilt_max_deg=float("nan"), tilt_rms_deg=float("nan"),
                    time_above_3_8_s=0.0, time_above_11_3_s=0.0, n=0)
    return dict(
        tilt_max_deg=float(t.max()),
        # RMS, not max, is primary: max rewards a trial that was terrible once
        # and fine otherwise, while the ball integrates the whole trace.
        tilt_rms_deg=float(np.sqrt(np.mean(t ** 2))),
        time_above_3_8_s=float((t > 3.8).sum() * dt),
        time_above_11_3_s=float((t > 11.3).sum() * dt),
        n=int(t.size))


def handover_ordering_ok(t_receiver_closed, t_giver_opened):
    """T4: a handover is receiver-closes THEN giver-opens.

    The reverse ordering is a drop that happened to be caught, and scoring it
    as a handover would credit the worst possible execution.
    """
    if t_receiver_closed is None or t_giver_opened is None:
        return False
    return t_giver_opened > t_receiver_closed


def scene_fault(kind, detail=""):
    """A scene problem, NOT a participant failure. Marks the trial invalid."""
    return dict(scene_fault=kind, detail=detail)


PRESENCE_FAULTS = (
    "object_absent",      # gripper closed to free air at pick
    "box_lost",           # T2 holding gripper reached free air
    "tray_absent",        # T3 either gripper reached free air at grip
    "grip_lost",          # T3 separation left 300 +/- 40 mm
    "dropped_at_transfer",  # T4 receiver slammed shut as giver opened
    "block_absent",
    "tool_absent",        # T5 cradle empty
    "dropped_in_transit",
    "arm_not_driven",     # T1 an EE never moved
)
