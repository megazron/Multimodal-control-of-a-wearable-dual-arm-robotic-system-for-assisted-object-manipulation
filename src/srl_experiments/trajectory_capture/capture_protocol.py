#!/usr/bin/env python3
"""capture_protocol.py — the segment list, and only the segment list.

Separated from the recorder so the protocol can be printed, costed and
reviewed without a running stack:

    python3 src/srl_experiments/trajectory_capture/capture_protocol.py

EVERY CHANNEL IS RECORDED IDENTICALLY, AND NONE IS PRE-LABELLED.

Earlier data called right j3/j5/j7 and left j7 dead. That determination is
deliberately NOT carried into this protocol: the operator is given the same
instruction for every joint, and no `expected_flat` flag reaches the data.

The reason is not tidiness. A channel previously called dead that now shows
movement is a FINDING -- the wiring may have been disturbed, a connector may
have reseated, the earlier session may have had a fault elsewhere. Telling the
operator "this one is dead, expect a flat trace" invites a less committed
sweep, and stamping `expected_flat` into the CSV invites the analysis to agree
with itself. Both would hide exactly the result worth having.

The prior verdicts still exist in PRIOR_VERDICT below, but they are used ONLY
after the fact, by the analyser, to flag disagreement. They never reach the
operator and never reach the data.
"""
import sys

ARMS = ("left", "right")

# PRIOR verdicts from the 2026-07-31 session (20440 frames). NOT an input to
# this capture: not shown to the operator, not written to the CSV. The
# analyser uses them only to say "this disagrees with what we thought", which
# is the finding. Kept here so that comparison has a single source.
PRIOR_VERDICT = {
    ("left", 1): ("alive", ""),
    ("left", 2): ("alive", ""),
    ("left", 3): ("alive", ""),
    ("left", 4): ("alive", ""),
    ("left", 5): ("degraded", "2.8% zeros, 14% railed"),
    ("left", 6): ("degraded", "12.8% clamped at 0"),
    ("left", 7): ("flat", "75% railed - EXPECTED FLAT, negative control"),
    ("right", 1): ("alive", ""),
    ("right", 2): ("alive", ""),
    ("right", 3): ("flat", "70% zeros - EXPECTED FLAT, negative control"),
    ("right", 4): ("intermittent", "12.9% zeros in bursts - the one that "
                                   "matters, spherical mode uses it"),
    ("right", 5): ("flat", "99% zeros - EXPECTED FLAT, negative control"),
    ("right", 6): ("alive", ""),
    ("right", 7): ("flat", "99% zeros - EXPECTED FLAT, negative control"),
}

# Physical joint names, so "joint 4" is never a guess at the mannequin.
JOINT_NAME = {1: "shoulder roll", 2: "shoulder bend", 3: "upper-arm roll",
              4: "elbow bend", 5: "forearm roll", 6: "wrist bend",
              7: "wrist roll"}

# ---------------------------------------------------------------------------
# BLOCK G: STATION, THEN SWEEP.  The block that targets the lateral defect.
#
# WHY A GRID AND NOT MORE SWEEPS.  Vertical and fore-and-aft command track
# correctly; lateral does not.  The mechanism is known: azimuth is taken from
# joint 1 alone, and joint 1 is a ROLL ABOUT THE ARM'S OWN AXIS, so how much
# the tip moves sideways for a given joint-1 rotation depends on HOW BENT THE
# ARM IS.  Two directions intended to be 90 deg apart were measured
# 137.7 deg apart.
#
# That makes the error CONFIGURATION-DEPENDENT, and it is why the existing
# directional block cannot settle it.  "Move left to the extreme and back"
# sweeps through a whole range of arm extensions at once and returns one
# number, which is consistent with two very different explanations:
#
#     a FIXED mis-orientation  -- one wrong rotation between the master frame
#                                 and the robot frame, correctable by a matrix
#     a CONFIGURATION-DEPENDENT gain -- the model is wrong about the geometry,
#                                 and no fixed matrix can fix it
#
# The discriminating experiment is to run the SAME elementary motion at MANY
# DIFFERENT arm configurations.  If the error is a fixed rotation it is the
# same at every station.  If it grows with extension it is not, and the
# decomposition needs replacing rather than re-calibrating.
#
# So: park at a station, hold still, then sweep ONE axis and return.  The hold
# gives a clean per-station datum; the sweep gives the local gain.  Every
# station is probed on the FAILING axis and on a WORKING one, because a
# measurement of a broken axis with no working axis beside it cannot separate
# "this axis is wrong" from "this session was wrong".
# ---------------------------------------------------------------------------
STATIONS = (
    ("centre", "at the neutral resting pose, arm relaxed"),
    ("near",   "hand pulled IN close to your chest, elbow well bent"),
    ("far",    "arm reaching OUT, elbow nearly straight"),
    ("front",  "hand FORWARD at about mid height"),
    ("back",   "hand drawn BACK beside your body"),
    ("high",   "hand raised ABOVE shoulder height"),
    ("low",    "hand lowered toward your HIP"),
)
#: The probe swept at each station. `left_right` is the axis under
#: investigation; `up_down` is the control, and it is not optional -- without
#: it a bad session and a bad axis look the same.
PROBES = (
    ("left_right", "LEFT then RIGHT then back to the station"),
    ("up_down",    "UP then DOWN then back to the station"),
)

DIRECTIONS = ("front", "back", "left", "right", "up", "down")
COMBINATIONS = ("front_left", "front_right", "back_left", "back_right",
                "up_front", "down_front", "up_back", "down_back")
IMU_SWEEPS = ("pitch_up_down", "roll_left_right",
              "diagonal_up_left_down_right", "diagonal_up_right_down_left")
SPEEDS = ("slow", "medium", "fast")

# Seconds per segment, from the recorded 2026-07-31 sessions: a full sweep and
# return at a comfortable pace runs 12-20 s, plus repositioning between.
T_SEGMENT = 18.0
T_BETWEEN = 8.0


def segments():
    """The whole protocol, in execution order.

    Each entry: (block, label, arm, instruction, meta)
    """
    out = []

    # ---- A. SINGLE JOINT ISOLATION -------------------------------------
    for arm in ARMS:
        for j in range(1, 8):
            # Identical instruction for every joint. No hint, no expectation.
            out.append(dict(
                block="A", label="A_%s_j%d" % (arm, j), arm=arm, rep=0,
                channel=j,
                instruction="%s arm, joint %d ONLY (%s): sweep slowly through "
                            "its full range and back. Hold every other joint "
                            "still." % (arm.upper(), j, JOINT_NAME[j])))

    # ---- B. IMU SWEEPS -------------------------------------------------
    for arm in ARMS:
        for s in IMU_SWEEPS:
            out.append(dict(
                block="B", label="B_%s_%s" % (arm, s), arm=arm, rep=0,
                channel=None,
                instruction="%s arm WRIST: %s. Keep the pots as still as you "
                            "can - this isolates the IMU."
                            % (arm.upper(), s.replace("_", " "))))

    # ---- C. DIRECTIONAL ------------------------------------------------
    for arm in ARMS:
        for d in DIRECTIONS:
            out.append(dict(
                block="C", label="C_%s_%s" % (arm, d), arm=arm, rep=0,
                channel=None, direction=d,
                instruction="%s arm: move %s to the extreme and back."
                            % (arm.upper(), d.upper())))

    # ---- D. COMBINATIONAL ----------------------------------------------
    for arm in ARMS:
        for d in COMBINATIONS:
            out.append(dict(
                block="D", label="D_%s_%s" % (arm, d), arm=arm, rep=0,
                channel=None, direction=d,
                instruction="%s arm: move %s to the extreme and back."
                            % (arm.upper(), d.replace("_", "-").upper())))

    # ---- E. REPEATABILITY: C again, twice more -------------------------
    # Reps 1 and 2; rep 0 is block C above, so three in total per direction.
    for rep in (1, 2):
        for arm in ARMS:
            for d in DIRECTIONS:
                out.append(dict(
                    block="E", label="E_%s_%s_r%d" % (arm, d, rep), arm=arm,
                    rep=rep, channel=None, direction=d,
                    instruction="REPEAT %d of 3 - %s arm: move %s to the "
                                "extreme and back. Match your earlier pace."
                                % (rep + 1, arm.upper(), d.upper())))

    # ---- G. STATION, THEN SWEEP ----------------------------------------
    # The grid: park somewhere, hold, sweep one axis, return. See the
    # STATIONS comment above for why this and not more directional sweeps.
    for arm in ARMS:
        for st, where in STATIONS:
            for probe, how in PROBES:
                out.append(dict(
                    block="G", label="G_%s_%s_%s" % (arm, st, probe),
                    arm=arm, rep=0, channel=None,
                    direction=probe, station=st,
                    instruction=(
                        "%s arm. FIRST go to the station: %s. HOLD STILL "
                        "there for about two seconds. THEN sweep %s. Move "
                        "only that one axis -- try to keep the station's "
                        "depth and height otherwise unchanged."
                        % (arm.upper(), where, how))))

    # ---- F. SPEED VARIATION --------------------------------------------
    for arm in ARMS:
        for sp in SPEEDS:
            out.append(dict(
                block="F", label="F_%s_front_%s" % (arm, sp), arm=arm, rep=0,
                channel=None, direction="front",
                speed=sp,
                instruction="%s arm: move FRONT to the extreme and back, "
                            "%s." % (arm.upper(), sp.upper())))
    return out


def estimate(segs):
    n = len(segs)
    secs = n * (T_SEGMENT + T_BETWEEN)
    return n, secs


def print_plan():
    segs = segments()
    n, secs = estimate(segs)
    by_block = {}
    for s in segs:
        by_block.setdefault(s["block"], []).append(s)
    names = {"A": "single joint isolation, all 14 channels",
             "B": "IMU sweeps", "C": "directional",
             "D": "combinational", "E": "repeatability (C x3)",
             "F": "speed variation",
             "G": "station, then sweep (the lateral-axis grid)"}
    print("TRAJECTORY CAPTURE PROTOCOL")
    print()
    for b in sorted(by_block):
        bs = by_block[b]
        mins = len(bs) * (T_SEGMENT + T_BETWEEN) / 60.0
        print("  %s  %-48s %3d segments  ~%4.1f min"
              % (b, names[b], len(bs), mins))
    print()
    print("  %d segments total, ~%.0f min (%.1f h) at %.0f s per segment "
          "plus %.0f s between." % (n, secs / 60.0, secs / 3600.0,
                                    T_SEGMENT, T_BETWEEN))
    print("  Every channel is recorded identically; none is pre-labelled.")
    print()
    # The fatigue ceiling is 45 min of SESSION, and the master arm is
    # unassisted - the operator's own arm carries its weight throughout.
    cap = 45.0
    mins = secs / 60.0
    if mins > cap:
        print("  %.0f min EXCEEDS the %.0f min fatigue cap. Split at a block"
              % (mins, cap))
        print("  boundary: --start-at <label> resumes, and every block is")
        print("  independently analysable.")
    else:
        print("  %.0f min is inside the %.0f min fatigue cap, but that is the"
              % (mins, cap))
        print("  RECORDING time only. Add setup and rests; blocks A and E are")
        print("  the tiring ones. --start-at <label> resumes at any segment,")
        print("  and every block is independently analysable.")
    print("  Take the 2-min rest between blocks - fatigue is collinear with")
    print("  segment order here, so a tired operator biases the LAST block.")


if __name__ == "__main__":
    print_plan()
    sys.exit(0)
