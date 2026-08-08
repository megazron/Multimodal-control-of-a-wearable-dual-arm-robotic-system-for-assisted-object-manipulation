#!/usr/bin/env python3
"""T2 DISCRETE OUTCOME: blocks placed, blocks dropped, blocks missed.

T2 earns its slot beside T3/T6/T7 precisely because its outcome is DISCRETE
where theirs are continuous. Until now the runner logged trajectories and
timing and printed a caveat saying so; this is the missing half.

EVERYTHING HERE COMES FROM /joint_states AND TF. No vision, no AprilTags --
the same sources T3 and T6 use, and the same detection pattern:

    a state change in the gripper, cross-referenced against WHERE the
    gripper was when it changed.

WHY A TRANSITION AND NOT A LEVEL
--------------------------------
The mock gripper boots at 0.79 rad, which is already past any "closed"
threshold. Scoring on a level made every trial after the first log an instant
grasp, and the original pilot passed with that bug in it. A pick is therefore
an observed OPEN -> HOLDING transition INSIDE the trial, and a release is an
observed HOLDING -> OPEN transition inside the same trial. A gripper that was
already closed when the trial started has not picked anything.

`gripper_state` already distinguishes the third case that matters:
FREE_AIR, a gripper closed all the way, means it closed on NOTHING. That is a
scene fault (`block_absent`), not a participant failure, and conflating the
two makes the experiment blame the participant for an empty table.

THE OPENING MOVES WITH THE HOLDING ARM
--------------------------------------
The container is HELD, not fixed to the world, so its opening is wherever the
holding gripper has carried it:

    opening_now = hold_ee_now + (opening_centre - hold_pose)

Scoring a release against a world-fixed opening would credit a block dropped
into thin air whenever the holding arm had drifted, and would penalise an
operator who tracked the drifting container correctly. This is also what
couples the two metrics: hold disturbance is not a separate nicety, it is
part of whether the block goes in.
"""
import math

import numpy as np

from bimanual_metrics import gripper_state          # noqa: F401  (re-export)

# A release only counts if the gripper was above the opening rim when it let
# go. Below this the block was placed ON the container, not IN it.
MIN_RELEASE_HEIGHT_M = -0.005
# How far above the opening is still a plausible drop rather than a throw.
MAX_RELEASE_HEIGHT_M = 0.150
# The holding arm is allowed to move this far before the hold is "disturbed".
HOLD_TOL_MM = 30.0


class T2Outcome:
    """Streaming detector. Feed it one sample per control cycle.

    Deliberately streaming rather than post-hoc: a transition is a
    two-sample event, and a trial that ends mid-pick must be scored as a
    pick that did not complete, not silently rounded to whichever level the
    last sample happened to hold.
    """

    def __init__(self, hold_pose, opening_centre, tol_mm,
                 hold_tol_mm=HOLD_TOL_MM):
        self.hold0 = np.asarray(hold_pose, float)
        self.offset = np.asarray(opening_centre, float) - self.hold0
        self.tol = float(tol_mm) / 1000.0
        self.hold_tol = float(hold_tol_mm) / 1000.0

        self.prev_fill = None          # previous fill-gripper state
        self.prev_hold = None
        self.carrying = False
        self.pick_t = None
        self.hold_ref = None           # holding EE at the trial's first sample

        self.placed = 0
        self.missed = 0
        self.dropped = 0
        self.picks = 0
        self.events = []               # (t, kind, detail)
        self.faults = []               # scene faults, not participant failures
        self.hold_disturbance = []
        self.cycle_times = []
        self.first_place_t = None

    # ------------------------------------------------------------------
    def opening_now(self, hold_ee):
        """Where the opening is RIGHT NOW, carried by the holding arm."""
        if hold_ee is None:
            return None
        return np.asarray(hold_ee, float) + self.offset

    def _inside(self, p_release, hold_ee):
        """Did the block land in the opening? Horizontal footprint + height.

        Split into the two questions rather than one 3-D distance, because
        they fail for different reasons and the analysis wants to know which:
        horizontal miss = aiming, vertical miss = released too low (placed on
        the rim) or too high (dropped from a height, which bounces out).
        """
        o = self.opening_now(hold_ee)
        if o is None or p_release is None:
            return False, "no pose"
        p = np.asarray(p_release, float)
        radial = float(np.linalg.norm(p[:2] - o[:2]))
        dz = float(p[2] - o[2])
        if radial > self.tol:
            return False, "%.0f mm outside the %.0f mm opening" % (
                1000 * radial, 1000 * self.tol)
        if dz < MIN_RELEASE_HEIGHT_M:
            return False, "released %.0f mm BELOW the rim" % (-1000 * dz)
        if dz > MAX_RELEASE_HEIGHT_M:
            return False, "released %.0f mm above the rim" % (1000 * dz)
        return True, ""

    # ------------------------------------------------------------------
    def update(self, t, fill_knuckle, hold_knuckle, fill_ee, hold_ee):
        """One sample. Returns a short event string, or None."""
        f = gripper_state(fill_knuckle)
        h = gripper_state(hold_knuckle)
        if self.hold_ref is None and hold_ee is not None:
            self.hold_ref = np.asarray(hold_ee, float)
        if hold_ee is not None and self.hold_ref is not None:
            self.hold_disturbance.append(
                float(np.linalg.norm(np.asarray(hold_ee, float)
                                     - self.hold_ref)))

        ev = None
        # ---- the container itself
        if h == "free_air" and self.prev_hold in ("holding", None):
            self._fault("box_lost", "holding gripper reached free air at "
                                    "t=%.2f s" % t)
            ev = "box_lost"

        # ---- pick: OPEN -> HOLDING, a transition inside this trial
        if self.prev_fill is not None:
            if self.prev_fill == "open" and f == "holding":
                self.picks += 1
                self.pick_t = t
                self.carrying = True
                ev = "pick"
                self.events.append((t, "pick", ""))
            elif self.prev_fill == "open" and f == "free_air":
                # closed all the way: there was nothing there
                self._fault("block_absent",
                            "fill gripper closed to free air at t=%.2f s" % t)
                ev = "block_absent"
            elif self.carrying and self.prev_fill == "holding" and f == "open":
                # ---- release: score it where it happened
                ok, why = self._inside(fill_ee, hold_ee)
                if ok:
                    self.placed += 1
                    if self.first_place_t is None:
                        self.first_place_t = t
                    ev = "placed"
                else:
                    self.missed += 1
                    ev = "missed"
                self.events.append((t, ev, why))
                if self.pick_t is not None:
                    self.cycle_times.append(t - self.pick_t)
                self.carrying = False
                self.pick_t = None
            elif self.carrying and self.prev_fill == "holding" \
                    and f == "free_air":
                # the fingers closed further with a block between them: the
                # block is gone
                self.dropped += 1
                ev = "dropped"
                self.events.append((t, "dropped", "gripper reached free air "
                                                  "while carrying"))
                self.carrying = False
                self.pick_t = None

        self.prev_fill, self.prev_hold = f, h
        return ev

    def _fault(self, kind, detail):
        if not any(k == kind for k, _ in self.faults):
            self.faults.append((kind, detail))

    # ------------------------------------------------------------------
    def summary(self, duration_s=float("nan")):
        att = self.placed + self.missed + self.dropped
        d = np.asarray(self.hold_disturbance, float)
        return dict(
            blocks_placed=int(self.placed),
            blocks_missed=int(self.missed),
            blocks_dropped=int(self.dropped),
            blocks_attempted=int(att),
            picks=int(self.picks),
            # Reported as NaN, never 0.0, when nothing was attempted: a rate
            # of zero and an absence of data are different findings and only
            # one of them is about the participant.
            success_rate=(float(self.placed) / att if att else float("nan")),
            carrying_at_end=bool(self.carrying),
            hold_disturbance_max_mm=(1000.0 * float(d.max()) if d.size
                                     else float("nan")),
            hold_disturbance_rms_mm=(1000.0 * float(np.sqrt((d ** 2).mean()))
                                     if d.size else float("nan")),
            hold_disturbed=bool(d.size and d.max() > self.hold_tol),
            time_to_first_place_s=(self.first_place_t
                                   if self.first_place_t is not None
                                   else float("nan")),
            mean_cycle_time_s=(float(np.mean(self.cycle_times))
                               if self.cycle_times else float("nan")),
            scene_faults=";".join(k for k, _ in self.faults),
            duration_s=duration_s)


def scene_fault_reason(outcome):
    """The invalidation cause, or None. A scene fault is not a trial result.

    Keeping this separate from the summary is deliberate: a trial with an
    empty table must be INVALIDATED with a cause, not recorded as a 0%
    success rate that later reads as a participant who could not do the task.
    """
    if not outcome.faults:
        return None
    k, detail = outcome.faults[0]
    return "%s: %s" % (k, detail)
