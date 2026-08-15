"""Predictive avoidance: see it coming, move the ELBOW, keep the grasp.

WHAT THIS REPLACES. Today a near-collision is handled by `ik_follower_node`
measuring the arm's CURRENT clearance and, if it is under the floor, blocking
and publishing nothing. That is safe and it is also the failure mode this
project has hit five times: a check that can only say no. The arm stops with
the operator still pushing, and the task does not complete.

THREE THINGS, IN ORDER, AND THE FLOOR STAYS UNDERNEATH ALL OF THEM:

  1. LOOKAHEAD -- extrapolate the commanded pose forward from its recent
     velocity and evaluate the PREDICTED pose, not only the one being asked
     for now. Acting early and gently beats acting late and hard.
  2. NULL-SPACE AVOIDANCE -- the arm is 7-DOF, so for any gripper pose there
     is a family of elbow positions. When a link approaches the wearer, move
     the ELBOW through that family while the gripper pose is unchanged.
  3. THE FLOOR -- unchanged, last, and only when nothing in the family clears
     it. It is a floor, not the strategy.

HOW THE NULL SPACE IS ACTUALLY SEARCHED, STATED PLAINLY BECAUSE THE NAME
PROMISES MORE THAN THE METHOD. This follower has no local Jacobian; it has a
`/compute_ik` service. So the family is SAMPLED, not projected: the same EE
pose is requested several times from different seeds, every returned solution
is scored by wearer clearance, and the clearest wins. That is a genuine use of
the redundancy -- the solutions differ only in the null space, because they
all satisfy the same end-effector constraint -- and it is what the available
interface supports. It is not a Jacobian null-space projection and this
docstring does not call it one.

`srl_console` used to display "tangential" and "null-space" as avoidance
stages that did not exist, and they were removed rather than faked. This
module makes the second one real. TANGENTIAL IS STILL NOT IMPLEMENTED and is
still not displayed; three honest stages beat five claimed ones.
"""
import math


class Lookahead:
    """Where the commanded pose will be in `horizon_s`, from recent motion.

    Deliberately a straight-line extrapolation of the last two accepted
    samples rather than a filter: the input is an operator's hand, the horizon
    is a fraction of a second, and a smoother would add lag to the one signal
    whose whole purpose is to be early. Samples older than `stale_s` are
    dropped, so a paused master predicts NO motion rather than repeating its
    last velocity for ever.
    """

    def __init__(self, horizon_s=0.30, stale_s=0.50):
        self.horizon_s = float(horizon_s)
        self.stale_s = float(stale_s)
        self._prev = None            # (t, (x, y, z))

    def reset(self):
        self._prev = None

    def update(self, t, xyz):
        """Feed a commanded pose. Returns the predicted pose, or None.

        None means "no usable velocity estimate" -- the first sample, a
        stale gap, or a zero time step -- and the caller must treat that as
        "no prediction available", never as "no motion predicted". They are
        different: the second is a measurement and the first is its absence.
        """
        xyz = tuple(float(v) for v in xyz)
        prev, self._prev = self._prev, (float(t), xyz)
        if prev is None:
            return None
        dt = float(t) - prev[0]
        if dt <= 1e-6 or dt > self.stale_s:
            return None
        vel = [(xyz[i] - prev[1][i]) / dt for i in range(3)]
        return tuple(xyz[i] + vel[i] * self.horizon_s for i in range(3)), vel

    @property
    def last(self):
        return None if self._prev is None else self._prev[1]


def seed_fan(seed, n, spread_rad=0.45, joints=(2, 3, 5)):
    """`n` seeds around `seed`, spread over the joints that move the elbow.

    WHICH JOINTS, AND WHY THESE. On a 7-DOF Gen3 the elbow's position for a
    fixed wrist pose is controlled by the redundant circle about the
    shoulder-wrist axis; joints 3, 4 and 6 (0-based 2, 3, 5) are the ones that
    move a solution around it without the solver having to give up the pose.
    The existing redundancy re-seed in ik_follower_node perturbs joint_3 only,
    which samples a line through the family rather than the family.

    The fan is SYMMETRIC about the seed so the search is not biased toward one
    side of the wearer, and it always contains the unperturbed seed first, so
    "no better solution" costs one solve and returns exactly what the current
    code would have returned.
    """
    out = [list(seed)]
    if n <= 1:
        return out
    for k in range(1, n):
        step = spread_rad * ((k + 1) // 2) / max(1, (n - 1) // 2)
        sign = 1.0 if k % 2 else -1.0
        s = list(seed)
        for j in joints:
            if j < len(s):
                s[j] += sign * step
        out.append(s)
    return out


class AvoidanceDecision:
    """What the avoider concluded, in a form the caller can log verbatim."""

    def __init__(self, action, clearance, predicted_clearance=None,
                 candidates=0, ee_residual_m=None, part=None, joints=None):
        self.action = action          # "clear" | "avoided" | "refuse"
        self.clearance = clearance
        self.predicted_clearance = predicted_clearance
        self.candidates = candidates
        self.ee_residual_m = ee_residual_m
        self.part = part
        self.joints = joints

    def __repr__(self):                                   # pragma: no cover
        return ("AvoidanceDecision(%s, clr=%s, pred=%s, cands=%d, "
                "ee_resid=%s, part=%s)"
                % (self.action, self.clearance, self.predicted_clearance,
                   self.candidates, self.ee_residual_m, self.part))


def choose(candidates, floor_m, trigger_m):
    """Pick a solution from the null-space family. Pure, so it is testable.

    `candidates` is [(joints, clearance, ee_residual_m)], the FIRST of which
    must be the unperturbed solution -- the one the follower would have used
    without this module.

    THE RULES, IN THE ORDER THEY APPLY:

      * if the unperturbed solution is already clear of `trigger_m`, use it.
        Avoidance that runs when nothing is near the wearer is just jitter the
        operator can feel, and (b) of the brief is explicit that they should
        not be able to feel it.
      * otherwise take the candidate with the greatest clearance. It is the
        same EE pose, so this costs the task nothing.
      * refuse ONLY if no candidate clears `floor_m`. That is the last resort
        and it is the existing behaviour, unchanged.
    """
    if not candidates:
        return AvoidanceDecision("refuse", None, candidates=0)
    base = candidates[0]
    if base[1] is not None and base[1] >= trigger_m:
        return AvoidanceDecision("clear", base[1], candidates=len(candidates),
                                 ee_residual_m=base[2], joints=base[0])
    usable = [c for c in candidates if c[1] is not None]
    if not usable:
        return AvoidanceDecision("refuse", None, candidates=len(candidates))
    best = max(usable, key=lambda c: c[1])
    if best[1] < floor_m:
        return AvoidanceDecision("refuse", best[1], candidates=len(candidates),
                                 joints=best[0])
    action = "avoided" if best is not base else "clear"
    return AvoidanceDecision(action, best[1], candidates=len(candidates),
                             ee_residual_m=best[2], joints=best[0])


def ee_residual(a, b):
    """Straight-line distance between two EE positions, in metres."""
    return math.dist(a, b)
