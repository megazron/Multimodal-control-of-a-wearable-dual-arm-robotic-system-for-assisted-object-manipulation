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


# ---------------------------------------------------------------------------
# THE NULL SPACE, FOR REAL THIS TIME
#
# The sampler above is honest about what it is, and it was MEASURED and found
# useless: driven at the wearer it bought 4.4 mm at the head and 0.1 mm at the
# torso, because re-seeding TRAC-IK returns solutions that differ in the WRIST
# rather than in the elbow swivel. Seed sampling is not redundancy resolution.
# `docs/system/findings.md`, 2026-08-15.
#
# This is the redundancy resolution. For a 7-DOF arm the end-effector task is
# 6-dimensional, so the Jacobian J (6 x 7) has a one-dimensional null space:
# joint velocities qdot with J qdot = 0 move the arm WITHOUT moving the hand.
# Project a clearance gradient into that null space and integrate:
#
#     qdot = N grad(clearance),      N = I - pinv(J) J
#
# and every step moves the elbow away from the wearer while the gripper pose
# is unchanged to first order. The residual is not assumed: it is measured with
# forward kinematics after each step and reported, which is what (b) of the
# brief asks for.
#
# NO ANALYTIC JACOBIAN IS NEEDED AND NONE IS CLAIMED. J is built by finite
# differences from whatever forward-kinematics callable the caller passes, so
# this module still depends on nothing but arithmetic, and the same code runs
# against /compute_fk offline and against a local KDL chain in the follower.
# The cost is 7 extra FK evaluations per Jacobian, and the achieved rate is
# reported by the verification rather than promised here.


def _rotvec(R):
    """Rotation matrix to rotation vector, for orientation differences."""
    import numpy as np
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) * 0.5))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.zeros(3)
    if abs(math.pi - ang) < 1e-6:                       # near pi, use the
        w, V = np.linalg.eigh(R)                        # eigenvector for +1
        axis = np.real(V[:, int(np.argmax(np.real(w)))])
        return axis / (np.linalg.norm(axis) or 1.0) * ang
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * (ang / (2.0 * math.sin(ang)))


def pose_error(a, b):
    """6-vector from pose b to pose a, as (position, rotation vector).

    Poses are (position(3), rotation matrix(3x3)). Used both for the Jacobian
    columns and for measuring how far the hand actually moved.
    """
    import numpy as np
    dp = np.asarray(a[0], float) - np.asarray(b[0], float)
    dR = _rotvec(np.asarray(a[1], float) @ np.asarray(b[1], float).T)
    return np.concatenate([dp, dR])


def jacobian(fk, q, h=1e-5):
    """6 x n end-effector Jacobian by central differences.

    `fk(q)` returns (position, rotation matrix) for the end effector.
    """
    import numpy as np
    q = np.asarray(q, float)
    cols = []
    for i in range(len(q)):
        qp, qm = q.copy(), q.copy()
        qp[i] += h
        qm[i] -= h
        cols.append(pose_error(fk(qp), fk(qm)) / (2.0 * h))
    return np.column_stack(cols)


def null_projector(J, rcond=1e-6):
    """N = I - pinv(J) J. Projects a joint velocity onto ker(J)."""
    import numpy as np
    Jp = np.linalg.pinv(np.asarray(J, float), rcond=rcond)
    n = np.asarray(J).shape[1]
    return np.eye(n) - Jp @ np.asarray(J, float)


def clearance_gradient(clearance_fn, q, h=1e-4):
    """d(clearance)/dq by central differences, as a vector.

    `clearance_fn(q)` returns the worst clearance in metres, or None. A None
    anywhere makes the whole gradient None: half a gradient points somewhere
    nobody chose.
    """
    import numpy as np
    q = np.asarray(q, float)
    g = np.zeros(len(q))
    for i in range(len(q)):
        qp, qm = q.copy(), q.copy()
        qp[i] += h
        qm[i] -= h
        cp, cm = clearance_fn(qp), clearance_fn(qm)
        if cp is None or cm is None:
            return None
        g[i] = (cp - cm) / (2.0 * h)
    return g


class NullSpaceRetreat:
    """Move the ELBOW away from the wearer with the gripper pose unchanged.

    Iterates qdot = N grad(clearance) with a step size chosen so the joint
    move per step is bounded, stopping when the clearance target is met, the
    gradient dies, or the step budget runs out. Every step measures the ACTUAL
    end-effector displacement by forward kinematics and abandons a step that
    moves the hand more than `max_ee_drift_m`, because an avoidance the
    operator can feel is a different failure from a collision but it is still a
    failure.
    """

    def __init__(self, max_steps=8, step_rad=0.05, max_ee_drift_m=0.0005,
                 joint_limits=None):
        self.max_steps = int(max_steps)
        self.step_rad = float(step_rad)
        self.max_ee_drift_m = float(max_ee_drift_m)
        self.joint_limits = joint_limits

    def retreat(self, fk, clearance_fn, q0, target_m):
        """Returns (q, clearance, ee_residual_m, steps_taken, why_stopped)."""
        import numpy as np
        q = np.asarray(q0, float).copy()
        p0 = fk(q)
        c = clearance_fn(q)
        if c is None:
            return list(q), None, 0.0, 0, "no clearance estimate"
        why = "budget"
        steps = 0
        for _ in range(self.max_steps):
            if c >= target_m:
                why = "target met"
                break
            g = clearance_gradient(clearance_fn, q)
            if g is None:
                why = "gradient unavailable"
                break
            d = null_projector(jacobian(fk, q)) @ g
            nrm = float(np.linalg.norm(d))
            if nrm < 1e-9:
                why = "null space offers no direction"
                break
            trial = q + d * (self.step_rad / nrm)
            if self.joint_limits is not None:
                lo, hi = self.joint_limits
                trial = np.clip(trial, lo, hi)
            # PULL THE HAND BACK. N grad is tangent to the constraint, so a
            # FINITE step along it leaves the manifold and the hand drifts --
            # measured on the constructed planar arm, 5.5 mm over eight steps
            # of 0.05 rad, which an operator would feel. One Newton correction
            # in the task space per step removes it:
            #
            #     q <- q - pinv(J) * pose_error(fk(q), p0)
            #
            # This is the difference between an open-loop null-space step and
            # a closed-loop one, and it is why the residual below is microns
            # rather than millimetres.
            for _ in range(2):
                err = pose_error(fk(trial), p0)
                if float(np.linalg.norm(err[:3])) < 1e-7:
                    break
                trial = trial - np.linalg.pinv(
                    jacobian(fk, trial), rcond=1e-6) @ err
                if self.joint_limits is not None:
                    lo, hi = self.joint_limits
                    trial = np.clip(trial, lo, hi)
            drift = float(np.linalg.norm(
                np.asarray(fk(trial)[0]) - np.asarray(p0[0])))
            if drift > self.max_ee_drift_m:
                why = "would move the hand %.4f m" % drift
                break
            c_new = clearance_fn(trial)
            if c_new is None or c_new <= c + 1e-6:
                why = "no improvement"
                break
            q, c = trial, c_new
            steps += 1
        resid = float(np.linalg.norm(
            np.asarray(fk(q)[0]) - np.asarray(p0[0])))
        return list(q), c, resid, steps, why


def tangential_target(xyz, step, away, max_into_m=0.0):
    """Slide ALONG the wearer instead of stopping dead in front of them.

    THE STAGE THIS MAKES REAL. `srl_console` used to display "tangential" as
    an avoidance stage and nothing computed one; the label was removed rather
    than faked, and the console now names only the stages that exist. This is
    the stage, so it can be named again.

    WHAT IT DOES. `step` is the motion the operator asked for and `away` is the
    unit direction from the nearest wearer point toward the arm -- the
    direction clearance increases in. Split the step into the part along
    `away` and the part perpendicular to it, and keep the perpendicular part
    plus at most `max_into_m` of the inward part. The hand then travels ACROSS
    the person rather than into them, which is what lets the task keep going
    when refusing would end it.

    WHY IT IS THE LAST STAGE AND NOT THE FIRST. It changes the commanded pose,
    so the operator gets something they did not ask for; the null-space stage
    does not, and is always tried first. This runs only when the null space has
    nothing left, and it is still bounded underneath by the floor -- a slide
    that would breach it is refused like anything else.

    Returns the adjusted target position.
    """
    import numpy as np
    p = np.asarray(xyz, float)
    d = np.asarray(step, float)
    a = np.asarray(away, float)
    n = float(np.linalg.norm(a))
    if n < 1e-9:
        return tuple(p + d)
    a = a / n
    into = float(np.dot(d, a))            # negative means heading INTO them
    tang = d - into * a
    keep = into if into >= 0.0 else max(into, -abs(max_into_m))
    return tuple(p + tang + keep * a)
