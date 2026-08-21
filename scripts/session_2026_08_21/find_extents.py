"""How far can each arm ACTUALLY go, per direction? Bisect until it fails.

The previous calibration used a fixed 100 mm step, which measures a box
around home and says nothing about the envelope. This searches each direction
outward until one of the real limits bites, and REPORTS WHICH ONE -- reach,
the wearer, or a joint stop -- because "cannot go further" without a cause is
not a measurement.

Every candidate is checked N times over the WHOLE PATH from home, not just at
the endpoint: a target that solves but cannot be reached without crossing the
clearance floor is not reachable.
"""
import json, math, os, sys
sys.path.insert(0, os.path.join(os.getcwd(), "scripts"))
sys.path.insert(0, os.path.join(os.getcwd(), "config"))
import numpy as np
from scipy.optimize import minimize
import solve_home_pose as S
import home_positions as hp

sc = S.Scorer()
FLOOR = 0.150            # HARD CONSTRAINT 11
IK_TOL = 0.006
DIRS = {"+X": (1,0,0), "-X": (-1,0,0), "+Y": (0,1,0), "-Y": (0,-1,0),
        "+Z": (0,0,1), "-Z": (0,0,-1)}

def ee(arm, q):
    return np.array(sc.arm_terms(arm, q, with_clearance=False)["hand"], float)

# THE SEAM, AGAIN, AND IT NEARLY WENT TO THE ROBOT.
#
# The first run of this search had no seam term, and the left arm's -Y
# extreme came back with joint_3 at 180.00 deg and joint_5 at -180.00 --
# sitting exactly on the +/-pi wrap point. A continuous joint parked there
# reads +pi one sample and -pi the next with the arm STATIONARY, and every
# consumer that differences joint angles (the bridge's require_homed gate,
# the homing node's shortest-path error, the follower's lag monitor) reads
# that as a 2*pi jump. It is the same defect as the home pose carried this
# morning; an IK that is free to park a joint on the seam will do it.
SEAM_MARGIN_RAD = 0.30
CONT_IDX = (0, 2, 4, 6)

def seam_margin(q):
    return min(math.pi - abs(float(q[i])) for i in CONT_IDX)

def solve(arm, goal, seed, lo, hi, home):
    def cost(q):
        p = ee(arm, q)
        c = 400.0*float(np.sum((p-goal)**2)) + 0.05*float(np.sum((q-home)**2))
        # Solved INSIDE the limit, like everything else here: a hinge with no
        # margin parks the optimiser exactly on the boundary.
        for i in CONT_IDX:
            short = (SEAM_MARGIN_RAD + 0.05) - (math.pi - abs(float(q[i])))
            if short > 0:
                c += 50.0 * short * short
        return c
    best = None
    for s0 in (seed, home):
        r = minimize(cost, s0, method="L-BFGS-B", bounds=list(zip(lo,hi)),
                     options=dict(maxiter=800, ftol=1e-14))
        if best is None or r.fun < best.fun: best = r
    return best.x

def check(arm, q, goal, home):
    """(ok, why). Endpoint IK, endpoint clearance, and the whole path."""
    p = ee(arm, q)
    if float(np.linalg.norm(p-goal)) > IK_TOL:
        return False, "unreachable (IK residual %.0f mm)" % (
            np.linalg.norm(p-goal)*1000)
    worst, who = 9.0, None
    for k in range(31):
        qq = home + (k/30.0)*(q-home)
        t = sc.arm_terms(arm, qq)
        if t["clearance_m"] < worst:
            worst, who = t["clearance_m"], t.get("clearance_to", "wearer")
    if worst < FLOOR:
        return False, "wearer clearance %.3f m on the path" % worst
    m = seam_margin(q)
    if m < SEAM_MARGIN_RAD:
        return False, ("continuous joint %.3f rad from the +/-pi seam "
                       "(limit %.2f)" % (m, SEAM_MARGIN_RAD))
    # AND THE WHOLE PATH, not just the endpoint: a joint that crosses the
    # seam mid-travel is fine, one that DWELLS there is not.
    for k in range(31):
        qq = home + (k/30.0)*(q-home)
        if seam_margin(qq) < 0.10:
            return False, "path dwells within 0.10 rad of the +/-pi seam"
    return True, "ok (min clearance %.3f m, seam %.2f rad)" % (worst, m)

out = {"floor_m": FLOOR, "arms": {}}
for arm in ("left", "right"):
    home = np.array(hp.load_home_radians(arm), float)
    lo, hi, _ = sc.lim[arm]
    p0 = ee(arm, home)
    out["arms"][arm] = {"home_ee": [float(v) for v in p0], "dirs": {}}
    print("\n%s  home EE %s" % (arm.upper(), np.round(p0,3)))
    for name, d in DIRS.items():
        d = np.array(d, float)
        loD, hiD = 0.0, 1.60
        bestq, bestwhy = None, "nothing solved"
        seed = home.copy()
        # coarse march outward, then bisect the last good/bad bracket
        step = 0.05
        good = 0.0
        while good + step <= hiD:
            cand = good + step
            q = solve(arm, p0 + cand*d, seed, lo, hi, home)
            ok, why = check(arm, q, p0 + cand*d, home)
            if not ok:
                bestwhy = why
                break
            good, bestq, seed = cand, q, q
            step = 0.05
        # bisect for the last 10 mm
        loB, hiB = good, min(good+0.05, hiD)
        for _ in range(5):
            mid = 0.5*(loB+hiB)
            q = solve(arm, p0 + mid*d, seed, lo, hi, home)
            ok, why = check(arm, q, p0 + mid*d, home)
            if ok: loB, bestq = mid, q
            else:  hiB, bestwhy = mid, why
        reach = loB
        print("   %-3s reach %5.0f mm   limited by: %s" % (name, reach*1000, bestwhy))
        out["arms"][arm]["dirs"][name] = {
            "reach_m": round(float(reach), 4), "limit": bestwhy,
            "q_max": [float(v) for v in bestq] if bestq is not None else None}
json.dump(out, open(sys.argv[1], "w"), indent=1)
print("\n-> %s" % sys.argv[1])
