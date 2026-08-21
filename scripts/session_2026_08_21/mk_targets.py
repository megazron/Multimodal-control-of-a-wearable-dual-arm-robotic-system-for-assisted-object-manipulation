"""Precompute joint targets for +-X/+-Y/+-Z moves from home, both arms.

IK here is numerical and seeded from home, using the SAME compiled FK the
rest of this repo measures with, so the target joints and the achieved
position are scored on one model. Orientation is held near home: a
calibration that let the wrist rotate would be measuring two things at once.
"""
import json, math, sys, os
sys.path.insert(0, os.path.join(os.getcwd(), "scripts"))
sys.path.insert(0, os.path.join(os.getcwd(), "config"))
import numpy as np
from scipy.optimize import minimize
import solve_home_pose as S
import home_positions as hp

sc = S.Scorer()
STEP = 0.10                      # 100 mm per axis
DIRS = {"+X": (1,0,0), "-X": (-1,0,0), "+Y": (0,1,0),
        "-Y": (0,-1,0), "+Z": (0,0,1), "-Z": (0,0,-1)}

def ee(arm, q):
    t = sc.arm_terms(arm, q, with_clearance=False)
    return np.array(t["hand"], dtype=float)

out = {"step_m": STEP, "arms": {}}
for arm in ("left", "right"):
    home = np.array(hp.load_home_radians(arm), float)
    p0 = ee(arm, home)
    lo, hi, _ = sc.lim[arm]
    out["arms"][arm] = {"home_rad": [float(v) for v in home],
                        "home_ee": [float(v) for v in p0], "targets": {}}
    print("%s home EE %s" % (arm, np.round(p0, 4)))
    for name, d in DIRS.items():
        goal = p0 + STEP*np.array(d, float)
        def cost(q):
            p = ee(arm, q)
            c = 400.0*float(np.sum((p-goal)**2))
            c += 0.5*float(np.sum((q-home)**2))     # stay near home posture
            return c
        best = None
        for seed in (home, home+0.05*np.random.default_rng(3).normal(size=7)):
            r = minimize(cost, seed, method="L-BFGS-B",
                         bounds=list(zip(lo,hi)),
                         options=dict(maxiter=600, ftol=1e-14))
            if best is None or r.fun < best.fun: best = r
        q = best.x
        p = ee(arm, q)
        err = float(np.linalg.norm(p-goal))
        clear = sc.arm_terms(arm, q)["clearance_moving_m"]
        # path clearance, home -> target
        worst = 9.0
        for k in range(41):
            qq = home + (k/40.0)*(q-home)
            worst = min(worst, sc.arm_terms(arm, qq)["clearance_moving_m"])
        ok = err < 0.005 and worst > 0.20
        print("   %-3s ik_err %6.2f mm  clearance(min on path) %.4f m  %s"
              % (name, err*1000, worst, "OK" if ok else "REJECT"))
        out["arms"][arm]["targets"][name] = {
            "q": [float(v) for v in q], "goal_ee": [float(v) for v in goal],
            "ik_ee": [float(v) for v in p], "ik_err_m": err,
            "path_min_clearance_m": float(worst), "usable": bool(ok)}
json.dump(out, open(sys.argv[1], "w"), indent=1)
print("\n-> %s" % sys.argv[1])
