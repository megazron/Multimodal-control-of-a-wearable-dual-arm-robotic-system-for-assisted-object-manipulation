"""Waypoints across the WHOLE envelope, at fractions of each direction's
own measured reach -- not a fixed box.

Fractions rather than fixed distances because the envelope is wildly
anisotropic: +Y runs out at 253 mm while -Y has 1506 mm. A fixed step either
falls off the end of the short axis or barely leaves home on the long one,
which is exactly what the 100 mm version did.
"""
import json, math, os, sys
sys.path.insert(0, "scripts"); sys.path.insert(0, "config")
import numpy as np
from scipy.optimize import minimize
import solve_home_pose as S, home_positions as hp
sc = S.Scorer()
EXT = json.load(open(sys.argv[1]))
FRACS = [0.25, 0.50, 0.75, 1.00]
SEAM, CONT = 0.30, (0,2,4,6)
DIRS = {"+X":(1,0,0),"-X":(-1,0,0),"+Y":(0,1,0),"-Y":(0,-1,0),"+Z":(0,0,1),"-Z":(0,0,-1)}
def ee(a,q): return np.array(sc.arm_terms(a,q,with_clearance=False)["hand"],float)
def seam(q): return min(math.pi-abs(float(q[i])) for i in CONT)
out={"fracs":FRACS,"arms":{}}
for arm in ("left","right"):
    home=np.array(hp.load_home_radians(arm),float)
    lo,hi,_=sc.lim[arm]; p0=ee(arm,home)
    out["arms"][arm]={"home_rad":[float(v) for v in home],
                      "home_ee":[float(v) for v in p0],"targets":{}}
    print("\n%s"%arm.upper())
    for name,d in DIRS.items():
        reach=EXT["arms"][arm]["dirs"][name]["reach_m"]
        d=np.array(d,float); seed=home.copy()
        for f in FRACS:
            dist=reach*f; goal=p0+dist*d
            def cost(q):
                c=400.0*float(np.sum((ee(arm,q)-goal)**2))+0.05*float(np.sum((q-home)**2))
                for i in CONT:
                    sh=(SEAM+0.05)-(math.pi-abs(float(q[i])))
                    if sh>0: c+=50.0*sh*sh
                return c
            best=None
            for s0 in (seed,home):
                r=minimize(cost,s0,method="L-BFGS-B",bounds=list(zip(lo,hi)),
                           options=dict(maxiter=800,ftol=1e-14))
                if best is None or r.fun<best.fun: best=r
            q=best.x; err=float(np.linalg.norm(ee(arm,q)-goal)); m=seam(q)
            worst=min(sc.arm_terms(arm,home+(k/30.0)*(q-home))["clearance_m"]
                      for k in range(31))
            ok = err<0.008 and m>=SEAM and worst>=0.150
            key="%s@%d"%(name,int(f*100))
            out["arms"][arm]["targets"][key]={
                "q":[float(v) for v in q],"goal_ee":[float(v) for v in goal],
                "dist_m":round(dist,4),"ik_err_m":round(err,5),
                "seam_rad":round(m,4),"path_min_clearance_m":round(worst,4),
                "usable":bool(ok)}
            print("   %-8s %6.0f mm  ik %4.1f mm  seam %.2f  clear %.3f  %s"
                  %(key,dist*1000,err*1000,m,worst,"OK" if ok else "REJECT"))
            if ok: seed=q
json.dump(out,open(sys.argv[2],"w"),indent=1)
n=sum(1 for a in out["arms"].values() for t in a["targets"].values() if t["usable"])
print("\n%d usable waypoints -> %s"%(n,sys.argv[2]))
