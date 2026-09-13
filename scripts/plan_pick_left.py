"""Plan (and optionally execute) a top-down grasp of the located cube.

IK is damped least squares on the PAD MIDPOINT pose, with a finite-difference
Jacobian taken from srl_fk -- the same FK that produced the cube measurement,
so plan and measurement cannot drift apart.  Nothing here reads TF: the sim
TF tree disagreed with the real arm by 36 deg on joint 6 and that error is
exactly what put the table 27 deg off level earlier.
"""
import sys,json,math,time,os
sys.path.insert(0,"/home/gausms/kortex_ws/scripts")
import numpy as np
from srl_fk import FK

SC=os.environ.get("SRL_SCRATCH","/tmp/srl_pick")

# THE ARM IS A MODULE VARIABLE, NOT A LITERAL.  Every solve below used to say
# "left", so importing this to plan a RIGHT-arm grasp silently solved the
# LEFT arm's kinematics and returned joint angles for the wrong robot.  It
# also loaded a cube measurement from a scratchpad path AT IMPORT TIME, so
# once that session's directory was cleaned the module could not be imported
# at all -- and it is imported by the servo and by srl_pick_cube.
ARM = "left"

def set_arm(arm):
    """Point every solve in this module at one arm. Returns the previous."""
    global ARM
    prev, ARM = ARM, arm
    return prev

fk=FK()
LINKS=["end_effector_link","robotiq_85_left_finger_tip_link","robotiq_85_right_finger_tip_link"]

def pads(q,grip=0.0):
    Tee,Tl,Tr=fk.poses(ARM,q,LINKS,gripper=grip)
    mid=(Tl[:3,3]+Tr[:3,3])/2.0
    return Tee,mid

def pose_of(q,grip=0.0):
    """(pad-mid position, EE rotation)"""
    Tee,mid=pads(q,grip)
    return mid,Tee[:3,:3]

def rot_err(Rc,Rd):
    E=Rd@Rc.T
    v=np.array([E[2,1]-E[1,2],E[0,2]-E[2,0],E[1,0]-E[0,1]])
    s=np.linalg.norm(v); c=(np.trace(E)-1)/2
    if s<1e-9: return np.zeros(3) if c>0 else np.array([np.pi,0,0])
    return v/s*math.atan2(s,c)

def ik(p_des,R_des,q0,grip=0.0,iters=300,tol_p=3e-4,tol_r=0.5*math.pi/180):
    q=np.array(q0,float)
    lo,hi,cont=fk.limits(ARM)
    for it in range(iters):
        p,R=pose_of(q,grip)
        ep=p_des-p; er=rot_err(R,R_des)
        if np.linalg.norm(ep)<tol_p and np.linalg.norm(er)<tol_r:
            return q,np.linalg.norm(ep),np.linalg.norm(er),it
        J=np.zeros((6,7)); d=1e-6
        for j in range(7):
            qd=q.copy(); qd[j]+=d
            p2,R2=pose_of(qd,grip)
            J[:3,j]=(p2-p)/d
            J[3:,j]=rot_err(R,R2)/d
        e=np.r_[ep,er]
        lam=0.05
        dq=J.T@np.linalg.solve(J@J.T+lam*lam*np.eye(6),e)
        n=np.linalg.norm(dq)
        if n>0.15: dq*=0.15/n
        q=q+dq
        for j in range(7):
            if not cont[j]: q[j]=min(hi[j],max(lo[j],q[j]))
    p,R=pose_of(q,grip)
    return q,np.linalg.norm(p_des-p),np.linalg.norm(rot_err(R,R_des)),iters

def _main():
    # ---------------- build the grasp frame ----------------
    CUBE=json.load(open(SC+"/cube_v2.json"))
    q_now=np.array(CUBE["q"],float)
    cube_w=np.array(CUBE["grasp_world"])
    nrm=np.array(CUBE["table_normal_world"]); nrm/=np.linalg.norm(nrm)

    Tee0,mid0=pads(q_now)
    approach_world=(mid0-Tee0[:3,3]); approach_world/=np.linalg.norm(approach_world)
    approach_ee=Tee0[:3,:3].T@approach_world
    print("pad-mid offset from EE  : %.5f m"%np.linalg.norm(mid0-Tee0[:3,3]))
    print("approach axis in EE frame: [%+.3f %+.3f %+.3f]"%tuple(approach_ee))
    print("current pad mid (world) : (%+.4f,%+.4f,%+.4f)"%tuple(mid0))
    print("cube grasp centre       : (%+.4f,%+.4f,%+.4f)"%tuple(cube_w))
    print("table normal            : (%+.3f,%+.3f,%+.3f)"%tuple(nrm))

    # desired: approach axis points DOWN into the table (-normal)
    a_des=-nrm
    # free spin about a_des: pick the one closest to the current wrist
    def R_from_axis(a_des,ref_R):
        a_ee=approach_ee/np.linalg.norm(approach_ee)
        # any R with R@a_ee == a_des ; start from minimal rotation off current
        a_cur=ref_R@a_ee
        v=np.cross(a_cur,a_des); s=np.linalg.norm(v); c=float(a_cur@a_des)
        if s<1e-9:
            Rd=np.eye(3) if c>0 else -np.eye(3)
        else:
            vx=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
            Rd=np.eye(3)+vx+vx@vx*((1-c)/(s*s))
        return Rd@ref_R
    R_des=R_from_axis(a_des,Tee0[:3,:3])
    chk=R_des@(approach_ee/np.linalg.norm(approach_ee))
    print("achieved approach dir   : (%+.3f,%+.3f,%+.3f)  vs desired (%+.3f,%+.3f,%+.3f)"%(*chk,*a_des))

    OPEN=0.0; GRIP=0.55
    # THE PAD MIDPOINT IS A CURVE, NOT A CONSTANT.
    #
    # The Robotiq 85's fingers swing on a four-bar, so wrist-to-pad depends on how
    # far the hand is closed: 0.09833 m wide open, 0.10976 m closed on a 40 mm
    # cube -- 11.43 mm apart.  Solving the grasp with the hand OPEN and then
    # closing it drives the pads 11.43 mm FURTHER along the tool axis than
    # planned, which is straight into the table.  This is docs/ENGINEERING_LOG.md's own
    # "T1's grasp was built 11.43 mm short".
    #
    # So the GRASP pose is solved at the opening the hand will actually be at when
    # it holds the cube.  Measured from FK: finger-tip separation minus 50.7 mm is
    # the real gap, giving 84.8 mm at 0.0 rad (the Robotiq 85's rated 85 mm) and
    # 40 mm at 0.447 rad.
    CUBE_GRIP=0.447
    STAND=0.10; LIFT=0.12
    stages=[("PRE-GRASP", cube_w+nrm*STAND, OPEN),
            ("GRASP",     cube_w,           CUBE_GRIP),
            ("LIFT",      cube_w+nrm*LIFT,  GRIP)]
    lo,hi,cont=fk.limits(ARM)
    plan=[]; q=q_now.copy()
    print("\n%-10s %-34s %8s %8s %6s %8s"%("stage","pad-mid target (world)","pos_mm","rot_deg","iters","max_dq"))
    ok=True
    for name,tgt,grip in stages:
        qs,ep,er,it=ik(tgt,R_des,q,grip=grip)
        dq=np.degrees(np.abs(qs-q)).max()
        p,_=pose_of(qs,grip)
        # table clearance of the pad mid
        clr=(p-cube_w)@nrm + CUBE["height_mm"]/2000.0
        flag="" if (ep<0.002 and er<0.02) else "   <-- POOR"
        if ep>=0.002 or er>=0.02: ok=False
        print("%-10s (%+.4f,%+.4f,%+.4f) %8.2f %8.2f %6d %8.2f%s"%(name,*tgt,ep*1000,math.degrees(er),it,dq,flag))
        plan.append({"name":name,"q":qs.tolist(),"grip":grip,"pos_err_mm":ep*1000,
                     "rot_err_deg":math.degrees(er),"max_dq_deg":dq,
                     "pad_above_table_mm":float(clr*1000)})
        q=qs
    print("\nIK all converged:",ok)
    for s in plan:
        print("  %-10s q(deg)= %s"%(s["name"],[round(math.degrees(x),2) for x in s["q"]]))
        print("             pad above table: %.1f mm"%s["pad_above_table_mm"])
    # joint-limit check
    for s in plan:
        q_=np.array(s["q"])
        bad=[(i+1,round(math.degrees(q_[i]),2)) for i in range(7)
             if not cont[i] and (q_[i]<lo[i]-1e-6 or q_[i]>hi[i]+1e-6)]
        if bad: print("  !! %s out of limits: %s"%(s["name"],bad)); ok=False
    json.dump({"plan":plan,"ok":bool(ok),"R_des":R_des.tolist(),"normal":nrm.tolist(),
               "cube_world":cube_w.tolist()},open(SC+"/pick_plan.json","w"),indent=1)
    print("\nplan written. SAFE TO EXECUTE:",ok)


if __name__ == '__main__':
    _main()
