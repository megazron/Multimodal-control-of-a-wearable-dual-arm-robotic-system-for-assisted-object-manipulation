"""Scan the wrist RGB-D over the workspace; FastSAM segments, colour picks,
depth positions. Prints every candidate in ROBOT coordinates."""
import json, math, subprocess, sys, time, os
sys.path.insert(0,"src/srl_perception"); sys.path.insert(0,"scripts"); sys.path.insert(0,"config")
import numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from scipy.optimize import minimize
import solve_home_pose as S, home_positions as hp
from srl_perception import srl_cameras as CAMS
from srl_perception.prompt_detector import PromptDetector
try:
    cv2.setLogLevel(0)
except AttributeError:
    pass
SC=os.path.dirname(os.path.abspath(__file__))+"/"
IP="192.168.1.10"; ARM="left"
sc=S.Scorer(); IDX=S.IDX; CONT=(0,2,4,6); SEAM=0.30; FLOOR=0.150
rng=np.random.default_rng(21)
def cam(q):
    T=sc.cf[ARM](q)[IDX["camera_link"]]; return T[:3,3],T[:3,:3]
def solve_look(look,want,home,lo,hi):
    def cost(q):
        p,R=cam(q); z=R[:,2]; to=look-p; nn=np.linalg.norm(to)
        if nn<1e-6: return 1e3
        c=60.0*(1.0-float(np.dot(z,to/nn)))+6.0*float(np.sum((p-want)**2))
        for i in CONT:
            sh=(SEAM+0.05)-(math.pi-abs(float(q[i])))
            if sh>0: c+=20.0*sh*sh
        t=sc.arm_terms(ARM,q); sh=(FLOOR+0.03)-t["clearance_m"]
        if sh>0: c+=200.0*sh*sh
        return c
    best=None
    for s0 in [home]+[rng.uniform(lo,hi) for _ in range(14)]:
        r=minimize(cost,s0,method="L-BFGS-B",bounds=list(zip(lo,hi)),
                   options=dict(maxiter=350,ftol=1e-13))
        if best is None or r.fun<best.fun: best=r
    q=best.x; p,R=cam(q); to=look-p
    ang=math.degrees(math.acos(np.clip(np.dot(R[:,2],to/np.linalg.norm(to)),-1,1)))
    t=sc.arm_terms(ARM,q); m=min(math.pi-abs(float(q[i])) for i in CONT)
    pmin=min(sc.arm_terms(ARM,home+(k/25.)*(q-home))["clearance_m"] for k in range(26))
    return (q if (ang<35 and t["clearance_m"]>=FLOOR and m>=SEAM and pmin>=FLOOR) else None), ang, pmin
class N(Node):
    def __init__(self):
        super().__init__("srl_scan"); self.q={}
        self.create_subscription(JointState,"/real/joint_states",self._cb,50)
        self.pub=self.create_publisher(JointTrajectory,"/real/%s_arm_controller/joint_trajectory"%ARM,10)
    def _cb(self,m):
        for k,p in zip(m.name,m.position): self.q[k]=p
    def pose(self):
        nm=["%s_joint_%d"%(ARM,i) for i in range(1,8)]
        return np.array([self.q[k] for k in nm]) if all(k in self.q for k in nm) else None
    def spin(self,s):
        t=time.time()
        while time.time()-t<s: rclpy.spin_once(self,timeout_sec=0.02)
    def goto(self,q,tmo=45):
        t0=time.time()
        while time.time()-t0<tmo:
            m=JointTrajectory(); m.joint_names=["%s_joint_%d"%(ARM,i) for i in range(1,8)]
            pt=JointTrajectoryPoint(); pt.positions=[float(v) for v in q]
            pt.time_from_start=Duration(sec=3); m.points=[pt]; self.pub.publish(m)
            self.spin(0.08); p=self.pose()
            if p is None: continue
            e=max(abs((lambda v:(v+math.pi)%(2*math.pi)-math.pi)(q[i]-p[i])) if i in CONT
                  else abs(q[i]-p[i]) for i in range(7))
            if e<math.radians(0.8): break
        self.spin(1.5); return self.pose()
rclpy.init(); n=N(); n.spin(2.0)
if n.pose() is None: print("NO JOINT STATES"); sys.exit(2)
home=np.array(hp.load_home_radians(ARM),float); lo,hi,_=sc.lim[ARM]
det=PromptDetector(backend="segment", far_m=1.9)
gc=CAMS.GripperCamera(IP)
LOOKS=[np.array(v) for v in ([0.05,0.30,1.00],[0.20,0.30,1.00],[0.05,0.30,1.10],
                             [0.20,0.35,1.08],[0.10,0.22,0.95],[0.30,0.30,1.02])]
# STAND OFF 0.40 m ON THE ARM'S OWN SIDE, per look point. The previous version
# pinned the camera to one WANT for every aim, so all six poses put it within
# 10 mm of the same place -- six pictures of the same view, not a scan.
def want_for(L):
    # IN FRONT OF THE WEARER, LOOKING BACK AND DOWN.
    #
    # The previous standoff put the camera beside and above the look point,
    # so it stared into the mannequin's own chest -- the returned frames were
    # skin, harness strap and the teal robot behind. The cube sits on a tray
    # IN FRONT of the torso, so anything approaching from behind or beside is
    # occluded by the body itself.
    return np.array([L[0]+0.22, L[1]+0.30, L[2]+0.22])
found=[]
print("RGB-D SCAN: FastSAM segments -> green mean -> depth -> robot frame\n")
for k,L in enumerate(LOOKS):
    q,ang,pmin=solve_look(L,want_for(L),home,lo,hi)
    if q is None:
        print("  aim %d %s : no safe pose (%.0f deg, clear %.3f)"%(k,np.round(L,2),ang,pmin)); continue
    qa=n.goto(q); cp,cR=cam(qa)
    dep=None
    for attempt in range(3):
        try:
            col=gc.read_colour(); dep=gc.read_depth(seconds=9)
            break
        except Exception as e:
            if attempt==2:
                print("  aim %d stream after 3 tries: %s"%(k,str(e)[:80])); dep=None
            time.sleep(2.0)
    if dep is None: continue
    cv2.imwrite(SC+"scan_%d.png"%k,col)
    hits=det.detect(col,"the green cube",depth_m=dep,
                    K=CAMS.KINOVA_COLOR_K,depth_K=CAMS.KINOVA_DEPTH_K)
    msg=""
    for h in hits[:1]:
        if h.xyz_cam is None: msg="green seg, no depth"; continue
        P=cp+cR@h.xyz_cam
        msg="FOUND %s @ %.3f m  robot %s"%(h.extra["mean_bgr"],h.depth_m,np.round(P,3))
        found.append(dict(aim=k,xyz=[float(v) for v in P],z=h.depth_m,
                          area=h.extra["area_px"],bgr=h.extra["mean_bgr"]))
    print("  aim %d %s cam %s -> %d seg %s"%(k,np.round(L,2),np.round(cp,2),len(hits),msg))
gc.close(); n.goto(home)
json.dump(found,open(SC+"cube_found.json","w"),indent=1)
print("\nCANDIDATES: %d"%len(found))
if found:
    P=np.array([f["xyz"] for f in found])
    print("  mean %s   spread %s mm"%(np.round(P.mean(axis=0),3),np.round(P.std(axis=0)*1000,1)))
n.destroy_node(); rclpy.shutdown()
