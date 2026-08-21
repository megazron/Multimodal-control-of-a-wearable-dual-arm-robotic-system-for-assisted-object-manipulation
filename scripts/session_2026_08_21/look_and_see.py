import json, math, sys, time, os
sys.path.insert(0,"scripts"); sys.path.insert(0,"config")
import numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import solve_home_pose as S
cv2.setLogLevel(0)
SC="/tmp/claude-1000/-home-gausms-kortex-ws/6fb0cbe9-3047-4da2-9236-822179e1174c/scratchpad/"
POSES=json.load(open(SC+"look_poses.json"))
ARM=sys.argv[1]; VIEW=int(sys.argv[2]); IP=sys.argv[3]
sc=S.Scorer(); CONT=(0,2,4,6)
def wrap(v):
    while v>math.pi: v-=2*math.pi
    while v<-math.pi: v+=2*math.pi
    return v
class N(Node):
    def __init__(self):
        super().__init__("srl_look"); self.q={}
        self.create_subscription(JointState,"/real/joint_states",self._cb,50)
        self.pub=self.create_publisher(JointTrajectory,
            "/real/%s_arm_controller/joint_trajectory"%ARM,10)
    def _cb(self,m):
        for n,p in zip(m.name,m.position): self.q[n]=p
    def pose(self):
        nm=["%s_joint_%d"%(ARM,i) for i in range(1,8)]
        return np.array([self.q[n] for n in nm]) if all(n in self.q for n in nm) else None
    def spin(self,s):
        t=time.time()
        while time.time()-t<s: rclpy.spin_once(self,timeout_sec=0.02)
    def send(self,q):
        m=JointTrajectory(); m.joint_names=["%s_joint_%d"%(ARM,i) for i in range(1,8)]
        p=JointTrajectoryPoint(); p.positions=[float(v) for v in q]
        p.time_from_start=Duration(sec=3); m.points=[p]; self.pub.publish(m)
rclpy.init(); n=N(); n.spin(2.0)
if n.pose() is None: print("no joint states"); sys.exit(2)
qg=np.array(POSES[ARM][VIEW]["q"],float)
print("moving %s to view %d ..."%(ARM,VIEW))
t0=time.time()
while time.time()-t0<60:
    n.send(qg); n.spin(0.08)
    p=n.pose()
    e=max(abs(wrap(qg[i]-p[i]) if i in CONT else qg[i]-p[i]) for i in range(7))
    if e<math.radians(0.6): break
n.spin(2.0)
p=n.pose()
M=sc.cf[ARM](p); C=M[S.IDX["camera_link"]]
print("  arrived. camera at %s  axis %s"%(np.round(C[:3,3],3),np.round(C[:3,2],3)))
json.dump({"arm":ARM,"view":VIEW,"q":[float(v) for v in p],
           "cam_pos":[float(v) for v in C[:3,3]],
           "cam_R":[[float(x) for x in r] for r in C[:3,:3]]},
          open(SC+"look_%s_%d.json"%(ARM,VIEW),"w"),indent=1)
cap=cv2.VideoCapture("rtsp://%s/color"%IP, cv2.CAP_FFMPEG)
f=None; t0=time.time()
while time.time()-t0<15:
    ok,x=cap.read()
    if ok and x is not None: f=x
    if f is not None and time.time()-t0>3: break
cap.release()
if f is None: print("  NO WRIST FRAME"); sys.exit(3)
path=SC+"wrist_%s_%d.png"%(ARM,VIEW); cv2.imwrite(path,f)
print("  wrist frame %dx%d -> %s"%(f.shape[1],f.shape[0],path))
hsv=cv2.cvtColor(f,cv2.COLOR_BGR2HSV)
m=cv2.inRange(hsv,np.array([35,70,50]),np.array([88,255,255]))
m=cv2.morphologyEx(m,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
nn,lab,st,ce=cv2.connectedComponentsWithStats(m,8)
b=[(st[i,cv2.CC_STAT_AREA],i) for i in range(1,nn) if st[i,cv2.CC_STAT_AREA]>120]
b.sort(reverse=True)
print("  green blobs: %d"%len(b))
for a,i in b[:3]:
    print("     area %5d  centre (%.0f,%.0f)  bbox %dx%d"
          %(a,ce[i][0],ce[i][1],st[i,cv2.CC_STAT_WIDTH],st[i,cv2.CC_STAT_HEIGHT]))
n.destroy_node(); rclpy.shutdown()
