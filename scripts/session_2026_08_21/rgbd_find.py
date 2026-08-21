"""FIND OBJECTS WITH THE GRIPPER'S RGB-D CAMERA AND PUT THEM IN ROBOT COORDINATES.

Colour gives WHAT, depth gives WHERE, and depth also settles the argument
colour alone could not: this room contains teal robots whose shadowed hue
(76) is within 2 of the green cube's (74), and they already fooled two
filters. They are also 2.8 m away while anything on the rig is under 1.5 m,
so a range gate separates them on a property that has nothing to do with
colour.

Intrinsics are the FACTORY values read from the robot over the Kortex API,
not nominal figures off a spec sheet. That matters: the colour principal
point is cy = 238.3, not the 360 that "centre of a 720-high image" would
suggest, and using the wrong one throws every ray by 122 px.
"""
import json, math, subprocess, sys, time
sys.path.insert(0,"scripts"); sys.path.insert(0,"config")
import numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import solve_home_pose as S
cv2.setLogLevel(0)
SC="/tmp/claude-1000/-home-gausms-kortex-ws/6fb0cbe9-3047-4da2-9236-822179e1174c/scratchpad/"
IP="192.168.1.10"; ARM="left"
C=dict(w=1280,h=720,fx=1297.6729,fy=1298.6313,cx=620.914,cy=238.28032)
D=dict(w=480,h=270,fx=342.2138,fy=342.2138,cx=233.06856,cy=132.48465)
sc=S.Scorer()

def grab_depth(sec=8):
    raw=SC+"d.raw"
    subprocess.run(["bash","-c","rm -f %s; timeout %d gst-launch-1.0 rtspsrc "
        "location=rtsp://%s/depth latency=30 ! rtpgstdepay ! filesink location=%s "
        ">/dev/null 2>&1"%(raw,sec,IP,raw)])
    b=open(raw,"rb").read(); FS=D["w"]*D["h"]*2
    if len(b)<FS: return None
    return np.frombuffer(b[-FS:],dtype="<u2").reshape(D["h"],D["w"]).astype(np.float32)

def grab_color(sec=8):
    cap=cv2.VideoCapture("rtsp://%s/color"%IP,cv2.CAP_FFMPEG)
    f=None; t0=time.time()
    while time.time()-t0<sec:
        ok,x=cap.read()
        if ok and x is not None: f=x
        if f is not None and time.time()-t0>3: break
    cap.release(); return f

class J(Node):
    def __init__(self):
        super().__init__("srl_rgbd"); self.q={}
        self.create_subscription(JointState,"/real/joint_states",self._cb,50)
    def _cb(self,m):
        for n,p in zip(m.name,m.position): self.q[n]=p
    def pose(self):
        nm=["%s_joint_%d"%(ARM,i) for i in range(1,8)]
        return np.array([self.q[n] for n in nm]) if all(n in self.q for n in nm) else None

rclpy.init(); n=J()
t0=time.time()
while time.time()-t0<8 and n.pose() is None: rclpy.spin_once(n,timeout_sec=0.05)
q=n.pose()
if q is None: print("no joint states"); sys.exit(2)
M=sc.cf[ARM](q); T=M[S.IDX["camera_link"]]
cam_p=T[:3,3]; cam_R=T[:3,:3]
print("camera at %s in robot frame"%np.round(cam_p,3))

col=grab_color(); dep=grab_depth()
if col is None or dep is None: print("stream failure"); sys.exit(3)
cv2.imwrite(SC+"rgbd_color.png",col)
print("colour %dx%d   depth %dx%d  valid %.0f%%"
      %(col.shape[1],col.shape[0],dep.shape[1],dep.shape[0],
        100*np.count_nonzero(dep)/dep.size))

hsv=cv2.cvtColor(col,cv2.COLOR_BGR2HSV)
mask=cv2.inRange(hsv,np.array([40,90,20]),np.array([90,255,255]))
mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
nn,lab,st,ce=cv2.connectedComponentsWithStats(mask,8)
print("\n%-6s %-10s %-18s %-8s %-22s %s"%("blob","area","pixel","depth m","3D robot frame","verdict"))
found=[]
for i in range(1,nn):
    a=st[i,cv2.CC_STAT_AREA]
    if a<50: continue
    u,v=ce[i]
    # colour pixel -> bearing (factory intrinsics)
    d_cam=np.array([(u-C["cx"])/C["fx"],(v-C["cy"])/C["fy"],1.0])
    # same bearing projected into the DEPTH image
    du=D["cx"]+d_cam[0]*D["fx"]; dv=D["cy"]+d_cam[1]*D["fy"]
    if not (0<=du<D["w"] and 0<=dv<D["h"]):
        print("%-6d %-10d (%4.0f,%4.0f)      %-8s %-22s outside depth FOV"%(i,a,u,v,"-","-")); continue
    win=dep[max(0,int(dv)-3):int(dv)+4, max(0,int(du)-3):int(du)+4]
    val=win[win>0]
    if val.size==0:
        print("%-6d %-10d (%4.0f,%4.0f)      %-8s %-22s no depth return"%(i,a,u,v,"-","-")); continue
    z=float(np.median(val))/1000.0
    P_cam=d_cam/d_cam[2]*z
    P=cam_p+cam_R@P_cam
    px=hsv[int(v),int(u)]
    near = z<1.6
    bgr=col[int(v),int(u)]
    green = int(bgr[1])>int(bgr[0])*1.25 and int(bgr[1])>int(bgr[2])*1.8
    verdict=("CUBE CANDIDATE" if (near and green) else
             "too far (%.1f m)"%z if not near else "not green (B/G %.2f)"%(bgr[0]/max(bgr[1],1)))
    print("%-6d %-10d (%4.0f,%4.0f)      %-8.3f %-22s %s"
          %(i,a,u,v,z,str(np.round(P,3)),verdict))
    if near and green: found.append(dict(uv=[float(u),float(v)],z=z,
                                         xyz=[float(x) for x in P],area=int(a)))
json.dump(dict(cam_p=[float(x) for x in cam_p],found=found),
          open(SC+"rgbd_found.json","w"),indent=1)
print("\ncube candidates: %d"%len(found))
n.destroy_node(); rclpy.shutdown()
