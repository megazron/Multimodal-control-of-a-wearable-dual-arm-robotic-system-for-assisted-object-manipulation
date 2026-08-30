import sys,time,math,json,statistics
sys.path.insert(0,"/home/gausms/kortex_ws/scripts")
import numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo,JointState
from rclpy.qos import qos_profile_sensor_data
st={}; js=[]
class G(Node):
    def __init__(s):
        super().__init__("locate_cube")
        f=lambda T,t,k: s.create_subscription(T,t,lambda m,k=k: st.setdefault(k,m),qos_profile_sensor_data)
        f(Image,"/left_camera/color/image_raw","c");  f(CameraInfo,"/left_camera/color/camera_info","ck")
        f(Image,"/left_camera/depth/image_raw","d");  f(CameraInfo,"/left_camera/depth/camera_info","dk")
        s.create_subscription(JointState,"/real/joint_states",lambda m: js.append(m),10)
def plane(P,it=500,tol=0.006,seed=0):
    rng=np.random.default_rng(seed); best=None;bc=-1
    for _ in range(it):
        p=P[rng.choice(len(P),3,replace=False)]
        nn=np.cross(p[1]-p[0],p[2]-p[0]); L=np.linalg.norm(nn)
        if L<1e-9: continue
        nn/=L; dd=-nn@p[0]; cc=int((np.abs(P@nn+dd)<tol).sum())
        if cc>bc: bc,best=cc,(nn,dd)
    nn,dd=best; m=np.abs(P@nn+dd)<tol; Q=P[m]; cen=Q.mean(0)
    C=np.cov((Q-cen).T); w_,V_=np.linalg.eigh(C); nn=V_[:,0]/np.linalg.norm(V_[:,0]); dd=-nn@cen
    return nn,dd,np.abs(P@nn+dd)<tol
rclpy.init(); n=G(); t0=time.time()
while time.time()-t0<30 and (len(st)<4 or len(js)<10): rclpy.spin_once(n,timeout_sec=0.05)
assert len(st)==4 and js, (sorted(st), len(js))
c,ck,d,dk=st["c"],st["ck"],st["d"],st["dk"]
z={}
for m in js:
    for nm,p in zip(m.name,m.position): z.setdefault(nm,[]).append(p)
q=[statistics.median(z[f"left_joint_{i}"]) for i in range(1,8)]
print("real joints (deg):",[round(math.degrees(x),3) for x in q])
from srl_fk import FK
fk=FK()
Tw_d,Tw_c,Tw_b,Tw_ee=fk.poses("left",q,["camera_depth_frame","camera_color_frame","base_link","end_effector_link"])
T_cd=np.linalg.inv(Tw_c)@Tw_d          # depth -> colour
# ---- deproject depth ----
dep=np.frombuffer(d.data,np.uint16).reshape(d.height,d.width).astype(np.float32)*0.001
fx,fy,cx,cy=dk.k[0],dk.k[4],dk.k[2],dk.k[5]
v,u=np.nonzero((dep>0.10)&(dep<1.5)); Z=dep[v,u]; X=(u-cx)*Z/fx; Y=(v-cy)*Z/fy
P=np.stack([X,Y,Z],1)
nrm,dd,minl=plane(P)
if dd<0: nrm,dd=-nrm,-dd
h=P@nrm+dd
# ---- colour gate ----
col=np.frombuffer(c.data,np.uint8).reshape(c.height,c.width,-1)
if c.encoding=="rgb8": col=cv2.cvtColor(col,cv2.COLOR_RGB2BGR)
hsv=cv2.cvtColor(col,cv2.COLOR_BGR2HSV)
mask=cv2.inRange(hsv,(40,80,40),(85,255,255))
mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((9,9),np.uint8))
Pcol=(T_cd@np.c_[P,np.ones(len(P))].T).T[:,:3]
uu=Pcol[:,0]*ck.k[0]/Pcol[:,2]+ck.k[2]; vv=Pcol[:,1]*ck.k[4]/Pcol[:,2]+ck.k[5]
ok=(uu>=0)&(uu<c.width)&(vv>=0)&(vv<c.height)
green=np.zeros(len(P),bool); green[ok]=mask[vv[ok].astype(int),uu[ok].astype(int)]>0
cube=green&(h>0.005)
print(f"table plane: {minl.mean()*100:.1f}% inliers, RMS {np.sqrt((h[minl]**2).mean())*1000:.2f} mm")
print(f"green px {int(mask.sum()//255)}, cube depth points {int(cube.sum())}")
Pc=P[cube]
e1=np.cross(nrm,[0,0,1.0]); e1/=np.linalg.norm(e1); e2=np.cross(nrm,e1)
a,b=Pc@e1,Pc@e2; hc=Pc@nrm+dd
print(f"cube footprint {(a.max()-a.min())*1000:.1f} x {(b.max()-b.min())*1000:.1f} mm, height {hc.max()*1000:.1f} mm")
cen_cam=Pc.mean(0)
# grasp point: centre of the cube body (half height above table, under the centroid)
foot=cen_cam-nrm*(cen_cam@nrm+dd)
grasp_cam=foot+nrm*(hc.max()/2)
def tf(T,p): return (T@np.r_[p,1])[:3]
gw=tf(Tw_d,grasp_cam); gb=tf(np.linalg.inv(Tw_b)@Tw_d,grasp_cam)
nw=Tw_d[:3,:3]@nrm
if nw[2]<0: nw=-nw
tilt=math.degrees(math.acos(min(1,nw[2])))
print(f"\nrange camera->cube: {np.linalg.norm(grasp_cam):.4f} m")
print(f"CUBE GRASP CENTRE  world           : ({gw[0]:+.4f},{gw[1]:+.4f},{gw[2]:+.4f})")
print(f"CUBE GRASP CENTRE  left_base_link  : ({gb[0]:+.4f},{gb[1]:+.4f},{gb[2]:+.4f})")
print(f"table normal world ({nw[0]:+.3f},{nw[1]:+.3f},{nw[2]:+.3f}) -> {tilt:.2f} deg off vertical  [SANITY: should be small]")
eew=tf(Tw_ee,[0,0,0])
print(f"EE now world: ({eew[0]:+.4f},{eew[1]:+.4f},{eew[2]:+.4f})   EE->cube dist {np.linalg.norm(gw-eew):.4f} m")
json.dump({"q":q,"grasp_world":gw.tolist(),"grasp_base":gb.tolist(),"grasp_cam":grasp_cam.tolist(),
"table_normal_world":nw.tolist(),"tilt_deg":tilt,"height_mm":float(hc.max()*1000),
"footprint_mm":[float((a.max()-a.min())*1000),float((b.max()-b.min())*1000)],
"plane_rms_mm":float(np.sqrt((h[minl]**2).mean())*1000),"n_cube":int(cube.sum())},
 open("/tmp/claude-1000/-home-gausms-kortex-ws/0353f722-b129-4dcb-bf5e-6f4c8caf94a7/scratchpad/cube_v2.json","w"),indent=1)
ov=col.copy()
cont,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE); cv2.drawContours(ov,cont,-1,(0,255,255),2)
gu,gv=uu[cube].astype(int),vv[cube].astype(int)
for Xp,Yp in zip(gu,gv): cv2.circle(ov,(Xp,Yp),1,(0,0,255),-1)
cv2.imwrite("/tmp/claude-1000/-home-gausms-kortex-ws/0353f722-b129-4dcb-bf5e-6f4c8caf94a7/scratchpad/cam/locate.png",ov)
print("wrote locate.png")
