import math, sys, time, statistics
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
sys.path.insert(0,"/home/gausms/kortex_ws/config")
import home_positions as hp
NAMES=["left_joint_%d"%i for i in range(1,8)]; CONT=(0,2,4,6)
TGT=list(hp.load_home_radians("left"))
def wrap(v):
    while v>math.pi: v-=2*math.pi
    while v<-math.pi: v+=2*math.pi
    return v
class R(Node):
    def __init__(self):
        super().__init__("srl_gap"); self.s=[]
        self.create_subscription(JointState,"/real/joint_states",self.cb,50)
    def cb(self,m):
        d=dict(zip(m.name,m.position))
        if all(k in d for k in NAMES): self.s.append([d[k] for k in NAMES])
rclpy.init(); n=R(); t=time.time()
while time.time()-t<6: rclpy.spin_once(n,timeout_sec=0.05)
if not n.s: print("no joint states"); sys.exit(2)
print("SIM (config) vs REAL, at rest -- %d samples over 6 s\n"%len(n.s))
print("  %-9s %11s %11s %10s %9s"%("joint","sim deg","real deg","err deg","noise"))
worst=0
for i in range(7):
    col=[q[i] for q in n.s]
    mean=statistics.fmean(col)
    sd=statistics.pstdev(col) if len(col)>1 else 0.0
    e=wrap(TGT[i]-mean) if i in CONT else TGT[i]-mean
    worst=max(worst,abs(e))
    print("  joint_%-3d %11.3f %11.3f %10.3f %9.4f"
          %(i+1,math.degrees(TGT[i]),math.degrees(mean),
            math.degrees(e),math.degrees(sd)))
print("\n  WORST STEADY-STATE DISAGREEMENT: %.3f deg (%.5f rad)"
      %(math.degrees(worst),worst))
print("  homing deadband 0.15 deg | exit deadband 0.35 | tolerance 2.86")
print("  bridge require_homed tolerance 0.05 rad = 2.86 deg")
n.destroy_node(); rclpy.shutdown()
