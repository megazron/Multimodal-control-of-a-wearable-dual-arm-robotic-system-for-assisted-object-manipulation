"""REAL ARM CONNECTION INDICATORS -- one line per fact, no inference."""
import math, os, subprocess, sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
sys.path.insert(0,"/home/gausms/kortex_ws/config")
import home_positions as hp
CONT=(0,2,4,6)
def wrap(v):
    while v>math.pi: v-=2*math.pi
    while v<-math.pi: v+=2*math.pi
    return v
IPS={"left":"192.168.1.10","right":"192.168.1.9"}
class S(Node):
    def __init__(self):
        super().__init__("srl_status")
        self.real={}; self.sim={}; self.estop=None
        self.create_subscription(JointState,"/real/joint_states",
            lambda m:self.grab(m,self.real),50)
        self.create_subscription(JointState,"/joint_states",
            lambda m:self.grab(m,self.sim),50)
        self.create_subscription(Bool,"/estop_state",
            lambda m:setattr(self,"estop",m.data),10)
    def grab(self,m,d):
        for n,p in zip(m.name,m.position): d[n]=p
rclpy.init(); s=S(); t=time.time()
while time.time()-t<5: rclpy.spin_once(s,timeout_sec=0.05)
print("="*66)
print("REAL ARM CONNECTION INDICATORS      %s"%time.strftime("%H:%M:%S"))
print("="*66)
for arm,ip in IPS.items():
    ping=subprocess.run(["ping","-c","1","-W","2",ip],
        capture_output=True).returncode==0
    proc=subprocess.run(["pgrep","-f","kortex_highlevel_bridge.*arm:=%s"%arm],
        capture_output=True).stdout.decode().strip()
    names=["%s_joint_%d"%(arm,i) for i in range(1,8)]
    live=all(n in s.real for n in names)
    print("\n  %s ARM  (%s)"%(arm.upper(),ip))
    print("    network reachable      %s"%("YES" if ping else "NO"))
    print("    kortex session/driver  %s"%("UP (pid %s)"%proc if proc else "NOT RUNNING"))
    print("    /real/joint_states     %s"%("LIVE" if live else "ABSENT"))
    if live:
        cur=[s.real[n] for n in names]
        tgt=list(hp.load_home_radians(arm))
        e=[wrap(tgt[i]-cur[i]) if i in CONT else tgt[i]-cur[i] for i in range(7)]
        w=max(range(7),key=lambda i:abs(e[i]))
        print("    real vs SIM-config     worst %.3f deg on joint_%d"
              %(math.degrees(abs(e[w])),w+1))
        print("    AT HOME?               %s (gate 2.86 deg)"
              %("YES" if abs(e[w])<=math.radians(2.86) else "NO"))
    simn=["%s_joint_%d"%(arm,i) for i in range(1,8)]
    print("    sim /joint_states      %s"%("LIVE" if all(n in s.sim for n in simn) else "ABSENT"))
print("\n  E-STOP  /estop_state = %s"%
      ("LATCHED (STOPPED)" if s.estop else ("clear" if s.estop is False else "NO PUBLISHER")))
print("  camera  %s"%("/dev/video0 present" if os.path.exists("/dev/video0") else "absent"))
print("="*66)
s.destroy_node(); rclpy.shutdown()
