"""REAL-ARM DIRECTIONAL + SPEED CALIBRATION, with scene-camera verification.

For each arm, each of the six Cartesian directions, and each commanded joint
speed: drive there, measure what actually happened, photograph it, come back.

WHAT IS MEASURED, and why each one is here
  settle_s        how long until the arm stops moving. The number a shared
                  autonomy layer needs to know before it can plan a dwell.
  ss_joint_deg    steady-state joint error. This is the DEADBAND, and it is
                  the floor on everything downstream.
  ss_cart_mm      steady-state Cartesian error of the end effector against the
                  position that was asked for. Joint error is what the
                  controller sees; THIS is what the gripper misses by.
  overshoot_mm    peak excursion past the target. A rate limit hides this at
                  low speed and cannot at high speed -- which is the whole
                  point of sweeping speed.
  peak_speed      achieved joint speed, so the commanded vmax can be checked
                  against what the link actually delivered rather than
                  assumed.

FK IS THE SAME COMPILED CHAIN the rest of this repo measures with, so the
target and the achievement are scored on one model and cannot disagree by
construction.
"""
import json, math, os, sys, time
sys.path.insert(0, os.path.join(os.getcwd(), "scripts"))
import numpy as np, cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import solve_home_pose as S
cv2.setLogLevel(0)

OUT   = sys.argv[1]
FRAMES= sys.argv[2]
TGT   = json.load(open(sys.argv[3]))
SPEEDS= [0.40]                       # reach map first; speed sweep is a second pass
CONT  = (0, 2, 4, 6)
os.makedirs(FRAMES, exist_ok=True)
sc = S.Scorer()

def wrap(v):
    while v > math.pi: v -= 2*math.pi
    while v < -math.pi: v += 2*math.pi
    return v

class Cal(Node):
    def __init__(self):
        super().__init__("srl_calibrate")
        self.q = {}
        self.create_subscription(JointState, "/real/joint_states", self._cb, 50)
        self.pub = {a: self.create_publisher(
            JointTrajectory, "/real/%s_arm_controller/joint_trajectory" % a, 10)
            for a in ("left", "right")}
        self.par = {a: self.create_client(
            SetParameters, "/kortex_highlevel_bridge_%s/set_parameters" % a)
            for a in ("left", "right")}
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        for _ in range(8): self.cap.read()
    def _cb(self, m):
        for n, p in zip(m.name, m.position): self.q[n] = p
    def pose(self, arm):
        names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
        if not all(n in self.q for n in names): return None
        return np.array([self.q[n] for n in names], float)
    def spin(self, s):
        t = time.time()
        while time.time()-t < s: rclpy.spin_once(self, timeout_sec=0.01)
    def send(self, arm, q):
        m = JointTrajectory()
        m.joint_names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
        p = JointTrajectoryPoint(); p.positions = [float(v) for v in q]
        p.time_from_start = Duration(sec=2)
        m.points = [p]; self.pub[arm].publish(m)
    def set_vmax(self, arm, v):
        cli = self.par[arm]
        if not cli.wait_for_service(timeout_sec=4.0): return False
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name="vmax_rad_s",
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(v)))]
        f = cli.call_async(req); t = time.time()
        while time.time()-t < 6 and not f.done(): rclpy.spin_once(self, timeout_sec=0.02)
        return f.done()
    def shot(self, tag):
        best = None
        for _ in range(5):
            ok, f = self.cap.read()
            if ok and f is not None: best = f
        if best is None: return None, None
        p = os.path.join(FRAMES, tag + ".jpg")
        cv2.imwrite(p, best, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return p, float(best.mean())

    def drive(self, arm, qgoal, goal_ee, timeout=75.0):
        """Command, track the whole transient, return the measurements."""
        t0 = time.time(); samples = []; last = None; still = 0.0
        while time.time()-t0 < timeout:
            self.send(arm, qgoal); self.spin(0.06)
            p = self.pose(arm)
            if p is None: continue
            samples.append((time.time()-t0, p.copy()))
            e = max(abs(wrap(qgoal[i]-p[i]) if i in CONT else qgoal[i]-p[i])
                    for i in range(7))
            if last is not None:
                moved = max(abs(p[i]-last[i]) for i in range(7))
                still = still + 0.06 if moved < 2e-4 else 0.0
            last = p.copy()
            if e < math.radians(0.35) and still > 1.2: break
        self.spin(1.0)
        fin = self.pose(arm)
        # steady state
        ss_j = [math.degrees(wrap(qgoal[i]-fin[i]) if i in CONT
                             else qgoal[i]-fin[i]) for i in range(7)]
        ee_fin = np.array(sc.arm_terms(arm, fin, with_clearance=False)["hand"])
        ss_cart = float(np.linalg.norm(ee_fin - np.array(goal_ee)))
        # overshoot: furthest the EE got PAST the goal along the travel axis
        over = 0.0; peak = 0.0
        if len(samples) > 2:
            ee0 = np.array(sc.arm_terms(arm, samples[0][1],
                                        with_clearance=False)["hand"])
            axis = np.array(goal_ee) - ee0
            n = float(np.linalg.norm(axis))
            if n > 1e-6:
                u = axis/n
                for (_, qq) in samples:
                    pe = np.array(sc.arm_terms(arm, qq,
                                               with_clearance=False)["hand"])
                    proj = float(np.dot(pe-ee0, u))
                    over = max(over, proj-n)
            for k in range(1, len(samples)):
                dt = samples[k][0]-samples[k-1][0]
                if dt > 1e-3:
                    peak = max(peak, max(abs(samples[k][1]-samples[k-1][1]))/dt)
        return dict(settle_s=round(samples[-1][0], 2) if samples else None,
                    ss_joint_deg=[round(v, 4) for v in ss_j],
                    ss_joint_worst_deg=round(max(abs(v) for v in ss_j), 4),
                    ss_cart_mm=round(ss_cart*1000, 3),
                    overshoot_mm=round(over*1000, 3),
                    peak_joint_speed_rad_s=round(peak, 4),
                    achieved_ee=[round(float(v), 5) for v in ee_fin],
                    n_samples=len(samples))

rclpy.init(); c = Cal(); c.spin(2.0)
res = {"fracs": TGT.get("fracs"), "speeds_rad_s": SPEEDS, "runs": []}
if c.pose("left") is None or c.pose("right") is None:
    print("NO JOINT STATES"); sys.exit(2)
print("CALIBRATION: %d arms x %d speeds x 6 directions\n"
      % (2, len(SPEEDS)))
for arm in ("left", "right"):
    home = np.array(TGT["arms"][arm]["home_rad"], float)
    for v in SPEEDS:
        c.set_vmax(arm, v)
        print("== %s arm, vmax %.2f rad/s ==" % (arm.upper(), v))
        for name, spec in TGT["arms"][arm]["targets"].items():
            if not spec["usable"]: continue
            qg = np.array(spec["q"], float)
            m = c.drive(arm, qg, spec["goal_ee"])
            tag = "%s_v%03d_%s" % (arm, int(v*100), name.replace("+","p").replace("-","m"))
            fp, mean = c.shot(tag)
            m.update(arm=arm, vmax_rad_s=v, direction=name,
                     dist_m=spec.get("dist_m"), seam_rad=spec.get("seam_rad"),
                     goal_ee=spec["goal_ee"], frame=fp, frame_mean=mean)
            res["runs"].append(m)
            print("   %-3s settle %5.2fs  ss_joint %6.3f deg  ss_cart %7.3f mm"
                  "  overshoot %6.3f mm  peak %.3f rad/s"
                  % (name, m["settle_s"] or -1, m["ss_joint_worst_deg"],
                     m["ss_cart_mm"], m["overshoot_mm"],
                     m["peak_joint_speed_rad_s"]))
            c.drive(arm, home, TGT["arms"][arm]["home_ee"])   # back to home
        json.dump(res, open(OUT, "w"), indent=1)
    c.set_vmax(arm, 0.30)
json.dump(res, open(OUT, "w"), indent=1)
print("\n-> %s   (%d runs, frames in %s)" % (OUT, len(res["runs"]), FRAMES))
c.cap.release(); c.destroy_node(); rclpy.shutdown()
