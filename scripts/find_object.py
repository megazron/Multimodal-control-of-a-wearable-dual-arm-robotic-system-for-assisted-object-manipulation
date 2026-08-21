#!/usr/bin/env python3
"""Say what you want; get its position, a grasp, and whether an arm can reach.

    python3 scripts/find_object.py "the green cube"
    python3 scripts/find_object.py "a bottle" --camera scene
    python3 scripts/find_object.py "the green cube" --arm right --json out.json

MOVES NOTHING. It looks, plans and reports. Commanding the arm is a separate,
deliberate act -- this is the step you run first to find out whether the thing
you want is somewhere the arm can actually go.

WHICH CAMERA, AND WHY IT MATTERS
  gripper  the Kinova wrist module: colour AND depth, so a detection becomes
           a POSITION and a grasp can be planned. Needs the arm powered and
           on the network, and the arm's joint states, because the camera's
           pose comes from forward kinematics.
  scene    the room webcam: colour only. It can say what it sees and where in
           the image, and it CANNOT say how far away, so it cannot produce a
           grasp. Reported as such rather than quietly returning a 2D answer.

RUN IT UNDER .venv_vision FOR OPEN-VOCABULARY DETECTION. Under the system
interpreter the colour backend still works and still says so.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(WS, "src/srl_perception"))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(WS, "config"))

IPS = {"left": "192.168.1.10", "right": "192.168.1.9"}


def arm_joints(arm, timeout_s=6.0):
    """The arm's live joints, or None with the reason printed."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except Exception as exc:                                  # noqa: BLE001
        print("   (no rclpy: %s)" % exc)
        return None
    import time
    names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
    got = {}

    class R(Node):
        def __init__(self):
            super().__init__("srl_find_object")
            self.create_subscription(JointState, "/real/joint_states",
                                     self._cb, 20)

        def _cb(self, m):
            for n, p in zip(m.name, m.position):
                got[n] = p

    rclpy.init()
    n = R()
    t0 = time.time()
    while time.time() - t0 < timeout_s and not all(k in got for k in names):
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node()
    rclpy.shutdown()
    if not all(k in got for k in names):
        return None
    return [got[k] for k in names]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prompt")
    ap.add_argument("--camera", choices=("gripper", "scene"), default="gripper")
    ap.add_argument("--arm", choices=("left", "right"), default="left")
    ap.add_argument("--backend", default="auto",
                    choices=("auto", "both", "colour", "yoloworld"))
    ap.add_argument("--far-m", type=float, default=1.8,
                    help="drop detections beyond this range. The room's teal "
                         "robots sit at 2.8 m and match the target cube's hue "
                         "to within 2 -- range separates them on a property "
                         "that has nothing to do with colour.")
    ap.add_argument("--standoff-m", type=float, default=0.12)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    import numpy as np
    from srl_perception import srl_cameras as CAMS
    from srl_perception.grasp_pipeline import PlanFailure, plan_grasp, reachable
    from srl_perception.prompt_detector import PromptDetector

    det = PromptDetector(backend=a.backend, far_m=a.far_m)
    print("looking for %r" % a.prompt)
    print("   backend  %s   (available: %s)"
          % (det.backend, ", ".join(k for k, v in det.available().items() if v)))

    if a.camera == "scene":
        cam = CAMS.SceneCamera().open()
        try:
            bgr = cam.read()
        finally:
            cam.close()
        hits = det.detect(bgr, a.prompt)
        print("   scene camera: %d match(es)" % len(hits))
        for h in hits[:5]:
            print("      %s" % h)
        print("\nNO GRASP: the scene camera has no depth, so this is a "
              "direction and not a position.\nUse --camera gripper for a "
              "graspable pose.")
        return 0 if hits else 1

    ip = IPS[a.arm]
    print("   gripper camera on the %s arm (%s)" % (a.arm, ip))
    q = arm_joints(a.arm)
    if q is None:
        print("REFUSING: no /real/joint_states for the %s arm. The camera's "
              "pose comes from forward kinematics, so without joints any 3D "
              "answer would describe a robot that is not there." % a.arm)
        return 2
    import solve_home_pose as SHP
    import home_positions as hp
    sc = SHP.Scorer()
    M = sc.cf[a.arm](np.array(q, float))
    T = M[SHP.IDX["camera_link"]]
    cam_p, cam_R = T[:3, 3], T[:3, :3]
    print("   camera at %s" % np.round(cam_p, 3))

    gc = CAMS.GripperCamera(ip)
    bgr = gc.read_colour()
    depth = gc.read_depth()
    gc.close()
    print("   colour %dx%d, depth %dx%d (%.0f%% valid)"
          % (bgr.shape[1], bgr.shape[0], depth.shape[1], depth.shape[0],
             100.0 * np.count_nonzero(depth) / depth.size))
    try:
        plan = plan_grasp(bgr, depth, a.prompt, cam_p, cam_R,
                          CAMS.KINOVA_COLOR_K, CAMS.KINOVA_DEPTH_K,
                          detector=det, standoff_m=a.standoff_m)
    except PlanFailure as pf:
        print("\nREFUSED at the %s stage" % pf.stage)
        print("   %s" % pf.reason)
        return 3
    c = plan["centre"]
    print("\nFOUND  %s   via %s" % (plan["detection"]["label"], plan["backend"]))
    print("   position   x=%+.4f  y=%+.4f  z=%+.4f  m" % (c[0], c[1], c[2]))
    print("   range      %.3f m from the camera" % plan["detection"]["depth_m"])
    print("   grasp      %.0f mm across, %d depth points"
          % (plan["width_m"] * 1000, plan["n_points"]))
    print("   jaw axis   %s" % np.round(plan["close_axis"], 3))
    print("   approach   %s" % np.round(plan["approach"], 3))
    print("   pregrasp   %s" % np.round(plan["pregrasp"], 3))
    home = np.array(hp.load_home_radians(a.arm), float)
    print("\nREACHABILITY (multi-start IK, wearer floor 0.150 m)")
    for label, pt in (("grasp", plan["centre"]), ("pregrasp", plan["pregrasp"])):
        ok, _q, why = reachable(sc, a.arm, pt, home)
        print("   %-9s %-4s %s" % (label, "OK" if ok else "NO", why))
    other = "right" if a.arm == "left" else "left"
    ok2, _q2, why2 = reachable(sc, other, plan["centre"],
                               np.array(hp.load_home_radians(other), float))
    print("   %-9s %-4s %s" % ("(%s arm)" % other, "OK" if ok2 else "NO", why2))
    if a.json:
        out = {k: (v.tolist() if hasattr(v, "tolist") else v)
               for k, v in plan.items()}
        json.dump(out, open(a.json, "w"), indent=1, default=float)
        print("\n-> %s" % a.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
