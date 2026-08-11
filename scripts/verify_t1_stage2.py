"""T1 Stage 2, N=10 over the FULL densified path, across seeds."""
import sys, json, os
sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
sys.path.insert(0, "/home/gausms/kortex_ws/src/srl_experiments/experiments/abc")
import rclpy
from verify_task_scenes import Solver, HOME_TOL_RAD
import msc_clip_tasks as MCT, clip_scene as CS, clip_tasks as CT
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import time as _t

N = 10
rclpy.init(); n = Solver(); n.spin(8.0)
if not n.ik.wait_for_service(timeout_sec=25.0): print("no /compute_ik"); sys.exit(2)
for arm in ("left","right"):
    w,_ = n.home_ok(arm)
    if w > HOME_TOL_RAD: print("REFUSING: %s %.4f from home" % (arm,w)); sys.exit(3)
q = {a: n.ee_quat(a) for a in ("left","right")}
tmp = CS.Scene.__new__(CS.Scene)
cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(timeout_sec=15.0)
CS.remove_furniture(n)
objs = CS.Scene._collision_furniture(tmp, "t1")
ps = PlanningScene(); ps.is_diff = True; ps.world.collision_objects = objs
f = cli.call_async(ApplyPlanningScene.Request(scene=ps))
end = _t.time()+20
while _t.time()<end and not f.done(): rclpy.spin_once(n, timeout_sec=0.05)

calls = [0]
def ok(arm, p):
    for _ in range(N):
        calls[0]+=1
        if not n.solve(arm, list(p), q[arm], tries=6): return False
    return True

ctl_far = n.solve("left",[1.6,0.35,1.15],q["left"],tries=6)
print("control 1.6 m out ->", "REACHABLE (BAD)" if ctl_far else "unreachable", flush=True)
tot = 0; out={}
for seed in (0,1,2,3,4):
    path = MCT.TASKS["t1s2"]["build"](seed)
    bad = 0; tested = 0
    for arm in ("left","right"):
        seen=set(); uniq=[]
        for w in path[arm]:
            k=tuple(round(v,4) for v in w)
            if k not in seen: seen.add(k); uniq.append(list(w))
        for w in uniq:
            tested += 1
            if not ok(arm, w): bad += 1
    out[seed]=dict(tested=tested, failures=bad); tot += bad
    print("  seed %d: %d distinct waypoints, %d failures" % (seed, tested, bad), flush=True)
print("T1 STAGE 2: %d seeds, %d IK calls, TOTAL FAILURES %d" % (len(out), calls[0], tot))
json.dump(dict(seeds=out, repeats=N, ik_calls=calls[0], failures=tot,
               control_far_unreachable=(not ctl_far)),
          open("/home/gausms/kortex_ws/recordings/baselines/t1_stage2_verification.json","w"), indent=2)
CS.remove_furniture(n); n.destroy_node(); rclpy.shutdown()
sys.exit(0 if (tot==0 and not ctl_far) else 1)
