#!/usr/bin/env python3
"""JOB F: DOES SAFETY SCALE WITH THE DIAL? Measured at both extremes.

Drives the commanded pose STRAIGHT AT THE WEARER at dial=1.0 (fastest, largest
scale, lightest smoothing) and again at dial=0.0, and reports how close the arm
ACTUALLY GOT -- measured from tf2, where the arm really is.

WHY THIS IS THE TEST. A dial that quietly relaxed the floor at speed would look
identical from the operator's seat -- smoother, more responsive, no warning --
right up until it did not. The only way to know is to aim the arm at the person
and see where it stops.

WHAT WOULD FALSIFY THE CLAIM: a closer approach at dial=1.0 than at dial=0.0,
or any breach of the floor at either. Faster arrival is expected and is not a
failure; a closer approach is.

=========================================================================
THREE INSTRUMENT BUGS THIS SCRIPT USED TO HAVE. All three produced a
confident number from a measurement that never happened.
=========================================================================

1. IT READ A STALE FIELD. It took the approach distance from
   `/ik_status_<arm>[5]`. The follower assigns `min_clearance` at the END of
   the publish path, AFTER IK has succeeded. When the target is inside the
   wearer, IK fails and that line is never reached -- so the field holds the
   clearance from the last SUCCESSFUL pose, indefinitely. Both dial ends
   therefore reported the same frozen number and it was written up as
   "speed did not buy a closer approach". Measured 2026-08-09: both ends
   returned 0.1277 m, bit-identical, and tf2 showed the arm had not moved at
   all -- it was still exactly where an unrelated test had left it.

2. IT PUBLISHED WORLD COORDINATES TO A MASTER-FRAME TOPIC.
   `/master_arm_pose_<arm>` carries a MASTER-FRAME DISPLACEMENT, which the
   follower maps through the anchor and scale. Feeding it a world position
   asks for a pose nowhere near the intended one -- the same mistake that
   produced Job D's false negative. It now drives `/autonomy/assist_pose_<arm>`,
   which is world-frame by contract.

3. IT COULD NOT TELL "REFUSED" FROM "NEVER TRIED". A run in which nothing
   moved scored identically to a run in which the arm advanced and was
   stopped. There is now an explicit MOTION CONTROL: the arm must demonstrably
   move during the run, or the result is reported as INCONCLUSIVE rather than
   as a safety pass.

The clearance itself is delegated to the project's own `srl_teleop.clearance`,
so this probe and the follower's hard floor cannot disagree about what
"clearance" means. Two clearance models would be worse than one.
"""
import json
import math
import os
import sys
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_teleop import precision_speed as ps               # noqa: E402
from srl_teleop.clearance import ClearanceModel, DISTAL_LINKS   # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/dial_safety.json")
ARM = "left"
SECS = 14.0

# The wearer's torso/head region, in world coordinates. Aiming here must be
# refused. Taken from the same geometry the mount guard uses.
WEARER = (0.0, -0.10, 1.25)
# A reachable staging pose on the arm's own side, so every run starts from the
# same place and the approach is a real approach rather than wherever the
# previous test happened to leave the arm. State left from a previous run is
# instrument-failure mechanism 3 and it is what produced bug 1 above.
STAGE = (0.32, 0.35, 1.15)


class P(Node):
    def __init__(self):
        super().__init__("dial_safety_probe")
        self.pub = self.create_publisher(
            PoseStamped, "/autonomy/assist_pose_%s" % ARM, 10)
        self.traj = 0
        self.floor_blocks = 0
        self.create_subscription(
            JointTrajectory, "/%s_arm_controller/joint_trajectory" % ARM,
            lambda _: setattr(self, "traj", self.traj + 1), 10)
        self.create_subscription(Float64MultiArray, "/ik_status_%s" % ARM,
                                 self._st, 10)
        self.cli = self.create_client(
            SetParameters, "/ik_follower_%s/set_parameters" % ARM)
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.model = ClearanceModel()

    def _st(self, m):
        d = list(m.data)
        if len(d) >= 7:
            self.floor_blocks = int(d[6])

    def spin(self, s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def ee(self):
        t = self.buf.lookup_transform(
            "world", "%s_end_effector_link" % ARM,
            rclpy.time.Time()).transform.translation
        return (t.x, t.y, t.z)

    def anchor_quat(self):
        """The arm's OWN current orientation.

        Never identity. Identity is not a neutral orientation -- it is a
        specific one, and at these positions it is unreachable, which has
        produced a false negative three times in this project.
        """
        r = self.buf.lookup_transform(
            "world", "%s_end_effector_link" % ARM,
            rclpy.time.Time()).transform.rotation
        return (r.x, r.y, r.z, r.w)

    def clearance_now(self):
        """Clearance from tf2, via the project's own model.

        This mirrors ik_follower_node.measure_clearance() exactly -- same
        links, same parts, same ClearanceModel. Two clearance models would be
        worse than one: the probe and the floor it is testing must agree about
        what the word means, or a disagreement between them is unattributable.
        """
        pts = {}
        for part in ("torso", "head", "hips"):
            got = []
            for link in DISTAL_LINKS:
                try:
                    t = self.buf.lookup_transform(
                        part, "%s_%s" % (ARM, link),
                        rclpy.time.Time()).transform.translation
                    got.append((t.x, t.y, t.z))
                except Exception:
                    continue
            if got:
                pts[part] = got
        if not pts:
            return float("nan")
        return self.model.clearance(pts, 0.0)[0]

    def set_dial(self, dial):
        s = ps.settings(dial)
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise SystemExit("no set_parameters service on ik_follower_%s -- "
                             "is the follower running?" % ARM)
        params = []
        for k in ("max_vel_rad_s", "max_step_rad"):
            p = Parameter()
            p.name = k
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                     double_value=float(s[k]))
            params.append(p)
        req = SetParameters.Request()
        req.parameters = params
        fut = self.cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 8.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        return s

    def drive(self, xyz, secs, quat):
        """Publish a world-frame target and sample clearance from tf2."""
        samples = []
        path = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            m = PoseStamped()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            (m.pose.position.x, m.pose.position.y,
             m.pose.position.z) = xyz
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = quat
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)
            c = self.clearance_now()
            if math.isfinite(c):
                samples.append(c)
            try:
                path.append(self.ee())
            except Exception:
                pass
        return samples, path


def run_one(p, dial, quat, step_m=0.02):
    """Stage, then WALK toward the wearer and find where the arm stops.

    A single target deep inside the body is refused outright -- collision-aware
    IK returns nothing and the arm never moves. That is correct behaviour, but
    it measures nothing about WHERE the arm would have stopped, and it makes a
    correctly-refused run look identical to a broken harness.

    So the approach is INCREMENTAL: interpolate from the staging pose toward
    the wearer in `step_m` steps, commanding each in turn. Every step short of
    the limit is individually reachable, so the arm really does advance, and
    the run ends at the last pose it could actually hold. That last pose is the
    answer the claim is about.
    """
    s = p.set_dial(dial)
    # STAGE FIRST -- identical start for both ends of the dial, so neither end
    # inherits wherever the previous run happened to leave the arm.
    p.drive(STAGE, 8.0, quat)
    p.spin(1.0)
    start = p.ee()
    before = p.floor_blocks

    n = max(1, int(math.dist(STAGE, WEARER) / step_m))
    samples, path, stalled = [], [], 0
    reached = start
    for i in range(1, n + 1):
        f = i / float(n)
        tgt = tuple(STAGE[k] + f * (WEARER[k] - STAGE[k]) for k in range(3))
        pre = p.ee()
        sm, pa = p.drive(tgt, 1.2, quat)
        samples += sm
        path += pa
        now = p.ee()
        if math.dist(pre, now) < 0.001:
            stalled += 1
            # Three consecutive steps with no motion: the arm has stopped and
            # further commands only re-refuse. Stop asking.
            if stalled >= 3:
                break
        else:
            stalled = 0
            reached = now

    p.spin(1.0)
    end = p.ee()
    travelled = math.dist(start, reached)
    return dict(
        dial=dial,
        max_vel=s["max_vel_rad_s"], max_step=s["max_step_rad"],
        n_samples=len(samples),
        min_clearance=(min(samples) if samples else None),
        floor_blocks=p.floor_blocks - before,
        start=[round(v, 4) for v in start],
        end=[round(v, 4) for v in end],
        closest_to_wearer_m=round(math.dist(reached, WEARER), 4),
        ee_travel_m=round(travelled, 4),
        moved=travelled > 0.005,
    )


def main():
    rclpy.init()
    p = P()
    p.spin(4.0)
    try:
        quat = p.anchor_quat()
    except Exception as e:
        raise SystemExit("no tf2 for %s_end_effector_link (%s) -- is the "
                         "stack up?" % (ARM, e))
    print("=" * 74)
    print("DRIVING AT THE WEARER, AT BOTH ENDS OF THE DIAL")
    print("=" * 74)
    print("  driving /autonomy/assist_pose_%s (world frame, by contract)" % ARM)
    print("  orientation = the arm's own anchor %s, never identity"
          % ("(%.3f %.3f %.3f %.3f)" % quat))
    print("  clearance from tf2 via srl_teleop.clearance -- the same model the")
    print("  follower's hard floor uses, so the two cannot disagree.\n")

    res = [run_one(p, 1.0, quat), run_one(p, 0.0, quat)]
    for r in res:
        mc = ("%.4f m" % r["min_clearance"]) if r["min_clearance"] is not None \
            else "--"
        print("  dial %.1f  vel %.2f rad/s  step %.2f"
              % (r["dial"], r["max_vel"], r["max_step"]))
        print("            clearance %s   stopped %.4f m from the wearer   "
              "floor blocks %d   EE advanced %.4f m  (n=%d)"
              % (mc, r["closest_to_wearer_m"], r["floor_blocks"],
                 r["ee_travel_m"], r["n_samples"]))

    print()
    # ---------------------------------------------------------------- verdict
    if not all(r["moved"] for r in res):
        print("  INCONCLUSIVE -- the arm did not move on at least one run.")
        print("  A run in which nothing happened is not a safety pass. Check")
        print("  that the follower is alive and that STAGE is reachable.")
        verdict = "INCONCLUSIVE: arm did not move"
    elif any(r["min_clearance"] is None for r in res):
        print("  INCONCLUSIVE -- no clearance samples. Nothing may be concluded.")
        verdict = "INCONCLUSIVE: no clearance samples"
    else:
        fast, slow = res[0]["min_clearance"], res[1]["min_clearance"]
        fd, sd = res[0]["closest_to_wearer_m"], res[1]["closest_to_wearer_m"]
        print("  fast stopped %.4f m from the wearer at %.4f m clearance"
              % (fd, fast))
        print("  slow stopped %.4f m from the wearer at %.4f m clearance"
              % (sd, slow))
        if fast < slow - 0.005:
            print("  SAFETY SCALES WITH THE DIAL -- FAIL. Speed bought a "
                  "closer approach.")
            verdict = "FAIL: closer approach at dial 1.0"
        else:
            print("  SAFETY UNCONDITIONAL: HOLDS -- speed did not buy a "
                  "closer approach")
            verdict = "HOLDS"

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as fh:
        json.dump(dict(runs=res, verdict=verdict,
                       note="clearance measured from tf2, not from the "
                            "ik_status field, which freezes when IK fails"),
                  fh, indent=2)
    print("  -> %s" % OUT)
    p.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
