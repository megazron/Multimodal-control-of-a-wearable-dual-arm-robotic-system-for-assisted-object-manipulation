#!/usr/bin/env python3
"""JOB C: the gripper penetrates objects. Measure it, explain it, fix it.

THE FAULT. `grasp_library.candidates()` returns `position = p.copy()` -- the
object CENTROID, unmodified -- and `grasp_generator` commands
`<arm>_end_effector_link` there. But the fingers extend along the tool's +z,
and the grasp quaternion aims that axis straight down, so the fingertips end
up a full finger-length BELOW the point the gripper was asked to reach. The
gripper does not close on the object; it drives through it.

WHY /compute_ik DOES NOT REFUSE. `grasp_generator` sets
`avoid_collisions=True`, which looks like it covers this. It does not:
NOTHING IN THE AUTONOMY PATH EVER PUTS THE PERCEIVED OBJECT INTO THE PLANNING
SCENE. `world_model` and `grasp_generator` publish no CollisionObject at all
(only `scripted_pick_place` and the experiment scene do). So the collision
check runs against a world in which the object does not exist, and passes.
This is the project's own "a marker is decoration" failure, one layer up.

Both halves are measured here, not asserted: the geometry comes from TF on the
live model, and the IK behaviour is measured with the object absent and then
present.
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (CollisionObject, PlanningScene, RobotState,
                             PositionIKRequest)
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_autonomy"))
from srl_autonomy import grasp_library as gl                  # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/grasp_penetration.json")
PAD_M = 0.005            # clearance left between fingertip and the underside
N_IK = 10                # TRAC-IK restarts randomly; one call is one flip
# Where the object sits for the reachability half of the test: inside the
# volume Job B measured as solvable, so a FAIL here is about the grasp offset
# and not about the arm being unable to get there at all.
PLACE = {"left": np.array([0.32, 0.35, 1.15]),
         "right": np.array([-0.32, 0.35, 1.15])}


class G(Node):
    def __init__(self):
        super().__init__("grasp_penetration")
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.js = {}
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.scene = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")

    def _js(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.05)

    def tip_offset(self, arm):
        """Fingertip reach along the tool +z, MEASURED, not hardcoded."""
        zs = []
        for side in ("left", "right"):
            f = "%s_robotiq_85_%s_finger_tip_link" % (arm, side)
            t = self.buf.lookup_transform("%s_end_effector_link" % arm, f,
                                          rclpy.time.Time())
            zs.append(t.transform.translation.z)
        return float(np.mean(zs))

    def set_object(self, name, xyz, size, present):
        ps = PlanningScene()
        ps.is_diff = True
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = name
        if present:
            sp = SolidPrimitive()
            sp.type = SolidPrimitive.BOX
            sp.dimensions = [float(v) for v in size]
            p = Pose()
            p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
            p.orientation.w = 1.0
            co.primitives = [sp]
            co.primitive_poses = [p]
            co.operation = CollisionObject.ADD
        else:
            co.operation = CollisionObject.REMOVE
        ps.world.collision_objects = [co]
        for _ in range(3):
            self.scene.publish(ps)
            self.spin(0.15)
        self.spin(0.5)

    def ik_rate(self, arm, pos, quat, n=N_IK):
        ok = 0
        for _ in range(n):
            r = PositionIKRequest()
            r.group_name = "%s_arm" % arm
            r.ik_link_name = "%s_end_effector_link" % arm
            r.avoid_collisions = True
            r.timeout.sec = 0
            r.timeout.nanosec = int(0.05 * 1e9)
            rs = RobotState()
            names = [n2 for n2 in self.js if n2.startswith(arm)]
            rs.joint_state.name = names
            rs.joint_state.position = [float(self.js[n2]) for n2 in names]
            r.robot_state = rs
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = \
                (float(v) for v in pos)
            ps.pose.orientation.x, ps.pose.orientation.y, \
                ps.pose.orientation.z, ps.pose.orientation.w = \
                (float(v) for v in quat)
            r.pose_stamped = ps
            req = GetPositionIK.Request()
            req.ik_request = r
            # call_async + explicit spin. A synchronous .call() blocks
            # forever here because nothing else is spinning this node, which
            # is how the first version of this script hung rather than
            # returning a wrong answer.
            fut = self.ik.call_async(req)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 2.0:
                rclpy.spin_once(self, timeout_sec=0.02)
            res = fut.result() if fut.done() else None
            if res is not None and res.error_code.val == 1:
                ok += 1
        return 100.0 * ok / n


def corrected_offset(size_z, tip_z, pad=PAD_M):
    """Distance to RAISE the tool above the centroid for a top-down grasp.

    Derived, not tuned: we want the fingertip to sit `pad` above the object's
    underside, so with the tool at centroid + d and the tip a distance tip_z
    down the approach axis,

        tip_world = centroid + d - tip_z  ==  (centroid - size_z/2) + pad
        =>  d = tip_z - size_z/2 + pad

    which is exactly "object half-width plus pad", with the finger length that
    made the offset necessary in the first place.
    """
    return tip_z - size_z / 2.0 + pad


def main():
    rclpy.init()
    n = G()
    n.spin(4.0)
    if not n.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik")
        return 2
    tip = {a: n.tip_offset(a) for a in ("left", "right")}
    print("=" * 78)
    print("MEASURED GRIPPER GEOMETRY (from TF, not hardcoded)")
    print("=" * 78)
    for a, v in tip.items():
        print("  %-5s fingertip is %.4f m along the tool +z from "
              "end_effector_link" % (a, v))
    print("  the grasp quaternion aims that +z straight DOWN, so the tool")
    print("  point commanded at a centroid puts the tips that far below it.")

    print("\n" + "=" * 78)
    print("PENETRATION PER OBJECT -- BEFORE (ships today) vs AFTER")
    print("=" * 78)
    t = tip["left"]
    print("  %-12s %7s   %9s %9s   %9s %9s"
          % ("object", "size_z", "before", "clears?", "after", "clears?"))
    rows = {}
    for oid, spec in gl.OBJECTS.items():
        sz = spec["size"][2]
        # BEFORE: the tool at the bare centroid, which is what shipped.
        before = t - sz / 2.0
        # AFTER: read the LIBRARY's actual output rather than recomputing the
        # formula here. If the two were computed the same way this script
        # would agree with itself no matter what the library does, which is
        # not a regression test.
        cand = gl.candidates(oid, np.zeros(3))[0]
        d = float(cand["position"][2])
        after = t - d - sz / 2.0
        rows[oid] = dict(name=spec["name"], size_z=sz,
                         before_penetration_m=before,
                         offset_m=d, after_penetration_m=after)
        print("  %-12s %7.3f   %+9.4f %9s   %+9.4f %9s"
              % (spec["name"], sz, before, "NO" if before > 0 else "yes",
                 after, "NO" if after > 0 else "yes"))
    print("\n  positive = the fingertip is THAT FAR BELOW the object's")
    print("  underside, i.e. driven through the object and into whatever")
    print("  it is standing on. Negative = clearance.")
    worst = max(rows.values(), key=lambda r: r["before_penetration_m"])
    print("  worst: %s at %.1f mm of penetration"
          % (worst["name"], 1000 * worst["before_penetration_m"]))

    print("\n" + "=" * 78)
    print("WHY /compute_ik DOES NOT REFUSE -- measured, object absent v present")
    print("=" * 78)
    arm = "left"
    oid = "tag_0"
    spec = gl.OBJECTS[oid]
    c = PLACE[arm]
    cand = gl.candidates(oid, c)[0]
    quat = cand["quat"]
    n.set_object("probe", c, spec["size"], present=False)
    absent = n.ik_rate(arm, c, quat)
    n.set_object("probe", c, spec["size"], present=True)
    present = n.ik_rate(arm, c, quat)
    d = corrected_offset(spec["size"][2], t)
    fixed_pos = c + np.array([0.0, 0.0, d])
    fixed = n.ik_rate(arm, fixed_pos, quat)
    # WHY IS THE CORRECTED POSE ALSO REFUSED? A grasp pose puts the fingers
    # AROUND the object, so the finger links overlap its collision box by
    # construction. Collision-aware IK cannot tell that apart from driving
    # through it. The pre-grasp standoff is the discriminator: if the standoff
    # passes while the grasp fails, the refusal is the gripper enveloping the
    # object, which is what a grasp IS -- not penetration.
    # CONTROLS FIRST. A 0% with the object present means nothing until the
    # same pose has been tried with the object ABSENT: an unreachable pose
    # scores 0% either way, and reading that as "collision refused it" would
    # be the instrument talking. The corrected pose sits ~0.10 m and the
    # standoff ~0.20 m above the centroid, and Job B only established
    # reachability to +0.10 m, so the standoff is genuinely in doubt.
    pre = c + np.array([0.0, 0.0, d + spec["approach"]])
    n.set_object("probe", c, spec["size"], present=False)
    fixed_absent = n.ik_rate(arm, fixed_pos, quat)
    pre_absent = n.ik_rate(arm, pre, quat)
    n.set_object("probe", c, spec["size"], present=True)
    pre_rate = n.ik_rate(arm, pre, quat)
    n.set_object("probe", c, spec["size"], present=False)
    print("  centroid pose, object NOT in the planning scene : %5.1f%% accepted"
          % absent)
    print("  centroid pose, object IS  in the planning scene : %5.1f%% accepted"
          % present)
    print("  corrected pose (+%.4f m), object present        : %5.1f%% accepted"
          % (d, fixed))
    print("  PRE-GRASP standoff (+%.4f m), object present    : %5.1f%% accepted"
          % (d + spec["approach"], pre_rate))
    print("  CONTROLS, object absent:  corrected %5.1f%%   standoff %5.1f%%"
          % (fixed_absent, pre_absent))
    print()
    if absent > present:
        print("  -> IK DOES refuse the penetrating pose, but ONLY when the")
        print("     object is in the scene. The autonomy path never puts it")
        print("     there, so the check it relies on never runs.")
    else:
        print("  -> IK accepts the penetrating pose EVEN WITH the object")
        print("     present. The offset fix is then the only defence.")
    print("  Three separate things, and they must not be conflated:")
    if absent > present:
        print("   1. the penetrating pose IS refused by collision-aware IK,")
        print("      but only when the object is in the scene, and the")
        print("      autonomy path never puts it there.")
    if fixed_absent > 50.0 and fixed < 50.0:
        print("   2. the CORRECTED grasp is refused too, for a different")
        print("      reason: it is reachable with no object (%.1f%%) and")
        print("      refused with one (%.1f%%), because the fingers envelop"
              % (fixed_absent, fixed))
        print("      the object -- which is what a grasp IS. Adding the")
        print("      object to the scene is therefore NOT sufficient on its")
        print("      own: the target must be excluded from the gripper's")
        print("      ACM, or attached, or only the standoff validated.")
    if pre_absent < 50.0:
        print("   3. the PRE-GRASP standoff is unreachable with NO object")
        print("      present (%.1f%%), so that 0%% is REACH, not collision."
              % pre_absent)
        print("      The offset raises the whole approach by ~0.10 m and")
        print("      pushes the standoff out of the arm's volume. Objects")
        print("      must be placed lower to pay for the correction.")
    if False:
        print("  -> the corrected pose is UNREACHABLE even with no object")
        print("     (%.1f%% absent), so its 0%% with the object present is"
              % fixed_absent)
        print("     about REACH, not collision. Raising the tool ~0.10 m")
        print("     above the centroid moves the grasp out of the volume")
        print("     Job B verified. THE OFFSET FIX COSTS REACHABILITY, and")
        print("     the object must be placed lower to pay for it.")
    elif pre_rate > fixed:
        print("  -> and the CORRECTED grasp is refused for a DIFFERENT")
        print("     reason: the fingers envelop the object, which is what a")
        print("     grasp is. The pre-grasp standoff passes at %.1f%%, so"
              % pre_rate)
        print("     collision-aware IK cannot validate a grasp pose at all.")
        print("     Validate the STANDOFF with collisions on; the grasp needs")
        print("     the target object excluded from the gripper's ACM.")

    print("\n" + "=" * 78)
    print("POSITION TOLERANCE -- what real-world object error costs")
    print("=" * 78)
    print("  The design assumes the object centroid is known to better than")
    print("  half the free span between the pads and the object:")
    ungraspable = []
    for oid, spec in gl.OBJECTS.items():
        w = min(spec["size"][0], spec["size"][1])
        ok_w, _ = gl.is_graspable(oid)
        if not ok_w:
            # NOT a tolerance failure. Saying "flat_plate misses at +/-10 mm"
            # implies it works at 0 mm, and it does not: its 90 mm minor
            # extent exceeds the 85 mm stroke, so it is ungraspable outright
            # and is already refused upstream by is_graspable().
            ungraspable.append(spec["name"])
            rows[oid]["ungraspable"] = True
            continue
        lateral = (gl.GRIPPER_MAX_WIDTH_M - w) / 2.0
        rows[oid]["lateral_tolerance_m"] = lateral
        rows[oid]["vertical_tolerance_m"] = PAD_M + spec["size"][2] / 2.0
        print("    %-12s width %.3f -> lateral +/-%.1f mm, "
              "vertical +/-%.1f mm"
              % (spec["name"], w, 1000 * lateral,
                 1000 * (PAD_M + spec["size"][2] / 2.0)))
    if ungraspable:
        print("\n  EXCLUDED as ungraspable at any tolerance (minor extent")
        print("  exceeds the %.0f mm safe stroke): %s"
              % (1000 * gl.GRIPPER_SAFE_WIDTH_M, ", ".join(ungraspable)))
    print("\n  at +/-10 mm of object position error:")
    bad10 = [r["name"] for r in rows.values()
             if r.get("lateral_tolerance_m", 9) < 0.010]
    print("    lateral : %s"
          % ("all objects still fit between the pads"
             if not bad10 else "MISSES on " + ", ".join(bad10)))
    print("    vertical: fingertip moves into the pad clearance; objects")
    print("              shorter than %.0f mm lose the %.0f mm pad and touch"
          % (1000 * 2 * (0.010 - PAD_M), 1000 * PAD_M))
    print("  at +/-20 mm:")
    bad20 = [r["name"] for r in rows.values()
             if r.get("lateral_tolerance_m", 9) < 0.020]
    print("    lateral : %s"
          % ("all objects still fit"
             if not bad20 else "MISSES on " + ", ".join(bad20)))

    json.dump(dict(tip_offset_m=tip, pad_m=PAD_M, objects=rows,
                   ik_absent_pct=absent, ik_present_pct=present,
                   ik_corrected_pct=fixed),
              open(OUT, "w"), indent=2)
    print("\n  -> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
