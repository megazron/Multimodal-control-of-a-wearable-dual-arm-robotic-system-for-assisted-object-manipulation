#!/usr/bin/env python3
"""THE MISSING SEAM: measured objects -> MoveIt's planning scene.

    ros2 run srl_perception map_obstacles_node

The two obstacle stores have been DISJOINT for the life of the project:
`world_model.Map` feeds `joint_planner.VoxelWorld` and never reaches MoveIt;
the wearer reaches MoveIt and nothing else does. So MoveIt has planned around
a person and a coordinate table, never around a measured object. This node
closes that seam from two inputs:

  * robot-frame objects from `/perception/scene/objects` (wrist cameras only
    -- a room camera's frame is its own, and pushing an unextrinsic'd box
    into the world frame would be inventing a transform);
  * `recordings/baselines/world_map.json`, on the Trigger service
    `/perception/load_world_map` -- the sweep's map, already in robot frame.

THE DIRECTION IS LAW, copied from `pick_from_map`: the map may ADD an
obstacle; it may not remove one. An object that vanishes from view stays in
the scene until the Trigger `/perception/clear_mapped` -- deliberate, logged
-- because "the camera stopped seeing it" and "it is gone" are different
claims and only a person gets to make the second. Within one id the box only
GROWS (union of everything observed there); a smaller observation never
shrinks it.

ONE WRITER PER NAMESPACE. This node writes ids `mapped_*` and nothing else.
Two clients writing one planning scene cost this project a day (18 of 171
false obstructions); the wearer ids belong to `wearer_tracker_node`, the task
furniture to `scene_spawner`. For the same reason the wearer's own volume is
EXCLUDED here: a wrist camera that sees the person must not file them as
furniture -- the wearer nodes own that space, with a body that tracks.
"""
from __future__ import annotations

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# A conservative axis-aligned bound around the mannequin INCLUDING its arms
# (`wearer_posture.py` is the source of the real body; this box only decides
# what this node refuses to claim as furniture, so too big is safe and too
# small files a person as a box).
WEARER_LO = (-0.45, -0.35, 0.60)
WEARER_HI = (0.45, 0.30, 1.85)


def observation_box(centre, extents):
    """(lo, hi) for one measured object, in world.

    The perception report does not carry the object's yaw, so the horizontal
    footprint is the CIRCUMSCRIBED square on the longest extent: the box may
    only overstate the object, never understate it -- an obstacle drawn too
    small is a collision, one drawn too big is a detour.
    """
    cx, cy, cz = [float(v) for v in centre]
    ex = [abs(float(v)) for v in extents]
    half_xy = max(ex[0], ex[1]) / 2.0
    half_z = ex[2] / 2.0
    return ((cx - half_xy, cy - half_xy, cz - half_z),
            (cx + half_xy, cy + half_xy, cz + half_z))


def _overlaps(lo_a, hi_a, lo_b, hi_b):
    return all(lo_a[i] <= hi_b[i] and lo_b[i] <= hi_a[i] for i in range(3))


class MappedStore:
    """Pure add-only store. No ROS, so the known-answer test drives it dry."""

    def __init__(self, merge_m=0.06, min_extent_m=0.015,
                 wearer_lo=WEARER_LO, wearer_hi=WEARER_HI):
        self.merge_m = float(merge_m)
        self.min_extent_m = float(min_extent_m)
        self.wearer_lo = tuple(float(v) for v in wearer_lo)
        self.wearer_hi = tuple(float(v) for v in wearer_hi)
        self.objs = {}                 # id -> [lo, hi], only ever grows
        self._published = {}           # id -> signature of what MoveIt has
        self._n = 0
        self.last_excluded = []

    @staticmethod
    def _sig(lo, hi):
        return tuple(round(v, 3) for v in (*lo, *hi))

    def update(self, observations):
        """Fold one cycle's observations in. ADD or GROW; never shrink,
        never remove. Returns {"added": [...], "grown": [...],
        "excluded": [...]} with every exclusion NAMED."""
        added, grown, excluded = [], [], []
        for obs in observations:
            centre, extents = obs["centre"], obs["extents_m"]
            if min(abs(float(v)) for v in extents) < self.min_extent_m:
                excluded.append(dict(
                    centre=centre,
                    why="smallest extent %.0f mm is under the %.0f mm floor"
                        % (min(abs(float(v)) for v in extents) * 1000,
                           self.min_extent_m * 1000)))
                continue
            lo, hi = observation_box(centre, extents)
            if _overlaps(lo, hi, self.wearer_lo, self.wearer_hi):
                excluded.append(dict(
                    centre=centre,
                    why="inside the wearer's volume -- the wearer nodes own "
                        "that space and it must never be filed as furniture"))
                continue
            oid = self._match(lo, hi)
            if oid is None:
                oid = "mapped_%d" % self._n
                self._n += 1
                self.objs[oid] = [list(lo), list(hi)]
                added.append(oid)
            else:
                slo, shi = self.objs[oid]
                nlo = [min(a, b) for a, b in zip(slo, lo)]
                nhi = [max(a, b) for a, b in zip(shi, hi)]
                if self._sig(nlo, nhi) != self._sig(slo, shi):
                    self.objs[oid] = [nlo, nhi]
                    grown.append(oid)
        self.last_excluded = excluded
        return dict(added=added, grown=grown, excluded=excluded)

    def _match(self, lo, hi):
        cx = [(lo[i] + hi[i]) / 2.0 for i in range(3)]
        for oid, (slo, shi) in self.objs.items():
            if _overlaps(lo, hi, slo, shi):
                return oid
            sc = [(slo[i] + shi[i]) / 2.0 for i in range(3)]
            if math.dist(cx, sc) < self.merge_m:
                return oid
        return None

    def diff(self):
        """Only what MoveIt does not already have -- a stream of identical
        scenes forces re-planning mid-solve (`wearer_tracker_node` learned
        this the hard way)."""
        out = []
        for oid, (lo, hi) in self.objs.items():
            if self._published.get(oid) != self._sig(lo, hi):
                out.append((oid, tuple(lo), tuple(hi)))
        return out

    def mark_published(self, ids):
        for oid in ids:
            lo, hi = self.objs[oid]
            self._published[oid] = self._sig(lo, hi)

    def clear(self):
        """The ONE removal path, and it is a deliberate call, never a timeout."""
        ids = sorted(self.objs)
        self.objs.clear()
        self._published.clear()
        return ids


def load_world_map(path):
    """(observations, note) from a `world_model.save()` file.

    `world_model` lives in this same package, but its `Map` object wants the
    point clouds back; the saved `as_dict` already carries centre and extents
    per object, which is all a collision box needs -- so a light parse, and
    the note says which file and how many objects it contributed.
    """
    if not os.path.exists(path):
        raise FileNotFoundError("no world map at %s -- run the calibration "
                                "sweep first" % path)
    with open(path) as f:
        d = json.load(f)
    objs = d.get("objects") or []
    if not objs:
        raise ValueError("%s holds no objects -- an empty map adds nothing "
                         "and this is said rather than silently succeeding"
                         % path)
    out = [dict(centre=o["centre"], extents_m=o["extents"]) for o in objs]
    return out, "%d object(s) from %s" % (len(out), path)


def collision_objects_for(entries, remove=False, frame="world"):
    """Store entries -> CollisionObject list. Ids are mapped_* BY CONSTRUCTION
    -- they come from the store and nowhere else."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    out = []
    for oid, lo, hi in entries:
        co = CollisionObject()
        co.header.frame_id = frame
        co.id = oid
        co.operation = (CollisionObject.REMOVE if remove
                        else CollisionObject.ADD)
        if not remove:
            sp = SolidPrimitive()
            sp.type = SolidPrimitive.BOX
            sp.dimensions = [max(hi[i] - lo[i], 0.01) for i in range(3)]
            p = Pose()
            p.position.x = (lo[0] + hi[0]) / 2.0
            p.position.y = (lo[1] + hi[1]) / 2.0
            p.position.z = (lo[2] + hi[2]) / 2.0
            p.orientation.w = 1.0
            co.primitives.append(sp)
            co.primitive_poses.append(p)
        out.append(co)
    return out


class MapObstacles(Node):
    def __init__(self):
        super().__init__("map_obstacles")
        self.declare_parameter("scene_hz", 2.0)
        self.declare_parameter("merge_m", 0.06)
        self.declare_parameter("min_extent_m", 0.015)
        self.declare_parameter(
            "world_map_path",
            os.path.expanduser("~/kortex_ws/recordings/baselines/world_map.json"))
        self.store = MappedStore(
            merge_m=float(self.get_parameter("merge_m").value),
            min_extent_m=float(self.get_parameter("min_extent_m").value))
        self.scene_pub = None
        self._last_scene = 0.0
        self.create_subscription(String, "/perception/scene/objects",
                                 self._on_objects, 10)
        self.create_service(Trigger, "/perception/load_world_map",
                            self._srv_load)
        self.create_service(Trigger, "/perception/clear_mapped",
                            self._srv_clear)
        self.get_logger().info(
            "map obstacles up: mapped_* ids only, add/grow only; removal is "
            "the /perception/clear_mapped Trigger and nothing else")

    def _on_objects(self, msg):
        try:
            rep = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn("unparseable /perception/scene/objects")
            return
        obs = []
        for name, ent in (rep.get("cameras") or {}).items():
            # ROBOT-FRAME OBJECTS ONLY. A room camera's centres are in its
            # own frame; pushing them into world would invent an extrinsic.
            if ent.get("frame") != "robot":
                continue
            for o in ent.get("objects") or []:
                if "centre" in o and "extents_m" in o:
                    obs.append(o)
        if obs:
            res = self.store.update(obs)
            for e in res["excluded"]:
                self.get_logger().info("excluded: %s" % e["why"], once=True)
        self._publish_diff()

    def _publish_diff(self, force=False):
        now = time.monotonic()
        hz = max(0.2, float(self.get_parameter("scene_hz").value))
        if not force and now - self._last_scene < 1.0 / hz:
            return
        entries = self.store.diff()
        if not entries:
            return
        ps = self._scene_msg(collision_objects_for(entries))
        if ps is None:
            return
        self.scene_pub.publish(ps)
        self.store.mark_published([e[0] for e in entries])
        self._last_scene = now

    def _scene_msg(self, cos):
        try:
            from moveit_msgs.msg import PlanningScene
        except Exception as e:                                # noqa: BLE001
            self.get_logger().warn("moveit_msgs unavailable (%s) -- nothing "
                                   "reaches the planning scene" % e, once=True)
            return None
        if self.scene_pub is None:
            self.scene_pub = self.create_publisher(PlanningScene,
                                                   "/planning_scene", 4)
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.world.collision_objects.extend(cos)
        return ps

    def _srv_load(self, req, resp):
        try:
            obs, note = load_world_map(
                str(self.get_parameter("world_map_path").value))
        except (FileNotFoundError, ValueError, KeyError) as e:
            resp.success = False
            resp.message = str(e)
            return resp
        res = self.store.update(obs)
        self._publish_diff(force=True)
        resp.success = True
        resp.message = ("%s; %d added, %d grown, %d excluded"
                        % (note, len(res["added"]), len(res["grown"]),
                           len(res["excluded"])))
        for e in res["excluded"]:
            self.get_logger().info("world map object excluded: %s" % e["why"])
        return resp

    def _srv_clear(self, req, resp):
        ids = self.store.clear()
        if ids:
            ps = self._scene_msg(collision_objects_for(
                [(i, (0, 0, 0), (0, 0, 0)) for i in ids], remove=True))
            if ps is not None:
                self.scene_pub.publish(ps)
        # Logged loudly on purpose: this is the map forgetting obstacles, and
        # it must be traceable to the person who asked for it.
        self.get_logger().warn("CLEARED %d mapped obstacle(s) on explicit "
                               "request: %s" % (len(ids), ", ".join(ids)))
        resp.success = True
        resp.message = "cleared %d: %s" % (len(ids), ", ".join(ids))
        return resp


def main(args=None):
    rclpy.init(args=args)
    n = MapObstacles()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
