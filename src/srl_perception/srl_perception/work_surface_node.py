#!/usr/bin/env python3
"""MEASURE THE WORK SURFACE FROM DEPTH AND TELL EVERYONE. Closes U-3.

    ros2 run srl_perception work_surface_node --ros-args -p arm:=left

`srl_experiments.work_surface` has owned the declared/measured split since it
was written, detects a 20 mm error, and had NO PRODUCER: `set_measured()` was
called by the fault injector and by nothing else. So the split existed, the
check existed, and on real hardware a table 20 mm out would still have been
absorbed into every grasp. This node is the missing half.

WHAT IT PUBLISHES

    /perception/work_surface_z        Float64, metres, only when measured
    /perception/work_surface_status   String,  the same words work_surface
                                      .check() would produce, every tick

IT ALSO CALLS set_measured() IN ITS OWN PROCESS, which matters less than it
looks: module state does not cross a process boundary. A consumer in another
process gets the value from the topic, via
`srl_experiments.work_surface.subscribe(node)`. Both paths exist because a
node that only set its own global would be the "feature present but does
nothing" failure in a new costume.

REFUSAL IS A RESULT. If the camera is not looking at the work region the
estimator raises and this node publishes the reason and leaves the value
UNSET. It never falls back to the declared height quietly -- the whole point
of the declared/measured split is that "not measured" is a state you can see.

THE SCAN POSE IS A PRECONDITION, NOT A SUGGESTION. From the home pose the
wrist cameras do not see the work surface at all (docs/NEXT_SESSION.md, and
recordings/baselines/scan_pose.json holds the poses that do). A correctly
working camera at home returns an empty region, so this node names that case
rather than reporting a failure.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener

from srl_perception.surface_from_depth import (SurfaceRefusal,
                                               measure_surface)

try:
    from srl_experiments import work_surface as WSURF
except ImportError:                                            # pragma: no cover
    WSURF = None


def _t_from_tf(tr):
    """A 4x4 world<-camera matrix from a TransformStamped."""
    q = tr.transform.rotation
    v = tr.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
             2 * (x * z + y * w), v.x],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
             2 * (y * z - x * w), v.y],
            [2 * (x * z - y * w), 2 * (y * z + x * w),
             1 - 2 * (x * x + y * y), v.z],
            [0.0, 0.0, 0.0, 1.0]]


class WorkSurfaceNode(Node):

    def __init__(self):
        super().__init__("work_surface_node")
        self.declare_parameter("arm", "left")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("rate_hz", 2.0)
        # THE FOOTPRINT MEASURED OVER, in world metres. Defaults to the
        # table's own span so legs, floor and wearer cannot vote. Narrow it
        # from the launch file if a task needs a tighter region.
        self.declare_parameter("region_x", [-0.90, 0.90])
        self.declare_parameter("region_y", [0.10, 0.72])
        # How many consecutive agreeing frames before the value is published.
        # ONE FRAME IS ONE FLIP, exactly as with TRAC-IK: a single depth frame
        # with a hand in shot would otherwise redefine the table.
        self.declare_parameter("confirm_frames", 5)
        self.declare_parameter("agree_tol_m", 0.004)

        self.arm = self.get_parameter("arm").value
        self.frame = self.get_parameter("frame_id").value or \
            "%s_camera_color_frame" % self.arm
        self.world = self.get_parameter("world_frame").value
        rx = list(self.get_parameter("region_x").value)
        ry = list(self.get_parameter("region_y").value)
        self.region = ((rx[0], rx[1]), (ry[0], ry[1]))
        self.need = int(self.get_parameter("confirm_frames").value)
        self.tol = float(self.get_parameter("agree_tol_m").value)

        q = QoSProfile(depth=1)
        q.reliability = ReliabilityPolicy.RELIABLE
        base = "/%s_camera" % self.arm
        self.create_subscription(Image, base + "/depth/image_raw",
                                 self.on_depth, q)
        self.create_subscription(CameraInfo, base + "/color/camera_info",
                                 self.on_info, q)
        self.pub_z = self.create_publisher(Float64,
                                           "/perception/work_surface_z", 1)
        self.pub_s = self.create_publisher(String,
                                           "/perception/work_surface_status",
                                           1)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        self.intr = None
        self.depth = None
        self.encoding = None
        self.recent = []
        self.published = None
        self.no_depth = 0
        hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(
            "measuring the work surface from %s/depth, region x %.2f..%.2f "
            "y %.2f..%.2f, %d agreeing frames required"
            % (base, rx[0], rx[1], ry[0], ry[1], self.need))

    def on_info(self, m):
        k = m.k
        self.intr = (k[0], k[4], k[2], k[5])

    def on_depth(self, m):
        # Kept as the raw buffer and reshaped lazily; the estimator indexes
        # depth[v][u] and nothing here needs numpy.
        self.depth = m
        self.encoding = m.encoding

    def _rows(self, m):
        import array
        n = 2 if m.encoding in ("16UC1", "mono16") else 4
        typ = "H" if n == 2 else "f"
        a = array.array(typ)
        a.frombytes(bytes(m.data))
        if m.is_bigendian:
            a.byteswap()
        stride = m.step // n
        return [a[v * stride:v * stride + m.width] for v in range(m.height)]

    def _say(self, text):
        self.pub_s.publish(String(data=text))

    def tick(self):
        if self.intr is None or self.depth is None:
            self.no_depth += 1
            if self.no_depth in (10, 60) or self.no_depth % 300 == 0:
                # NAMED. An absent camera and a camera seeing nothing are
                # indistinguishable downstream, and that ambiguity has cost
                # this project days before.
                self._say("NO DEPTH YET on /%s_camera (intrinsics=%s, "
                          "frames=%s). The surface is UNMEASURED."
                          % (self.arm, self.intr is not None,
                             self.depth is not None))
            return
        try:
            tr = self.tfb.lookup_transform(self.world, self.frame,
                                           rclpy.time.Time())
        except Exception as e:                                 # noqa: BLE001
            self._say("NO TF %s <- %s (%s). The surface is UNMEASURED."
                      % (self.world, self.frame, e))
            return
        try:
            est = measure_surface(self._rows(self.depth), self.intr,
                                  _t_from_tf(tr), self.region,
                                  encoding=self.encoding)
        except SurfaceRefusal as e:
            self.recent.clear()
            self._say("SURFACE NOT MEASURED: %s" % e)
            return

        # AGREEING FRAMES, not one. See confirm_frames.
        self.recent.append(est.z_m)
        if len(self.recent) > self.need:
            self.recent.pop(0)
        if len(self.recent) < self.need:
            self._say("surface %.4f m, %d/%d agreeing frames"
                      % (est.z_m, len(self.recent), self.need))
            return
        if max(self.recent) - min(self.recent) > self.tol:
            self._say("SURFACE UNSTABLE: %d frames span %.1f mm (limit "
                      "%.1f mm). Something is moving in the region."
                      % (len(self.recent),
                         (max(self.recent) - min(self.recent)) * 1000.0,
                         self.tol * 1000.0))
            return

        z = sum(self.recent) / len(self.recent)
        self.pub_z.publish(Float64(data=float(z)))
        if WSURF is not None:
            WSURF.set_measured(z, est.note())
            ok, msg = WSURF.check()
            self._say(("%s" if ok else "FAIL %s") % msg)
        else:
            self._say("surface %.4f m (%s)" % (z, est.note()))
        if self.published is None or abs(self.published - z) > self.tol:
            self.get_logger().info("work surface MEASURED at %.4f m -- %s"
                                   % (z, est.note()))
            self.published = z


def main(argv=None):
    rclpy.init(args=argv)
    n = WorkSurfaceNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
