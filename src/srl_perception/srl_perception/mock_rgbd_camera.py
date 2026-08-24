#!/usr/bin/env python3
"""MOCK RGB-D on the wrist-camera topics, so the perception pipeline runs.

    ros2 run srl_perception mock_rgbd_camera --ros-args -p arm:=left -p task:=t1

Same pattern as `mock_real.launch.py` and `quest_vendor_mock.py`: stand a
plausible producer at the boundary so everything INSIDE the boundary can be
exercised, and then say precisely where the boundary is. The alternative --
deferring the whole pipeline until the Kinova cameras exist -- leaves the
frame chain, the intrinsics plumbing, the deprojection arithmetic and the
tracker's timeout untested on the day they are first needed.

WHAT IT PUBLISHES, on exactly the topics the real driver owns:

    /<arm>_camera/color/image_raw     Image, rgb8,   640 x 480
    /<arm>_camera/color/camera_info   CameraInfo,    plumb_bob, zero distortion
    /<arm>_camera/depth/image_raw     Image, 16UC1,  MILLIMETRES

Depth is 16UC1 in mm because that is what the Kinova driver emits, and
`vlm_object_locator` carries a mm-or-metres branch that has never had the mm
side exercised. A mock that published the convenient encoding would leave the
branch that will actually run untested.

The camera POSE is not invented: it is read from TF for
`<arm>_camera_color_frame`, which the real `robot_state_publisher` provides
from the same URDF the driver would. So the view moves with the arm, and the
optical-frame convention (z forward, x right, y down) is exercised for real.

=====================================================================
WHAT THIS CAN AND CANNOT VERIFY -- READ BEFORE CITING ANYTHING FROM IT
=====================================================================

The standing rule in CLAUDE.md is that synthetic ONLY counts where the ground
truth is CONSTRUCTED, not RENDERED. That line runs straight through this node,
and it is the whole reason the file has this section:

CAN be verified here, because the geometry is CONSTRUCTED and I know the
answer in advance to the millimetre:

  * the topics exist, with the right types, encodings and QoS, and the
    detector nodes actually match them;
  * CameraInfo intrinsics reach the consumer and are used, rather than a
    hardcoded default silently winning;
  * DEPROJECTION -- pixel + depth -> a 3-D point -- lands on the object's
    true world position, through the real TF chain. This is arithmetic
    against a known answer, which is exactly the case the rule permits;
  * the optical-frame convention and the world <- camera transform compose
    in the right order (getting this backwards is the AprilTag
    object-space-vs-world-space bug in a new costume);
  * timing, synchronisation and the tracker's 1.5 s drop.

CANNOT be verified here, and no amount of work on this file changes it:

  * DETECTION RATE OR ACCURACY, for YOLO-World or any learned detector.
    This is not a hedge, it is MEASURED: flat-shaded primitives on a flat
    background score 0-4% while the identical model scores 0.89-0.91 on a
    real photograph. Rendering prettier primitives would move the number
    without making it mean anything -- that is the trap the 0-4% figure
    already sprang once.
  * the REAL intrinsics, distortion and the colour-to-depth extrinsic. The
    values below are nominal, not the Kinova's, and they are marked so in
    the CameraInfo header comment. The driver is the only source for those.
  * exposure, white balance, motion blur, rolling shutter, IR dropout on
    dark or specular surfaces, and depth holes at range -- every one of
    which is a real failure mode of the real sensor and none of which is
    modelled here.
  * whether the Kinova vision module streams at all over this machine's
    network path, which is a live question given the UDP finding.

So: the pipeline is exercised UP TO the camera. The camera itself, and
anything that depends on how the world actually LOOKS, is unverified until
the real driver runs. AprilTag is the one detector whose accuracy is
partially transferable, because a tag is constructed geometry rather than
appearance -- and even there the earlier characterisation was invalidated
once by a mirrored renderer, so it is re-checked, not assumed.
"""
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

# THREE levels up, not four: srl_perception/srl_perception -> srl_perception
# -> src -> the workspace root. One extra ".." pointed outside the workspace,
# clip_scene failed to import, and _scene() returned [] -- so the node ran,
# published, and rendered an empty world while reporting success.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "scripts"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "src", "srl_experiments",
                                "experiments", "abc"))

W, H = 640, 480
# NOMINAL intrinsics for a 640x480 RGB-D wrist camera -- a ~55 deg horizontal
# field of view. THESE ARE NOT THE KINOVA'S. The real values arrive in the
# driver's own CameraInfo; anything calibrated against these numbers is
# calibrated against an assumption.
FX = FY = 615.0
CX, CY = W / 2.0 - 0.5, H / 2.0 - 0.5
DEPTH_MAX_M = 4.0

BG_RGB = (58, 60, 64)          # a flat backdrop, and it is flat ON PURPOSE:
BG_DEPTH_MM = 0                # 0 = invalid, the real sensor's no-return code

# CALIBRATION PROBES -- three markers at FIXED WORLD POSITIONS, published only
# with `probe:=true`.
#
# WHY THEY EXIST. Deprojection can only be checked against an object the camera
# can SEE, and at the home pose neither wrist camera frames the work surface:
# measured, the bench and table sit at pixel (954, 539) and (1115, 765) for the
# right camera -- in front of it, but outside a 640 x 480 frame. That is not a
# defect, it is the documented parking (CLAUDE.md: the left wrist "points
# UPWARD - sees nothing at table height from home"; point the arms first).
# Verifying against an empty frame measures nothing and reports it as a
# pipeline failure, which is the instrument-blaming-the-system error the
# standing rule exists to stop.
#
# THESE ARE CONSTANTS, NOT DERIVED AT RUNTIME. They were computed once from the
# measured home camera pose and then written down. A probe recomputed from the
# same TF the renderer uses could agree with a wrong frame composition; a fixed
# number cannot, and the transposed-rotation control in verify_mock_camera.py
# confirms the check discriminates. They are render targets in free space and
# are NOT collision objects -- nothing plans against them.
PROBE_WORLD = [
    (-0.4994, 0.4244, 1.4010),
    (-0.3522, 0.5664, 1.3627),
    (-0.5127, 0.6870, 1.5867),
]
PROBE_SIZE = 0.05


def _quat_to_R(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class MockRGBD(Node):
    """Renders the CONSTRUCTED scene from the live camera pose."""

    def __init__(self):
        super().__init__("mock_rgbd_camera")
        self.declare_parameter("arm", "left")
        self.declare_parameter("task", "t1")
        # STAGE 2 DRAWS ITS CUBES, so the renderer needs the same seed the
        # task and the scene node were given or it paints a different table
        # from the one the arm is working on.
        self.declare_parameter("seed", 0)
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("frame_id", "")
        self.declare_parameter("probe", False)
        self.arm = self.get_parameter("arm").value
        self.task = self.get_parameter("task").value
        self.seed = int(self.get_parameter("seed").value)
        self.frame = self.get_parameter("frame_id").value or \
            "%s_camera_color_frame" % self.arm

        q = QoSProfile(depth=1)
        q.reliability = ReliabilityPolicy.RELIABLE
        base = "/%s_camera" % self.arm
        self.pub_c = self.create_publisher(Image, base + "/color/image_raw", q)
        self.pub_i = self.create_publisher(CameraInfo,
                                           base + "/color/camera_info", q)
        self.pub_d = self.create_publisher(Image, base + "/depth/image_raw", q)
        # THE POSE THIS FRAME WAS RENDERED FROM, published with the frame's own
        # timestamp.
        #
        # WHY IT EXISTS. A consumer that deprojects a pixel needs the camera
        # pose the pixel was TAKEN from, and the only way to get it off TF is
        # to look it up again later and hope the arm has not moved. Measured
        # inside the recording sweep on 2026-08-17, it had: the four T1 cubes
        # came back 31.7, 32.7, 34.0 and 35.7 mm from truth against a 30 mm
        # capture gate, so NONE of the four was grasped -- and the pad miss at
        # closure was 31.6, 32.6, 33.9 and 35.6 mm, the SAME NUMBERS to
        # 0.1 mm. The whole grasp failure was the deprojection reading a pose
        # the frame did not come from. Standalone on the same stack, with
        # nothing else loading the machine, the same code returned 1.3-3.0 mm,
        # which is why it looked intermittent.
        #
        # Publishing it is the fix that cannot be got wrong later: the pose
        # travels WITH the frame, so a consumer either uses the right one or
        # has to ignore a message that is sitting there.
        self.pub_p = self.create_publisher(PoseStamped, base + "/render_pose", q)

        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        self.objects = self._scene()
        # MOVE THE WHOLE SCENE, for testing whether anything downstream
        # actually FINDS the table rather than assuming where it is.
        #
        # `SRL_SCENE_SHIFT_XY="dx,dy"` translates every rendered solid. The
        # task files are untouched, so the DECLARED coordinates stay where
        # they were and the rendered world moves -- which is exactly the real
        # situation the declared pipeline cannot survive, and the one a
        # simulation where "declared == truth" can otherwise never produce.
        sh = os.environ.get("SRL_SCENE_SHIFT_XY", "")
        if sh:
            try:
                dx, dy = (float(v) for v in sh.split(","))
            except Exception:                              # noqa: BLE001
                self.get_logger().error(
                    "SRL_SCENE_SHIFT_XY=%r is not 'dx,dy' -- NOT shifting, "
                    "rather than shifting by a number I invented" % sh)
            else:
                for o in self.objects:
                    o["xyz"] = [o["xyz"][0] + dx, o["xyz"][1] + dy,
                                o["xyz"][2]]
                self.get_logger().warn(
                    "SCENE SHIFTED by (%+.3f, %+.3f) m. The rendered world no "
                    "longer matches the task files, ON PURPOSE." % (dx, dy))
        self.n_pub = 0
        self.no_tf = 0
        hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / hz, self.tick)

        self.get_logger().warn(
            "MOCK CAMERA on %s -- geometry is real, APPEARANCE IS NOT. "
            "Deprojection and frames can be verified against this; "
            "detection rate CANNOT. See the module docstring." % base)
        self.get_logger().info("%d scene objects for task '%s', frame %s"
                               % (len(self.objects), self.task, self.frame))

    def _scene(self):
        """The objects, from the SAME definition the clips and tasks use.

        Reusing `clip_scene` rather than restating the geometry is the point:
        a mock with its own copy of the scene drifts from the real one and
        then verifies a world nothing else lives in.
        """
        out = []
        # PROBES FIRST, before any import that can fail. They were behind the
        # clip_scene guard, whose early return dropped them silently.
        if self.get_parameter("probe").value:
            for i, xyz in enumerate(PROBE_WORLD):
                out.append(dict(xyz=list(xyz),
                                size=[PROBE_SIZE] * 3,
                                rgb=(0.9, 0.2, 0.2),
                                name="probe_%d" % i))
        try:
            import clip_scene as CS
        except Exception as e:                        # pragma: no cover
            self.get_logger().error("no clip_scene: %s -- probes only" % e)
            return out
        # furniture_boxes() yields (name, xyz, size, rgba) TUPLES; fixtures_for()
        # yields bare NAMES with no geometry. Reading either as a dict is how the
        # first version of this node rendered an empty scene while reporting
        # success -- so the shapes are unpacked explicitly here.
        for name, xyz, size, rgba in CS.furniture_boxes(self.task):
            out.append(dict(xyz=list(xyz), size=list(size),
                            rgb=tuple(rgba[:3]), name=name))
        # The two coloured mats: geometry lives in the task spec, not in
        # fixtures_for(), which is a NAME list.
        if self.task in ("t1", "t1s2"):
            try:
                # T1's OWN GEOMETRY, from the module that owns it. Rebuilt
                # 2026-08-17: the pads rest ON the table rather than hanging
                # at BENCH_TOP, and they are T1's own size, not clip_scene's
                # generic one. Rendering them at the old height would put the
                # detector's depth step 120 mm from where the cubes are, and
                # the depth step is the ONLY thing that separates a cube from
                # the same-coloured pad it stands on.
                import t1_task as T1M
                for i, (px, py) in enumerate(T1M.T1_PLANES):
                    out.append(dict(
                        xyz=[px, py, T1M.TABLE_TOP + T1M.PLANE_T / 2.0],
                        size=[T1M.PAD_W, T1M.PAD_D, T1M.PLANE_T],
                        rgb=CS.BLUE[:3] if i == 0 else CS.GREEN[:3],
                        name="plane_%d" % i))
            except Exception as e:
                self.get_logger().warn("no plane geometry: %s" % e)
            # THE GRASPABLE OBJECTS. Without them the mock renders the
            # furniture and nothing else, and "can the camera see the
            # objects" -- the question a scan pose exists to answer -- is
            # unanswerable from it.
            try:
                import t1_task as T1M
                # THE CUBES OF WHICHEVER STAGE THIS IS. Stage 1's are fixed;
                # stage 2's are drawn from the seed, and its colours follow
                # the SIDE, because neither arm crosses the centreline and a
                # cube can only be delivered to the pad on its own side.
                if self.task == "t1s2":
                    lay = T1M.stage2_layout(self.seed)
                    cubes = [(c[0], c[1]) for c in lay]
                    rendered = [T1M.PLANE_COLOURS[c[2]] for c in lay]
                else:
                    cubes = [tuple(p) for p in T1M.T1_CUBES]
                    rendered = list(T1M.T1_RENDERED)
                for i, (px, py) in enumerate(cubes):
                    out.append(dict(
                        xyz=[px, py, T1M.T1_Z],
                        size=[T1M.CUBE_M] * 3,
                        # THE SCENE'S OWN COLOURS, NOT A SECOND COPY.
                        #
                        # These were literals here, and they had already
                        # drifted: clip_scene draws GREEN as (0.1, 0.9, 0.2)
                        # and this rendered (0.1, 0.8, 0.3) -- 25 counts of G
                        # and 25 of B apart at 8-bit. So the detector was
                        # being exercised against a green the clips never
                        # show, which is the whole class of fault this file
                        # avoids elsewhere by importing clip_scene rather than
                        # restating the scene. Blue happened to match exactly,
                        # which is how it went unnoticed.
                        #
                        # Checked before changing: the scene's green sits
                        # inside the detector's HSV band with margins
                        # H,S,V = 21, 29, 25 against the old 17, 32, 51, so
                        # the hue margin IMPROVES and the value margin drops
                        # to the same 25 the blue cube already had.
                        # THE CUBE'S DECLARED COLOUR, which the detector then has
                        # to READ back off the pixels. Two blue then two
                        # green, not alternating: the blue pair stands on
                        # the LEFT arm's side and the green pair on the
                        # RIGHT arm's, because a cube goes to the pad of
                        # its own colour and one arm cannot reach both.
                        # `T1_RENDERED`, NOT `T1_PAIR`. The pair table is
                        # what the TASK believes; this node paints what the
                        # camera SEES, and the mislabel control works only
                        # because the two are separate. Reading T1_PAIR here
                        # would repaint the scene whenever the declaration was
                        # flipped, so the control would compare a file with
                        # itself and always report agreement.
                        rgb=(CS.BLUE[:3] if rendered[i] == "blue"
                             else CS.GREEN[:3]),
                        name="cube_%d" % i))
            except Exception as e:
                self.get_logger().warn("no cube geometry: %s" % e)
        return out

    def _cam_pose(self):
        try:
            t = self.tfb.lookup_transform("world", self.frame,
                                          rclpy.time.Time())
        except Exception:
            return None, None
        tr, ro = t.transform.translation, t.transform.rotation
        return (np.array([tr.x, tr.y, tr.z]),
                _quat_to_R(ro.x, ro.y, ro.z, ro.w))

    def render(self, p_cam, R_wc):
        """Colour + PER-PIXEL depth, by ray/box intersection.

        R_wc maps CAMERA -> WORLD, so its transpose maps world points into the
        optical frame. Composing these the wrong way round is the failure this
        mock exists to let somebody catch cheaply, and the verifier's control
        deliberately does it wrong to prove the check can tell.

        THE FIRST VERSION PAINTED EACH BOX AS A FLAT CARD AT ITS CENTROID
        DEPTH, and that is worth recording because it looked entirely fine.
        A table is 1.7 m wide and 0.4 m from the wrist camera, so its card
        covered the WHOLE frame at one constant depth and overwrote every
        other object's depth with 0.44 m. The colour image was unchanged and
        plausible; only the depth was quietly, uniformly wrong -- which is the
        same shape as every other silent-wrong-data bug in this project. A
        real box has a depth GRADIENT across it, so the renderer computes one.
        """
        col = np.zeros((H, W, 3), np.uint8)
        col[:, :] = BG_RGB
        best = np.full((H, W), np.inf)          # nearest hit so far, metres
        dep = np.full((H, W), BG_DEPTH_MM, np.uint16)
        R_cw = R_wc.T

        for o in self.objects:
            c = np.asarray(o["xyz"], float)
            h = 0.5 * np.asarray(o["size"], float)
            lo, hi = c - h, c + h
            pc = R_cw @ (c - p_cam)
            if pc[2] <= 0.02 or pc[2] > DEPTH_MAX_M:
                continue
            # Project the eight corners to bound the pixels worth testing.
            us, vs = [], []
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        q = R_cw @ (c + h * np.array([sx, sy, sz]) - p_cam)
                        if q[2] <= 0.02:
                            us, vs = [0, W], [0, H]
                            break
                        us.append(FX * q[0] / q[2] + CX)
                        vs.append(FY * q[1] / q[2] + CY)
            u0 = max(0, int(np.floor(min(us))))
            u1 = min(W, int(np.ceil(max(us))) + 1)
            v0 = max(0, int(np.floor(min(vs))))
            v1 = min(H, int(np.ceil(max(vs))) + 1)
            if u1 <= u0 or v1 <= v0:
                continue

            uu, vv = np.meshgrid(np.arange(u0, u1, dtype=float),
                                 np.arange(v0, v1, dtype=float))
            d_cam = np.stack([(uu - CX) / FX, (vv - CY) / FY,
                              np.ones_like(uu)], axis=-1)
            d_w = d_cam @ R_wc.T                       # ray dirs, world frame
            with np.errstate(divide="ignore", invalid="ignore"):
                inv = 1.0 / d_w
                t0 = (lo - p_cam) * inv
                t1 = (hi - p_cam) * inv
            tmin = np.nanmax(np.minimum(t0, t1), axis=-1)
            tmax = np.nanmin(np.maximum(t0, t1), axis=-1)
            hit = (tmax >= np.maximum(tmin, 0.0))
            if not hit.any():
                continue
            # depth is the optical-frame z of the hit, i.e. t scaled by the
            # ray's own z component -- NOT the distance along the ray.
            z = np.where(hit, np.maximum(tmin, 0.0), np.inf)
            z = np.where(hit & (z > 0), z, np.inf)
            sub = best[v0:v1, u0:u1]
            take = hit & (z < sub) & (z <= DEPTH_MAX_M)
            if not take.any():
                continue
            sub[take] = z[take]
            best[v0:v1, u0:u1] = sub
            csub = col[v0:v1, u0:u1]
            r, g, b = [int(255 * min(1.0, max(0.0, v))) for v in o["rgb"][:3]]
            csub[take] = (r, g, b)
            col[v0:v1, u0:u1] = csub

        finite = np.isfinite(best)
        dep[finite] = np.clip(best[finite] * 1000.0, 1, 65535).astype(np.uint16)
        return col, dep

    def _img(self, stamp, arr, enc, step):
        m = Image()
        m.header.stamp = stamp
        m.header.frame_id = self.frame
        m.height, m.width = arr.shape[0], arr.shape[1]
        m.encoding = enc
        m.is_bigendian = 0
        m.step = step
        m.data = arr.tobytes()
        return m

    @staticmethod
    def _quat_from_R(R):
        """Rotation matrix -> (x, y, z, w). Shepperd's branch, so the trace
        term is never the one being divided by when it is near zero."""
        t = R[0][0] + R[1][1] + R[2][2]
        if t > 0.0:
            s2 = np.sqrt(t + 1.0) * 2.0
            return ((R[2][1] - R[1][2]) / s2, (R[0][2] - R[2][0]) / s2,
                    (R[1][0] - R[0][1]) / s2, 0.25 * s2)
        if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
            s2 = np.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
            return (0.25 * s2, (R[0][1] + R[1][0]) / s2,
                    (R[0][2] + R[2][0]) / s2, (R[2][1] - R[1][2]) / s2)
        if R[1][1] > R[2][2]:
            s2 = np.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
            return ((R[0][1] + R[1][0]) / s2, 0.25 * s2,
                    (R[1][2] + R[2][1]) / s2, (R[0][2] - R[2][0]) / s2)
        s2 = np.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        return ((R[0][2] + R[2][0]) / s2, (R[1][2] + R[2][1]) / s2,
                0.25 * s2, (R[1][0] - R[0][1]) / s2)

    def _render_pose(self, stamp, p_cam, R_wc):
        m = PoseStamped()
        m.header.stamp = stamp
        m.header.frame_id = "world"
        m.pose.position.x = float(p_cam[0])
        m.pose.position.y = float(p_cam[1])
        m.pose.position.z = float(p_cam[2])
        qx, qy, qz, qw = self._quat_from_R(np.asarray(R_wc, float))
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = (
            float(qx), float(qy), float(qz), float(qw))
        return m

    def tick(self):
        p_cam, R_wc = self._cam_pose()
        if p_cam is None:
            self.no_tf += 1
            if self.no_tf in (15, 150) or self.no_tf % 450 == 0:
                # NAMED, not silent. A camera publishing nothing because TF is
                # missing looks exactly like a camera that is not running.
                self.get_logger().warn(
                    "no TF world <- %s after %d ticks -- PUBLISHING NOTHING. "
                    "Is robot_state_publisher up?" % (self.frame, self.no_tf))
            return
        col, dep = self.render(p_cam, R_wc)
        stamp = self.get_clock().now().to_msg()

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame
        info.height, info.width = H, W
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.pub_i.publish(info)
        self.pub_c.publish(self._img(stamp, col, "rgb8", W * 3))
        self.pub_d.publish(self._img(stamp, dep, "16UC1", W * 2))
        self.pub_p.publish(self._render_pose(stamp, p_cam, R_wc))
        self.n_pub += 1
        if self.n_pub == 1 or self.n_pub % 300 == 0:
            self.get_logger().info(
                "published %d frames; camera at (%.3f, %.3f, %.3f), "
                "%d objects in view"
                % (self.n_pub, p_cam[0], p_cam[1], p_cam[2],
                   int((dep > 0).any() and (dep > 0).sum() > 0)))


def main(argv=None):
    rclpy.init(args=argv)
    n = MockRGBD()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
