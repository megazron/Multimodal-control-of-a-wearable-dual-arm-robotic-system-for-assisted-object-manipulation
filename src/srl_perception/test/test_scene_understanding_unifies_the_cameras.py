"""One vision pipeline over four cameras, and every answer says how it knew.

CONSTRUCTED ground truth only, per the standing rule: the frames here are
built by arithmetic -- a pinhole projection of a fronto-parallel plane with
two boxes of KNOWN size at KNOWN places -- never rendered. What is pinned:

  * the measurement: two boxes of known geometry come back within tolerance,
    with the derived name and the sampled colour;
  * the refusals: zero K, empty depth, and too-sparse depth each produce a
    NAMED refusal, not an empty result -- a check that cannot fail on a
    deliberately broken input is not a check;
  * the frames: only a camera with a pose gets "robot"; the colour-only
    camera yields pixel boxes and a `why_no_metric`, and NO metric fields --
    a zero-K camera claiming metres would be the "feature present but does
    nothing" row inverted: a number present that means nothing.
"""
import numpy as np
import pytest

pytest.importorskip("rclpy")

from srl_perception import scene_understanding_node as SU  # noqa: E402
from srl_perception.prompt_detector import PromptDetector  # noqa: E402

# The constructed camera: 160x120, fx = fy = 200, principal point centred.
W, H = 160, 120
K4 = (200.0, 200.0, 80.0, 60.0)
PLANE_Z = 0.90

# Ground truth, chosen so the two objects are unambiguous: a 40 mm cube
# (pickable) and a 120 x 120 x 60 mm box (wider than the 85 mm jaws).
CUBE = dict(centre_xy=(0.05, 0.02), size=0.040, height=0.040)
BOX = dict(centre_xy=(-0.08, -0.03), size=0.120, height=0.060)


def frame_with_boxes(objects=(CUBE, BOX), plane_z=PLANE_Z, colour=True):
    """Depth + BGR built by projection arithmetic. The top face of each box
    sits `height` closer than the plane; sides are invisible fronto-parallel,
    which is exactly what a downward camera returns."""
    fx, fy, cx, cy = K4
    depth = np.full((H, W), plane_z, np.float32)
    bgr = np.full((H, W, 3), 120, np.uint8)
    for i, o in enumerate(objects):
        z = plane_z - o["height"]
        x0, x1 = o["centre_xy"][0] - o["size"] / 2, o["centre_xy"][0] + o["size"] / 2
        y0, y1 = o["centre_xy"][1] - o["size"] / 2, o["centre_xy"][1] + o["size"] / 2
        u0, u1 = int(round(fx * x0 / z + cx)), int(round(fx * x1 / z + cx))
        v0, v1 = int(round(fy * y0 / z + cy)), int(round(fy * y1 / z + cy))
        depth[v0:v1 + 1, u0:u1 + 1] = z
        if colour:
            # The cube is green (BGR), the box stays plain grey.
            bgr[v0:v1 + 1, u0:u1 + 1] = (30, 200, 40) if i == 0 else (120, 120, 120)
    return bgr, depth


def objects_by_width(res):
    return sorted(res["objects"], key=lambda o: o["width_mm"])


def test_two_known_boxes_are_measured_within_tolerance():
    bgr, depth = frame_with_boxes()
    res = SU.analyse_metric(bgr, depth, K4)
    assert "refusal" not in res, res
    assert res["table"]["rms_mm"] < 5.0
    objs = objects_by_width(res)
    assert len(objs) == 2, objs
    cube, box = objs
    assert abs(cube["width_mm"] - 40.0) < 6.0, cube
    assert abs(cube["height_mm"] - 40.0) < 6.0, cube
    assert abs(box["width_mm"] - 120.0) < 8.0, box
    assert abs(box["height_mm"] - 60.0) < 6.0, box
    # Centres in the camera frame, against the constructed truth. The points
    # are the TOP FACE, so z is plane_z - height.
    cc = np.array(cube["centre"])
    truth = np.array([CUBE["centre_xy"][0], CUBE["centre_xy"][1],
                      PLANE_Z - CUBE["height"]])
    assert np.linalg.norm(cc - truth) < 0.010, (cc, truth)


def test_the_name_and_pickability_are_derived_from_the_measurement():
    bgr, depth = frame_with_boxes()
    cube, box = objects_by_width(SU.analyse_metric(bgr, depth, K4))
    assert cube["label"] == "cube" and cube["pickable"] is True
    assert box["label"] == "box" and box["pickable"] is False


def test_colour_is_sampled_from_the_pixels_not_declared():
    bgr, depth = frame_with_boxes()
    cube, box = objects_by_width(SU.analyse_metric(bgr, depth, K4))
    assert cube["colour"] == "green", cube
    assert box["colour"] != "green", box


def test_unaligned_colour_is_reported_unknown_not_sampled_wrongly():
    """The wrist pair is 480x270 depth against 1280x720 colour, ~27 mm apart;
    sampling colour at scaled depth pixels put boxes in the wrong corner of
    the frame (measured in the standalone script). bgr=None is that case."""
    _, depth = frame_with_boxes()
    cube, _ = objects_by_width(SU.analyse_metric(None, depth, K4))
    assert "unknown" in cube["colour"]


# ------------------------------------------------------- the broken inputs

def test_zero_k_is_a_named_refusal():
    bgr, depth = frame_with_boxes()
    res = SU.analyse_metric(bgr, depth, (0.0, 0.0, 0.0, 0.0))
    assert "refusal" in res
    assert "zero" in res["refusal"].lower()
    assert "objects" not in res


def test_empty_depth_is_a_named_refusal():
    res = SU.analyse_metric(None, np.zeros((H, W), np.float32), K4)
    assert "refusal" in res
    assert "no depth" in res["refusal"]


def test_too_sparse_depth_refuses_rather_than_fitting_a_plane():
    depth = np.zeros((H, W), np.float32)
    depth[0, :100] = 0.9                        # 100 returns: below the floor
    res = SU.analyse_metric(None, depth, K4)
    assert "refusal" in res
    assert "not enough" in res["refusal"]


def test_a_clean_frame_passes_so_the_refusals_are_not_a_wall():
    """A gate that rejects everything is not a gate."""
    bgr, depth = frame_with_boxes()
    assert "refusal" not in SU.analyse_metric(bgr, depth, K4)


# ------------------------------------------------------------- the frames

def test_to_world_is_the_constructed_rigid_transform():
    """90 deg about z plus a translation, checked by arithmetic."""
    q = (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5))
    p = SU.to_world([(1.0, 0.0, 0.0)], (1.0, 2.0, 3.0), q)[0]
    assert np.allclose(p, (1.0, 3.0, 3.0), atol=1e-9), p


def test_the_plane_transforms_with_the_points():
    """Heights above the plane are invariant under the rigid transform --
    if they were not, the segment path would gate on the wrong points."""
    rng = np.random.default_rng(3)
    P = rng.uniform(-0.3, 0.3, (50, 3)) + [0, 0, 0.9]
    n, d = np.array([0.0, 0.0, -1.0]), 0.9
    t, q = (0.4, -0.2, 1.1), (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5))
    nw, dw = SU.plane_to_world(n, d, t, q)
    Pw = SU.to_world(P, t, q)
    assert np.allclose(P @ n + d, Pw @ nw + dw, atol=1e-9)


def test_markers_carry_only_robot_frame_objects_and_wipe_first():
    objs = [dict(centre=[0.4, 0.2, 1.1], extents_m=[0.04, 0.04, 0.04])]
    arr = SU.markers_for(objs)
    from visualization_msgs.msg import Marker
    assert arr.markers[0].action == Marker.DELETEALL
    cubes = [m for m in arr.markers if m.type == Marker.CUBE]
    assert len(cubes) == 1
    assert cubes[0].header.frame_id == "world"


# ------------------------------------------- the colour-only scene camera

def test_the_colour_only_camera_makes_no_metric_claims():
    """Pixel boxes and provenance only. The HSV fallback finds the green
    square; 'person' is listed NOT-detectable with the reason, because the
    fallback genuinely cannot answer it and must say so."""
    det = PromptDetector(backend="colour")
    bgr = np.full((120, 160, 3), 120, np.uint8)
    bgr[40:80, 60:100] = (30, 200, 40)          # a green square
    rep = SU.colour_report(bgr, det, labels=["person", "robot arm", "table"],
                           probes=["green object"])
    assert rep["backend"] == "colour"
    labels_out = {d["label"] for d in rep["detections"]}
    assert "green object" in labels_out, rep
    not_det = {d["label"] for d in rep["not_detectable"]}
    assert {"person", "robot arm", "table"} <= not_det, rep
    for d in rep["detections"]:
        assert d["backend"] == "colour"
        assert "bbox_px" in d
        for metric_key in ("centre", "extents_m", "width_mm", "depth_m"):
            assert metric_key not in d, "metric claim from a 2-D detection"
    assert rep["backend_note"], "the fallback must SAY it is the fallback"


def test_provenance_fields_are_present_on_every_measured_object():
    bgr, depth = frame_with_boxes()
    res = SU.analyse_metric(bgr, depth, K4)
    for o in res["objects"]:
        for key in ("label", "centre", "extents_m", "n_points", "colour"):
            assert key in o, (key, o)
    # The table itself carries its fit quality, so a bad plane is visible.
    assert set(res["table"]) >= {"normal", "rms_mm", "inliers"}


def test_a_stale_camera_refuses_instead_of_republishing_its_last_frame():
    """Stale-data-with-a-fresh-timestamp is the instrument-failure table's
    third row. Injected: a camera whose last frame is older than the gate
    must produce a refusal naming the age, not an object list."""
    rclpy = pytest.importorskip("rclpy")
    import time
    rclpy.init()
    try:
        n = SU.SceneUnderstanding()
        name, cfg = "gripper/left", SU.CAMS["gripper/left"]
        from sensor_msgs.msg import Image
        im = Image()
        im.height, im.width, im.encoding = 4, 4, "bgr8"
        im.data = bytes(4 * 4 * 3)
        n.col[name] = im
        n.t[name] = time.time() - 10.0          # ten seconds silent
        ent = n._camera_entry(name, cfg, [])
        assert "refusal" in ent and "stale" in ent["refusal"], ent
        # And a FRESH frame with no depth refuses about the depth instead --
        # the staleness gate must not swallow every path after it.
        n.t[name] = time.time()
        ent = n._camera_entry(name, cfg, [])
        assert "refusal" in ent and "depth" in ent["refusal"], ent
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_segmenter_availability_is_a_statement_not_a_crash():
    ok, why = SU.segmenter_available()
    assert ok in (True, False)
    if not ok:
        assert why, "an unavailable segmenter must say why"
