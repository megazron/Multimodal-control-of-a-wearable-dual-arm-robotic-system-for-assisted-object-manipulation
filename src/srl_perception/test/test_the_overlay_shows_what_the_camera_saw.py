"""The overlay is the JSON burned onto the frame it came from, and a camera
that refuses shows WHY on its own picture.

CONSTRUCTED ground truth only, per the standing rule: frames are flat arrays
built by assignment, entries are dicts written by hand, and every assertion
compares pixels against a copy of the input -- arithmetic, never rendering.
What is pinned:

  * purity: `draw_overlay` returns a NEW image and never touches its input;
  * the boxes: pixels change where the entry says an object is, and do NOT
    change far from the boxes and below the banner -- a function that
    repainted the whole frame would pass a weaker check;
  * the refusal: a refusing entry still yields an overlay, the banner strip
    is drawn, and two DIFFERENT refusal strings produce different pixels,
    so the text is burned in rather than merely received;
  * the no-republish rule, at the node level like the staleness test: no
    fresh frame since the last publish -> publish NOTHING.
"""
import numpy as np
import pytest

pytest.importorskip("rclpy")

from srl_perception import scene_understanding_node as SU  # noqa: E402

W, H = 640, 360
GREY = 120


def flat_frame():
    return np.full((H, W, 3), GREY, np.uint8)


def metric_entry():
    """An entry shaped exactly like `_camera_entry`'s metric answer."""
    return {
        "method": "plane_cluster", "frame": "robot",
        "table": {"normal": [0.0, -0.6, -0.8], "d_m": 0.9,
                  "inliers": 5000, "rms_mm": 1.2},
        "objects": [dict(label="cube", pickable=True, colour="green",
                         centre=[0.4, 0.2, 1.1], width_mm=40.0,
                         height_mm=40.0, extents_m=[0.04, 0.04, 0.04],
                         n_points=500,
                         bbox_depth_px=[200, 150, 280, 230])]}


def test_the_overlay_is_pure_and_changes_pixels_where_the_box_is():
    bgr = flat_frame()
    before = bgr.copy()
    out = SU.draw_overlay(bgr, metric_entry(), name="gripper/left")
    assert np.array_equal(bgr, before), "draw_overlay mutated its input"
    assert out is not bgr and out.shape == bgr.shape
    # The box edge: pixels along the declared bbox border changed.
    assert not np.array_equal(out[150, 200:280], before[150, 200:280])
    assert not np.array_equal(out[150:230, 200], before[150:230, 200])
    # The banner: the top strip changed.
    assert not np.array_equal(out[0:10], before[0:10])
    # And far from box and banner the frame is untouched -- a function that
    # repainted everything would pass the two checks above.
    assert np.array_equal(out[300:340, 450:600], before[300:340, 450:600])


def test_depth_pixel_boxes_are_scaled_onto_a_larger_colour_frame():
    """bbox_depth_px is in DEPTH pixels; on a colour frame twice the size
    the drawn box must land at twice the coordinates, by arithmetic."""
    bgr = flat_frame()
    before = bgr.copy()
    ent = metric_entry()
    ent["objects"][0]["bbox_depth_px"] = [100, 75, 140, 115]
    out = SU.draw_overlay(bgr, ent, name="gripper/left",
                          depth_shape=(H // 2, W // 2))
    assert not np.array_equal(out[150, 200:280], before[150, 200:280])
    assert np.array_equal(out[300:340, 450:600], before[300:340, 450:600])


def test_a_refusing_camera_still_gets_an_overlay_with_the_reason_burned_in():
    bgr = flat_frame()
    before = bgr.copy()
    refusal = ("camera_info K is zero for scene/rs -- cannot deproject, "
               "so no metric claims from this camera")
    out = SU.draw_overlay(bgr, {"refusal": refusal}, name="scene/rs")
    assert np.array_equal(bgr, before)
    assert not np.array_equal(out[0:12], before[0:12]), "no banner drawn"
    # No object boxes on a refusal: below the banner the frame is untouched.
    assert np.array_equal(out[H // 2:], before[H // 2:])
    # The TEXT is burned, not merely received: a different refusal string
    # must produce different pixels.
    out2 = SU.draw_overlay(bgr, {"refusal": "no frames on /x -- this camera "
                                            "has never spoken"},
                           name="scene/rs")
    assert not np.array_equal(out, out2)


def test_2d_detections_are_drawn_and_the_backend_is_in_the_banner():
    """The colour-only scene camera's entry: pixel boxes plus provenance."""
    bgr = flat_frame()
    before = bgr.copy()
    ent = {"backend": "colour", "frame": "camera:scene/usb (pixels)",
           "why_no_metric": "zero K",
           "detections": [dict(label="green object", score=0.91,
                               backend="colour",
                               bbox_px=[300, 200, 380, 280])]}
    out = SU.draw_overlay(bgr, ent, name="scene/usb")
    assert not np.array_equal(out[200, 300:380], before[200, 300:380])
    # The backend NAME reaches the banner: renaming it changes the pixels.
    ent2 = dict(ent, backend="owlv2")
    out2 = SU.draw_overlay(bgr, ent2, name="scene/usb")
    assert not np.array_equal(out[0:20], out2[0:20])


def test_no_fresh_frame_means_no_publish():
    """The node's no-republishing rule, at the node level: one arrival, one
    overlay -- a second cycle with no new frame publishes NOTHING, a stale
    frame publishes nothing, and a refusing camera with a FRESH frame does
    publish (the refusal rides the frame it refused about)."""
    rclpy = pytest.importorskip("rclpy")
    import time
    from sensor_msgs.msg import Image
    rclpy.init()
    try:
        n = SU.SceneUnderstanding()
        name = "gripper/left"
        sent = []
        n.overlay_pubs[name].publish = sent.append
        im = Image()
        im.height, im.width, im.encoding = 8, 8, "bgr8"
        im.data = bytes(8 * 8 * 3)
        n.col[name] = im
        n.t[name] = time.time()
        ent = n._camera_entry(name, SU.CAMS[name], [])
        assert "refusal" in ent               # fresh frame, but no depth yet
        n._publish_overlay(name, ent)
        assert len(sent) == 1, "a refusing camera with a fresh frame " \
                               "must still show why"
        assert sent[0].encoding == "bgr8"
        assert (sent[0].height, sent[0].width) == (8, 8)
        # Same stamp, next cycle: nothing.
        n._publish_overlay(name, ent)
        assert len(sent) == 1, "re-published an overlay with no fresh frame"
        # A new arrival publishes again.
        n.t[name] = time.time()
        n._publish_overlay(name, ent)
        assert len(sent) == 2
        # A stale frame publishes nothing, matching the analysis gate.
        n.t[name] = time.time() - 10.0
        n._publish_overlay(name, ent)
        assert len(sent) == 2, "published an overlay for a stale frame"
        # And a camera that never spoke publishes nothing.
        n._publish_overlay("scene/rs", {"refusal": "no frames"})
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_the_horizon_line_is_the_constructed_vanishing_line():
    """For n = (0, n1, n2) the ray through pixel v is parallel to the plane
    exactly when n1 (v - cy) / fy + n2 = 0, so the horizon sits at
    v = cy - fy * n2 / n1: pure arithmetic, checked against it. The plane
    is tilted 80 deg so the line actually crosses the 120 px frame."""
    fx, fy, cx, cy = 200.0, 200.0, 80.0, 60.0
    a = np.deg2rad(80.0)
    n = (0.0, -np.sin(a), -np.cos(a))
    ends = SU._horizon_endpoints(n, (fx, fy, cx, cy), 160, 120)
    assert ends is not None
    v_truth = cy - fy * n[2] / n[1]
    for (_, v) in ends:
        assert abs(v - v_truth) < 1e-6, (v, v_truth)
    # A fronto-parallel plane has no horizon in frame -- None, not a crash.
    assert SU._horizon_endpoints((0.0, 0.0, -1.0),
                                 (fx, fy, cx, cy), 160, 120) is None
