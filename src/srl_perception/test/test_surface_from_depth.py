#!/usr/bin/env python3
"""KNOWN ANSWERS FOR THE WORK-SURFACE ESTIMATOR.

TASK_SPEC.md U-3: table height is declared and never measured. The estimator
that closes it is only worth having if it is right, and "right" here means a
CONSTRUCTED scene whose true surface height I wrote down before running
anything. No renderer is involved: every depth value in these tests is
computed from the plane equation, so the ground truth is arithmetic.

WHAT EACH TEST IS DEFENDING AGAINST, in one line:

  * the flat case            -- does it find a height it was given
  * cubes on the surface     -- does a mean creep upward (this is why it is
                                a mode; the assertion fails on a mean)
  * a 20 mm error            -- does it actually resolve the size of error
                                work_surface.TOL_M exists to catch
  * empty and clutter        -- does it REFUSE rather than return a number
  * the units branch         -- 16UC1 is mm and 32FC1 is m, read from the
                                encoding and never from the magnitude
  * a bin-edge surface       -- does the refinement recover the sub-bin
                                position instead of sitting a bin off
"""

import math
import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src", "srl_perception"))

from srl_perception.surface_from_depth import (                # noqa: E402
    SurfaceRefusal, depth_to_metres, measure_surface)

W, H = 320, 240
FX = FY = 300.0
PPX, PPY = W / 2.0, H / 2.0
INTR = (FX, FY, PPX, PPY)

# THE CAMERA LOOKS STRAIGHT DOWN from 1.60 m, and the transform is written out
# rather than composed from a library so the test cannot inherit a library's
# convention bug. Optical frame is z forward, x right, y down. Looking down:
#   world x =  cam x
#   world y = -cam y      (cam y points "down" the image, which is -world y
#                          for a camera whose image top is further away)
#   world z =  CAM_H - cam z
CAM_H = 1.60
T_DOWN = [[1.0, 0.0, 0.0, 0.0],
          [0.0, -1.0, 0.0, 0.30],
          [0.0, 0.0, -1.0, CAM_H],
          [0.0, 0.0, 0.0, 1.0]]

# The footprint measured over, in world metres. Wide enough to hold the whole
# image at this height and narrow enough to exclude nothing that matters.
REGION = ((-0.90, 0.90), (-0.20, 0.80))


def _plane(z_world, objects=(), noise=None):
    """A depth frame of a horizontal plane at `z_world`, in 16UC1 mm.

    `objects` are (x0, x1, y0, y1, top_z) boxes in WORLD coordinates that
    stand on the plane and occlude it.
    """
    img = []
    for v in range(H):
        row = []
        for u in range(W):
            # Straight-down camera: the ray through (u, v) hits height z at
            # cam-z = CAM_H - z, and the world x/y follow from deprojection.
            zc = CAM_H - z_world
            xw = (u - PPX) * zc / FX
            yw = 0.30 - (v - PPY) * zc / FY
            top = z_world
            for (x0, x1, y0, y1, tz) in objects:
                if x0 <= xw <= x1 and y0 <= yw <= y1:
                    top = max(top, tz)
            d = CAM_H - top
            if noise is not None:
                d += noise(u, v)
            row.append(int(round(d * 1000.0)))
        img.append(row)
    return img


def test_units_come_from_the_encoding_not_the_magnitude():
    assert depth_to_metres(1200, "16UC1") == 1.2
    assert depth_to_metres(1.2, "32FC1") == 1.2
    # 50 mm reads as 0.05 m, where the "z > 100 means millimetres" heuristic
    # used elsewhere in this package would call it 50 metres.
    assert depth_to_metres(50, "16UC1") == 0.05
    with pytest.raises(SurfaceRefusal):
        depth_to_metres(1200, "bgr8")


def test_flat_surface_at_the_declared_height():
    est = measure_surface(_plane(0.950), INTR, T_DOWN, REGION)
    assert abs(est.z_m - 0.950) < 0.002, est.as_dict()
    assert est.share > 0.9
    assert est.spread_m < 0.002


def test_objects_standing_on_it_do_not_drag_the_answer_up():
    """THE TEST THE MEAN FAILS.

    Four 40 mm cubes and two mats over roughly a fifth of the region. A mean
    of the visible points sits several millimetres high and moves whenever an
    object moves; the mode does not move at all.
    """
    objs = [(-0.50, -0.30, 0.15, 0.35, 0.990),      # a row of cubes
            (-0.20, 0.20, 0.15, 0.35, 0.990),
            (0.30, 0.55, 0.10, 0.40, 0.954)]        # a thin mat
    est = measure_surface(_plane(0.950, objs), INTR, T_DOWN, REGION)
    assert abs(est.z_m - 0.950) < 0.002, est.as_dict()
    assert est.n_inliers < est.n_points        # the objects were excluded


def test_it_resolves_the_error_the_tolerance_exists_for():
    """work_surface.TOL_M is 10 mm and the fault table injects +/-20 mm.

    An estimator that cannot separate 0.950 from 0.970 makes that whole
    check decorative.
    """
    lo = measure_surface(_plane(0.930), INTR, T_DOWN, REGION)
    hi = measure_surface(_plane(0.970), INTR, T_DOWN, REGION)
    assert abs(lo.z_m - 0.930) < 0.002
    assert abs(hi.z_m - 0.970) < 0.002
    assert hi.z_m - lo.z_m > 0.035


def test_a_surface_on_a_bin_edge_is_not_a_bin_off():
    """2 mm bins, a surface at exactly a boundary.

    Without the inlier refinement the answer sits at the bin centre, up to
    1 mm out, and it does so systematically rather than randomly.
    """
    for z in (0.950, 0.9500 + 0.001, 0.952, 0.9530):
        est = measure_surface(_plane(z), INTR, T_DOWN, REGION)
        assert abs(est.z_m - z) < 0.0015, (z, est.as_dict())


def test_noise_does_not_move_it():
    """+/- 3 mm of sensor noise, deterministic so the test cannot flake."""
    def n(u, v):
        return 0.003 * math.sin(u * 0.7 + v * 1.3)
    est = measure_surface(_plane(0.950, noise=n), INTR, T_DOWN, REGION)
    assert abs(est.z_m - 0.950) < 0.002, est.as_dict()


# ---- THE REFUSALS. A check that cannot fail is not a check. --------------
def test_empty_frame_refuses():
    with pytest.raises(SurfaceRefusal):
        measure_surface([[0] * W for _ in range(H)], INTR, T_DOWN, REGION)


def test_camera_pointed_away_refuses_rather_than_answering():
    """The region is somewhere the camera cannot see.

    This is the scan-pose failure: a correctly working camera at the home
    pose returns an empty scene, and an empty scene must not be reported as
    a measurement.
    """
    with pytest.raises(SurfaceRefusal) as e:
        measure_surface(_plane(0.950), INTR, T_DOWN,
                        ((5.0, 6.0), (5.0, 6.0)))
    assert "scan pose" in str(e.value)


def test_clutter_with_no_dominant_plane_refuses():
    """A staircase: every height equally represented, no surface.

    The estimator must not pick the tallest bin of a flat histogram and call
    it a table.
    """
    img = []
    for v in range(H):
        row = []
        for u in range(W):
            z = 0.80 + (u % 64) * 0.006      # 64 distinct heights, even
            row.append(int(round((CAM_H - z) * 1000.0)))
        img.append(row)
    with pytest.raises(SurfaceRefusal) as e:
        measure_surface(img, INTR, T_DOWN, REGION)
    assert "dominant" in str(e.value)
