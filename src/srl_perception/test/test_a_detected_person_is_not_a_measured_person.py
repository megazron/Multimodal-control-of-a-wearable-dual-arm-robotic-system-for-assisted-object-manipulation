"""A pose detector reports elbows on a body that has none, at high confidence.

REPORTED FROM THE RIG: "I don't feel like the cameras are properly detecting
the arms and the mannequin." They were not, and the numbers say so exactly.

The wearer the cameras watch is a TORSO MANNEQUIN WITH NO ARMS. MediaPipe
fits a whole-body prior, so it returns elbows and wrists anyway -- at
visibility 0.95 to 0.98, which is above every confidence gate in the tracker.
Measured on the recorded scene frames of 2026-08-30 against a real person
photographed in the same laboratory:

    forearm / upper arm      real person   0.93, 0.92
                             mannequin     0.57, 0.36  (HD camera)
                             mannequin     0.47, 0.11  (depth camera)

and the left and right landmarks of one body sit 1.88 m apart in depth, on a
body about 0.4 m wide -- half of them are on the back wall.

THE SAFETY CASE HOLDS, and this test is what says so. `plausible()` rejects
every one of those limbs on ABSOLUTE length, so `fuse()` falls back to the
mannequin and the clearance floor is never driven by the invention. What was
wrong was the REPORTING: the figure drew the skeleton, printed "1 person(s)",
and said nothing about the fact that the safety path would throw the arms
away.

These are the recorded numbers, so this test fails if the bounds are ever
widened to the point where the hallucination would be accepted.
"""
import math
import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src", "srl_perception"))

from srl_perception.wearer_tracking import PLAUSIBLE, plausible  # noqa: E402

#: The scene depth camera's intrinsics, as the device reported them.
FX, FY, CX, CY = 603.0215, 603.1288, 318.8748, 231.2179

#: Landmarks as RECORDED on 2026-08-30, with the depth read at each pixel:
#: (u, v, range_m, visibility).
MANNEQUIN = {
    "l_shoulder": (364, 170, 4.632, 1.000),
    "l_elbow":    (382, 185, 4.5753, 0.913),
    "l_wrist":    (375, 176, 4.6179, 0.855),
    "r_elbow":    (318, 189, 2.756, 0.767),
    "r_wrist":    (321, 189, 2.781, 0.583),
}


def _p(u, v, z):
    return ((u - CX) / FX * z, (v - CY) / FY * z, z)


def _seg(a, b):
    return math.dist(_p(*MANNEQUIN[a][:3]), _p(*MANNEQUIN[b][:3]))


@pytest.mark.parametrize("name,a,b,kind", [
    ("left upper arm", "l_shoulder", "l_elbow", "upperarm"),
    ("left forearm", "l_elbow", "l_wrist", "forearm"),
    ("right forearm", "r_elbow", "r_wrist", "forearm"),
])
def test_the_hallucinated_limbs_are_refused_on_length(name, a, b, kind):
    """The gate that keeps the safety case standing."""
    length = _seg(a, b)
    lo, hi = PLAUSIBLE[kind]
    assert not plausible(kind, length), (
        "%s measures %.3f m and the %s range is %.2f-%.2f m, so it is now "
        "ACCEPTED. On the recorded frames this limb does not exist -- the "
        "mannequin has no arms -- and accepting it puts an invented body "
        "part into the clearance model." % (name, length, kind, lo, hi))


def test_confidence_alone_would_have_accepted_every_one_of_them():
    """Why a confidence gate is not enough, stated as a test.

    Visibility is the detector's belief that the landmark is in frame and
    unoccluded. It is not a belief that the landmark corresponds to a joint
    that exists, and on this frame it is high for joints that do not.
    """
    worst = min(v[3] for v in MANNEQUIN.values())
    assert worst > 0.5, (
        "the recorded visibilities have changed; the point of this test is "
        "that they were all comfortably above any confidence threshold")


def test_the_two_sides_of_one_body_are_metres_apart_in_depth():
    """Half the landmarks are not on the body at all."""
    gap = abs(MANNEQUIN["l_shoulder"][2] - MANNEQUIN["r_elbow"][2])
    assert gap > 1.0, "%.2f m" % gap
    # A human body is not two metres deep, so this is not a pose, it is two
    # different surfaces. Recorded for the record rather than gated on,
    # because the length check above already refuses the limbs.


def test_a_real_person_passes_the_same_gate():
    """THE CONTROL. A gate that refuses everything is not a gate.

    Measured on `recordings/scene_camera/one_person.jpg`: forearm over upper
    arm 0.93 and 0.92, against 0.11-0.57 on the mannequin. Reconstructed here
    at a plausible working range so the ABSOLUTE check is exercised too.
    """
    for kind, length in (("upperarm", 0.31), ("forearm", 0.28),
                         ("shoulder_width", 0.40), ("torso_height", 0.52)):
        assert plausible(kind, length), (
            "%.2f m is a normal adult %s and the gate refuses it; the bounds "
            "have been tightened onto real people" % (length, kind))


def test_the_figure_reports_the_verdict_rather_than_only_the_detection():
    """The defect was in the reporting, so the reporting is pinned."""
    src = open(os.path.join(WS, "scripts", "cv_pickpose_visuals.py")).read()
    i = src.index('"32  PEOPLE IN THE PICTURE"')
    stage = src[max(0, i - 6000):i + 3000]
    assert "REFUSED by the safety path" in stage, \
        "the people figure no longer says which limbs the safety path refuses"
    assert "LINK_KIND" in stage, \
        "the per-link plausibility check has gone from the figure"
    assert "A DETECTED PERSON IS NOT A MEASURED PERSON" in stage, \
        "the note that separates detection from measurement has gone"
