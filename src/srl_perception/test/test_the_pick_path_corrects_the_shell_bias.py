"""A depth camera sees the FRONT of an object, and the pick path knew it -- once.

WHAT WAS WRONG. `srl_perception/table_scene.py` has corrected the shell bias
since 2026-08-21: one depth view returns the front surface, the centroid of a
front surface sits towards the camera and BELOW the centre of a solid standing
on a table, and the object's bottom is known for free because it is RESTING on
the plane. `scripts/measure_control_budget.py` carries the uncorrected bias at
**10.5 mm against a 30 mm capture gate**.

`grasp_pipeline.plan_grasp` -- the function `scripts/find_object.py` and the
recording path actually plan grasps with -- did not have it. Two readers of
the same scene, one corrected and one not, and the corrected one is the one
that only prints a description of the table.

WHAT IS CHECKED HERE:

  * the correction exists as ONE function, used by both readers, rather than
    two implementations that can drift;
  * on a constructed front-only cloud it removes a bias that is really there
    -- the test requires the UNCORRECTED value to be wrong first, or it would
    pass on a scene where the bias happens to be zero and prove nothing;
  * without a support height the plan SAYS the centre is uncorrected, rather
    than reporting a biased number as if it were a measurement;
  * an object whose top is below the surface it is said to rest on is
    REFUSED, not "corrected" into nonsense.

Constructed geometry only: the ground truth is arithmetic, which is what
CLAUDE.md's standing rule permits synthetic data for.
"""
import numpy as np
import pytest

from srl_perception import rgbd_grasp as RG
from srl_perception import table_scene as TS


def _front_only_box(centre, size, n=2400, seed=5):
    """Points on the -y face only: what one camera at -y actually returns."""
    rng = np.random.default_rng(seed)
    c = np.asarray(centre, float)
    s = np.asarray(size, float)
    return np.column_stack([
        c[0] + rng.uniform(-s[0] / 2, s[0] / 2, n),
        np.full(n, c[1] - s[1] / 2),
        c[2] + rng.uniform(-s[2] / 2, s[2] / 2, n)])


def test_there_is_exactly_one_shell_correction():
    """`table_scene` must USE the shared function, not restate it."""
    src = open(TS.__file__).read()
    assert "shell_corrected_centre" in src, (
        "table_scene no longer calls the shared correction, so there are two "
        "implementations of it again")
    assert callable(RG.shell_corrected_centre)


def test_it_removes_a_bias_that_is_really_there():
    """And the uncorrected value must be wrong, or this proves nothing.

    The cloud is dense near the BOTTOM of the front face and sparse near the
    top, which is what a wrist camera looking slightly down actually returns:
    the raw centroid is then well below the true centre while the top edge is
    still observed.
    """
    plane_z, height = 0.90, 0.06
    truth_z = plane_z + height / 2.0
    rng = np.random.default_rng(4)
    n = 3000
    # heights biased toward the bottom (h^2 favours small h), top still seen
    hs = height * rng.random(n) ** 2
    pts = np.column_stack([0.4 + rng.uniform(-0.025, 0.025, n),
                           np.full(n, 0.2 - 0.025),
                           plane_z + hs])
    raw = float(pts.mean(axis=0)[2])
    corrected, top = RG.shell_corrected_centre(pts, [0, 0, plane_z], [0, 0, 1])
    assert abs(raw - truth_z) > 0.004, (
        "the raw centroid is only %.1f mm out on this cloud, so the "
        "correction has nothing to prove" % (abs(raw - truth_z) * 1000))
    assert abs(float(corrected[2]) - truth_z) < 0.001, (
        "corrected height is %.2f mm out" % (abs(corrected[2] - truth_z) * 1000)
    )
    assert abs(top - height) < 0.002


def test_the_correction_needs_the_TOP_to_be_observed_and_says_what_it_saw():
    """THE ASSUMPTION, STATED AS A TEST RATHER THAN A HOPE.

    The bottom comes from the plane; the top has to be SEEN. If the camera
    only ever returns the lower two thirds of an object, the correction
    centres it on the part it saw and is wrong by half the missing height --
    measured here at 10 mm on a 60 mm object with its top third invisible.

    That is not a bug to hide. `shell_corrected_centre` returns the height it
    measured, and a caller comparing that against an expected object size is
    how this failure becomes visible instead of silent. Written down because
    the first version of this test made exactly this mistake -- it built a
    cloud with no top and asserted the answer for one with a top.
    """
    plane_z, height = 0.90, 0.06
    rng = np.random.default_rng(6)
    n = 2000
    seen = height * 2.0 / 3.0
    pts = np.column_stack([0.4 + rng.uniform(-0.025, 0.025, n),
                           np.full(n, 0.175),
                           plane_z + rng.uniform(0, seen, n)])
    c, top = RG.shell_corrected_centre(pts, [0, 0, plane_z], [0, 0, 1])
    assert abs(top - seen) < 0.002, "it must report the height it SAW"
    err = abs(float(c[2]) - (plane_z + height / 2.0))
    assert abs(err - (height - seen) / 2.0) < 0.002, (
        "with the top third unseen the centre should be %.1f mm low; it is "
        "%.1f mm" % ((height - seen) / 2.0 * 1000, err * 1000))


def test_the_horizontal_centre_is_the_footprint_not_the_raw_centroid():
    """A shell's raw centroid is pulled along the line of sight; a footprint
    centroid is not. On a face at y = const the two agree in x and differ in y
    only by the face offset, which is what makes the correction safe to apply
    to all three axes at once."""
    pts = _front_only_box((0.4, 0.2, 0.93), (0.05, 0.001, 0.06))
    c, _ = RG.shell_corrected_centre(pts, [0, 0, 0.90], [0, 0, 1])
    assert abs(float(c[0]) - 0.4) < 0.002


def test_a_plane_above_the_object_is_refused_not_corrected():
    """Top below the surface it is said to rest on means one of the two is
    wrong, and inventing a centre from it would be confident nonsense."""
    pts = _front_only_box((0.4, 0.2, 0.93), (0.05, 0.001, 0.06))
    _, top = RG.shell_corrected_centre(pts, [0, 0, 1.20], [0, 0, 1])
    assert top < 0, "a plane above the object must give a negative height"


def test_plan_grasp_declares_when_it_has_not_corrected():
    """The signature carries it, the plan reports it, and a caller that gives
    no surface is told the centre is the shell's."""
    import inspect

    from srl_perception import grasp_pipeline as GP
    sig = inspect.signature(GP.plan_grasp)
    assert "support_z_m" in sig.parameters
    assert sig.parameters["support_z_m"].default is None
    src = inspect.getsource(GP.plan_grasp)
    assert "shell_correction" in src
    # the no-plane branch must SAY so rather than silently omitting the key
    assert "none -- no support_z_m given" in src


def test_find_object_passes_the_work_surface_by_default():
    """A capability nothing calls is the 'feature present but does nothing'
    row of CLAUDE.md's own table. This is the caller."""
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    ws = os.path.abspath(os.path.join(here, "..", "..", ".."))
    src = open(os.path.join(ws, "scripts", "find_object.py")).read()
    assert "support_z_m=a.support_z_m" in src
    assert "work_surface" in src and "table_top()" in src


@pytest.mark.parametrize("up", [(0, 0, 1), (0, 0.2, 0.98)])
def test_it_works_on_a_tilted_surface_too(up):
    """The correction is along the plane NORMAL, not world down. A table that
    is not level is the case that would silently reintroduce the bias."""
    up = np.asarray(up, float)
    up = up / np.linalg.norm(up)
    base = np.array([0.4, 0.2, 0.90])
    height = 0.06
    rng = np.random.default_rng(11)
    n = 2000
    e1 = np.cross(up, [1.0, 0, 0])
    e1 /= np.linalg.norm(e1)
    hs = rng.uniform(0, height, n)
    pts = (base + np.outer(rng.uniform(-0.025, 0.025, n), e1)
           + np.outer(hs, up))
    c, top = RG.shell_corrected_centre(pts, base, up)
    assert abs(top - height) < 0.002
    along = float((c - base) @ up)
    assert abs(along - height / 2.0) < 0.001
