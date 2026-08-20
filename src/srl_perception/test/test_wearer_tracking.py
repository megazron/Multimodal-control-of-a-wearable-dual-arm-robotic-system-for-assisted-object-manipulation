"""The tracked wearer must never be less conservative than the mannequin.

EVERY FAILURE IN THIS FILE IS INJECTED, NOT ARGUED. The brief is explicit:
"test each by injecting it, not by reasoning about it". So each gate is driven
with a body that is deliberately wrong in one specific way, and required to
refuse -- and, just as important, a HEALTHY body is driven through the same
path and required to be accepted, because a gate that rejects everything is
not a gate.

THE ONE TEST THAT MATTERS is `test_a_body_further_away_never_improves_clearance`.
If that passes, this feature cannot reduce safety however wrong the camera is;
if it could not be written, the feature should not be built. It is the
executable form of the rule the whole module is arranged around:

    THE CAMERA MAY ONLY EVER MAKE THE WEARER BIGGER, NEVER SMALLER.
"""
import pytest

from srl_perception import wearer_tracking as WT


# The mannequin, in the shape `wearer_posture.wearer_model()` produces. Built
# here rather than imported so this file can run with srl_teleop absent; the
# agreement between the two is asserted in
# test_tracked_wearer_matches_the_one_source below.
MANNEQUIN = [
    ("torso", "box", (0.36, 0.22, 0.48), (0.0, 0.0, 1.22), (0.0, 0.0, 0.0)),
    ("head", "sphere", (0.105,), (0.0, 0.0, 1.645), (0.0, 0.0, 0.0)),
    ("hips", "box", (0.32, 0.21, 0.18), (0.0, 0.0, 0.94), (0.0, 0.0, 0.0)),
    ("L-upperarm", "cylinder", (0.050, 0.30), (0.21, 0.0, 1.28), (0.0, 0.0, 0.0)),
    ("L-forearm", "cylinder", (0.045, 0.26), (0.21, 0.0, 1.00), (0.0, 0.0, 0.0)),
    ("L-hand", "box", (0.09, 0.05, 0.18), (0.21, 0.0, 0.78), (0.0, 0.0, 0.0)),
    ("R-upperarm", "cylinder", (0.050, 0.30), (-0.21, 0.0, 1.28), (0.0, 0.0, 0.0)),
    ("R-forearm", "cylinder", (0.045, 0.26), (-0.21, 0.0, 1.00), (0.0, 0.0, 0.0)),
    ("R-hand", "box", (0.09, 0.05, 0.18), (-0.21, 0.0, 0.78), (0.0, 0.0, 0.0)),
]

# THE ROBOT WORKS IN FRONT OF THE CHEST. The probe point is where the arms
# actually are -- the work plane is z = 1.100 and the near table edge is
# 0.430 forward -- so "closer to the robot" means closer to here.
PROBE = (0.0, 0.30, 1.20)


def healthy_joints(forward=0.0):
    """A plausible standing body, arms down. `forward` shifts it toward the
    robot, which is how a test asks for a body that is CLOSER."""
    y = forward
    return {
        "nose": (0.0, y, 1.65),
        "L-shoulder": (0.19, y, 1.43), "R-shoulder": (-0.19, y, 1.43),
        "L-elbow": (0.21, y, 1.15), "R-elbow": (-0.21, y, 1.15),
        "L-wrist": (0.22, y, 0.92), "R-wrist": (-0.22, y, 0.92),
        # THE KNUCKLE, 95 mm below the wrist -- which is what the detector
        # actually reports. The fixture said 170 mm at first, i.e. a whole
        # hand, and so agreed with a model that stopped at the knuckle.
        "L-hand-tip": (0.22, y, 0.825), "R-hand-tip": (-0.22, y, 0.825),
        "L-hip": (0.11, y, 1.00), "R-hip": (-0.11, y, 1.00),
    }


def all_visible(pts, v=0.95):
    return {k: v for k in pts}


def build(pts, vis=None, prev=None, dt=0.1, now=100.0):
    return WT.segments_from_joints(pts, vis or all_visible(pts),
                                   prev=prev, dt=dt, now=now, source="test")


# ===================================================================== 0
def test_a_healthy_body_is_actually_accepted():
    """The negative control for every rejection test below. Without this,
    a module that refused everything would pass the whole file."""
    est = build(healthy_joints())
    usable = est.usable_names(now=100.0)
    for part in WT.TRACKED_PARTS:
        assert part in usable, "%s rejected on a healthy body: %r" % (
            part, est.segments[part].reasons)


def test_a_healthy_body_that_is_closer_is_used():
    est = build(healthy_joints(forward=0.28))
    _, dec = WT.fuse(MANNEQUIN, est, now=100.0, probe=PROBE)
    used = [k for k, (w, _) in dec.items() if w == "tracked"]
    assert used, "a body standing 280 mm nearer the robot changed nothing"


# ===================================================================== 1
def test_low_confidence_reverts_only_that_segment():
    """The folded-arms case, which is measured and not invented: on real
    photographs the shoulders and hips read visibility 1.00 while the elbows
    read 0.09-0.11. The torso must survive; the arms must not."""
    pts = healthy_joints(forward=0.28)
    vis = all_visible(pts)
    vis["L-elbow"] = 0.10
    vis["R-elbow"] = 0.11
    est = build(pts, vis)
    _, dec = WT.fuse(MANNEQUIN, est, now=100.0, probe=PROBE)
    assert dec["L-upperarm"][0] == "mannequin"
    assert dec["L-forearm"][0] == "mannequin"
    assert dec["R-upperarm"][0] == "mannequin"
    # ... and the parts that WERE seen are still used. A gate that takes the
    # whole body down whenever one joint is unclear is a gate nobody can use.
    assert dec["torso"][0] == "tracked"


# ===================================================================== 2
def test_the_hand_is_modelled_past_the_knuckle():
    """The detector's hand landmark is the index KNUCKLE, so the measured
    segment is a palm and the fingers are past it.

    Found on the first real photograph: both hands were refused with
    "0.079 m is not a hand", which was true about the number and wrong about
    the question. A hand model that stops at the knuckle leaves the fingers
    outside the collision model, and the fingers are the part of a person a
    gripper reaches first.
    """
    pts = healthy_joints(forward=0.28)
    est = build(pts, all_visible(pts, 0.99))
    seg = est.segments["L-hand"]
    assert seg.usable, seg.reasons
    assert seg.dims[1] > 0.15, (
        "the hand is only %.3f m long -- the fingers are not in the model"
        % seg.dims[1])


def test_an_impossible_limb_length_is_refused_however_confident():
    """Confidence is the detector's opinion of its own output. Limb length is
    a fact about human beings, so it overrules the opinion."""
    pts = healthy_joints(forward=0.28)
    pts["L-elbow"] = (0.20, 0.28, 1.38)          # 50 mm "upper arm"
    est = build(pts, all_visible(pts, 0.99))
    seg = est.segments["L-upperarm"]
    assert not seg.usable and seg.conf >= 0.99
    assert any("not a upperarm" in r for r in seg.reasons), seg.reasons
    _, dec = WT.fuse(MANNEQUIN, est, now=100.0, probe=PROBE)
    assert dec["L-upperarm"][0] == "mannequin"


def test_a_shoulder_width_that_is_not_a_person_is_refused():
    pts = healthy_joints(forward=0.28)
    pts["L-shoulder"] = (0.60, 0.28, 1.43)       # 790 mm across
    pts["R-shoulder"] = (-0.19, 0.28, 1.43)
    est = build(pts, all_visible(pts, 0.99))
    assert not est.segments["torso"].usable


# ===================================================================== 3
def test_a_joint_that_moved_impossibly_fast_is_refused():
    prev = healthy_joints(forward=0.28)
    cur = dict(prev)
    cur["L-wrist"] = (0.22, 0.28 + 1.2, 0.92)    # 1.2 m in 0.1 s = 12 m/s
    est = build(cur, all_visible(cur, 0.99), prev=prev, dt=0.1)
    fore = est.segments["L-forearm"]
    assert not fore.usable
    assert any("moved" in r for r in fore.reasons), fore.reasons
    _, dec = WT.fuse(MANNEQUIN, est, now=100.0, probe=PROBE)
    assert dec["L-forearm"][0] == "mannequin"
    # The OTHER side is untouched: a speed gate that takes the whole body out
    # on one bad joint hands the robot a mannequin every time somebody waves.
    assert dec["R-upperarm"][0] == "tracked"


def test_a_fast_but_possible_movement_is_kept():
    """2 m/s is a brisk reach and must survive, or the gate is a filter on
    normal movement rather than on impossible movement."""
    prev = healthy_joints(forward=0.28)
    cur = dict(prev)
    cur["L-wrist"] = (0.22, 0.28 + 0.20, 0.92)   # 2.0 m/s
    est = build(cur, all_visible(cur, 0.99), prev=prev, dt=0.1)
    assert est.segments["L-forearm"].usable


# ===================================================================== 4
def test_a_stale_estimate_contributes_nothing():
    est = build(healthy_joints(forward=0.28), now=100.0)
    _, dec = WT.fuse(MANNEQUIN, est, now=100.0 + WT.MAX_AGE_S + 0.01,
                     probe=PROBE)
    assert all(w == "mannequin" for w, _ in dec.values())


def test_no_estimate_at_all_is_exactly_todays_model():
    fused, dec = WT.fuse(MANNEQUIN, None, now=100.0, probe=PROBE)
    assert fused == MANNEQUIN
    assert all(w == "mannequin" for w, _ in dec.values())


# ===================================================================== 5
# THE ONE THAT MATTERS
def test_a_body_further_away_never_improves_clearance():
    """Inject a body displaced AWAY from the robot and require that the model
    does not follow it.

    This is doc 16 section 6.1 made executable. Every other test here says
    "a wrong reading is rejected"; this one says "even a reading that is
    accepted cannot make the wearer smaller", which is the property that
    holds when the rejection tests are all somehow wrong.
    """
    for retreat in (0.10, 0.30, 1.00, 5.00):
        pts = healthy_joints(forward=-retreat)
        est = build(pts, all_visible(pts, 1.0))
        fused, dec = WT.fuse(MANNEQUIN, est, now=100.0, probe=PROBE)
        assert fused == MANNEQUIN, (
            "a body %.2f m FURTHER from the robot changed the model" % retreat)
        assert all(w == "mannequin" for w, _ in dec.values())


def test_a_body_closer_does_change_the_model():
    """The other half. Without it the previous test passes on a module that
    ignores the camera entirely."""
    pts = healthy_joints(forward=0.30)
    est = build(pts, all_visible(pts, 1.0))
    fused, dec = WT.fuse(MANNEQUIN, est, now=100.0, probe=PROBE)
    assert fused != MANNEQUIN
    assert any(w == "tracked" for w, _ in dec.values())


# ===================================================================== 6
@pytest.mark.parametrize("luma,span,n,ok", [
    (110.0, 90.0, 1, True),      # a normal room
    (4.0, 90.0, 1, False),       # lights off
    (110.0, 2.0, 1, False),      # lens cap, or a repeated buffer
    (110.0, 90.0, 2, False),     # somebody walked behind the wearer
    (110.0, 90.0, 0, False),     # nobody there
])
def test_a_frame_is_refused_before_a_body_is_read_out_of_it(luma, span, n, ok):
    got, reason = WT.frame_is_usable(luma, span, n)
    assert got is ok, reason
    if not ok:
        assert reason and "camera" in reason or "person" in reason or \
            "nobody" in reason, reason


def test_a_mid_grey_blank_is_caught_by_the_span_and_not_the_mean():
    """Mean luma alone passes a uniform grey frame, which is what a covered
    lens and a repeated buffer both look like."""
    assert WT.frame_is_usable(128.0, 90.0, 1)[0] is True
    assert WT.frame_is_usable(128.0, 1.0, 1)[0] is False


# ===================================================================== 7
def test_tracked_wearer_matches_the_one_source():
    """`wearer_posture.py` is the ONE source for the wearer, and this module
    restates its rpy convention. Restating is allowed; drifting is not.

    Skipped rather than failed where srl_teleop is not importable, because
    this file must run on its own.
    """
    wp = pytest.importorskip("srl_teleop.wearer_posture")
    for d in ((0, 0, -1), (1, 0, 0), (0.3, -0.5, 0.8), (0, 1, 0)):
        a = wp.rpy_for(wp._unit(d))
        b = WT.rpy_from_direction(d)
        assert all(abs(a[i] - b[i]) < 1e-9 for i in range(3)), (d, a, b)


def test_every_tracked_part_names_a_real_mannequin_part():
    """A tracked part whose name does not match keeps the mannequin FOR EVER
    and says nothing. Names are the whole interface here."""
    wp = pytest.importorskip("srl_teleop.wearer_posture")
    names = {n for n, _k, _d, _c, _r in wp.wearer_model("down")}
    for part in WT.TRACKED_PARTS:
        assert part in names, "%s is not a part of the wearer model" % part


def test_the_summary_never_claims_tracking_it_does_not_have():
    _, dec = WT.fuse(MANNEQUIN, None, now=100.0, probe=PROBE)
    n, total, text = WT.summarise(dec)
    assert n == 0 and "MANNEQUIN" in text
    est = build(healthy_joints(forward=0.30))
    _, dec = WT.fuse(MANNEQUIN, est, now=100.0, probe=PROBE)
    n, total, text = WT.summarise(dec)
    assert n > 0 and ("TRACKED" in text)
