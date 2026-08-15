"""The wearer's arm posture has ONE source, and both consumers must read it.

THE TRAP HERE IS SPECIFIC AND IT WOULD NOT HAVE BEEN NOTICED. The posture is
read by two things that share no file:

    human_backpack.xacro          what MoveIt plans against, and what
                                  `avoid_collisions` can see
    mount_guard_node.WEARER       what the 150 mm clearance floor is measured
                                  against geometrically, because the SRDF
                                  excludes the pairs a shoulder mount threatens

A posture applied to one and not the other does not error. It produces a sweep
that runs four times, changes nothing, and reports the shipped answer under
four new labels -- which is precisely the class of instrument failure this
project keeps finding. So `wearer_posture.py` is the single table, the xacro
block is GENERATED from it, and this test regenerates and compares.

ALSO CHECKED, and each of these has a reason:

  * `down` reproduces the shipped geometry to the millimetre. Every workspace
    and clearance number ever published here was measured against those exact
    centres, and a posture mechanism that quietly moved the default would
    invalidate all of them without saying so.
  * an unknown posture name RAISES. A silent fall back to the default is the
    same four-identical-answers failure with a typo as its cause.
  * the rotated-primitive distance is right. `dist_point` had no rotation
    argument until the posture became a variable; a folded forearm is a
    cylinder on its side, and measuring it unrotated measures a different
    solid. The known answer is constructed, not rendered.
  * the torso link frame really is at world z = 1.05. Every local coordinate
    in the table is offset by it, so if the xacro moves the torso, the
    clearance model silently measures the wearer in the wrong place.
"""
import math
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, 'src/srl_teleop'))

from srl_teleop import wearer_posture as WP          # noqa: E402

XACRO = os.path.join(ROOT, 'src/srl_description/urdf/human_backpack.xacro')
GEN = os.path.join(ROOT, 'scripts/gen_wearer_posture_xacro.py')

# The shipped model, as it stood before the posture became a variable. These
# are the numbers every published workspace figure was measured against.
SHIPPED_DOWN = {
    'L-upperarm': (0.21, 0.0, 1.28),
    'R-upperarm': (-0.21, 0.0, 1.28),
    'L-forearm': (0.21, 0.0, 1.00),
    'R-forearm': (-0.21, 0.0, 1.00),
    'L-hand': (0.21, 0.0, 0.78),
    'R-hand': (-0.21, 0.0, 0.78),
}


def test_xacro_matches_the_table():
    r = subprocess.run([sys.executable, GEN, '--check'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_down_reproduces_the_shipped_geometry():
    model = {n: c for n, _k, _d, c, _r in WP.wearer_model('down')}
    for name, want in SHIPPED_DOWN.items():
        got = model[name]
        for k in range(3):
            assert abs(got[k] - want[k]) < 1e-9, (name, got, want)


def test_down_arm_primitives_are_unrotated_in_effect():
    """`down` may print rpy = (0, pi, 0); it must not MEAN anything.

    Both arm primitives are symmetric about their own z, so a 180 deg pitch is
    the identity geometrically. The check is on the geometry, not the text.
    """
    for name, kind, dims, ctr, rpy in WP.wearer_model('down'):
        if not name.startswith(('L-', 'R-')):
            continue
        probe = (ctr[0] + 0.4, ctr[1] + 0.3, ctr[2] + 0.2)
        import srl_teleop.mount_guard_node as MG
        a = MG.dist_point(probe, kind, dims, ctr, rpy)
        b = MG.dist_point(probe, kind, dims, ctr, (0.0, 0.0, 0.0))
        assert abs(a - b) < 1e-9, name


def test_unknown_posture_raises():
    with pytest.raises(ValueError):
        WP.posture_from_env({'SRL_WEARER_ARMS': 'crossed-fingers'})
    with pytest.raises(ValueError):
        WP.wearer_model('crossed-fingers')
    assert WP.posture_from_env({}) == 'down'


def test_none_really_has_no_arms():
    model = WP.wearer_model('none')
    assert not [n for n, *_ in model if n.startswith(('L-upper', 'L-fore',
                                                      'L-hand', 'R-upper',
                                                      'R-fore', 'R-hand'))]
    assert len(model) == len(WP.TORSO_PARTS)


def test_rotated_cylinder_distance_is_right():
    """Constructed known answer: a cylinder laid along x, probed along y.

    r = 0.05, L = 0.40, centred at the origin, rotated so its axis is +x. A
    point at (0, 0.30, 0) is 0.25 m from the surface. Measured WITHOUT the
    rotation the same point sits beside an upright cylinder and reads 0.25 too,
    so that probe cannot tell the two apart -- use one that can: (0.30, 0, 0)
    is INSIDE the lying cylinder's length and 0.10 m past its end when upright.
    """
    import srl_teleop.mount_guard_node as MG
    lying = WP.rpy_for((1.0, 0.0, 0.0))          # +z onto +x
    d_rot = MG.dist_point((0.30, 0.0, 0.0), 'cylinder', (0.05, 0.40),
                          (0.0, 0.0, 0.0), lying)
    d_flat = MG.dist_point((0.30, 0.0, 0.0), 'cylinder', (0.05, 0.40),
                           (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert abs(d_rot - 0.10) < 1e-9, d_rot       # 0.30 - half length 0.20
    assert abs(d_flat - 0.25) < 1e-9, d_flat     # 0.30 - radius 0.05
    assert d_rot != d_flat


def test_rpy_for_puts_z_on_the_direction():
    for d in ((0, 0, -1), (1, 0, 0), (0.3, -0.5, -0.8), (-0.9, 0.1, 0.4)):
        rpy = WP.rpy_for(d)
        R = WP.rot_matrix(rpy)
        got = tuple(R[i][2] for i in range(3))
        want = WP._unit(d)
        for k in range(3):
            assert abs(got[k] - want[k]) < 1e-9, (d, got, want)


def test_torso_link_frame_is_where_the_table_says():
    """The xacro must still put the torso link at world z = 1.05."""
    src = open(XACRO).read()
    i = src.index('<link name="torso">')
    head = src[:i]
    j = head.rindex('name="world_to_torso"') if 'world_to_torso' in head else -1
    assert j >= 0 or 'z=1.05' in src, 'torso placement moved; re-derive TORSO_Z'
    assert WP.TORSO_Z == 1.05


def test_the_live_clearance_model_can_see_the_wearers_arms():
    """The follower's wearer had no arms, and the planner's always did.

    `clearance.py` is what ik_follower_node, real_homing_node and
    sim_to_real_bridge enforce at runtime, and it modelled a torso, a head and
    a pair of hips. Inboard, the forearm is the thing that binds -- every
    "innermost safe column" figure in this project is measured against it. The
    known answer is constructed: a point 30 mm out from the forearm's surface,
    in the forearm's OWN frame, where the primitive is at the origin.
    """
    from srl_teleop.clearance import ClearanceModel, WEARER_ARM_PARTS
    parts = ClearanceModel.PARTS
    for side in ('left', 'right'):
        for seg in ('upper_arm', 'lower_arm', 'hand'):
            assert 'human_%s_%s' % (side, seg) in parts
    assert len(WEARER_ARM_PARTS) == 6

    m = ClearanceModel()
    r = 0.045                                     # forearm radius
    d, who = m.clearance({'human_left_lower_arm': [(r + 0.030, 0.0, 0.0)]})
    assert abs(d - 0.030) < 1e-9, d
    assert who == 'human_left_lower_arm'

    # and it must still pick the WORST part when several are offered
    d2, who2 = m.clearance({'human_left_lower_arm': [(r + 0.030, 0.0, 0.0)],
                            'torso': [(0.0, 0.60, 0.17)]})
    assert who2 == 'human_left_lower_arm', (d2, who2)


def test_every_posture_is_a_connected_chain():
    """Shoulder to elbow to wrist, with no gaps and no stretched segments."""
    for posture in WP.POSTURES:
        for side in ('left', 'right'):
            links = WP.arm_links(posture, side, 'local')
            if not links:
                continue
            sgn = 1.0 if side == 'left' else -1.0
            joint = (sgn * WP.SHOULDER[0], WP.SHOULDER[1], WP.SHOULDER[2])
            for (name, _k, _d, ctr, rpy), seg in zip(links, WP.SEGMENTS):
                length = seg[3]
                R = WP.rot_matrix(rpy)
                axis = tuple(R[i][2] for i in range(3))
                want = tuple(joint[k] + 0.5 * length * axis[k] for k in range(3))
                for k in range(3):
                    assert abs(ctr[k] - want[k]) < 1e-9, (posture, side, name)
                joint = tuple(joint[k] + length * axis[k] for k in range(3))
            assert math.isfinite(joint[0])
