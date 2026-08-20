#!/usr/bin/env python3
"""wearer_posture.py — WHERE THE WEARER'S OWN ARMS ARE, and one place that says it.

WHY THIS EXISTS. Every workspace number this project has published was measured
against a wearer whose arms hang rigidly at their sides, because that is the
only posture `human_backpack.xacro` could draw. The inboard limit was then
reported as "the wearer's own forearm", which is true of the model and is a
MODELLING CHOICE rather than a fact about people. A person standing in a rig
while two robot arms work in front of their chest does not stand like a
mannequin; they get their arms out of the way, and asking them to is a normal
operating instruction.

So the posture is now a variable, and this module is its single source. The
xacro's HUMAN ARMS block is GENERATED from the table below by
`scripts/gen_wearer_posture_xacro.py`, and `test_wearer_posture_has_one_source`
regenerates it and fails if the file and this table have drifted. That matters
more here than in most places: the geometric clearance check
(`mount_guard_node`) and the collision model MoveIt plans against are two
different pieces of code reading two different files, and a posture applied to
one and not the other is a measurement that quietly reports the old answer.

    SRL_WEARER_ARMS=folded ros2 launch srl_moveit_config demo.launch.py

Postures, and each is a thing a person can actually hold:

    down    arms hanging at the sides. The shipped model, and the default.
    behind  clasped behind the back, hands crossed at the small of the back.
    folded  folded across the chest. Included because it is what people
            actually do when they are told to keep their arms out of the way,
            and it is the WORST posture for this rig -- the forearms end up in
            the work volume. A posture sweep that only tries helpful postures
            has no control.
    out     held out to the sides and back, elbows out, hands low and behind.
    none    no arms at all. Not a posture: the LIMIT. It answers "what binds
            when the wearer's arms cannot possibly bind", which is the
            question the torso-or-mount answer needs.

FRAME. Everything here is in the `torso` LINK frame, whose origin sits at world
z = 1.05 (`human_backpack.xacro` says so in a comment and the test checks it).
World = local + (0, 0, TORSO_Z). The shoulder is at local z = 0.38, which is
world 1.43, the shoulder line the chest box was built to.

The chain is shoulder -> upper arm -> forearm -> hand, each segment a unit
direction from the previous joint. Segment lengths and radii are unchanged from
the shipped model, so `down` reproduces the shipped geometry exactly; that is
the regression control and the generator asserts it.
"""
import math
import os

# The torso LINK frame in world. The chest BOX is offset +0.17 from it.
TORSO_Z = 1.05

# Shoulder, LEFT arm, in the torso link frame. Right is the x mirror.
SHOULDER = (0.21, 0.0, 0.38)

# name suffix, primitive, dimensions, length along the chain, material
#
# THE SHIPPED MANNEQUIN. These are the numbers every clearance figure in this
# project was measured against, and they are one body: a 300 mm upper arm, a
# 260 mm forearm, a 360 mm chest.
SEGMENTS = (
    ('upper_arm', 'cylinder', (0.050, 0.30), 0.30, 'shirt'),
    ('lower_arm', 'cylinder', (0.045, 0.26), 0.26, 'skin'),
    ('hand',      'box',      (0.09, 0.05, 0.18), 0.18, 'skin'),
)

# ---------------------------------------------------------------- SIZE
# THE WEARER'S SIZE IS A VARIABLE NOW, AND IT HAS THE SAME ONE SOURCE THE
# POSTURE HAS.
#
# Posture was made a variable on 2026-08-15 and size was not, so every figure
# since then has described a person 300 mm in the upper arm and 360 mm across
# the chest. Measured from a photograph through the scene camera path on
# 2026-08-20, an ordinary adult reads 234 mm and 330 mm -- 66 mm and 30 mm
# from the model, in the direction that makes the mannequin too big in one
# place and, for a broader person, too small in another.
#
# A size reaches BOTH the URDF block and the clearance floor, through the same
# generator the posture does, because a size that reached one and not the
# other would measure the old body under a new name -- which is the exact trap
# `test_wearer_posture_has_one_source.py` was written for.
SIZE_ENV = 'SRL_WEARER_SIZE'
SIZE_DIR = 'config/wearer_sizes'

# Everything a body's SIZE consists of, and the shipped values. A profile
# gives some or all of these; anything it omits keeps the shipped number, so a
# profile that only knows the arm lengths is a legal profile.
SHIPPED_SIZE = dict(
    upper_arm_len=0.30, upper_arm_rad=0.050,
    lower_arm_len=0.26, lower_arm_rad=0.045,
    hand_len=0.18, hand_w=0.09, hand_d=0.05,
    chest_w=0.36, chest_d=0.22, chest_h=0.48,
    shoulder_x=0.21, shoulder_z=0.38,
    head_r=0.105, hips_w=0.32, hips_d=0.21,
)


def size_profile(name=None, env=None):
    """The wearer's measurements. `mannequin` is the shipped body exactly.

    A name is a file in config/wearer_sizes/<name>.json, or a path. Unknown
    names are an ERROR: silently falling back to the mannequin on a typo is
    how a session measures the wrong person and files it under the right one.
    """
    import json
    env = os.environ if env is None else env
    name = (name or env.get(SIZE_ENV, 'mannequin')).strip() or 'mannequin'
    out = dict(SHIPPED_SIZE)
    if name == 'mannequin':
        out['profile'] = 'mannequin'
        return out
    path = name if os.path.isabs(name) or name.endswith('.json') else \
        os.path.join(_ws(), SIZE_DIR, name + '.json')
    if not os.path.exists(path):
        raise ValueError(
            '%s=%r: no such size profile (%s). Known: %s'
            % (SIZE_ENV, name, path, ', '.join(known_sizes())))
    with open(path) as fh:
        d = json.load(fh)
    unknown = set(d) - set(SHIPPED_SIZE) - {'profile', 'note', 'measured_by',
                                            'date'}
    if unknown:
        raise ValueError('%s: unknown field(s) %s' % (path,
                                                      ', '.join(sorted(unknown))))
    out.update({k: v for k, v in d.items() if k in SHIPPED_SIZE})
    out['profile'] = d.get('profile', name)
    return out


def known_sizes():
    d = os.path.join(_ws(), SIZE_DIR)
    if not os.path.isdir(d):
        return ['mannequin']
    return sorted({'mannequin'} | {
        f[:-5] for f in os.listdir(d) if f.endswith('.json')})


def _ws():
    return os.environ.get('SRL_WS') or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')


def segments_for(size=None):
    """SEGMENTS, resized. Identical to SEGMENTS for the mannequin."""
    s = size if isinstance(size, dict) else size_profile(size)
    return (
        ('upper_arm', 'cylinder', (s['upper_arm_rad'], s['upper_arm_len']),
         s['upper_arm_len'], 'shirt'),
        ('lower_arm', 'cylinder', (s['lower_arm_rad'], s['lower_arm_len']),
         s['lower_arm_len'], 'skin'),
        ('hand', 'box', (s['hand_w'], s['hand_d'], s['hand_len']),
         s['hand_len'], 'skin'),
    )

# Unit-ish direction of each segment for the LEFT arm, from its proximal joint.
# They are normalised on use, so these are readable numbers rather than exact
# ones. +x is the wearer's left, +y is forward, +z is up.
POSTURES = {
    'down': dict(
        doc='arms hanging at the sides (the shipped model)',
        dirs=[(0.0, 0.0, -1.0), (0.0, 0.0, -1.0), (0.0, 0.0, -1.0)],
    ),
    'behind': dict(
        doc='clasped behind the back, hands crossed at the small of the back',
        dirs=[(-0.09, -0.42, -0.90), (-0.94, -0.10, -0.32), (-0.60, -0.20, -0.77)],
    ),
    'folded': dict(
        doc='folded across the chest (the forearms are IN the work volume)',
        dirs=[(0.05, 0.18, -0.98), (-0.95, 0.28, 0.14), (-0.93, 0.15, 0.34)],
    ),
    'out': dict(
        doc='held out to the sides and back, elbows out, hands low and behind',
        dirs=[(0.65, -0.35, -0.68), (0.72, -0.50, -0.48), (0.70, -0.52, -0.49)],
    ),
    'none': dict(
        doc='no wearer arms at all -- the limit, not a posture',
        dirs=None,
    ),
}

DEFAULT = 'down'
ENV_VAR = 'SRL_WEARER_ARMS'

# The wearer's body, minus the arms. Unchanged, and posture cannot move it.
# (name, primitive, dimensions, centre in WORLD, rpy)
TORSO_PARTS = [
    ('torso',   'box',      (0.36, 0.22, 0.48), (0.0, 0.0, 1.22),   (0.0, 0.0, 0.0)),
    ('head',    'sphere',   (0.105,),           (0.0, 0.0, 1.645),  (0.0, 0.0, 0.0)),
    ('neck',    'cylinder', (0.055, 0.16),      (0.0, 0.0, 1.50),   (0.0, 0.0, 0.0)),
    ('hips',    'box',      (0.32, 0.21, 0.18), (0.0, 0.0, 0.94),   (0.0, 0.0, 0.0)),
    ('L-thigh', 'cylinder', (0.075, 0.44),      (0.09, 0.0, 0.68),  (0.0, 0.0, 0.0)),
    ('R-thigh', 'cylinder', (0.075, 0.44),      (-0.09, 0.0, 0.68), (0.0, 0.0, 0.0)),
]


def posture_from_env(env=None):
    """Which posture the stack was launched with. Unknown names are an ERROR.

    Silently falling back to `down` on a typo is exactly how a posture sweep
    reports four identical answers and calls it a finding.
    """
    env = os.environ if env is None else env
    name = env.get(ENV_VAR, DEFAULT).strip() or DEFAULT
    if name not in POSTURES:
        raise ValueError(
            '%s=%r is not a posture. Known: %s'
            % (ENV_VAR, name, ', '.join(sorted(POSTURES))))
    return name


def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-9:
        raise ValueError('zero-length segment direction')
    return tuple(c / n for c in v)


def rpy_for(d):
    """Fixed-axis rpy taking +z onto direction `d`, with yaw held at 0.

    URDF composes R = Rz(yaw) Ry(pitch) Rx(roll), so with yaw = 0,
    R z = (sin p cos r, -sin r, cos p cos r). Invert that.
    """
    dx, dy, dz = _unit(d)
    roll = math.asin(max(-1.0, min(1.0, -dy)))
    c = math.cos(roll)
    if abs(c) < 1e-9:                       # the arm points straight along y
        return (roll, 0.0, 0.0)
    return (roll, math.atan2(dx / c, dz / c), 0.0)


def rot_matrix(rpy):
    """R = Rz(y) Ry(p) Rx(r), the URDF convention, as a 3x3 tuple of rows."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,                cp * cr),
    )


def arm_links(posture, side, frame='local', size=None):
    """The three links of one wearer arm.

    Returns (link_suffix, primitive, dimensions, centre, rpy). `side` is
    'left' or 'right'; the right arm is the left one mirrored in x. `frame` is
    'local' (the torso link, which is what the xacro wants) or 'world' (which
    is what the clearance check wants). `size` is a per-participant profile;
    None means SRL_WEARER_SIZE, whose default is the shipped mannequin.
    """
    spec = POSTURES[posture]
    if spec['dirs'] is None:
        return []
    prof = size if isinstance(size, dict) else size_profile(size)
    segs = segments_for(prof)
    sgn = 1.0 if side == 'left' else -1.0
    joint = [sgn * prof['shoulder_x'], SHOULDER[1], prof['shoulder_z']]
    dz = TORSO_Z if frame == 'world' else 0.0
    out = []
    for (name, kind, dims, length, _mat), raw in zip(segs, spec['dirs']):
        d = _unit((sgn * raw[0], raw[1], raw[2]))
        centre = tuple(joint[k] + 0.5 * length * d[k] for k in range(3))
        joint = [joint[k] + length * d[k] for k in range(3)]
        out.append((name, kind, dims,
                    (centre[0], centre[1], centre[2] + dz), rpy_for(d)))
    return out


def torso_parts_for(size=None):
    """TORSO_PARTS, resized. Identical to TORSO_PARTS for the mannequin."""
    s = size if isinstance(size, dict) else size_profile(size)
    return [
        ('torso', 'box', (s['chest_w'], s['chest_d'], s['chest_h']),
         (0.0, 0.0, 1.22), (0.0, 0.0, 0.0)),
        ('head', 'sphere', (s['head_r'],), (0.0, 0.0, 1.645), (0.0, 0.0, 0.0)),
        ('neck', 'cylinder', (0.055, 0.16), (0.0, 0.0, 1.50), (0.0, 0.0, 0.0)),
        ('hips', 'box', (s['hips_w'], s['hips_d'], 0.18), (0.0, 0.0, 0.94),
         (0.0, 0.0, 0.0)),
        ('L-thigh', 'cylinder', (0.075, 0.44), (0.09, 0.0, 0.68),
         (0.0, 0.0, 0.0)),
        ('R-thigh', 'cylinder', (0.075, 0.44), (-0.09, 0.0, 0.68),
         (0.0, 0.0, 0.0)),
    ]


def wearer_model(posture=None, size=None):
    """The whole wearer as world-frame primitives: (name, kind, dims, ctr, rpy).

    This is what the geometric clearance check measures against. It is NOT the
    SRDF and it is not `avoid_collisions`; both of those are blind to the pairs
    a shoulder mount threatens.

    `size` is a per-participant profile: a name in config/wearer_sizes, a path,
    or a dict. Omitted, it is whatever SRL_WEARER_SIZE names, whose default is
    the shipped mannequin -- so every existing caller keeps the body it has
    always had, byte for byte.
    """
    posture = posture_from_env() if posture is None else posture
    if posture not in POSTURES:
        raise ValueError('unknown posture %r' % (posture,))
    prof = size if isinstance(size, dict) else size_profile(size)
    parts = list(torso_parts_for(prof))
    for side, tag in (('left', 'L'), ('right', 'R')):
        for name, kind, dims, ctr, rpy in arm_links(posture, side, 'world',
                                                    prof):
            label = {'upper_arm': 'upperarm', 'lower_arm': 'forearm',
                     'hand': 'hand'}[name]
            parts.append(('%s-%s' % (tag, label), kind, dims, ctr, rpy))
    return parts
