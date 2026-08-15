#!/usr/bin/env python3
"""Generate the HUMAN ARMS block of human_backpack.xacro from wearer_posture.py.

    python3 scripts/gen_wearer_posture_xacro.py            # rewrite the block
    python3 scripts/gen_wearer_posture_xacro.py --check    # exit 1 if stale

WHY GENERATE IT. The wearer's arm posture is read by two pieces of code that
share no file: the URDF, which is what MoveIt plans against, and
`mount_guard_node.WEARER`, which is what the geometric clearance floor is
measured against. A posture applied to one and not the other produces a sweep
that reports the old answer with a new label, and nothing downstream disagrees.
So there is one table (`wearer_posture.POSTURES`) and the xacro is written from
it, with `test_wearer_posture_has_one_source` re-running this in --check mode.
"""
import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/srl_teleop'))

from srl_teleop import wearer_posture as WP          # noqa: E402

XACRO = os.path.join(ROOT, 'src/srl_description/urdf/human_backpack.xacro')
BEGIN = '    <!-- BEGIN GENERATED WEARER ARMS - edit wearer_posture.py, not this -->'
END = '    <!-- END GENERATED WEARER ARMS -->'


def _n(v):
    """Print a number the way a person would, not the way a float prints."""
    s = '%.6g' % (v + 0.0)
    return '0' if s in ('-0', '0') else s


def _xyz(t):
    return ' '.join(_n(v) for v in t)


def _comment(text):
    """XML forbids `--` inside a comment, and the posture docs are full of it."""
    return re.sub(r'-{2,}', '-', text)


def _geom(kind, dims):
    if kind == 'cylinder':
        return '<cylinder radius="%s" length="%s"/>' % (_n(dims[0]), _n(dims[1]))
    if kind == 'box':
        return '<box size="%s"/>' % _xyz(dims)
    return '<sphere radius="%s"/>' % _n(dims[0])


def block():
    lines = [BEGIN,
             '    <!-- Which posture the wearer holds. See wearer_posture.py;',
             '         %s to pick one. -->' % WP.ENV_VAR,
             '    <xacro:property name="wearer_arms" value="$(optenv %s %s)"/>'
             % (WP.ENV_VAR, WP.DEFAULT)]
    known = ' or '.join("wearer_arms == '%s'" % p for p in sorted(WP.POSTURES))
    lines += [
        '    <!-- An unknown posture must STOP the build. A typo that silently',
        '         falls back to the default is a sweep that measures the same',
        '         wearer five times and reports it as five postures. -->',
        '    <xacro:unless value="${%s}">' % known,
        '      <!-- a property body is evaluated lazily and would never fire;',
        '           an attribute is substituted eagerly, so this one does. -->',
        '      <link name="unknown_wearer_arms_posture_${1/0}"/>',
        '    </xacro:unless>',
    ]
    for posture in sorted(WP.POSTURES):
        spec = WP.POSTURES[posture]
        lines.append('    <xacro:if value="${wearer_arms == \'%s\'}">' % posture)
        lines.append('      <!-- %s -->' % _comment(spec['doc']))
        if spec['dirs'] is None:
            lines.append('      <!-- no links: the wearer has no arms in this'
                         ' configuration -->')
        for side in ('left', 'right'):
            for name, kind, dims, ctr, rpy in WP.arm_links(posture, side, 'local'):
                link = 'human_%s_%s' % (side, name)
                mat = dict((s[0], s[4]) for s in WP.SEGMENTS)[name]
                g = _geom(kind, dims)
                lines += [
                    '      <link name="%s">' % link,
                    '        <collision><geometry>%s</geometry></collision>' % g,
                    '        <visual><geometry>%s</geometry>' % g,
                    '          <material name="%s"/></visual>' % mat,
                    '      </link>',
                    '      <joint name="torso_to_%s" type="fixed">' % link,
                    '        <parent link="torso"/><child link="%s"/>' % link,
                    '        <origin xyz="%s" rpy="%s"/>' % (_xyz(ctr), _xyz(rpy)),
                    '      </joint>',
                ]
        lines.append('    </xacro:if>')
    lines.append(END)
    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    a = ap.parse_args()
    with open(XACRO) as fh:
        src = fh.read()
    pat = re.compile(re.escape(BEGIN) + r'.*?' + re.escape(END) + r'\n',
                     re.S)
    if not pat.search(src):
        print('markers not found in %s' % XACRO)
        return 2
    new = pat.sub(lambda _m: block(), src)
    if new == src:
        print('wearer arms block is up to date')
        return 0
    if a.check:
        print('STALE: %s does not match wearer_posture.py. Run '
              'scripts/gen_wearer_posture_xacro.py' % XACRO)
        return 1
    with open(XACRO, 'w') as fh:
        fh.write(new)
    print('rewrote the wearer arms block in %s' % XACRO)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
