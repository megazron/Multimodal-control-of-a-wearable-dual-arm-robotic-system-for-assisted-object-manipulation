"""The home pose has ONE source, and everything else must load it.

THE "UPDATE BOTH PLACES" TRAP IS REAL AND IT WAS UNDERCOUNTED. The pose was
carried in FIVE places, not two, and the two nobody names are the dangerous
ones:

    config/home_positions_{arm}.txt   THE SOURCE. Five live runtime consumers:
                                      sim_to_real_bridge (the enable gate),
                                      real_homing_node (the target the REAL
                                      arm is driven to), ik_follower_node (the
                                      IK seed), mock_real_stack, session_manager
    srl_dual.urdf.xacro x2            where the SIM arm spawns, baked at launch
    pot_bridge.py                     had its own hardcoded copy
    srl_teleop_node.py                had its own hardcoded copy
    config/real_home_reference.txt    reference only, read by no code

Both hardcoded copies PUBLISH TO THE ARM CONTROLLERS, and both are installed
executables that no launch file references -- so nothing exercised them, and
when the home moved on 2026-08-15 they would have driven the arms to the
superseded pose. They now load the source like everything else, and this test
is what stops a sixth copy appearing.

`config/home_positions_{arm}.txt` is what every node loads at runtime; the two
`initial_positions` blocks of `srl_dual.urdf.xacro` are where the SIM arm
actually starts. Updating one and not the other is a documented trap in this
project, and it is invisible: the sim comes up at one pose while every text
file, every readiness gate and the sim-to-real bridge's enable check all read
the other. The bridge would then refuse to enable against a home the sim is
not at, or -- worse -- agree with a home the arm is not at.

It was a comment in both files saying "keep the two in step". A comment cannot
fail. This can.

ALSO CHECKED: continuous joints inside +/-pi. `ik_follower_node` refuses to
start on real hardware with a continuous joint wound beyond +/-pi and performs
an automatic unwind in sim, so a home stored outside that range is a home that
either blocks real bring-up or commands a 360 deg move at every start. The
2026-08-15 pose change hit exactly this: the search returned the right arm's
joint_7 at 265.75 deg and it had to be stored wrapped, as -94.25.
"""
import math
import os
import re
import sys

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "config"))

# joint_1, joint_3, joint_5, joint_7 -- the same tuple kortex_convention and
# ik_follower_node use. Named here rather than imported so this test does not
# pass by agreeing with a constant it is meant to be checking.
CONTINUOUS_IDX = (0, 2, 4, 6)
URDF = os.path.join(WS, "src", "srl_description", "urdf", "srl_dual.urdf.xacro")
TOL_RAD = 1e-3          # the text files are stored to 0.01 deg = 1.7e-4 rad


def _urdf_initial_positions():
    """[(arm, [q7])] in the order the blocks appear -- left then right."""
    txt = open(URDF).read()
    blocks = re.findall(r'initial_positions="\$\{dict\(([^)]*)\)\}"', txt)
    out = []
    for arm, blk in zip(("left", "right"), blocks):
        vals = {}
        for part in blk.split(","):
            k, v = part.split("=")
            vals[k.strip()] = float(v)
        out.append((arm, [vals["joint_%d" % i] for i in range(1, 8)]))
    return out


def test_urdf_has_exactly_two_initial_positions_blocks():
    """If this fires, the parser below is reading the wrong thing."""
    assert len(_urdf_initial_positions()) == 2


def test_urdf_and_config_agree():
    import home_positions as hp
    for arm, urdf_q in _urdf_initial_positions():
        cfg_q = hp.load_home_radians(arm)
        worst = max(abs(urdf_q[i] - cfg_q[i]) for i in range(7))
        assert worst < TOL_RAD, (
            "%s home has DRIFTED between its two storage sites: worst "
            "|delta| = %.6f rad on joint_%d.\n"
            "  urdf   %s\n  config %s\n"
            "srl_dual.urdf.xacro is where the sim arm starts; "
            "config/home_positions_%s.txt is what every node loads. "
            "They must be the same pose."
            % (arm, worst,
               max(range(7), key=lambda i: abs(urdf_q[i] - cfg_q[i])) + 1,
               ["%.4f" % v for v in urdf_q], ["%.4f" % v for v in cfg_q], arm))


def test_no_node_hardcodes_a_home_pose():
    """Any node that commands the arms must LOAD home, not carry a copy.

    Detected structurally rather than by matching the old numbers: a literal
    list of seven floats assigned to something called `home` is a copy of the
    pose whatever its values are, so this still fires if someone pastes the
    NEW pose in. Matching the legacy numbers would pass the moment the copy
    was updated once, which is exactly the failure being prevented.
    """
    import ast
    import glob
    src = os.path.join(WS, "src", "srl_teleop", "srl_teleop")

    # PARSED, NOT PATTERN-MATCHED, and it took two goes to get here.
    #
    # v1 anchored on `home... = [` and MISSED the dict form the real code
    # used -- `self.home = {"left": [ ...seven... ], "right": [...]}` -- so it
    # passed a deliberately broken input, which makes it not a check.
    # v2 matched any seven-float list with "home" in the preceding 200
    # characters and FALSE-POSITIVED on mock_real_stack's `start_offset_deg`,
    # which is an offset FROM home and sits under a comment mentioning it.
    #
    # The property that actually matters is "a literal pose assigned to
    # something called home", and that is a question about the syntax tree,
    # not about the characters.
    def _seven_floats(node):
        if isinstance(node, ast.List) and len(node.elts) == 7:
            return all(isinstance(e, ast.Constant)
                       and isinstance(e.value, (int, float))
                       or (isinstance(e, ast.UnaryOp)
                           and isinstance(e.op, ast.USub))
                       for e in node.elts)
        return False

    def _named_home(target):
        if isinstance(target, ast.Name):
            return "home" in target.id.lower()
        if isinstance(target, ast.Attribute):
            return "home" in target.attr.lower()
        return False

    bad = []
    for path in glob.glob(os.path.join(src, "*.py")):
        tree = ast.parse(open(path).read())
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign):
                continue
            if not any(_named_home(t) for t in node.targets):
                continue
            if any(_seven_floats(sub) for sub in ast.walk(node.value)):
                bad.append("%s:%d" % (os.path.basename(path), node.lineno))
    assert not bad, (
        "a node carries its own copy of the home pose at %s. Load it from "
        "config/home_positions_{arm}.txt instead -- pot_bridge.py and "
        "srl_teleop_node.py both did this, both publish to the arm "
        "controllers, and both would have commanded the superseded pose after "
        "the 2026-08-15 change." % ", ".join(bad))


def test_the_nodes_that_command_the_arms_load_the_source():
    """The positive half of the check above: they must actually load it."""
    src = os.path.join(WS, "src", "srl_teleop", "srl_teleop")
    for name in ("pot_bridge.py", "srl_teleop_node.py", "ik_follower_node.py",
                 "sim_to_real_bridge.py", "real_homing_node.py"):
        txt = open(os.path.join(src, name)).read()
        assert "home_positions" in txt, (
            "%s commands or gates arm motion but never reads "
            "config/home_positions_*.txt" % name)


def test_the_urdf_comments_are_legal_xml():
    """`--` cannot appear inside an XML comment, and the URDF is comment-heavy.

    Caught the hard way on 2026-08-15: a comment added beside the home change
    used `--` the way the Python files in this repository use it, and xacro
    refused the whole description with "not well-formed (invalid token)". That
    is not a cosmetic failure -- it takes out the arms, the wearer, MoveIt and
    every node that reads /robot_description, and the error names a line and a
    column rather than the thing that is wrong.

    A full xacro parse would be the complete check; it needs ROS and the
    vendor packages on the path, which a unit test should not require. This
    catches the one syntax rule that a prose-heavy file actually trips over.
    """
    txt = open(URDF).read()
    bad = []
    for i, block in enumerate(re.findall(r"<!--(.*?)-->", txt, re.S)):
        if "--" in block:
            bad.append(block.strip().splitlines()[0][:70])
    assert not bad, (
        "srl_dual.urdf.xacro has '--' inside an XML comment, which makes the "
        "whole file unparseable. Offending comment(s) start: %s" % bad)


def test_continuous_joints_are_inside_pi():
    import home_positions as hp
    for arm in ("left", "right"):
        q = hp.load_home_radians(arm)
        wound = [(i + 1, q[i]) for i in CONTINUOUS_IDX if abs(q[i]) > math.pi]
        assert not wound, (
            "%s home stores a continuous joint outside +/-pi: %s. "
            "ik_follower_node REFUSES to start on real hardware in that state "
            "and unwinds 360 deg in sim. Store the wrapped value -- it is the "
            "same physical pose, and the Kortex value is unchanged."
            % (arm, ", ".join("joint_%d=%.4f rad" % w for w in wound)))
