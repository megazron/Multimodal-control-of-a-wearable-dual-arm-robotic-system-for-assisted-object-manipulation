"""`require_observer_estop` must gate the FREEZE, not only the enable service.

THE DEFECT THIS PINS, found in the lab on 2026-08-20 with a real arm on the
bench and an operator in a headset.

`vr_safety_node._tick` decided whether to leave the freeze with

    elif self.frozen and self.observer_ok and not self._reference_bad():

-- `self.observer_ok` read directly, with no parameter anywhere near it. The
parameter `require_observer_estop` existed and was honoured in exactly one
place: `_srv_enable`, the REAL-ARM request. So setting it false disabled the
real-arm gate, which is the half you want kept, and left the clutch held shut,
which is the half you were trying to open.

WHAT IT LOOKED LIKE FROM INSIDE THE HEADSET. Grip pressed: the gripper moved,
because `vr_gripper_node` does not consult the freeze. The arm did nothing.
The HUD said `clutch out`. The only explanation was 400 identical lines of
"engage REFUSED: the safety node is holding a freeze" in a log on a machine
the operator could not see while wearing the headset. It read as a broken
clutch for an hour.

THE TWO HALVES MUST MOVE INDEPENDENTLY, and that is the whole point:

  * simulation, no observer posted   -> the clutch may engage
  * real arm,   no observer posted   -> REFUSED, unconditionally, always

so both directions are asserted here. A test that only checked the first
would pass on a node that had simply deleted the interlock.
"""


class Fake:
    """`_observer_required` / `_observer_satisfied` and the `_tick` branch.

    A transcription, kept honest by `test_the_transcription_matches_the_node`
    below -- the same device `test_observer_estop_silence.py` uses, and for
    the same reason: the real node needs rclpy, TF and a graph, none of which
    is what is under test.
    """

    def __init__(self, require_observer=True):
        self.require_observer = require_observer
        self.allow_real_arm_without_observer = False
        self.observer_ok = False
        self.frozen = True
        self.reason = 'startup'
        self.reference_bad = False
        self.poses_flowing = True

    def _observer_required(self):
        return bool(self.require_observer)

    def _observer_satisfied(self):
        return self.observer_ok or not self._observer_required()

    def tick(self):
        if not self.poses_flowing:
            self.frozen, self.reason = True, 'no controller pose'
        elif self.frozen and self._observer_satisfied() and not self.reference_bad:
            self.frozen, self.reason = False, ''
        elif self.frozen and not self._observer_satisfied():
            self.reason = 'awaiting observer e-stop confirmation'

    def srv_enable(self):
        """`_srv_enable`: the REAL-ARM request.

        Gated by its OWN parameter, never by `require_observer_estop`.
        """
        if not self.observer_ok:
            if not self.allow_real_arm_without_observer:
                return False, 'REFUSED: no observer e-stop confirmed'
        return True, 'enabled'


def test_simulation_unfreezes_without_an_observer():
    """The bug: this stayed frozen for ever with the parameter set false."""
    f = Fake(require_observer=False)
    f.tick()
    assert f.frozen is False, (
        'require_observer_estop:=false must let the clutch engage; this is '
        'the exact state that read as a broken clutch in the lab')
    assert f.reason == ''


def test_the_default_still_demands_an_observer():
    """Off by default would be a silent removal of the interlock."""
    f = Fake(require_observer=True)
    f.tick()
    assert f.frozen is True
    assert f.reason == 'awaiting observer e-stop confirmation'
    f.observer_ok = True
    f.tick()
    assert f.frozen is False


def test_the_real_arm_refuses_even_when_the_freeze_is_relaxed():
    """THE HALF THAT MUST NOT MOVE.

    The parameter is about whether the SIMULATED arm may move unobserved. A
    node that also opened the real-arm gate would pass the first test in this
    file and be far more dangerous than the bug it replaced.
    """
    f = Fake(require_observer=False)
    f.tick()
    assert f.frozen is False, 'precondition: the sim clutch is free'
    ok, why = f.srv_enable()
    assert ok is False, (
        'the real arm must refuse without a confirmed observer no matter '
        'what the freeze path was told')
    assert 'observer' in why.lower()


def test_a_relaxed_gate_does_not_defeat_the_other_freezes():
    """Poses and the tracking reference still freeze with no observer needed.

    Without this, "not required" could be read as "nothing can freeze", and
    the pose watchdog is the one that catches a headset going to sleep.
    """
    f = Fake(require_observer=False)
    f.tick()
    assert f.frozen is False
    f.poses_flowing = False
    f.tick()
    assert f.frozen is True and 'pose' in f.reason

    g = Fake(require_observer=False)
    g.reference_bad = True
    g.tick()
    assert g.frozen is True, 'a moved tracking reference must still freeze'


def test_the_transcription_matches_the_node():
    """The Fake above is only worth anything if the node really reads this way.

    Pins the two helpers and, critically, that the `_tick` branch calls
    `_observer_satisfied()` rather than `self.observer_ok` -- the literal
    text of the defect.
    """
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[1]
           / 'srl_vr_teleop' / 'vr_safety_node.py').read_text()

    assert 'def _observer_required(self):' in src
    assert 'def _observer_satisfied(self):' in src
    # PINNED ON BEHAVIOUR, NOT ON LINE BREAKS.
    #
    # This asserted one exact source line. The node then grew a third term
    # (the audited working-alone bypass) and the expression wrapped across two
    # lines, so the test failed while the behaviour it guards was intact --
    # a transcription check that broke on reformatting, which is the "test
    # breaks with no behaviour change" entry in CLAUDE.md's own table.
    #
    # What must be true is that _observer_satisfied() is the OR of the three
    # ways an observer requirement can be met, whatever the whitespace.
    body = src.split("def _observer_satisfied(self):", 1)[1]
    body = body.split("def ", 1)[0]
    flat = " ".join(body.split())
    assert "self.observer_ok" in flat
    assert "not self._observer_required()" in flat
    assert "self._bypass()" in flat, (
        "the working-alone bypass must be one of the ways the observer "
        "requirement is satisfied: %s" % flat)

    assert 'elif self.frozen and self._observer_satisfied()' in src, (
        'the unfreeze branch must go through _observer_satisfied()')
    assert 'elif self.frozen and self.observer_ok and' not in src, (
        'the original defect is back: the freeze path reads observer_ok '
        'directly and the parameter cannot reach it')

    # _srv_enable keeps the parameter AND the flag, unconditionally.
    assert "allow_real_arm_without_observer" in src, (
        'the real-arm gate must have its own parameter, so that relaxing the '
        'simulation freeze cannot open it as a side effect')
    # CODE ONLY. The body explains the coupling it removed, so a naive
    # substring search matches the prose that documents the fix and fails on
    # a correct node -- the "everything matches" instrument failure, one line
    # long.
    i = src.index('def _srv_enable')
    body = src[i:].split('def _boundary')[0]
    code = '\n'.join(ln for ln in body.splitlines()
                     if not ln.lstrip().startswith('#'))
    assert "get_parameter('require_observer_estop')" not in code, (
        'the real-arm enable gate reads require_observer_estop again: one '
        'switch is opening two doors')
    assert "get_parameter(\n                    'allow_real_arm_without_observer')" \
        in code or "'allow_real_arm_without_observer'" in code, (
        'the real-arm gate must read its own parameter')
