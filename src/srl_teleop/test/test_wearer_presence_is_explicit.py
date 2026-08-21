"""Declaring nobody is in the rig must be explicit, loud, and never a default.

WHY THIS EXISTS. The wearer's POSTURE was a variable and their SIZE was a
variable, but their EXISTENCE was not. So a bench-mounted arm with an empty
harness was still checked against a mannequin torso, and homing halted at
0.114 m from a person who was not in the room -- caught by a mock rehearsal in
a lab session, one command before the real arm would have done the same.

The only routes available at that moment were to lower the clearance floor or
to edit the model by hand. Both are changes that quietly become permanent, and
HARD CONSTRAINT 11 exists because of exactly that.

So there is now a way to say "nobody is wearing this", and it is arranged to
be impossible to leave on by accident. THIS IS THE MOST DANGEROUS SETTING IN
THE REPOSITORY and every test below is about making it hard to get wrong
rather than easy to use.
"""
import pytest

from srl_teleop import wearer_posture as WP


def test_the_default_is_that_somebody_IS_in_the_rig():
    """Absence is never the fallback."""
    assert WP.wearer_present({}) is True
    assert WP.wearer_present({"SRL_WEARER_PRESENT": ""}) is True
    assert len(WP.wearer_model("down")) == 12


@pytest.mark.parametrize("word", ["0", "no", "false", "none", "off", "absent",
                                  "NO", "False", " off "])
def test_absence_must_be_said_in_words_that_mean_absence(word):
    assert WP.wearer_present({"SRL_WEARER_PRESENT": word}) is False


@pytest.mark.parametrize("word", ["1", "yes", "true", "on", "present", "YES"])
def test_presence_words_are_accepted_too(word):
    assert WP.wearer_present({"SRL_WEARER_PRESENT": word}) is True


@pytest.mark.parametrize("word", ["maybe", "sometimes", "2", "nobody?", "-1",
                                  "n", "y"])
def test_anything_else_RAISES_rather_than_guessing(word):
    """A typo must not resolve to either answer.

    Silently reading `SRL_WEARER_PRESENT=maybe` as absent is how a bench
    setting reaches a session with a person in the harness; silently reading
    it as present is how somebody spends an afternoon fighting a clearance
    halt they thought they had turned off. Neither is acceptable, so it is an
    error.
    """
    with pytest.raises(ValueError) as e:
        WP.wearer_present({"SRL_WEARER_PRESENT": word})
    assert "not yes or no" in str(e.value)


def test_absent_means_NO_primitives_not_a_smaller_person():
    """A partial body would be the worst of both: a floor that still fires,
    protecting a shape that is not there."""
    assert WP.wearer_model("down", present=False) == []
    # ... and the default is untouched, byte for byte.
    assert len(WP.wearer_model("down", present=True)) == 12


def test_the_banner_names_the_variable_and_says_when_it_is_wrong():
    """Every consumer prints this while the wearer is declared away. It has
    to be readable by somebody who did not set it."""
    b = WP.absence_banner()
    assert "SRL_WEARER_PRESENT" in b
    assert "bench" in b.lower()
    assert "before anybody puts the rig on" in b.lower()


def test_the_consumers_actually_check_it():
    """A declaration nothing reads is worse than none: it looks like a
    supported feature and changes nothing."""
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for mod, why in (("real_homing_node.py",
                      "homing halts on wearer clearance"),
                     ("mount_guard_node.py",
                      "the guard enforces the clearance floor")):
        src = open(os.path.join(here, "srl_teleop", mod)).read()
        assert "wearer_present()" in src, "%s does not check it (%s)" % (mod,
                                                                        why)
        assert "absence_banner()" in src, (
            "%s honours it SILENTLY -- a quiet 'no wearer' is the state that "
            "gets somebody hurt" % mod)
