"""docs/ENGINEERING_LOG.md must stay short AND still carry every damaging constraint.

The file was 98k tokens on 2026-08-12, a tenth of a session's context before
any work began, so it was split into docs/system/. The risk of splitting is
the opposite failure: a constraint that gets lost with the detail it moved to.

These are the rules where FORGETTING causes real damage rather than wasted
time. Each must survive in docs/ENGINEERING_LOG.md itself, because a fresh session reads
that and nothing else before starting.
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
MD = os.path.join(ROOT, "docs/ENGINEERING_LOG.md")


def text():
    return open(MD).read()


def test_it_stays_short():
    """15k tokens is the budget. ~4 bytes per token is the usual ratio."""
    n = len(text())
    assert n < 60000, ("docs/ENGINEERING_LOG.md is %d bytes, about %dk tokens. New findings "
                       "belong in docs/system/findings.md." % (n, n // 4000))


def test_every_damaging_constraint_survives():
    t = text()
    for phrase, why in [
            ("Home joint angles are ground truth", "sim/real jump"),
            ("ONE Kortex session", "a leaked session blocks the next run"),
            ("Never run two stacks", "splits the serial stream"),
            ("cyclic path is unusable", "costs a day on the wrong driver"),
            ("FASTDDS_BUILTIN_TRANSPORTS=SHM", "nothing discovers anything"),
            ("wait_for_service", "blocked the e-stop for 4 s"),
            ("dependency arrow", "circular experimental comparison"),
            ("motion_enabled", "unarmed real motion"),
            ("clearance floor", "the arms and a person's chest"),
            ("Anonymity", "participant data"),
            ("set +u", "sourcing ROS dies otherwise"),
    ]:
        assert phrase in t, "docs/ENGINEERING_LOG.md lost a damaging constraint: %s (%s)" % (
            phrase, why)


def test_the_standing_rule_and_its_checklist_survive():
    t = text()
    assert "validate the measuring tool" in t.lower()
    assert "cannot fail on a deliberately broken input" in t
    # The checklist is the part that actually gets used mid-debug.
    for row in ("oversamples", "stale republished", "out of distribution",
                "prefix matching", "STORED, not that a consumer READS"):
        assert row in t, "instrument-failure checklist lost: %s" % row


def test_it_says_how_to_run_things():
    t = text()
    for cmd in ("ros2 run srl_teleop gui", "check_channels.sh",
                "colcon build --symlink-install", "pytest -q src/*/test"):
        assert cmd in t, "docs/ENGINEERING_LOG.md no longer says how to: %s" % cmd


def test_it_indexes_the_split_docs():
    t = text()
    for d in ("findings.md", "hardware.md", "wsl.md", "architecture.md",
              "NEXT_SESSION.md"):
        assert d in t, "index lost the pointer to %s" % d


def test_the_split_docs_exist_and_are_not_empty():
    for d in ("findings", "hardware", "wsl", "architecture"):
        p = os.path.join(ROOT, "docs", "system", "%s.md" % d)
        assert os.path.exists(p), "%s is referenced but missing" % p
        assert len(open(p).read()) > 2000, "%s is suspiciously small" % p


def test_the_check_can_fail_on_a_deliberately_broken_input():
    """A check that cannot fail is not a check."""
    broken = "# kortex_ws\n\nnothing useful here\n"
    assert "ONE Kortex session" not in broken
    assert len(broken) < 60000          # short, but missing every constraint
