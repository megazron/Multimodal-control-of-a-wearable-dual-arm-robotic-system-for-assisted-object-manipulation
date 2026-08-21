"""Working alone is allowed. Working alone BY ACCIDENT is not.

The observer interlock is the only thing in this system that asserts a second
person can see the arm and reach a physical e-stop. An operator who is
genuinely alone needs a way past it, because the alternatives they will
otherwise find -- `require_observer_estop:=false`, or editing the check -- are
worse in the specific way that matters: they are silent and they are
permanent.

So the bypass exists, and every one of these tests is about it being hard to
take by accident and impossible to take without leaving a record.

THE ONE-WAY RULE. Every error path in `observer_bypass.active()` returns
False. A missing file, a corrupt file, a file with no timestamp, a file from
before the machine booted, a file older than the TTL -- all of them mean "no
bypass", never "assume yes". There is no branch that turns a failure into
permission, and `test_every_broken_record_denies` walks them.
"""
import json
import os
import time

import pytest

from srl_teleop import observer_bypass as OB


@pytest.fixture(autouse=True)
def _isolated(tmp_path, monkeypatch):
    """Never touch the real scratch or the real audit log from a test."""
    monkeypatch.setenv(OB.STATE_ENV, str(tmp_path / "bypass.json"))
    monkeypatch.setenv("SRL_WS", str(tmp_path))
    yield


def test_the_default_is_no_bypass():
    on, why = OB.active()
    assert on is False
    assert "no bypass" in why


def test_granting_it_turns_it_on_and_clearing_turns_it_off():
    OB.grant(who="test", reason="alone")
    assert OB.active()[0] is True
    OB.clear(who="test")
    assert OB.active()[0] is False


def test_every_use_is_written_down_with_a_timestamp():
    OB.grant(who="gui", reason="single-operator session")
    rows = OB.history()
    assert rows, "granting the bypass wrote nothing to the log"
    row = [r for r in rows if r.get("event") == "granted"][-1]
    assert row["who"] == "gui"
    assert row["reason"] == "single-operator session"
    # A timestamp a person can read, not just an epoch float.
    assert row["granted_at_iso"][:2] == "20"
    assert float(row["granted_at"]) > 0


def test_cancelling_is_written_down_too():
    """Otherwise the log says a run had no observer and never says it was put
    back, and the record of the SESSION is wrong even though every grant in
    it is right."""
    OB.grant(who="gui")
    OB.clear(who="gui", note="unticked")
    events = [r.get("event") for r in OB.history()]
    assert "granted" in events and "cleared" in events


def test_clearing_when_there_is_nothing_writes_nothing():
    """The log records decisions, not every time a window opened."""
    assert OB.clear(who="gui") is False
    assert OB.history() == []


def test_it_expires_on_its_own():
    """THE BACKSTOP. The mechanism is that the window clears it on open; this
    is what catches the machine nobody closed. A bypass taken yesterday must
    not authorise a run this morning."""
    OB.grant(who="gui", ttl_s=1.0)
    rec = json.load(open(OB.state_path()))
    rec["granted_at"] = time.time() - 3600.0
    json.dump(rec, open(OB.state_path(), "w"))
    on, why = OB.active()
    assert on is False
    assert "expired" in why


def test_a_bypass_from_before_the_machine_booted_is_not_this_session():
    """A grant whose timestamp predates the last boot belongs to a session
    that has already ended, whatever the TTL says."""
    boot = OB._boot_time()
    if boot is None:
        pytest.skip("no /proc/uptime on this machine")
    # The TTL has to be long enough that it CANNOT be what denies this, or
    # the test passes without ever running the check it is named after. On a
    # box up for a day and a half the default TTL fires first and the
    # assertion below reads as a failure of the boot check.
    OB.grant(who="gui", ttl_s=(time.time() - boot) + 7200.0)
    rec = json.load(open(OB.state_path()))
    rec["granted_at"] = boot - 60.0
    json.dump(rec, open(OB.state_path(), "w"))
    assert time.time() - rec["granted_at"] < rec["ttl_s"], "TTL would fire"
    on, why = OB.active()
    assert on is False
    assert "before this machine last started" in why


@pytest.mark.parametrize("content", [
    "not json at all",
    "{}",
    '{"granted_at": "yesterday"}',
    '{"ttl_s": 3600}',
    '{"granted_at": 99999999999, "ttl_s": 3600}',      # dated in the future
])
def test_every_broken_record_denies(content):
    """THE ONE-WAY RULE, walked. No malformed input may read as permission."""
    with open(OB.state_path(), "w") as fh:
        fh.write(content)
    on, _why = OB.active()
    assert on is False, "a broken bypass record granted the bypass"


def test_an_unreadable_file_denies(monkeypatch):
    os.makedirs(OB.state_path(), exist_ok=True)      # a directory, not a file
    assert OB.active()[0] is False


def test_the_description_says_nobody_is_watching():
    """This sentence goes on the status line. It has to be unmistakable."""
    OB.grant(who="gui")
    text = OB.describe()
    assert "BYPASSED" in text
    assert "e-stop" in text
    assert "Nobody" in text


def test_the_state_file_lives_in_scratch_and_not_in_config(monkeypatch):
    """It must not be something that can be committed, deployed or inherited.

    A bypass in `config/` would arrive on another machine in a git pull.
    """
    monkeypatch.delenv(OB.STATE_ENV, raising=False)
    p = OB.state_path()
    assert "config" not in p
    assert p.endswith(OB.STATE_NAME)


def test_the_audit_log_is_append_only_from_here():
    """Two grants must both survive. Nothing in this module rewrites it."""
    OB.grant(who="a")
    OB.clear(who="a")
    OB.grant(who="b")
    who = [r.get("who") for r in OB.history() if r.get("event") == "granted"]
    assert who == ["a", "b"], who
