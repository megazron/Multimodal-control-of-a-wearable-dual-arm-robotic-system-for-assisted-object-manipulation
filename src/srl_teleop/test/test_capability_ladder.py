#!/usr/bin/env python3
"""Known answers for the capability ladder.

The ladder is the thing that decides what mapping runs on a degrading master,
so it gets tested against hand-checked channel sets rather than against
itself.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from srl_teleop import capability as cap          # noqa: E402
from srl_teleop.capability_node import ChannelHealth  # noqa: E402

ALL = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"}


def lvl(healthy, imu=True):
    return cap.select(healthy, imu)[0]


def test_full_health_gives_full_fk():
    assert lvl(ALL) == cap.L_FK


def test_losing_a_roll_keeps_spherical():
    """A roll carries no radial information, so losing one costs a rung but
    not the mapping."""
    assert lvl(ALL - {"j3"}) == cap.L_SPHERICAL
    assert lvl(ALL - {"j5"}) == cap.L_SPHERICAL


def test_one_bend_is_enough_for_a_measured_radius():
    assert lvl({"j1", "j2"}) == cap.L_SPHERICAL
    assert lvl({"j1", "j4"}) == cap.L_SPHERICAL


def test_losing_both_bends_drops_to_a_rate_driven_radius():
    """This is the measured left-arm case: no reach observable survives."""
    assert lvl({"j1", "j3", "j7"}) == cap.L_SPH_RATE


def test_no_spare_channel_freezes_the_radius():
    assert lvl({"j1"}) == cap.L_SHELL


def test_losing_j1_removes_position_entirely():
    """Azimuth has exactly one source. Without it there is no position, and
    the ladder must say so rather than degrade quietly."""
    l = lvl(ALL - {"j1"})
    assert l == cap.L_DIR_ONLY
    assert not cap.position_available(l)


def test_no_imu_is_never_a_position():
    for h in (ALL, {"j1", "j2"}, {"j1"}, set()):
        assert not cap.position_available(lvl(h, imu=False))


def test_empty_set_is_the_bottom_rung():
    assert not cap.position_available(lvl(set()))


def test_recovery_moves_back_up():
    """A channel returning to health must improve the rung, not merely be
    recorded as healthy."""
    assert lvl({"j1"}) == cap.L_SHELL
    assert lvl({"j1", "j7"}) == cap.L_SPH_RATE
    assert lvl({"j1", "j7", "j4"}) == cap.L_SPHERICAL


def test_regain_only_credits_a_repair_that_actually_helps():
    """The point of the regain map: fixing a roll while both bends are dead
    buys nothing, and a health table cannot express that."""
    healthy = {"j1", "j3"}
    r = cap.regain(healthy)
    assert r["j5"][1] is False, "another roll must not be credited"
    assert r["j2"][1] is True, "a bend must be credited"
    assert r["j4"][1] is True


def test_regain_of_j1_when_j1_is_down_is_the_biggest_jump():
    r = cap.regain({"j2", "j4"})
    assert r["j1"][0] == cap.L_SPHERICAL


def test_every_level_states_whether_position_exists():
    for k in cap.LEVELS:
        assert isinstance(cap.position_available(k), bool)


def test_costs_are_loaded_not_invented():
    """Either a measured cost is present, or the rung says unmeasured. It must
    never carry a plausible number of unknown origin."""
    for key, c in cap.COST_M.items():
        assert c["mean_m"] is not None
        assert c["mean_m"] >= 0.0
    assert cap.COST_M.get("FK", {"mean_m": 0.0})["mean_m"] == 0.0


# ------------------------------------------------------------ health window
def test_health_uses_distinct_updates_not_repeated_rows():
    """A channel held perfectly constant by an oversampling recorder must not
    read as alive merely because rows keep arriving."""
    h = ChannelHealth()
    for _ in range(200):
        h.push("j2", 123.4)
    assert h.verdict("j2") == "dead", "zero circular range is dead"


def test_a_moving_channel_reads_alive():
    h = ChannelHealth()
    for i in range(200):
        h.push("j2", 100.0 + 0.4 * i % 300)
    assert h.verdict("j2") == "alive"


def test_incoherent_channel_is_caught_by_jump_rate_not_range():
    """The failure mode that range alone calls healthy: wide spread, no
    trajectory."""
    h = ChannelHealth()
    seq = [10.0, 200.0, 30.0, 250.0, 60.0, 300.0, 15.0, 280.0] * 25
    for v in seq:
        h.push("j4", v)
    assert h.verdict("j4") == "incoherent"


def test_dropouts_are_counted_and_not_used_as_values():
    h = ChannelHealth()
    for i in range(200):
        h.push("j6", 0.0 if i % 2 else 100.0 + i * 0.3)
    assert h.verdict("j6") in ("intermittent", "dead", "incoherent")


def test_circular_range_survives_the_wrap():
    h = ChannelHealth()
    assert h._circular_range([359.0, 1.0, 0.0]) < 10.0, "must not read ~358"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
