#!/usr/bin/env python3
"""The master-arm tip smoother: the control law, and the state it must not carry.

Same two obligations as test_vr_smoothing.py, applied to the master path:

  THE CONTROL LAW must beat the fixed EMA(0.3) the node ran until 2026-08-27
  on stillness AND lag at once -- either alone is free (retune the alpha the
  other way and it wins that one back), which is exactly why the fixed alpha
  could not be tuned out of the "smaller is smoother but laggier" trade.

  THE STATE must not cross a clutch boundary. The master path's boundary is
  the clutch ENGAGE: the smoother kept following while disengaged, so its
  position, velocity estimate and adaptive cutoff are wrong by however far
  the operator recentred -- the same carried-state defect that cost
  02_vr_teleop every grasping task through the mapper's `filt`.

The node methods under test (`tip_filter_sync`, `smooth_tip`,
`reset_tip_filter`, `update_clutch`) are the SHIPPED ones, exercised on an
instance built without the serial port or rclpy init -- only the parameter
surface, clock and logger are faked, because those are rclpy's, not this
node's. That the read loop actually CALLS them is pinned separately below,
because "present but never read" is this repo's most repeated failure mode.
"""
import inspect
import types

import numpy as np

from srl_teleop import master_pose_node as mpn
from srl_teleop import smoothing as sm

RATE = 50.0            # the master serial frame rate the node was sized for
DT = 1.0 / RATE
OLD_ALPHA = 0.3        # the fixed EMA the node declared until 2026-08-27


class _Param:
    def __init__(self, v):
        self.value = v


class _Log:
    def info(self, *a, **k):
        pass

    warn = info
    error = info


class _Clock:
    def __init__(self):
        self.t = 100.0

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(self.t * 1e9))


def rig(**over):
    """A MasterPoseNode with only rclpy's surface faked.

    __new__ skips the constructor (which opens a serial port), then wires
    exactly the state the smoothing and clutch methods read. The wiring is
    the same the real __init__ performs; test_the_read_loop_uses_the_filter
    guards the other half -- that the shipped loop actually calls into it.
    """
    n = mpn.MasterPoseNode.__new__(mpn.MasterPoseNode)
    n.arms = ["left", "right"]
    params = {
        "smoothing": "one_euro", "ema_alpha": OLD_ALPHA,
        "min_cutoff_hz": 0.5, "beta": 100.0, "d_cutoff_hz": 0.2,
        "smooth_orientation": False,
        "left_clutch_button": 2, "right_clutch_button": 1,
        # update_clutch re-reads these EVERY frame as of 2026-08-30, so the
        # capture can pin the clutch without restarting the node. They are
        # here rather than as attributes for the same reason.
        "clutch_enabled": True, "force_clutch_engaged": False,
    }
    params.update(over)
    n.params = params
    n.get_parameter = lambda k: _Param(params[k])
    n.get_logger = lambda: _Log()
    clock = _Clock()
    n.get_clock = lambda: clock
    n.clock = clock
    # smoothing state, as __init__ builds it
    n._smooth_key = None
    n._smooth_rejected = None
    n.tip_filt = {}
    n.quat_filt = {}
    n.tip_filter_sync()
    n.filt_t = {a: None for a in n.arms}
    n.last_filt_dt = {a: 0.0 for a in n.arms}
    # clutch state, as __init__ builds it
    n.clutch_enabled = params["clutch_enabled"]
    n.force_clutch = params["force_clutch_engaged"]
    n.clutch_button = {"left": 2, "right": 1}
    n.btn_last = {a: None for a in n.arms}
    n.last_btn_edge_t = {a: 0.0 for a in n.arms}
    n.clutch_debounce = 0.05
    n.clutch_static_m = 0.002
    n.clutch_static_win = 0.1
    n.clutch_ref_frames = 2
    n.clutch_on = {a: True for a in n.arms}
    n.pending_engage = {a: False for a in n.arms}
    n.pos_anchor = {a: np.zeros(3) for a in n.arms}
    n.last_pos = {a: np.zeros(3) for a in n.arms}
    n.ref_accum = {a: [] for a in n.arms}
    n.d_ref = {a: np.zeros(3) for a in n.arms}
    n.recent_p = {a: [] for a in n.arms}
    n.azim_gyro = {a: None for a in n.arms}
    n.last_gyro_t = {a: None for a in n.arms}
    return n


def feed(n, arm, sig):
    out = []
    for x in sig:
        n.clock.t += DT
        out.append(n.smooth_tip(arm, x, n.clock.t))
    return np.asarray(out)


def _still(nsamp=900, sigma=0.0015, seed=7):
    return np.random.default_rng(seed).normal(0.0, sigma, (nsamp, 3))


def _ramp(nsamp=900, v=0.40):
    t = np.arange(nsamp) * DT
    return np.stack([v * t, np.zeros(nsamp), np.zeros(nsamp)], 1)


# ------------------------------------------------------------- the control law
def test_one_euro_beats_the_shipped_ema_0_3_on_both_axes():
    """Stillness AND lag at once, against the alpha the node actually ran.

    1.5 mm-rms tremor and a 0.40 m/s reach, at the master's own 50 Hz --
    not the VR path's 72 -- because the fixed EMA's response depends on the
    tick rate, so a comparison at the wrong rate compares the wrong filter.
    """
    noise, ramp = _still(), _ramp()

    oe = feed(rig(), "left", noise)
    em = feed(rig(smoothing="ema"), "left", noise)
    oe_still = float(np.sqrt((oe[200:] ** 2).sum(1).mean())) * 1000
    em_still = float(np.sqrt((em[200:] ** 2).sum(1).mean())) * 1000

    oe = feed(rig(), "left", ramp + noise)
    em = feed(rig(smoothing="ema"), "left", ramp + noise)
    oe_lag = float(np.mean(ramp[400:, 0] - oe[400:, 0])) * 1000
    em_lag = float(np.mean(ramp[400:, 0] - em[400:, 0])) * 1000

    assert oe_still < em_still, (
        "still: %.2f vs %.2f mm" % (oe_still, em_still))
    assert oe_lag < em_lag, "lag: %.2f vs %.2f mm" % (oe_lag, em_lag)


# ---------------------------------------------------------------- reproduction
def test_ema_mode_is_the_old_arithmetic_bit_for_bit():
    """`smoothing:=ema` exists so pre-2026-08-27 recordings reproduce.

    The reference below IS the removed code: seed on first sample, then
    alpha*tip + (1-alpha)*filt with alpha clamped and re-read per frame --
    and dt-BLIND, so irregular frame times must change nothing.
    """
    n = rig(smoothing="ema", ema_alpha=OLD_ALPHA)
    rng = np.random.default_rng(3)
    sig = rng.normal(0.0, 0.1, (200, 3)) + np.array([0.1, -0.2, 0.35])
    old = None
    t = 500.0
    for i, x in enumerate(sig):
        # deliberately irregular steps: the old filter never saw dt
        t += DT * (1.0 + 2.0 * (i % 3))
        got = n.smooth_tip("left", x, t)
        old = x.copy() if old is None else (
            OLD_ALPHA * x + (1.0 - OLD_ALPHA) * old)
        assert np.array_equal(got, old), (
            "sample %d: %r != %r -- `ema` no longer reproduces the "
            "recordings made before the 1-Euro change" % (i, got, old))


# ------------------------------------------------------------------ live params
def test_param_changes_take_effect_live():
    """The old alpha was re-read every frame so `ros2 param set` worked
    mid-session; the new law keeps that by rebuilding on any change."""
    n = rig()
    feed(n, "left", _still(50))
    assert isinstance(n.tip_filt["left"], sm.OneEuro)

    # law change: next frame runs the other filter, freshly seeded
    n.params["smoothing"] = "ema"
    x = np.array([0.3, 0.1, -0.2])
    n.clock.t += DT
    got = n.smooth_tip("left", x, n.clock.t)
    assert isinstance(n.tip_filt["left"], sm.LegacyEma)
    assert np.array_equal(got, x)          # first sample passes through
    # alpha change rebuilds too
    n.params["ema_alpha"] = 0.5
    n.clock.t += DT
    n.smooth_tip("left", x, n.clock.t)
    assert n.tip_filt["left"].alpha == 0.5

    # orientation toggle reaches the quaternion filter
    assert n.quat_filt["left"] is None
    n.params["smooth_orientation"] = True
    n.clock.t += DT
    n.smooth_tip("left", x, n.clock.t)
    assert isinstance(n.quat_filt["left"], sm.OneEuroQuat)


def test_a_live_typo_keeps_the_law_in_force_instead_of_crashing():
    """make() refuses unknown names; mid-session that refusal must not kill
    the node or silently select a different law -- the session keeps the
    filters it had, and the log says so."""
    n = rig(smoothing="ema", ema_alpha=0.5)
    feed(n, "left", _still(20))
    before = n.tip_filt["left"]
    n.params["smoothing"] = "one-euro"          # the typo make() refuses
    n.clock.t += DT
    n.smooth_tip("left", np.zeros(3), n.clock.t)   # must not raise
    assert n.tip_filt["left"] is before
    assert isinstance(n.tip_filt["left"], sm.LegacyEma)


# ------------------------------------------------------------------- per arm
def test_each_arm_has_its_own_filter_state():
    """Two arms move independently; a shared filter would blend one arm's
    velocity estimate into the other's cutoff."""
    n = rig()
    assert n.tip_filt["left"] is not n.tip_filt["right"]
    feed(n, "left", _ramp(100) + np.array([0.5, 0.0, 0.0]))
    x = np.array([0.0, 0.1, 0.0])
    n.clock.t += DT
    got = n.smooth_tip("right", x, n.clock.t)
    assert np.array_equal(got, x), (
        "the right arm's first sample was biased by the left arm's history")


# ------------------------------------------------------------ clutch boundary
def test_filter_state_resets_at_the_clutch_engage():
    """Through the SHIPPED update_clutch, not by calling reset directly.

    Disengage, walk the master 0.5 m away (the smoother follows -- output is
    frozen elsewhere, the filter is not), re-engage through the debounce,
    static gate and reference averaging, and the first sample after engage
    must pass through exactly -- no blend of the pre-disengage position, no
    inherited velocity estimate.
    """
    n = rig()
    far = np.array([0.5, -0.3, 0.9])
    feed(n, "left", np.tile(far, (100, 1)))     # the filter now holds `far`
    assert n.filt_t["left"] is not None

    btn = [0.0, 0.0]

    def press():
        btn[1] = 1.0 - btn[1]                   # firmware toggle: flip value
        n.clock.t += 0.2                        # past the 50 ms debounce
        n.update_clutch("left", tuple(btn), True, np.zeros(3))

    n.update_clutch("left", tuple(btn), True, np.zeros(3))  # first sight
    press()                                     # edge 1: DISENGAGE
    assert n.clutch_on["left"] is False

    # master is quasi-static at the new station
    n.recent_p["left"] = [(n.clock.t, np.zeros(3)) for _ in range(3)]
    press()                                     # edge 2: engage PENDING
    # ref averaging needs clutch_ref_frames=2 valid static frames
    n.update_clutch("left", tuple(btn), True, np.zeros(3))
    assert n.clutch_on["left"] is True

    assert n.filt_t["left"] is None, (
        "engage did not clear the filter clock -- the first post-engage dt "
        "spans the whole disengaged period")
    probe = np.array([0.1, 0.2, 0.3])
    n.clock.t += DT
    got = n.smooth_tip("left", probe, n.clock.t)
    assert np.array_equal(got, probe), (
        "the first sample after engage blended the pre-disengage position "
        "%r -- the 02_vr_teleop carried-state defect, on the master path"
        % (far,))


# ------------------------------------------------- the loop actually calls it
def test_the_read_loop_uses_the_filter_and_the_clutch_resets_it():
    """The rig above fakes the wiring, so pin the shipped call sites: a
    filter that exists but is never called is 'feature present but does
    nothing', row one of the instrument failure table."""
    loop = inspect.getsource(mpn.MasterPoseNode.read_serial)
    assert "self.smooth_tip(" in loop
    assert 'get_parameter("ema_alpha")' not in loop, (
        "read_serial still applies its own alpha beside the smoother")
    clutch = inspect.getsource(mpn.MasterPoseNode.update_clutch)
    assert "self.reset_tip_filter(" in clutch
    init = inspect.getsource(mpn.MasterPoseNode.__init__)
    assert "self.tip_filter_sync()" in init


# ---------------------------------------------- the clutch pin, made LIVE
#
# The trajectory capture gates each segment on the button of the arm being
# moved, which is what an operator holding one arm can actually reach. That
# is only safe while the button cannot toggle that arm's clutch, so the
# capture pins the clutch with `force_clutch_engaged` for its duration.
#
# The pin has to be honoured WHILE THE NODE IS RUNNING. Until 2026-08-30 both
# clutch parameters were copied into attributes in __init__ and never looked
# at again, so `ros2 param set` reported success and changed nothing -- and
# the capture's confirmation would have been a parameter read agreeing with
# itself. These exercise the shipped `update_clutch`.

def test_forcing_the_clutch_takes_effect_without_a_restart():
    n = rig(clutch_enabled=True, force_clutch_engaged=False)
    n.clutch_on["left"] = True
    n.btn_last["left"] = 0.0
    # a press, unpinned: the clutch toggles OUT -- the 2026-08-06 defect
    n.update_clutch("left", (0.0, 1.0), True, np.zeros(3))
    assert n.clutch_on["left"] is False, \
        "an unpinned own-arm press no longer disengages the clutch; if that " \
        "is a deliberate change the capture's pin can be simplified, but it " \
        "is the premise the pin exists for"
    # now pin it, the way `ros2 param set` would, mid-run
    n.params["force_clutch_engaged"] = True
    n.update_clutch("left", (0.0, 0.0), True, np.zeros(3))
    assert n.clutch_on["left"] is True, \
        "force_clutch_engaged was set at runtime and the clutch stayed out " \
        "-- the parameter is read once at construction again"
    # and further presses do nothing at all
    for val in (1.0, 0.0, 1.0):
        n.update_clutch("left", (0.0, val), True, np.zeros(3))
        assert n.clutch_on["left"] is True, \
            "a button press moved a PINNED clutch"


def test_the_pin_does_not_silence_the_buttons_it_ignores():
    """The capture reads the SAME buttons the pin tells the clutch to ignore.

    If pinning also stopped /master_fsr_buttons being published the gate
    would wait forever, and the operator would be pressing a button on a rig
    that had gone deaf with nothing on screen saying so. The publish must sit
    OUTSIDE the clutch logic, not inside a branch it can take.
    """
    src = inspect.getsource(mpn.MasterPoseNode.read_loop) \
        if hasattr(mpn.MasterPoseNode, "read_loop") \
        else inspect.getsource(mpn)
    i = src.index("self.fsr_btn_pub.publish(")
    before = src[:i]
    assert "force_clutch" not in before.rsplit("fsr_btn = ", 1)[-1], \
        "the button publish is downstream of the clutch pin"
    # and it precedes update_clutch, so no clutch branch can skip it
    j = src.find("update_clutch(", i)
    assert j > i, \
        "the buttons are published after the clutch update, so a return " \
        "inside the clutch logic can suppress them"


def test_disabling_the_clutch_at_runtime_is_also_live():
    """The other half of the same defect, and the one a lab-day operator hits
    first: `clutch_enabled:=false` to rule the clutch out as a cause."""
    n = rig(clutch_enabled=True, force_clutch_engaged=False)
    n.clutch_on["right"] = True
    n.btn_last["right"] = 0.0
    n.params["clutch_enabled"] = False
    n.update_clutch("right", (1.0, 0.0), True, np.zeros(3))
    assert n.clutch_on["right"] is True, \
        "clutch_enabled=false was set at runtime and a press still toggled"
