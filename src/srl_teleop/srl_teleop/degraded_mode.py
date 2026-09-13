#!/usr/bin/env python3
"""DEGRADED MODE -- run on the channels that still work, freeze the rest.

WHY THIS EXISTS, AND WHY IT IS NOT A FILTER
-------------------------------------------
7 of 14 master channels are INCOHERENT: over 5% of their *distinct* sensor
updates jump more than 60 deg between consecutive updates. At the sensor's
14.7-17 Hz that is ~2500 deg/s, faster than any hand can move a mannequin
arm. Those values are not noisy, they are WRONG -- consecutive samples are
unrelated to each other and to the joint's real angle.

A filter maps a signal to a smoothed version of itself. That helps when the
truth is buried in noise. It cannot help when the samples carry no
information about the truth: a low-pass over an incoherent channel produces a
smooth trajectory that is still unrelated to where the operator's arm is, and
it produces it CONFIDENTLY, which is worse than jitter because nothing
downstream can tell it is wrong. The sim is a relay; whatever enters it the
real arm reproduces a second later.

So the only correct handling of an incoherent channel is to STOP USING IT.

WHAT FREEZING MEANS
-------------------
An excluded channel is replaced, at parse time, by its ZERO-REFERENCE value
from `config/master_zero_<arm>.txt` -- the raw reading captured with the arm
hanging at rest. `PotCalibration.apply()` maps that to exactly 0 rad, so the
joint reads as "at its reference" rather than at some arbitrary angle. Three
consequences, all wanted:

  * the channel is constant, so it can never inject motion;
  * it never triggers the incoherence rejector or the frame validator, so a
    dead channel stops costing the OTHER channels their frames (a rejected
    frame freezes the WHOLE command, which is how one bad pot degrades
    everything);
  * the geometry stays self-consistent -- FK is evaluated at a real posture,
    not at a mixture of live and stale angles.

Freezing at 0.0 instead would be wrong: 0.0 raw is the firmware's dropout
clamp, and `zero_dropouts()` treats it as a fault marker.

WHAT IS DELIBERATELY NOT DONE HERE
----------------------------------
No interpolation, no extrapolation, no substitution from the other arm, no
model-based estimate of the frozen joint. Every one of those invents operator
intent that was never measured. The rig is bolted to a person; a confident
fabrication is the most dangerous failure mode available to it.
"""
import json
import os

# Verdicts from `channel_report.py`. ALIVE and INTERMITTENT are USABLE:
# INTERMITTENT means the channel drops out (exact 0.0), which the existing
# validator already handles correctly by rejecting the frame -- it is a
# *known* loss, not a plausible-looking wrong value.
USABLE_VERDICTS = ("ALIVE", "INTERMITTENT")
TOTAL_CHANNELS = 14
ARMS = ("left", "right")

# What losing each channel costs in SPHERICAL mode, which is the shipped
# position mode. Numbers are the measured pot-reduction ladder (docs/ENGINEERING_LOG.md,
# 20430 real frames): commanded master-tip error against the 4-pot reach.
CHANNEL_ROLE = {
    0: ("AZIMUTH (left/right)",
        "frozen -> NO lateral control on this arm; elevation and reach "
        "still respond"),
    1: ("reach, shoulder bend",
        "frozen -> reach loses the shoulder term; measured cost 0.029 m "
        "(left) / 0.027 m (right) mean tip error"),
    2: ("reach, upper-arm roll",
        "frozen -> negligible; already passed as 0 when it drops out. "
        "Measured cost 0.042 m (left) / 0.005 m (right)"),
    3: ("reach, ELBOW bend (dominant)",
        "frozen -> reach is nearly constant; this is the joint that sets "
        "how far the tip is from the shoulder"),
    4: ("wrist -- unused by spherical position",
        "frozen -> no effect on position; wrist roll unobservable"),
    5: ("wrist -- unused by spherical position",
        "frozen -> no effect on position; wrist pitch unobservable"),
    6: ("wrist -- unused by spherical position",
        "frozen -> no effect on position; wrist roll unobservable"),
}

# Channels spherical mode actually consumes for POSITION.
SPHERICAL_POSITION_IDX = (0, 1, 2, 3)


BASELINE_GLOB = "channels_*.json"


def baseline_dir(pkg_root=None):
    root = pkg_root or os.environ.get("SRL_WS", os.path.expanduser("~/kortex_ws"))
    return os.path.join(root, "recordings", "baselines")


def default_baseline_path(pkg_root=None):
    """The NEWEST channel baseline, not a hardcoded filename.

    THIS WAS THE BUG THAT MADE THE POT REPAIR INVISIBLE, and it was not the
    warning being too quiet -- it was the runtime reading a different file
    from the one the lab writes.

    `check_channels.sh` ends by telling the operator to save the new reference
    as `channels_$(date +%Y%m%d).json`, and it diffs against
    `ls -t channels_*.json | head -1`. This function used to return the
    literal string "channels_20260806.json". So a lab session could repair
    every pot, re-capture, save `channels_20260812.json`, watch the script
    confirm 14/14 -- and `master_pose_node` would go on reading the 08-06 file
    and freezing eight working channels for ever. Nothing anywhere would
    disagree out loud.

    Two "which baseline is current" rules in one repository is the drift class
    this project has met repeatedly. There is now one rule, and it is this
    one; the shell script's `ls -t` selects the same file.
    """
    d = baseline_dir(pkg_root)
    try:
        cands = [os.path.join(d, f) for f in os.listdir(d)
                 if f.startswith("channels_") and f.endswith(".json")]
    except OSError:
        cands = []
    if not cands:
        # Name the conventional path anyway, so the "no baseline" message can
        # say WHERE it looked rather than just that it failed.
        return os.path.join(d, "channels_20260806.json")
    return max(cands, key=os.path.getmtime)


def load_baseline(path):
    """Returns {} when the file is missing -- an absent baseline must not
    stop the node, it must only stop AUTO from claiming to know anything."""
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _key(arm, idx):
    return "%s_j%d" % (arm[0], idx + 1)


def verdict(baseline, arm, idx):
    return (baseline.get(_key(arm, idx)) or {}).get("verdict", "UNKNOWN")


def n_coherent(baseline):
    """How many of the 14 channels are usable. UNKNOWN counts as NOT."""
    return sum(1 for a in ARMS for i in range(7)
               if verdict(baseline, a, i) in USABLE_VERDICTS)


def excluded_channels(baseline):
    """{arm: [joint indices to freeze]}. Everything not USABLE is frozen."""
    return {a: [i for i in range(7)
                if verdict(baseline, a, i) not in USABLE_VERDICTS]
            for a in ARMS}


def baseline_age_days(path=None):
    """How old the stored channel baseline is, in days.

    THE BASELINE IS STATE FROM A PREVIOUS RUN, which is one of the four
    instrument-failure mechanisms this project has catalogued. It is a
    snapshot of channel health taken on one day, and nothing invalidates it
    when the hardware changes. Repair every pot on the bench and this file
    still says the arm is broken, so degraded mode still engages and still
    freezes eight working channels.
    """
    import time
    if path is None:
        path = default_baseline_path()
    if not path or not os.path.exists(path):
        return None
    return (time.time() - os.path.getmtime(path)) / 86400.0


def staleness_warning(baseline, path=None):
    """A sentence to log whenever the baseline is about to degrade the arm.

    AGE IS THE WRONG TRIGGER, and trying it first showed why. A baseline
    written three days ago is recent, and if the pots were repaired yesterday
    it is still completely wrong: what is stale is the CONTENT, not the
    timestamp, and no age threshold can see that. A file rewritten by a
    re-capture would also reset its own age and hide the problem it caused.

    So this fires whenever the stored file is about to freeze channels, every
    time, and names the file and its date so the reader can judge. Freezing
    working channels is worth one line of log unconditionally; a warning that
    only appears sometimes is one an operator learns to wait for.

    Returned rather than logged so this module stays free of ROS handles.
    """
    n = n_coherent(baseline)
    if n >= 12:
        return None                     # nothing is being frozen
    if path is None:
        path = default_baseline_path()
    age = baseline_age_days(path)
    when = ("%.1f days old" % age) if age is not None else "unknown age"
    return ("THE CHANNEL SET BELOW COMES FROM A FILE ON DISK, NOT FROM THIS "
            "ARM. %s (%s) says %d/%d coherent, so %d channels are about to be "
            "FROZEN. If any pot has been repaired since that file was "
            "written, this is freezing a channel that works and NOTHING WILL "
            "SAY SO. Re-capture: bash scripts/check_channels.sh  (~3 min, "
            "FIRST ACTION OF EVERY LAB SESSION). Override: "
            "degraded_mode:=off."
            % (os.path.basename(path or "?"), when, n, TOTAL_CHANNELS,
               TOTAL_CHANNELS - n))


def decide(baseline, mode, min_coherent=12):
    """(active, {arm: [frozen idx]}, reason).

    mode: "auto" | "on" | "off".

    AUTO is the default and engages degraded mode whenever fewer than
    `min_coherent` of the 14 channels pass. It deliberately engages on an
    EMPTY baseline too: not knowing the channel health is not evidence of
    health, and the failure this guards against is silent.
    """
    n = n_coherent(baseline)
    exc = excluded_channels(baseline)
    if mode == "off":
        return False, {a: [] for a in ARMS}, (
            "degraded_mode:=off -- ALL channels in use regardless of health "
            "(%d/%d coherent)" % (n, TOTAL_CHANNELS))
    if mode == "on":
        return True, exc, ("degraded_mode:=on -- forced (%d/%d coherent)"
                           % (n, TOTAL_CHANNELS))
    if not baseline:
        return True, {a: [] for a in ARMS}, (
            "degraded_mode:=auto and NO channel baseline was found. Engaging "
            "with nothing excluded: an unknown channel state is not a "
            "healthy one, but neither is it grounds for freezing a channel "
            "that may be fine. Run scripts/check_channels.sh.")
    if n >= min_coherent:
        return False, {a: [] for a in ARMS}, (
            "degraded_mode:=auto and %d/%d channels are coherent (>= %d) -- "
            "running on the FULL channel set."
            % (n, TOTAL_CHANNELS, min_coherent))
    return True, exc, (
        "degraded_mode:=auto and only %d/%d channels are coherent (< %d)."
        % (n, TOTAL_CHANNELS, min_coherent))


def position_capability(arm, frozen):
    """One line per arm: what position control survives the exclusions."""
    lost = [i for i in SPHERICAL_POSITION_IDX if i in frozen]
    if not lost:
        return "FULL spherical position (azimuth + elevation + reach)"
    bits = []
    bits.append("elevation LIVE (IMU gravity, unaffected by any pot)")
    bits.append("azimuth %s" % ("FROZEN -- no lateral control"
                                if 0 in frozen else "LIVE (j1)"))
    live_reach = [i for i in (1, 2, 3) if i not in frozen]
    if not live_reach:
        bits.append("reach FROZEN -- motion is confined to a spherical shell")
    elif 3 in frozen:
        bits.append("reach PARTIAL -- elbow (j4) frozen, only j%s contribute"
                    % "/".join(str(i + 1) for i in live_reach))
    else:
        bits.append("reach LIVE (j%s)"
                    % "/".join(str(i + 1) for i in live_reach))
    return "; ".join(bits)


def _wrap(text, width=68, indent="  "):
    out, line = [], indent
    for w in text.split():
        if len(line) + len(w) + 1 > width and line.strip():
            out.append(line)
            line = indent + w
        else:
            line = (line + " " + w) if line.strip() else (indent + w)
    if line.strip():
        out.append(line)
    return out


def banner(baseline, active, exc, reason, min_coherent=12, path=None):
    """The startup block. Loud on purpose: which channels are driving the
    robot is the single most load-bearing fact about a run, and it has
    historically been discoverable only by reading a log for rejection
    counts."""
    n = n_coherent(baseline)
    L = ["=" * 72,
         "MASTER CHANNEL SET -- %s"
         % ("DEGRADED MODE ACTIVE" if active else "full channel set"),
         reason, ""]
    # THE STALENESS WARNING LIVES INSIDE THE BANNER, not beside it.
    # It was previously a separate log call at a different point in startup,
    # and that call raised NameError on every single run for days -- swallowed
    # by a bare `except`, so the mitigation shipped, was verified in isolation,
    # and never once fired. A warning that has to be remembered separately is
    # a warning that can go missing separately.
    stale = staleness_warning(baseline, path) if active else None
    if stale:
        L += ["!" * 72] + _wrap(stale) + ["!" * 72, ""]
    for a in ARMS:
        frozen = exc.get(a, [])
        use = [i for i in range(7) if i not in frozen]
        L.append("  %-5s IN USE : %s" % (
            a, ", ".join("j%d(%s)" % (i + 1, verdict(baseline, a, i))
                         for i in use) or "NONE"))
        L.append("  %-5s FROZEN : %s" % (
            a, ", ".join("j%d(%s)" % (i + 1, verdict(baseline, a, i))
                         for i in frozen) or "none"))
        L.append("  %-5s -> %s" % ("", position_capability(a, frozen)))
        for i in frozen:
            role, cost = CHANNEL_ROLE[i]
            L.append("           j%d %-34s %s" % (i + 1, role, cost))
    L += ["",
          "  %d/%d channels coherent; degraded mode auto-clears at %d."
          % (n, TOTAL_CHANNELS, min_coherent),
          "  Re-test after any wiring work: bash scripts/check_channels.sh",
          "=" * 72]
    return "\n".join(L)


# ---------------------------------------------------------------- reach
# RADIAL FALLBACK.
#
# MEASURED, on 42953/43710 rows of the 2026-08-06 capture: regressing the
# true 4-pot reach on EVERY channel that still passes coherence, plus the
# IMU,
#
#     left   j1, j7 + IMU   R^2 = 0.133   residual RMS 51 mm  (93% of the
#                                          55 mm true spread) -> NO SIGNAL
#     right  j1,j2,j4,j6    R^2 = 0.986   residual RMS  9.1 mm -> RECOVERED
#
# So the left arm has no reach observable at all and the right arm needs no
# help. Physically that is expected: reach is set by shoulder flexion (j2)
# and elbow (j4); j1 and j7 are ROLLS about the arm's own axis and carry
# zero radial information, and one wrist IMU cannot see the elbow without a
# second IMU on the upper arm.
#
# The fallback therefore gives the operator a DELIBERATE radial input on a
# channel that is alive, coherent and otherwise unused. j7 (left) and j6
# (right) qualify: spherical position never reads them, so nothing is taken
# away, and unlike the FSR (the gripper) or the button (the clutch) there is
# no conflict -- the operator can extend while gripping and while indexing.
#
# RATE, not absolute. l_j7 spans only 26 deg, so an absolute map would give
# 26 deg of resolution over the whole radial range and would peg the reach
# to a wrist angle the operator must hold. A rate law turns a small range
# into unbounded travel and returns to zero when the wrist is neutral.
FALLBACK_PREFERENCE = (6, 5, 4)          # j7, then j6, then j5


def reach_is_observable(frozen):
    """Reach survives while at least one of j2/j4 is live. j3's measured
    contribution is 4.5 mm (right) and it is already passed as 0 when it
    drops out, so it does not count."""
    return not ({1, 3} <= set(frozen))


def pick_fallback_channel(baseline, arm, frozen):
    """Highest-preference channel that is USABLE and unused by position."""
    for i in FALLBACK_PREFERENCE:
        if i in frozen:
            continue
        if verdict(baseline, arm, i) in USABLE_VERDICTS:
            return i
    return None


def reach_fallback_plan(baseline, arm, frozen, mode="auto"):
    """(active, channel_index, reason)."""
    if mode == "off":
        return False, None, "reach_fallback:=off"
    obs = reach_is_observable(frozen)
    if mode == "auto" and obs:
        return False, None, (
            "reach is still observable on %s (j2 and/or j4 live) -- no "
            "fallback needed" % arm)
    ch = pick_fallback_channel(baseline, arm, frozen)
    if ch is None:
        return False, None, (
            "%s has NO reach signal and NO usable spare channel to put one "
            "on. Radial motion is unavailable until the wiring is repaired."
            % arm)
    return True, ch, (
        "%s reach is UNOBSERVABLE (j2 and j4 both frozen); radial control "
        "moved to j%d, which is coherent and unused by spherical position"
        % (arm, ch + 1))
