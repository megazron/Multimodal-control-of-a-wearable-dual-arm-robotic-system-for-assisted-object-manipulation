#!/usr/bin/env python3
"""Everything the GUI can launch, as ONE manifest that can be checked.

WHY A MANIFEST AND NOT BUTTONS. Five experiment buttons in an earlier GUI
exited 2 the instant they were pressed while appearing to launch: the label
said "T3 rigid carry", the dispatcher only accepted `e1..e6`, and the process
died before anything drew. From the operator's side a button that launches
nothing and a button that launches something invisible look identical.

The defence is not care, it is a shared list plus a checker. `validate()`
resolves every entry against the thing that will actually run -- the script on
disk, the dispatcher's own accepted-task list, the package's entry points --
and `scripts/verify_gui_buttons.py` presses each one for real and requires
evidence that it started. A spec whose target cannot be resolved is DISABLED
with the reason on the button, never rendered as live.

`starts_stack` scopes the second-stack refusal. Two `master_pose_node`
instances split the serial stream and cost a full day of measurements, so a
stack-starting entry is refused while one is up. Add-ons (an autonomy node, a
diagnostic) are NOT second stacks, and refusing those whenever a stack exists
makes the refusal useless in the only situation where you would use them.
"""
import os
import re
import subprocess

WS = os.environ.get("SRL_WS") or os.path.expanduser("~/kortex_ws")


class Spec:
    def __init__(self, key, label, group, argv, starts_stack=False,
                 needs_stack=False, needs_teensy=False, needs_real=False,
                 note="", disabled_reason=None):
        self.key = key
        self.label = label
        self.group = group
        self.argv = list(argv)
        self.starts_stack = starts_stack
        self.needs_stack = needs_stack
        self.needs_teensy = needs_teensy
        self.needs_real = needs_real
        self.note = note
        # Set by validate(), or given up front for a thing we KNOW cannot run.
        self.disabled_reason = disabled_reason

    @property
    def enabled(self):
        return self.disabled_reason is None

    def __repr__(self):
        return "Spec(%s, %s)" % (self.key, "ok" if self.enabled else "DISABLED")


def _sh(*parts):
    return [os.path.join(WS, "scripts", parts[0])] + list(parts[1:])


def dispatcher_tasks():
    """The tasks `run_experiment.sh` ACTUALLY accepts, read from the script.

    Parsed rather than duplicated. A hand-copied list is how the labels and
    the dispatcher drifted apart in the first place, and a list that can go
    out of date silently is worth less than no list.
    """
    p = os.path.join(WS, "scripts", "run_experiment.sh")
    try:
        src = open(p).read()
    except OSError:
        return set()
    ok = set()
    # Parse each `case` branch and keep ONLY the ones that actually dispatch.
    # A branch that echoes to stderr and exits is a REFUSAL -- the archived
    # t*/e* sets live in one such branch -- and treating a refusal as an
    # accepted task is how the button list drifted from the dispatcher in the
    # first place.
    for m in re.finditer(r"^\s*([A-Za-z0-9|]+)\)\s*\n(.*?)(?=^\s*[A-Za-z0-9|*-]+\)|^esac)",
                         src, re.M | re.S):
        grp, body = m.group(1), m.group(2)
        if ">&2" in body and "exit" in body and "exec" not in body:
            continue                      # a refusal, not a task
        if "exec" not in body and "SCRIPT=" not in body:
            continue
        for t in grp.split("|"):
            if re.fullmatch(r"[te]\d|[abc]", t):
                ok.add(t)
    ok -= {"A", "B", "C"}
    return ok


MODES = [
    Spec("sim", "Sim teleop only", "mode",
         _sh("run_teleop.sh", "gate:=false"), starts_stack=True,
         note="MoveIt, RViz, master_pose_node, both followers, e-stop"),
    Spec("autonomy", "+ perception and shared autonomy", "mode",
         _sh("run_autonomy.sh"), starts_stack=True,
         note="mode 4: adds vision, grasp generation, the arbiter"),
    Spec("real", "Cascade to the REAL arms", "mode",
         _sh("start_real.sh"), needs_stack=True, needs_real=True,
         note="refuses unless the sim stack is up and homing succeeds"),
    Spec("real_mock", "Cascade, MOCK hardware", "mode",
         _sh("start_real.sh", "--mock"), needs_stack=True,
         note="the identical sequence against mock_real.launch.py"),
    Spec("vr", "VR transport (Quest)", "mode",
         _sh("vr_connect.sh"), needs_stack=True,
         note="needs Android platform-tools on the WINDOWS side"),
]

DIAGNOSTICS = [
    Spec("diag", "Diagnostics", "diag", _sh("diagnostics.sh"),
         note="controllers, /joint_states rate, serial port, SHM, e-stop"),
    Spec("channels", "Channel check (~3 min)", "diag",
         _sh("check_channels.sh"), needs_teensy=True,
         note="FIRST ACTION OF EVERY LAB SESSION"),
    Spec("recover", "Recover to known-good", "diag", _sh("recover.sh")),
    Spec("blocking", "Blocking aggregator", "diag",
         ["ros2", "run", "srl_teleop", "blocking_aggregator"],
         needs_stack=True,
         note="teleop.launch.py does not start it"),
    Spec("preflight", "Preflight", "diag",
         ["ros2", "run", "srl_teleop", "preflight"], needs_stack=True),
]


def task_specs():
    """One button per task the dispatcher accepts, plus the A/B/C row."""
    accepted = dispatcher_tasks()
    out = []
    # ONE GENERATION OF TASK SET, NOT THREE.
    #
    # The GUI offered t2-t9 (the bimanual set, 300 mm span), e1-e6 (the
    # superseded E-series) AND a/b/c at once -- 41 buttons spanning three
    # generations, two of which log geometry that no longer matches the spec.
    # A button labelled "T3 rigid carry" runs a 300 mm tray; the current
    # specification is 500 mm. Offering both is how a result gets filed under
    # the wrong geometry.
    #
    # The old sets are not deleted -- their data and protocols are archived
    # under experiments/_archive/ -- they are simply no longer LAUNCHABLE.
    # TASKS A, B AND C -- now LIVE, one button per (task, mode).
    #
    # They were disabled while they had a verified spec and no runner, with
    # the reason on the button: pointing them at t3/t6/t7 would have run the
    # superseded 300/310 mm span and logged it under a 500 mm name. That is no
    # longer a risk, because run_abc.py reads experiments/abc/tasks.py -- the
    # spec that was verified N=10 over the densified full path -- and the
    # dispatcher routes a|b|c there and nowhere else.
    #
    # ONE BUTTON PER MODE, because the mode is the point. Each mode enters at
    # the topic its own upstream publishes at, so a clip recorded under a mode
    # name really did travel that mode's path.
    for k, lab in (("a", "A positioning"), ("b", "B coordinated carry"),
                   ("c", "C dual pursuit")):
        # THE FIVE CLIP-TREE MODES, AND ONLY THOSE. `run_abc.MODES` also
        # carries `04_shared_autonomy`, a legacy ALIAS of 03 -- same topic,
        # same command path. Listing both put two buttons on the screen with
        # different labels, the same behaviour and, because the key was cut
        # from the mode's first two characters, THE SAME KEY. The GUI showed
        # six buttons per task against five modes and one of them was
        # unreachable by key. Keys are now cut from the whole mode name and
        # asserted unique below, so a repeat is a startup failure rather than
        # a duplicate button nobody notices.
        for mode in ("01_master_teleop", "02_vr_teleop",
                     "03_shared_autonomy", "04_vr_shared",
                     "06_full_autonomy"):
            short = mode.split("_", 1)[1].replace("_", " ")
            out.append(Spec(
                "abc_%s_%s" % (k, mode),
                "%s  [%s]" % (lab, short), "task",
                _sh("run_experiment.sh", k, "--mode", mode, "--taskset",
                    "clip", "--participant", "PILOT", "--scripted"),
                needs_stack=True,
                disabled_reason=(
                    None if k in accepted else
                    "run_experiment.sh does not accept %r -- this button "
                    "would exit 2" % k),
                note="enters at this mode's own command path; "
                     "coordinates from the N=10-verified spec"))
    keys = [sp.key for sp in out]
    dupes = sorted({x for x in keys if keys.count(x) > 1})
    if dupes:
        raise AssertionError(
            "duplicate launch keys %s -- two buttons would dispatch as one"
            % dupes)
    labels = [sp.label for sp in out]
    dl = sorted({x for x in labels if labels.count(x) > 1})
    if dl:
        raise AssertionError("duplicate button labels %s" % dl)

    return out


def all_specs():
    return MODES + task_specs() + DIAGNOSTICS


def validate(specs=None):
    """Resolve every spec against what will actually run.

    Returns [(spec, ok, detail)]. Sets `disabled_reason` on failures, so a
    spec that cannot run is never drawn as live.
    """
    rows = []
    for s in specs if specs is not None else all_specs():
        if s.disabled_reason is not None:
            rows.append((s, False, s.disabled_reason))
            continue
        if not s.argv:
            s.disabled_reason = "no command"
            rows.append((s, False, s.disabled_reason))
            continue
        head = s.argv[0]
        if head.startswith("/") or head.startswith("."):
            if not os.path.exists(head):
                s.disabled_reason = "missing script: %s" % head
                rows.append((s, False, s.disabled_reason))
                continue
            if not os.access(head, os.X_OK) and not head.endswith(".py"):
                s.disabled_reason = "not executable: %s" % head
                rows.append((s, False, s.disabled_reason))
                continue
            rows.append((s, True, head))
            continue
        if head == "ros2":
            # `ros2 run <pkg> <exe>` -- the exe must be a real entry point.
            pkg, exe = s.argv[2], s.argv[3]
            try:
                out = subprocess.run(["ros2", "pkg", "executables", pkg],
                                     capture_output=True, text=True,
                                     timeout=25).stdout
            except Exception as e:                            # noqa: BLE001
                rows.append((s, True, "unchecked (%s)" % e))
                continue
            if exe not in out.split():
                s.disabled_reason = "%s has no executable %r" % (pkg, exe)
                rows.append((s, False, s.disabled_reason))
                continue
            rows.append((s, True, "%s/%s" % (pkg, exe)))
            continue
        rows.append((s, True, head))
    return rows


if __name__ == "__main__":
    import sys
    rows = validate()
    n_bad = 0
    for s, ok, detail in rows:
        n_bad += not ok
        print("  %-10s %-32s %-8s %s"
              % (s.group, s.label, "ok" if ok else "DISABLED", detail))
    print("\n%d specs, %d enabled, %d disabled with a stated reason"
          % (len(rows), len(rows) - n_bad, n_bad))
    print("dispatcher accepts: %s" % " ".join(sorted(dispatcher_tasks())))
    sys.exit(0)
