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
                 note="", disabled_reason=None, motion_generator=False):
        self.key = key
        self.label = label
        self.group = group
        self.argv = list(argv)
        self.starts_stack = starts_stack
        self.needs_stack = needs_stack
        self.needs_teensy = needs_teensy
        self.needs_real = needs_real
        self.note = note
        # Does this entry launch the followers, i.e. can it carry a
        # `motion_generator:=` argument. Declared per spec rather than guessed
        # from the argv, because `ros2 launch` fails outright on an argument
        # the launch file does not declare -- a button that appends one to a
        # stack that cannot take it does not degrade, it dies.
        self.motion_generator = motion_generator
        # Set by validate(), or given up front for a thing we KNOW cannot run.
        self.disabled_reason = disabled_reason

    @property
    def enabled(self):
        return self.disabled_reason is None

    def with_argv(self, argv):
        """A COPY of this spec carrying a different command line.

        For the things that are per-RUN rather than per-BUTTON: the sentence
        the operator typed, and which table stage 2 drew. The sweep presses
        the real button -- that is the point of driving it through
        `Gui.on_launch` -- and mutating the shared manifest entry instead
        would leak one run's instruction into the next one's clip.
        """
        out = Spec(self.key, self.label, self.group, argv,
                   self.starts_stack, self.needs_stack, self.needs_teensy,
                   self.needs_real, self.note, self.disabled_reason,
                   self.motion_generator)
        return out

    def __repr__(self):
        return "Spec(%s, %s)" % (self.key, "ok" if self.enabled else "DISABLED")


def _sh(*parts):
    return [os.path.join(WS, "scripts", parts[0])] + list(parts[1:])


def have_adb():
    """Is there an adb this machine can actually run?

    Asked of the filesystem, not of the platform. `vr_connect.sh` searches
    WSL's PATH and four Windows-side install locations; this mirrors that
    list, because a route is available exactly when the script that takes it
    can find its tool.
    """
    if any(os.access(os.path.join(d, "adb"), os.X_OK)
           for d in os.environ.get("PATH", "").split(os.pathsep) if d):
        return True
    user = os.environ.get("USER", "")
    for c in ("/mnt/c/platform-tools/adb.exe",
              "/mnt/c/Users/%s/AppData/Local/Android/Sdk/platform-tools/"
              "adb.exe" % user,
              "/mnt/c/Users/Gausms/AppData/Local/Android/Sdk/platform-tools/"
              "adb.exe",
              "/mnt/c/Program Files/platform-tools/adb.exe"):
        if os.access(c, os.X_OK):
            return True
    return False


def _vr_spec():
    """The VR transport spec, on the route this machine can take.

    See the comment at the call site. USB needs adb; wifi needs neither adb
    nor a sideloaded client, and is the route verified end to end here.
    """
    if have_adb():
        return Spec("vr", "VR link (Quest, USB cable)", "mode",
                    _sh("vr_connect.sh"), needs_stack=True,
                    note="adb found: reverse tunnel over USB to the "
                         "sideloaded client")
    return Spec("vr", "VR link (Quest, over wifi)", "mode",
                _sh("start_vr_wifi.sh"), needs_stack=True,
                note="no adb on this host, so the WEBXR route: open "
                     "https://<this machine>:8765/ in the headset's own "
                     "browser. Starts the bridge, the mapper, the gripper, "
                     "the safety node and feedback")


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
            # KEY FAMILIES: a|b|c (the A/B/C set), m\d (the MSc four --
            # m0-m3, displayed T0-T3), and t\d|e\d which exist only so the
            # ARCHIVED sets are recognised as keys and then dropped by the
            # refusal test above. A family the dispatcher gains must be added
            # here or its buttons render disabled with a message that is
            # wrong: the script does accept them.
            # STAGE SUFFIX: `m1s2` is T1 stage 2 -- the same task run with
            # both arms at once. It is a distinct dispatcher key, not a
            # variant of m1, because the two stages are separate cells in the
            # session plan and must be separately launchable. Without the
            # optional `s\d` here the dispatcher accepted m1s2 while every
            # one of its five buttons rendered DISABLED with the message
            # "run_experiment.sh does not accept 'm1s2'" -- a refusal that
            # was simply false, and which cost a recording run to diagnose
            # because the sweep's teardown crashed before printing it.
            if re.fullmatch(r"[temd]\d(s\d)?|[abc]", t):
                ok.add(t)
    ok -= {"A", "B", "C"}
    return ok


# use_rviz:=false ON EVERY STACK THE WINDOW LAUNCHES. The launch files
# default to starting their OWN RViz, so a GUI-launched stack opened a
# SECOND RViz window on top of the operations window (measured live,
# 2026-08-27: pid 172013 moveit.rviz over pid 168895 embedded) -- the
# operator reported "there is no rviz" while looking at the window the
# extra RViz had covered. The GUI provides the one RViz; a terminal
# launch keeps its own (the default is unchanged there).
MODES = [
    Spec("sim", "Simulation -- you drive with the master arm", "mode",
         _sh("run_teleop.sh", "gate:=false", "use_rviz:=false"),
         starts_stack=True,
         motion_generator=True,
         note="MoveIt, RViz, master_pose_node, both followers, e-stop"),
    # THE TELEOP STACK WITHOUT THE MASTER ARM, for the modes whose operator
    # is holding VR controllers. Same reasoning as `autonomy_nomaster` below:
    # `master_pose_node` is respawn=True, so with no Teensy on the bus it
    # dies and respawns for ever, and five live instances starved the
    # controller manager until joint_state_broadcaster never came up. Mode 1
    # keeps `sim` because the master arm IS its input; mode 2 must not.
    Spec("sim_nomaster", "Simulation -- ready for VR (no master arm)", "mode",
         _sh("run_teleop.sh", "gate:=false", "master:=false",
             "use_rviz:=false"),
         starts_stack=True, motion_generator=True,
         note="mode 2: MoveIt, RViz, both followers, e-stop -- with "
              "master_pose_node left out so an absent Teensy cannot starve "
              "the controllers"),
    Spec("autonomy", "Simulation + robot assistance (master arm)", "mode",
         _sh("run_autonomy.sh", "use_rviz:=false"),
         starts_stack=True, motion_generator=True,
         note="mode 3: adds vision, grasp generation, the arbiter, over the "
              "MASTER arm"),
    # THE SAME STACK WITHOUT THE MASTER ARM.
    #
    # Full autonomy and VR-shared never read master_pose_node, and that node
    # is respawn=True: with no Teensy on the bus it dies every 5 s for ever.
    # Measured 2026-08-26 -- five live instances, load average 8-11, the
    # controller manager overrunning, every spawner timing out, and
    # joint_state_broadcaster never coming up. /joint_states then never
    # published and the real-arm cascade refused with "Is the sim controller
    # up?", which points at the simulation when the cause is a missing cable.
    Spec("autonomy_nomaster", "Simulation + robot assistance (VR / autonomy)", "mode",
         _sh("run_autonomy.sh", "master:=false", "use_rviz:=false"),
         starts_stack=True,
         motion_generator=True,
         note="modes 4 and 6: the autonomy stack with master_pose_node left "
              "out, so an absent Teensy cannot starve the controllers"),
    Spec("real", "CONNECT THE REAL ARMS", "mode",
         _sh("start_real.sh"), needs_stack=True, needs_real=True,
         note="refuses unless the sim stack is up and homing succeeds"),
    # THE SAME END STATE WHEN THE ARMS ARE ALREADY CONNECTED.
    #
    # `real` above brings the real side up from nothing, so it starts a
    # `kortex_highlevel_bridge` per arm -- and the arm permits exactly ONE
    # session (HARD CONSTRAINT 2). Against a rig whose bridges are already
    # connected the second session is refused, the launch fails, and its own
    # cleanup sweeps the WORKING bridges as orphans: pressing the button
    # labelled "connect the real arms" DISCONNECTS them. `start_mode`
    # substitutes this entry whenever it finds a live bridge, so the same
    # SIM + REAL button reaches the same end state either way.
    Spec("real_cascade", "Relay onto the already-connected arms", "mode",
         _sh("start_cascade.sh"), needs_stack=True, needs_real=True,
         note="relays the sim onto the Kortex sessions already open -- opens "
              "no new one"),
    Spec("real_mock", "Rehearse the real-arm sequence (mock hardware)", "mode",
         _sh("start_real.sh", "--mock"), needs_stack=True,
         note="the identical sequence against mock_real.launch.py"),
    # THE VR TRANSPORT, BY WHICHEVER ROUTE THIS MACHINE CAN ACTUALLY TAKE.
    #
    # `vr_connect.sh` is route A: a sideloaded Unity client reaching
    # ws://127.0.0.1:8766 through `adb reverse` over USB. It dies on its
    # first line with "adb is not installed" on a host without Android
    # platform-tools -- which is this one, and which is why docs/ENGINEERING_LOG.md records
    # "no adb" under Blocked on the lab. The button therefore appeared to do
    # nothing: the process spawned, refused, and exited before anything drew.
    #
    # Route B is `start_vr_wifi.sh`: WebXR in the headset's own browser, page
    # and socket from one TLS origin, no sideloading and no account. It is
    # the route that has actually worked here (2026-08-19, verified end to
    # end). So the spec RESOLVES the route at import instead of hard-coding
    # the one that cannot run, and says which it picked.
    _vr_spec(),
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
    # THE UNIFIED VISION LAYER (2026-08-27). Every camera the rig models --
    # both wrist RGB-D, the RealSense, the USB scene camera -- through one
    # node that detects objects per camera and REFUSES by name for a camera
    # that has never spoken. Its health line is in the vision panel.
    Spec("scene_cv", "Scene vision (all cameras)", "diag",
         ["ros2", "run", "srl_perception", "scene_understanding_node"],
         note="one node, four cameras: plane, objects, colours, per-camera "
              "health on /perception/scene/objects. Works camera by camera; "
              "a camera with no frames is named, not invented"),
    # THE GRIPPER CAMERAS, WHICH HAD NO ENTRY ANYWHERE.
    #
    # `grep kinova_vision` over this file and srl_gui.py returned nothing on
    # 2026-08-30: the wrist cameras came up only when somebody typed the
    # launch, with six frame-id arguments, into a terminal. The operator's
    # report was "the gripper cameras are still not working" -- they were
    # never started. THE GUI RULE: a capability reachable only by typing is
    # one the person running the session does not have.
    #
    # These are Kinova's own cameras, on the ARM's network address. Nothing
    # to do with usbipd, nothing to do with /dev/video*, and they cannot come
    # up while the arm is off the network -- which the script says by name
    # rather than letting a launch time out in a log nobody reads.
    Spec("wrist_cams", "Gripper cameras (both wrists)", "diag",
         _sh("wrist_cameras.sh"),
         note="the cameras ON the arms, over the robot's network -- not USB, "
              "and not affected by usbipd. Needs the arm powered and "
              "reachable"),
    Spec("map_obstacles", "Planner avoids what the cameras saw", "diag",
         ["ros2", "run", "srl_perception", "map_obstacles_node"],
         note="feeds measured objects to MoveIt as mapped_* obstacles. "
              "Add-or-grow only: nothing it saw once is silently removed"),
]


def task_mode_locks():
    """{dispatcher key: (modes it will run under)} for every task that
    restricts them, read from the task specification itself.

    WHY THIS IS PARSED AND NOT LISTED HERE. T1 was rebuilt for
    `06_full_autonomy` alone -- it commands its own approach orientation
    instead of the pinned anchor -- and `run_abc` REFUSES it under any other
    mode, by name and with a reason. The GUI did not know that, so it drew
    four buttons per stage that could only ever exit 1: exactly the
    "button that exits 2 on press" failure this whole manifest exists to
    prevent, reintroduced from the other end.

    Read from `msc_clip_tasks.TASKS[..]["modes"]`, which is what the runner
    reads, so a task that gains or loses a restriction cannot leave a stale
    copy behind here.
    """
    import sys
    d = os.path.join(WS, "src/srl_experiments/experiments/abc")
    if d not in sys.path:
        sys.path.insert(0, d)
    try:
        import msc_clip_tasks as MCT
    except Exception:                                         # noqa: BLE001
        return {}
    # The dispatcher key is m0..m3 / m1s2; the task key is t0..t3 / t1s2. The
    # mapping is `run_abc._MSC_KEY` inverted, written explicitly rather than
    # as "t" + key[1], which silently maps m1s2 to t1.
    back = {"t0": "m0", "t1": "m1", "t1s2": "m1s2", "t2": "m2", "t3": "m3"}
    out = {}
    for tk, spec in MCT.TASKS.items():
        modes = spec.get("modes")
        if modes and tk in back:
            out[back[tk]] = tuple(modes)
    return out


def task_specs():
    """One button per task the dispatcher accepts, plus the A/B/C row."""
    accepted = dispatcher_tasks()
    locks = task_mode_locks()
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
    # ---------------------------------------------------------------- MSc
    # The four MSc tasks, keyed m0-m3 in the dispatcher and displayed T0-T3.
    # They are NOT keyed t0-t3: run_experiment.sh refuses t1..t9 BY NAME as
    # the archived 300/310 mm set, and a dispatcher that took `t3` would have
    # to pick one of two different T3s while silently orphaning the other.
    for k, lab in (("m0", "T0 target reaching"),
                   ("m1", "T1 pick and place"),
                   ("m1s2", "T1 stage 2, both arms"),
                   ("m2", "T2 coordinated carry"),
                   ("m3", "T3 circuit box + multimeter")):
        for mode in ("01_master_teleop", "02_vr_teleop",
                     "03_shared_autonomy", "04_vr_shared",
                     "06_full_autonomy"):
            short = mode.split("_", 1)[1].replace("_", " ")
            # T1 LOOKS BEFORE IT GRASPS, AND THE BUTTON SAYS SO.
            #
            # T1 is a COLOUR-MATCHED task and its colour used to be read from
            # T1_PAIR -- a declaration -- so every clip of it was produced
            # without a camera being consulted. With --vision, run_abc moves
            # to the observe pose, detects each cube, and builds the SAME path
            # from the seen position and the seen colour.
            #
            # IT REFUSES RATHER THAN FALLING BACK if no camera is publishing,
            # which is deliberate: a silent revert to the declared coordinate
            # is indistinguishable from a working perception path. The sweep
            # therefore starts `mock_rgbd_camera` for this task, and on real
            # hardware the wrist camera has to be streaming.
            #
            # T1 LOOKS BEFORE IT GRASPS, AND THE LOOK HAPPENS IN STAGING.
            #
            # THE ONE-SOURCE RULE IS WHY. `vision_grasp.observe_and_detect` commands the observe
            # pose by publishing a JointTrajectory to
            # /<arm>_arm_controller/joint_trajectory -- which is the topic
            # `ik_follower_node` publishes to (ik_follower_node.py:260). Under
            # a mode the follower owns that topic, so the observe move and the
            # follower fight for the arm. Measured: through the sweep the cell
            # recorded 8.90 m of travel, placed 101 mm from target and took no
            # grasp at all; run standalone with master_pose_node also live it
            # recorded 0.0000 m, because the follower was tracking the master
            # rather than the runner.
            #
            # So the look runs in STAGING, next to stage_presentation_pose.py
            # which the sweep already drives as a subprocess before pressing
            # the button. `stage_observe_and_detect.py` looks, detects, returns
            # the arm home and writes the cubes; --vision READS that file and
            # does no arm motion at all, so the follower stays the only
            # publisher on the arm controller.
            #
            # BOTH STAGES GET IT SINCE 2026-08-18. Stage 2 draws its layout
            # from the trial's seed, and the staged look now takes the seed
            # too, so the detections and the run are of the same table -- the
            # runner refuses a detection file whose seed is not its own.
            _extra = (["--vision", DETECTIONS_FILE]
                      if k in ("m1", "m1s2") else [])
            out.append(Spec(
                "msc_%s_%s" % (k, mode),
                "%s  [%s]" % (lab, short), "task",
                _sh("run_experiment.sh", k, "--mode", mode, "--taskset",
                    "msc", "--participant", "PILOT", "--scripted", *_extra),
                needs_stack=True,
                disabled_reason=(
                    ("run_experiment.sh does not accept %r -- this button "
                     "would exit 2" % k) if k not in accepted else
                    # THE TASK'S OWN MODE RESTRICTION, ON THE BUTTON.
                    # Pressing this would have exited 1 with a refusal nobody
                    # sees, because a launched job's output goes to a file.
                    ("this task runs under %s only -- it commands its own "
                     "approach orientation rather than the pinned anchor "
                     "every teleop mode sends, so a run under %s would be "
                     "measuring a different geometry under a mode's name"
                     % (" and ".join(locks[k]), short))
                    if k in locks and mode not in locks[k] else None),
                note="MSc set; coordinates verified N=10 over the full "
                     "densified CLIP path with the bench in scene"))

    # ------------------------------------------------------------- DEMO
    # THE CHOREOGRAPHED ROUTINES. Buttons because the sweep launches through
    # these specs, so a routine with no spec cannot be filmed -- which is
    # exactly how the first demo sweep failed, with "no GUI spec
    # 'demo_d1_06_full_autonomy'" on all three routines.
    #
    # They are tagged "demo", not "task", so nothing that enumerates tasks
    # picks them up: they produce no trial data and must never be counted as
    # a condition.
    for k, lab in (("d1", "Dance: flow (canon)"),
                   ("d2", "Dance: pulse (opposition)"),
                   ("d3", "Dance: play (call/response)")):
        for mode in ("01_master_teleop", "02_vr_teleop",
                     "03_shared_autonomy", "04_vr_shared",
                     "06_full_autonomy"):
            short = mode.split("_", 1)[1].replace("_", " ")
            out.append(Spec(
                "demo_%s_%s" % (k, mode),
                "%s  [%s]" % (lab, short), "demo",
                _sh("run_experiment.sh", k, "--mode", mode, "--taskset",
                    "demo", "--participant", "DEMO", "--scripted"),
                needs_stack=True,
                disabled_reason=(
                    None if k in accepted else
                    "run_experiment.sh does not accept %r -- this button "
                    "would exit 2" % k),
                note="DEMONSTRATION ONLY -- no trial data, no condition, no "
                     "metric. Every waypoint collision-checked against the "
                     "wearer at N=10 (2420 IK calls, 0 failures)"))

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


# Where the staged detection lands. One path, so the stage step and the run
# cannot disagree about which file carries the cubes.
DETECTIONS_FILE = "/tmp/srl_t1_detections.json"


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
