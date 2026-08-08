#!/usr/bin/env python3
"""Find resources that COLLIDE when two real arms are instantiated.

    python3 scripts/audit_dual_arm_collisions.py

WHY THIS EXISTS. ros2_control resource names must be unique across the whole
robot, but the vendor macros are written for ONE arm and hardcode several
names. Instantiate the macro twice and both components register the same
name; the second to load exports NOTHING, and the symptom is remote from the
cause -- an interface showing `[unavailable] [unclaimed]` while its twin is
`[available] [claimed]`, and a controller that cannot activate.

Four have been found by hand so far, each costing a debugging session:

  1. gripper hardware component NAME          (fixed: ${prefix} in the macro)
  2. `reactivate_gripper` GPIO                (fixed: patches/0003)
  3. gripper `COM_port` default /dev/ttyUSB0  (fixed: per-arm xacro args)
  4. `tcp/twist.*` + `reset_fault/*`          (fixed: patches/0001, 0002)

Finding the fifth by hand is not a plan. This renders the REAL dual-arm URDF,
extracts every ros2_control resource, and reports any name claimed by more
than one hardware component -- plus any hardware <param> whose value is
identical across components and looks like a device path or address, which is
how the COM_port collision presented (two drivers fighting over ttyUSB0).

It is a static check on the URDF, so it needs no hardware and no running
stack. Exit code is non-zero if anything collides.
"""
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "src/srl_description/urdf/srl_dual.urdf.xacro"

# Values that are SUPPOSED to be shared between the two components.
BENIGN_SHARED = {"username", "password", "port", "port_realtime",
                 "session_inactivity_timeout_ms",
                 "connection_inactivity_timeout_ms",
                 "use_internal_bus_gripper_comm", "state_following_offset",
                 "fake_sensor_commands", "mock_sensor_commands",
                 "gripper_max_velocity", "gripper_max_force",
                 "gripper_speed_multiplier", "gripper_force_multiplier",
                 "gripper_max_speed", "gripper_closed_position", "prefix",
                 "joint_commands_topic", "joint_states_topic"}

# A shared value for one of these is a genuine conflict: two components
# cannot own the same physical device or network endpoint.
DEVICEY = ("port", "ip", "com", "device", "tty", "address", "serial")


def render(real=True):
    args = ["xacro", str(XACRO)]
    if real:
        args += ["use_fake_hardware:=false", "sim_gazebo:=false",
                 "sim_isaac:=false"]
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        print("xacro FAILED:\n" + r.stderr[-2000:], file=sys.stderr)
        sys.exit(3)
    return r.stdout


def main():
    urdf = render(real=True)
    root = ET.fromstring(urdf)

    owners = defaultdict(set)      # "resource/interface" -> {component}
    res_owners = defaultdict(set)  # "resource"           -> {component}
    params = defaultdict(list)     # param name -> [(component, value)]
    comps = []

    for rc in root.iter("ros2_control"):
        comp = rc.get("name")
        comps.append(comp)
        for hw in rc.iter("hardware"):
            for p in hw.iter("param"):
                params[p.get("name")].append((comp, (p.text or "").strip()))
        for tag in ("joint", "gpio", "sensor"):
            for el in rc.iter(tag):
                res = el.get("name")
                res_owners[res].add(comp)
                for kind in ("command_interface", "state_interface"):
                    for itf in el.iter(kind):
                        owners["%s/%s" % (res, itf.get("name"))].add(comp)

    print("DUAL-ARM RESOURCE COLLISION AUDIT")
    print("rendered %s with use_fake_hardware:=false" % XACRO.name)
    print("hardware components: %d" % len(comps))
    for c in comps:
        print("    %s" % c)
    print()

    bad = 0

    dup_res = {r: sorted(o) for r, o in res_owners.items() if len(o) > 1}
    print("A. RESOURCES claimed by more than one component")
    if dup_res:
        bad += len(dup_res)
        for r, o in sorted(dup_res.items()):
            print("   COLLISION  %-34s claimed by %s" % (r, ", ".join(o)))
    else:
        print("   none")
    print()

    dup_itf = {k: sorted(o) for k, o in owners.items() if len(o) > 1}
    print("B. INTERFACES claimed by more than one component")
    if dup_itf:
        bad += len(dup_itf)
        for k, o in sorted(dup_itf.items()):
            print("   COLLISION  %-34s claimed by %s" % (k, ", ".join(o)))
    else:
        print("   none")
    print()

    print("C. HARDWARE PARAMS sharing a value that names a DEVICE or ENDPOINT")
    shared = []
    for name, entries in sorted(params.items()):
        if name in BENIGN_SHARED or len(entries) < 2:
            continue
        vals = {v for _, v in entries}
        if len(vals) > 1:
            continue
        if not any(d in name.lower() for d in DEVICEY):
            continue
        shared.append((name, entries[0][1], [c for c, _ in entries]))
    if shared:
        bad += len(shared)
        for name, val, cs in shared:
            print("   CONFLICT   %-22s = %-18s in %s" % (name, val, ", ".join(cs)))
    else:
        print("   none")
    print()

    # Informational: which resources ARE prefixed, so a reviewer can see the
    # fix took rather than trusting the absence of a collision.
    pref = sorted(r for r in res_owners if r.split("_")[0] in ("left", "right"))
    print("D. prefixed resources seen (%d): %s%s"
          % (len(pref), ", ".join(pref[:8]), " ..." if len(pref) > 8 else ""))
    print()

    # E. C++-EXPORTED resources. The URDF cannot show these, and the bug that
    # motivated this script lived here: "tcp" and "reset_fault" were string
    # literals in the driver, invisible to any URDF-level check. Any literal
    # resource name in a hardware component is a dual-arm collision unless it
    # is composed with a prefix.
    print("E. C++ hardware components exporting LITERAL resource names")
    import re as _re
    srcs = list((ROOT / "src").glob("ros2_*/**/*.cpp")) + \
        list((ROOT / "src").glob("ros2_*/**/*.hpp"))
    pat = _re.compile(r'(?:Command|State)Interface\(\s*"([^"]+)"')
    hits = []
    for f in srcs:
        try:
            txt = f.read_text()
        except (OSError, UnicodeDecodeError):
            continue
        for m in pat.finditer(txt):
            line = txt[:m.start()].count("\n") + 1
            hits.append((f.relative_to(ROOT), line, m.group(1)))
    if hits:
        bad += len(hits)
        for f, line, name in hits:
            print("   LITERAL    %-28s %s:%d" % (name, f, line))
        print("   Each is exported identically by BOTH arms. Compose it with a")
        print("   resource prefix, as patches/0001 does for tcp/reset_fault.")
    else:
        print("   none -- every exported resource name is composed, not literal")
    print()

    if bad:
        print("FAIL: %d collision(s). On real hardware the SECOND component to "
              "load will export nothing, and the symptom will appear far from "
              "this cause." % bad)
    else:
        print("PASS: no resource, interface or device-parameter collisions "
              "between the two arms.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
