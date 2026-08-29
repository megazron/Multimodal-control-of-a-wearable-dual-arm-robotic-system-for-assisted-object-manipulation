#!/usr/bin/env python3
"""
clear_stale_dds.py -- remove ORPHANED FastDDS shared-memory segments.

    python3 scripts/clear_stale_dds.py            # report only
    python3 scripts/clear_stale_dds.py --delete   # remove the orphans

THE FAULT THIS FIXES
==========================================================================
Every node is up and healthy, publishing to each other perfectly -- and a
FRESH process sees an empty graph. `ros2 topic list` hangs. `ros2 topic hz`
reports nothing on a topic that is plainly flowing. This is recorded in
CLAUDE.md as R-9 and it is what made a previous session pin the arms to
ROS_DOMAIN_ID=7: switching domain gave a clean shared-memory namespace and
looked like a cure, so the workaround outlived its cause and split the system
in half for months (see scripts/bringup_arm.sh).

THE ACTUAL CAUSE is orphaned segments in /dev/shm. FastDDS creates a segment
per participant; a process that dies without cleaning up -- SIGKILL, a crash,
a killed launch -- leaves its segment behind. They accumulate, and discovery
for a NEW participant has to wade through all of them. Measured 2026-08-29
after a session of heavy process churn: 116 fastrtps segments, 37 referenced
by a live process, 79 orphaned. `ros2 topic list` timed out at 25 s while a
fresh pub/sub pair still exchanged 27 of 27 messages -- because that pair only
had to find EACH OTHER, not the polluted graph.

WHY THIS IS SAFE, AND HOW IT KNOWS
==========================================================================
A segment is deleted ONLY if no live process references it. Both /proc/PID/fd
and /proc/PID/maps are consulted, because FastDDS mmaps its segments and a
check on open descriptors alone would miss a mapping whose fd was closed --
and deleting a segment a running node is mapping is how you break the working
half of the stack while trying to fix the broken half.

It is therefore safe to run with the stack UP. Nothing is stopped, no arm is
touched, no session is taken.
"""
import argparse
import os
import re
import sys

SHM = "/dev/shm"
PAT = re.compile(r"^(fastrtps|fastdds)[A-Za-z0-9_.]*$")


def referenced_names():
    """Every shm object name a LIVE process has open or mapped."""
    names = set()
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        # open descriptors
        d = "/proc/%s/fd" % pid
        try:
            for fd in os.listdir(d):
                try:
                    t = os.readlink(os.path.join(d, fd))
                except OSError:
                    continue
                if t.startswith("/dev/shm/"):
                    names.add(os.path.basename(t))
        except OSError:
            pass
        # mmaped regions -- the case an fd-only check misses
        try:
            with open("/proc/%s/maps" % pid) as f:
                for line in f:
                    i = line.find("/dev/shm/")
                    if i != -1:
                        names.add(os.path.basename(line[i:].strip()))
        except OSError:
            pass
    return names


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--delete", action="store_true",
                    help="actually remove the orphans (default: report only)")
    a = ap.parse_args()

    try:
        allnames = sorted(n for n in os.listdir(SHM) if PAT.match(n))
    except OSError as e:
        print("cannot read %s: %s" % (SHM, e))
        return 1
    live = referenced_names()
    orphans = [n for n in allnames if n not in live]
    inuse = [n for n in allnames if n in live]

    print("FastDDS shared memory in %s" % SHM)
    print("  total segments      %d" % len(allnames))
    print("  in use (live proc)  %d" % len(inuse))
    print("  ORPHANED            %d" % len(orphans))
    if not orphans:
        print("\nNothing to clear.")
        return 0
    if not a.delete:
        print("\nReport only. Re-run with --delete to remove the %d orphan(s)."
              % len(orphans))
        for n in orphans[:12]:
            print("    %s" % n)
        if len(orphans) > 12:
            print("    ... and %d more" % (len(orphans) - 12))
        return 0

    removed = failed = 0
    freed = 0
    for n in orphans:
        p = os.path.join(SHM, n)
        try:
            freed += os.path.getsize(p)
        except OSError:
            pass
        try:
            os.unlink(p)
            removed += 1
        except OSError as e:
            print("  could not remove %s: %s" % (n, e))
            failed += 1
    print("\nremoved %d orphan(s), %.1f MiB freed%s"
          % (removed, freed / (1024.0 * 1024.0),
             (", %d failed" % failed) if failed else ""))
    print("Discovery for NEW processes should recover immediately; already"
          "-running nodes are untouched.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
