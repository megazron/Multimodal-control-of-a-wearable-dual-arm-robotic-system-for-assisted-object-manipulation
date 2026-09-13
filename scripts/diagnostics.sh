#!/usr/bin/env bash
# diagnostics.sh - the checks worth running BEFORE blaming the code.
#
# Every item here corresponds to a failure that has actually happened on this
# rig and cost real time to diagnose. See docs/ENGINEERING_LOG.md.
set -uo pipefail
source "$(dirname "$0")/env.sh"

hr () { printf '\n== %s ==\n' "$1"; }

hr "ROS graph (bypassing the daemon, which caches network state and hangs)"
timeout 20 ros2 node list --no-daemon 2>/dev/null | sed 's/^/  /' || echo "  no nodes"

hr "controllers - joint_state_broadcaster and both arm controllers must be active"
timeout 20 ros2 control list_controllers 2>/dev/null | sed 's/^/  /' || echo "  controller_manager not reachable"

hr "/joint_states rate (expect ~100 Hz in sim)"
timeout 12 ros2 topic hz /joint_states 2>/dev/null | tail -2 | sed 's/^/  /' || echo "  silent"

hr "master arm serial port"
ls /dev/ttyACM* /dev/ttyUSB* /dev/teensy 2>/dev/null | sed 's/^/  /' || \
  echo "  none present - re-attach with: usbipd attach --wsl --hardware-id 16c0:0483"

hr "stale Fast DDS shared-memory segments (clear ONLY with the stack stopped)"
printf '  %s segments in /dev/shm\n' "$(ls /dev/shm 2>/dev/null | grep -c fastrtps || echo 0)"

hr "arm network"
bash "$(dirname "$0")/check_arm_network.sh" 2>/dev/null | tail -12 | sed 's/^/  /' || echo "  skipped"

hr "e-stop"
timeout 8 ros2 topic echo /estop_state --once 2>/dev/null | sed 's/^/  /' || echo "  /estop_state silent (estop_node not running)"
