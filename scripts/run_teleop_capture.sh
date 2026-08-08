#!/usr/bin/env bash
# Calibration capture: runs the scripted teleop recording in THIS terminal,
# so the prompts are clean, unprefixed and readable.
#
# Intended flow (two terminals - see NOTE below for why):
#     terminal 1:  ros2 launch srl_teleop teleop.launch.py
#     terminal 2:  bash ~/kortex_ws/run_teleop_capture.sh
#
# NOTE on why this is not folded into the launch file: under `ros2 launch`
# every node's stdout is merged into one stream and line-prefixed with
# [node-N], so RViz, MoveIt, both IK followers and master_pose_node bury the
# countdown, and the prefixes break in-place redraw. A `prompts_only` flag in
# the recorder cannot fix that - the noise comes from OTHER processes, which
# the recorder has no control over. Separate terminals is the fix that
# actually works, so `record:=true` was removed from the launch file.
#
# This script does NOT start master_pose_node if one is already running:
# only one process can hold the serial port, and a second would either fail
# or knock the first off the port.

# All ROS environment comes from scripts/env.sh so this script, start_real.sh
# and the launch files cannot drift apart on domain or discovery settings.
# It handles the `set -u` dance that ROS's own setup.bash requires.
# shellcheck disable=SC1091
source "$HOME/kortex_ws/scripts/env.sh" || {
  echo "cannot source scripts/env.sh" >&2; exit 1
}

cd "$HOME/kortex_ws"

# `ls /dev/ttyACM* /dev/teensy` returns non-zero when ANY argument is missing,
# so it reports "no device" whenever /dev/teensy is absent -- which it always
# is without a udev rule -- even with /dev/ttyACM0 sitting right there. Test
# each pattern independently instead.
have_serial() {
  compgen -G "/dev/ttyACM*" >/dev/null 2>&1 && return 0
  compgen -G "/dev/ttyUSB*" >/dev/null 2>&1 && return 0
  [ -e /dev/teensy ] && return 0
  return 1
}
list_serial() {
  { compgen -G "/dev/ttyACM*" || true
    compgen -G "/dev/ttyUSB*" || true
    [ -e /dev/teensy ] && echo /dev/teensy || true
  } 2>/dev/null | tr '\n' ' '
}

STARTED_MASTER=""
cleanup() {
  if [ -n "$STARTED_MASTER" ]; then
    echo
    echo "--- stopping the master_pose_node this script started (pid $STARTED_MASTER) ---"
    kill "$STARTED_MASTER" 2>/dev/null
    wait "$STARTED_MASTER" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------- Teensy ---
echo "=== 1/3  serial device ==="
if have_serial; then
  echo "    found: $(list_serial)"
else
  echo "    no /dev/ttyACM* present. In Windows PowerShell (Admin):"
  echo "        usbipd attach --wsl --hardware-id 16c0:0483"
  echo -n "    waiting "
  for _ in $(seq 1 60); do
    have_serial && break
    echo -n "."; sleep 1
  done
  echo
  if ! have_serial; then
    echo "!!! Teensy never appeared. Aborting."
    exit 1
  fi
  echo "    found: $(list_serial)"
fi

# -------------------------------------------------------- master_pose_node ---
echo "=== 2/3  master_pose_node ==="
if ros2 node list 2>/dev/null | grep -qx "/master_pose_node"; then
  echo "    ATTACHING to the master_pose_node already running (probably from"
  echo "    teleop.launch.py). Not starting another - only one process can"
  echo "    hold $(list_serial | awk '{print $1}')."
else
  echo "    none running - STARTING one (arm:=both, serial_port:=auto)."
  ros2 run srl_teleop master_pose_node --ros-args -p arm:=both \
    > /tmp/mpn_capture.log 2>&1 &
  STARTED_MASTER=$!
  echo "    started pid $STARTED_MASTER, log /tmp/mpn_capture.log"
fi

echo -n "    waiting for /master_arm_pose_left "
OK=""
for _ in $(seq 1 45); do
  if ros2 topic list 2>/dev/null | grep -qx "/master_arm_pose_left"; then OK=1; echo " ok"; break; fi
  echo -n "."; sleep 1
done
if [ -z "$OK" ]; then
  echo
  echo "!!! /master_arm_pose_left never appeared."
  if [ -n "$STARTED_MASTER" ]; then
    echo "    Last lines of /tmp/mpn_capture.log:"
    tail -20 /tmp/mpn_capture.log
  else
    echo "    A master_pose_node is running but not publishing. If it started"
    echo "    before the Teensy was attached, restart the launch - the serial"
    echo "    port is resolved once, at node startup."
  fi
  exit 1
fi

# ------------------------------------------------------------- recording ---
echo "=== 3/3  RECORDING - follow the prompts below ==="
echo
ros2 run srl_teleop teleop_recorder --ros-args -p arms:="['left','right']"
