#!/usr/bin/env bash
# recover.sh - back to a known-good state from ANY failure. PART 7f.
#
# Every step here corresponds to a state this rig has actually been stuck in.
# It is deliberately blunt: it kills everything srl_* and vendor-ROS related,
# clears the stale IPC that breaks discovery, and leaves you at a clean start.
set -uo pipefail
source "$(dirname "$0")/env.sh" 2>/dev/null || true

echo "== 1. stop every srl and MoveIt process, by explicit PID =="
# NEVER a broad `pkill -f "ros2 launch"` - that has killed a live recording
# session on this rig. Match the INSTALLED executable paths only.
PIDS=$(ps -eo pid,args | grep -E "lib/(srl_[a-z_]+|moveit_ros_move_group|controller_manager|robot_state_publisher)/" | grep -v grep | awk '{print $1}')
if [ -n "$PIDS" ]; then
  echo "$PIDS" | xargs -r kill -INT 2>/dev/null; sleep 4
  echo "$PIDS" | xargs -r kill -9  2>/dev/null
fi
echo "   remaining: $(ps -eo args | grep -cE 'lib/(srl_|moveit_ros_move_group)' || true)"

echo "== 2. clear stale Fast DDS shared memory =="
# Only safe with the stack STOPPED. Stale segments make discovery
# intermittent: a service is visible to one client and not another.
rm -f /dev/shm/fastrtps_* 2>/dev/null
echo "   /dev/shm fastrtps segments: $(ls /dev/shm 2>/dev/null | grep -c fastrtps || echo 0)"

echo "== 3. restart the ros2 daemon =="
# It caches network state, does not survive an interface change, and HANGS
# rather than failing - which reads as "nothing is running".
ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1

echo "== 4. clear any latched e-stop (service, not a topic) =="
timeout 10 ros2 service call /estop_reset std_srvs/srv/Trigger >/dev/null 2>&1 \
  && echo "   e-stop reset" || echo "   no estop_node running (fine)"

echo "== 5. master serial port =="
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || \
  echo "   none - re-attach: usbipd attach --wsl --hardware-id 16c0:0483"

echo
echo "RECOVERED. Now:  bash scripts/run_teleop.sh"
echo "Then verify:     ros2 run srl_teleop preflight"
