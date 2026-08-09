#!/usr/bin/env bash
# scripts/env.sh -- the single source of ROS 2 environment truth.
#
#   source scripts/env.sh          # SOURCE this, do not execute it
#
# Every script in this workspace sources this file so that terminal 1,
# terminal 3 and any shell you open by hand agree on the domain, the
# discovery settings, and the health of the ros2 daemon.
#
# WHY THIS FILE EXISTS
#   After switching WSL to networkingMode=mirrored, terminal 3 reported
#   "No ROS 2 nodes at all" while terminal 1 was running a healthy stack.
#   The graph was fine -- `ros2 node list --no-daemon` saw all 28 nodes the
#   whole time. What had broken was the ros2 DAEMON, which caches network
#   state and was still holding the pre-mirrored interface (172.21.248.59).
#   It did not fail, it HUNG, so `timeout 20 ros2 node list` returned an
#   empty string and the caller concluded nothing was running.
#
#   So the job here is not to pin interfaces. Measured on this host with a
#   healthy daemon, the default settings return 28/28 nodes on 5/5 runs
#   across eth0 + eth3 + lo. The job is to keep the daemon honest and to
#   keep every process in the same domain.
#
# ON ROS_LOCALHOST_ONLY
#   Deliberately NOT set, and actively unset below. It is deprecated in
#   Jazzy (rcl warns on every node), it was not the cause of the outage,
#   and a half-applied value is worse than none: the daemon caches the
#   environment of whichever shell first spawned it, so one shell setting
#   it and another not is exactly the confusing partition it is meant to
#   prevent. Unsetting it in one place is what makes every process agree.

# Idempotent: sourcing twice is a no-op, so scripts can source it freely.
if [ -z "${SRL_ENV_SOURCED:-}" ]; then

SRL_WS="${SRL_WS:-$HOME/kortex_ws}"

# ROS 2's setup.bash reads unbound variables internally, so it cannot run
# under `set -u`. Relax, source, then restore whatever the caller had.
_srl_had_u=0
case "$-" in *u*) _srl_had_u=1 ;; esac
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 || {
  echo "scripts/env.sh: ROS 2 Jazzy not found at /opt/ros/jazzy" >&2
  return 1 2>/dev/null || exit 1
}
# shellcheck disable=SC1091
source "$SRL_WS/install/setup.bash" >/dev/null 2>&1 || {
  echo "scripts/env.sh: workspace not built at $SRL_WS/install" >&2
  return 1 2>/dev/null || exit 1
}
[ "$_srl_had_u" = "1" ] && set -u
unset _srl_had_u

# ---------------------------------------------------------------- domain
# Pinned explicitly rather than left to default so that "both shells are on
# domain 0" is a fact you can read, not an accident you have to infer.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

# See "ON ROS_LOCALHOST_ONLY" above. If a shell inherited one from somewhere
# (a stale export, a terminal profile), drop it and say so -- a silent
# mismatch here is the single easiest way to recreate the original outage.
if [ -n "${ROS_LOCALHOST_ONLY:-}" ]; then
  echo "scripts/env.sh: unsetting inherited ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}" >&2
  echo "                (deprecated in Jazzy; see the note in scripts/env.sh)" >&2
  unset ROS_LOCALHOST_ONLY
fi

SRL_ENV_SOURCED=1
fi

# ---------------------------------------------------------------- helpers
# Defined unconditionally so a second source refreshes them.

# PIDs of the terminal-1 stack. Matched by executable, because these are ours
# and nothing else on the system runs them. This is the check that tells
# "nothing is running" apart from "running but invisible to this shell".
srl_stack_procs() {
  pgrep -f 'ros2 launch srl_teleop teleop\.launch\.py|moveit_ros_move_group/move_group|controller_manager/ros2_control_node' 2>/dev/null | tr '\n' ' '
}

# Hard reset of the ros2 daemon. `ros2 daemon stop` can itself hang when the
# daemon is wedged -- which is the exact case we are here to fix -- so it gets
# a timeout and a pkill fallback rather than being trusted.
srl_ros_daemon_reset() {
  timeout 10 ros2 daemon stop >/dev/null 2>&1
  pkill -f 'ros2cli\.daemon\.daemonize' >/dev/null 2>&1
  sleep 1
  timeout 25 ros2 daemon start >/dev/null 2>&1
  sleep 2
}

# Node list, with one automatic daemon reset if the first attempt hangs or
# comes back empty.
#
# The empty-result case matters as much as the timeout case: a wedged daemon
# returns "" with exit status 0, which is indistinguishable from a genuinely
# idle graph unless you also look at the process table.
#
# Whether a reset happened is reported through the EXIT STATUS, not a
# variable. Callers read the node list with `x="$(srl_ros_node_list)"`, and
# command substitution runs the function in a subshell -- so any variable it
# sets dies with that subshell and never reaches the caller. Exit status is
# the one channel that does survive.
#
#   0  nodes found
#  10  nodes found, but only after the daemon had to be restarted
#   1  no nodes
#  11  no nodes, and the daemon was restarted trying
srl_ros_node_list() {
  local timeout_s="${1:-20}"
  local out rc reset=0

  out="$(timeout "$timeout_s" ros2 node list 2>/dev/null)"; rc=$?
  if [ "$rc" -ne 0 ] || [ -z "$out" ]; then
    srl_ros_daemon_reset
    reset=1
    out="$(timeout "$timeout_s" ros2 node list 2>/dev/null)"
  fi

  [ -n "$out" ] && printf '%s\n' "$out"

  if [ -n "$out" ]; then
    [ "$reset" = "1" ] && return 10 || return 0
  fi
  [ "$reset" = "1" ] && return 11 || return 1
}

# Ground truth that bypasses the daemon entirely. Slower and noisier (counts
# vary run to run with discovery settle time), but it cannot be fooled by a
# wedged daemon, so it is the tiebreaker when the daemon path says nothing.
srl_ros_node_list_direct() {
  timeout "${1:-25}" ros2 node list --no-daemon 2>/dev/null
}

# ------------------------------------------------- stale Fast DDS SHM segments
# srl_clear_stale_shm -- remove /dev/shm Fast DDS segments left by killed
# stacks. Stale segments make Fast DDS log
#   RTPS_TRANSPORT_SHM Error ... Failed init_port fastrtps_port7002
# and discovery then goes INTERMITTENT: a service is visible to one client and
# not another, and wait_for_service times out on a service the graph can see.
# That is indistinguishable from a broken node, and it has cost this project
# two sessions of misdiagnosis.
#
# TWO GLOBS, NOT ONE. `rm /dev/shm/fastrtps_*` leaves the `sem.fastrtps_*`
# semaphores behind -- 188 segments cleared to 39 "remaining" on 2026-08-09,
# which read as a live writer and sent the session hunting for a process that
# did not exist. Both patterns or neither.
#
# ONLY SAFE WITH THE STACK STOPPED. Refuses otherwise, because removing a
# segment a live participant is using is worse than leaving it.
srl_clear_stale_shm() {
  local force="${1:-}"
  local live
  live=$(pgrep -c -f "opt/ros/jazzy/lib/(moveit_ros_move_group|controller_manager|robot_state_publisher)" 2>/dev/null || echo 0)
  if [ "$live" -gt 0 ] && [ "$force" != "--force" ]; then
    echo "srl_clear_stale_shm: $live stack process(es) running -- NOT clearing." >&2
    echo "  Stop the stack first, or pass --force if you know they are orphans." >&2
    return 1
  fi
  local n
  n=$(ls -1 /dev/shm 2>/dev/null | grep -c '^sem\.fastrtps_\|^fastrtps_' || true)
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  echo "srl_clear_stale_shm: removed $n stale Fast DDS segment(s)."
}
