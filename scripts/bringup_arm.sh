#!/usr/bin/env bash
# Bring up ONE arm's high-level bridge and its gripper camera, idempotently.
#
#   bash scripts/bringup_arm.sh left
#   bash scripts/bringup_arm.sh left --restart     # force a clean restart
#
# Everything in here was learned the hard way on 2026-08-25.  Each block says
# what goes wrong without it.
set -uo pipefail
ARM="${1:-left}"
FORCE="${2:-}"
case "$ARM" in
  left)  IP=192.168.1.10 ;;
  right) IP=192.168.1.9  ;;
  *) echo "usage: $0 left|right [--restart]"; exit 2 ;;
esac

# HARD CONSTRAINT 10: ROS setup.bash reads unbound variables.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/kortex_ws/install/setup.bash"
set -u

# ---------------------------------------------------------------- DDS domain
# INHERITED, NOT PINNED. This script used to `export ROS_DOMAIN_ID=7`
# unconditionally, and that is a defect the moment anything launches it:
# the GUI runs on domain 0 (as does sim_session.py, which pins 0), presses
# CONNECT, and the bridge it spawns lands on 7. The bridge then talks to the
# arm perfectly and publishes /real/joint_states into a graph the GUI cannot
# see. Observed exactly that on 2026-08-29 -- two healthy bridges, both arms
# answering, and a window reporting "bridge up but no data". It is this
# repository's own "feature present but does nothing", in the button whose
# entire job is to connect an arm.
#
# THE ORIGINAL REASON IS GONE, AND WAS RE-MEASURED RATHER THAN TRUSTED. The
# comment here claimed domain 0's shared memory was polluted so that NO new
# process could join it, citing 0 of 3 messages on 0 against 3 of 3 on 7. Re-
# run on 2026-08-29 with a minimal pub/sub on this host: domain 0 delivered
# 27 of 27 and domain 7 delivered 27 of 27. Identical. The pollution was a
# stale /dev/shm from a killed run, which is cleared by a reboot or by
# removing the segments -- not a property of domain 0.
#
# `:-7` keeps a bare terminal run behaving exactly as before; a caller that
# has already chosen a domain now KEEPS it.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
# Without this, rclpy picks a different RMW and discovers NOTHING while the
# graph is plainly there. Same silent failure.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=SHM

LOG="${TMPDIR:-/tmp}/srl_bringup"
mkdir -p "$LOG"
VENV="$HOME/kortex_ws/.kortex_venv/bin/python"

pid_of () { pgrep -f "kortex_highlevel_bridge_$1\b" | head -1; }
cam_of () { pgrep -f "kinova_vision_node.*__ns:=/${1}_camera" | head -1; }

stop_one () {
    local p="$1" what="$2"
    [ -z "$p" ] && return 0
    # HARD CONSTRAINT 2: the arm permits exactly ONE Kortex session, and
    # SIGKILL LEAKS IT -- the next run then cannot connect at all, while the
    # arm still pings and its API port still answers.  Always SIGINT and wait
    # for "kortex session closed cleanly".
    echo "  stopping $what ($p) with SIGINT"
    kill -INT "$p" 2>/dev/null
    for _ in $(seq 1 30); do kill -0 "$p" 2>/dev/null || return 0; sleep 1; done
    echo "  WARNING: $what did not exit on SIGINT; leaving it rather than "
    echo "           SIGKILLing and leaking the session"
    return 1
}

if [ "$FORCE" = "--restart" ]; then
    stop_one "$(pid_of "$ARM")" "$ARM bridge"
    CP="$(cam_of "$ARM")"; [ -n "$CP" ] && { kill -INT "$CP" 2>/dev/null; sleep 3; }
    # give the arm a moment to release anything the old session held
    sleep 15
fi

# ------------------------------------------------------------------- bridge
if [ -n "$(pid_of "$ARM")" ]; then
    echo "bridge for $ARM already up (pid $(pid_of "$ARM"))"
else
    echo "starting $ARM bridge -> $IP"
    # deadband 0.10 deg, NOT the shipped 1.00: inside the deadband the
    # proportional term is off by design, and a target ramped in small steps
    # then produces no velocity at all.  CLAUDE.md already records 0.10.
    setsid nohup "$VENV" -m srl_teleop.kortex_highlevel_bridge \
        --ros-args -r __node:="kortex_highlevel_bridge_$ARM" \
        -p arm:="$ARM" -p robot_ip:="$IP" -p rate_hz:=12.0 -p kp:=0.5 \
        -p vmax_rad_s:=0.40 -p deadband_deg:=0.10 -p watchdog_s:=0.5 \
        > "$LOG/bridge_$ARM.log" 2>&1 < /dev/null &
    disown
    for _ in $(seq 1 30); do
        grep -q "session created" "$LOG/bridge_$ARM.log" 2>/dev/null && break
        sleep 1
    done
    grep -m1 "session created" "$LOG/bridge_$ARM.log" | sed 's/^/  /' || {
        echo "  bridge did not create a session. Last lines:"
        tail -5 "$LOG/bridge_$ARM.log" | sed 's/^/    /'
        echo "  If this says 'Connection timed out' while the arm PINGS, a"
        echo "  previous session was leaked; wait ~30 s and retry --restart."
        exit 1; }
fi

# ----------------------------------------------------- vision module check
# THE ONE CHECK THAT TELLS THIS FAULT FROM EVERY OTHER CAMERA FAULT.
#
# When the wrist camera's RTSP server dies, `kinova_vision_node` retries for
# ever -- "[color]: Stream is PAUSED / Failed to start stream / attempt #16"
# -- while the arm pings, answers on port 80 and answers the Kortex API on
# 10000. Everything looks healthy and no camera ever appears. Observed on both
# arms at once on 2026-08-29, and it does NOT self-recover: still shut 45 s
# after every client had been stopped.
#
# Port 554 is the discriminator, so it is probed BEFORE a node is launched to
# hammer it. Launching anyway is not wrong -- the node's retry is harmless --
# but the operator is told what is actually broken and what fixes it, instead
# of reading "attempt #16" and guessing.
if command -v timeout >/dev/null 2>&1 && \
   ! timeout 3 bash -c "echo >/dev/tcp/$IP/554" 2>/dev/null; then
    echo "  WARNING: $ARM wrist camera RTSP (port 554 on $IP) is NOT LISTENING."
    echo "           The arm is otherwise fine -- this is the vision MODULE."
    echo "           The camera node will retry for ever and never connect."
    echo "           Recover it with (bridge must be stopped first):"
    echo "             .kortex_venv/bin/python scripts/reboot_vision_module.py $ARM"
fi

# ------------------------------------------------------------------- camera
if [ -n "$(cam_of "$ARM")" ]; then
    echo "camera for $ARM already up"
else
    echo "starting $ARM gripper camera"
    # launch_tf:=false and the URDF frame ids:  the driver's own static TF puts
    # camera_link->depth at (-0.0195,-0.005,0) while srl_description says
    # (-0.0275,-0.0096,0).  Letting both publish makes two writers fight over
    # one static edge.  Renaming the frames also fixes the OTHER default bug:
    # both arms' drivers publish the SAME unprefixed camera_link, so left and
    # right collide in one global TF tree.
    setsid nohup ros2 launch kinova_vision kinova_vision.launch.py \
        device:="$IP" camera:="${ARM}_camera" launch_tf:=false \
        camera_link_frame_id:="${ARM}_camera_link" \
        color_frame_id:="${ARM}_camera_color_frame" \
        depth_frame_id:="${ARM}_camera_depth_frame" \
        > "$LOG/cam_$ARM.log" 2>&1 < /dev/null &
    disown
    for _ in $(seq 1 40); do
        [ "$(grep -c 'Stream started' "$LOG/cam_$ARM.log" 2>/dev/null)" -ge 2 ] && break
        sleep 1
    done
    grep -c "Stream started" "$LOG/cam_$ARM.log" | xargs echo "  colour+depth streams started:"
fi

echo
echo "logs in $LOG"
echo "REMEMBER: everything that talks to this arm must export"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=SHM"
