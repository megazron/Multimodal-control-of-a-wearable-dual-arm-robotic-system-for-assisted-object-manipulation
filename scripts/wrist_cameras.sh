#!/usr/bin/env bash
# wrist_cameras.sh -- the two GRIPPER cameras, one command.
#
#     bash scripts/wrist_cameras.sh            # both arms
#     bash scripts/wrist_cameras.sh left       # one
#
# WHY THIS EXISTS. The wrist cameras are Kinova's own, on the ARM's network
# address -- nothing to do with usbipd and nothing to do with /dev/video*.
# They come up with `ros2 launch kinova_vision kinova_vision.launch.py` and a
# fistful of frame-id arguments per arm, and on 2026-08-30 that invocation
# existed in exactly one place: a terminal somebody had typed it into. It was
# in no launch file, no script and no spec, so `grep kinova_vision` over
# gui_launch_specs.py and srl_gui.py returned NOTHING.
#
# THE GUI RULE: a capability reachable only by typing is a capability the
# operator running the session does not have. The report was "the gripper
# cameras are still not working"; they were not broken, they were never
# started.
#
# The arm must be reachable -- these are cameras ON the robot, so a wrist
# camera cannot come up while the arm is off the network, and this says which
# rather than leaving a launch to time out in a log nobody reads.
set -uo pipefail
WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS_DIR"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
set -u
export FASTDDS_BUILTIN_TRANSPORTS=SHM

declare -A IP=( [left]=192.168.1.10 [right]=192.168.1.9 )
WANT=("$@")
[ ${#WANT[@]} -eq 0 ] && WANT=(left right)

PIDS=()
STARTED=0
for arm in "${WANT[@]}"; do
  ip="${IP[$arm]:-}"
  if [ -z "$ip" ]; then
    printf '  %-5s unknown arm -- skipped\n' "$arm" >&2
    continue
  fi
  if pgrep -f "camera:=${arm}_camera" >/dev/null 2>&1; then
    printf '  %-5s wrist camera ALREADY RUNNING -- left alone\n' "$arm"
    STARTED=$((STARTED + 1))
    continue
  fi
  if ! ping -c1 -W2 "$ip" >/dev/null 2>&1; then
    printf '  %-5s arm is OFF THE NETWORK at %s -- its wrist camera is ON\n' \
      "$arm" "$ip" >&2
    printf '        the robot, so it cannot come up while the arm is down.\n' >&2
    continue
  fi
  # ASK THE CAMERA IN RTSP'S OWN WORDS, NOT GStreamer'S.
  #
  # `ping` proves the arm's network stack is alive and says NOTHING about the
  # vision module: measured 2026-08-30, both arms answered ping AND served
  # HTTP 200 in 86 ms while one of their RTSP servers was wedged.
  #
  # AND THE FIRST VERSION OF THIS CHECK WAS WORSE THAN NONE. It ran
  # `gst-launch-1.0 rtspsrc location=... ! fakesink`, which cannot link
  # statically -- rtspsrc exposes its pads dynamically -- so it never reached
  # PLAYING no matter what the server did. It then refused to start a right
  # wrist camera that was answering perfectly: a gate inventing the fault it
  # was written to detect. `rtsp_probe.py` speaks OPTIONS and DESCRIBE and
  # believes the answer.
  if [ -x "$WS_DIR/scripts/rtsp_probe.py" ]; then
    if ! detail="$(python3 "$WS_DIR/scripts/rtsp_probe.py" "$ip" 2>&1)"; then
      printf '  %-5s CAMERA NOT SERVING: %s\n' "$arm" "$detail" >&2
      printf '        The arm itself is fine (it answers ping and HTTP). The\n' >&2
      printf '        VISION MODULE does not recover on its own:\n' >&2
      printf '        POWER-CYCLE THE ARM, or check http://%s\n' "$ip" >&2
      printf '        Starting the node anyway would publish a topic with no\n' >&2
      printf '        frames on it, which looks exactly like this fault.\n' >&2
      continue
    fi
    printf '  %-5s camera %s\n' "$arm" "$detail"
  fi
  printf '  %-5s starting the wrist camera at %s\n' "$arm" "$ip"
  setsid ros2 launch kinova_vision kinova_vision.launch.py \
    device:="$ip" \
    camera:="${arm}_camera" \
    launch_tf:=false \
    camera_link_frame_id:="${arm}_camera_link" \
    color_frame_id:="${arm}_camera_color_frame" \
    depth_frame_id:="${arm}_camera_depth_frame" >/dev/null 2>&1 &
  PIDS+=("$!")
  STARTED=$((STARTED + 1))
done

if [ "$STARTED" -eq 0 ]; then
  printf '\nNo wrist camera could be started. Check the arms are powered and\n' >&2
  printf 'on the network; these cameras live on the robot.\n' >&2
  exit 1
fi

# HARD CONSTRAINT 9: kill the process GROUP. `ros2 launch` is a wrapper and
# killing the wrapper leaves the camera nodes behind holding their topics.
cleanup() {
  printf '\n  stopping the wrist cameras\n'
  for p in "${PIDS[@]:-}"; do
    kill -INT -- "-$p" 2>/dev/null || kill -INT "$p" 2>/dev/null
  done
  sleep 2
  exit 0
}
trap cleanup INT TERM

printf '\n  %d wrist camera(s) starting. Topics: /<arm>_camera/color/image_raw\n' \
  "$STARTED"
printf '  They take ~15 s to publish their first frame.\n\n'
wait
