#!/usr/bin/env bash
# Automatic Quest connection: find adb, find the headset, open the reverse
# tunnel, start the bridge, and keep the tunnel alive.
#
# WHY adb reverse AND NOT THE NETWORK. The headset connects to
# ws://127.0.0.1:8766, which is a secure origin by definition. Over a LAN it
# would need a certificate carrying the address, a firewall exception, and an
# operator accepting a warning inside a headset they are wearing. It is also
# USB, so it does not share the wireless network the arms are on.
#
# WHY THE TUNNEL IS SUPERVISED RATHER THAN OPENED ONCE. `adb reverse` dies
# with the USB connection. Opened once at start-up, a cable nudge leaves the
# bridge running and the headset silently unable to reach it, which from the
# operator's side is indistinguishable from a frozen application.
set -u

PORT="${VR_PORT:-8766}"
POLL="${VR_POLL:-2}"
BRIDGE="${VR_BRIDGE:-1}"

say() { printf '  %s\n' "$*"; }
die() { printf '\nREFUSING: %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- find adb
find_adb() {
  if command -v adb >/dev/null 2>&1; then echo "adb"; return 0; fi
  # Windows-side installs, reachable from WSL. The lowercase /mnt/c path is
  # the one that works; /mnt/c/Windows returns EINVAL on this box.
  for c in \
    "/mnt/c/platform-tools/adb.exe" \
    "/mnt/c/Users/$USER/AppData/Local/Android/Sdk/platform-tools/adb.exe" \
    "/mnt/c/Users/Gausms/AppData/Local/Android/Sdk/platform-tools/adb.exe" \
    "/mnt/c/Program Files/platform-tools/adb.exe" ; do
    [ -x "$c" ] && { echo "$c"; return 0; }
  done
  return 1
}

ADB="$(find_adb)" || die "adb is not installed, in WSL or on Windows.

  It is the ONE hard prerequisite for the headset, and nothing here can work
  around it. Install Android Platform-Tools on the WINDOWS side, not in WSL:
  the headset enumerates as a Windows USB device, so a WSL-side adb cannot
  see it.

      https://developer.android.com/tools/releases/platform-tools

  Unzip it to C:\\platform-tools and re-run. This script finds it there."

say "adb: $ADB"

# ------------------------------------------------------------ find headset
devices() { "$ADB" devices 2>/dev/null | awk 'NR>1 && $2=="device" {print $1}'; }

if [ -z "$(devices)" ]; then
  say "no authorised device yet. Waiting..."
  say "  On the headset: enable Developer Mode, plug in USB, and ACCEPT the"
  say "  'Allow USB debugging' prompt. It appears INSIDE the headset, so put"
  say "  it on to answer it -- this is the step that traps everyone."
  for _ in $(seq 1 30); do
    sleep "$POLL"
    [ -n "$(devices)" ] && break
  done
fi

DEV="$(devices | head -1)"
[ -n "$DEV" ] || die "no authorised device after 60 s.

  '$ADB devices' shows nothing usable. If it lists a device as 'unauthorized',
  the prompt inside the headset has not been accepted."

say "headset: $DEV"

# ------------------------------------------------------------ the tunnel
open_tunnel() {
  "$ADB" -s "$DEV" reverse --remove tcp:$PORT >/dev/null 2>&1
  "$ADB" -s "$DEV" reverse tcp:$PORT tcp:$PORT >/dev/null 2>&1
}
tunnel_ok() {
  "$ADB" -s "$DEV" reverse --list 2>/dev/null | grep -q "tcp:$PORT"
}

open_tunnel
tunnel_ok || die "could not open the reverse tunnel on port $PORT."
say "tunnel: headset localhost:$PORT -> this host"

# ------------------------------------------------------------- the bridge
BRIDGE_PID=""
if [ "$BRIDGE" = "1" ]; then
  if pgrep -f "lib/srl_vr_teleop/quest_vendor_bridge" >/dev/null 2>&1; then
    say "bridge: already running, leaving it alone"
  else
    ros2 run srl_vr_teleop quest_vendor_bridge &
    BRIDGE_PID=$!
    say "bridge: started (pid $BRIDGE_PID)"
  fi
fi

cleanup() {
  printf '\n  closing\n'
  [ -n "$BRIDGE_PID" ] && kill "$BRIDGE_PID" 2>/dev/null
  "$ADB" -s "$DEV" reverse --remove tcp:$PORT >/dev/null 2>&1
  exit 0
}
trap cleanup INT TERM

say ""
say "CONNECTED. Start the Unity client on the headset; it should reach"
say "ws://127.0.0.1:$PORT immediately."
say "Supervising the tunnel. Ctrl-C to stop."

DOWN=0
while true; do
  sleep "$POLL"
  if ! tunnel_ok; then
    DOWN=$((DOWN + 1))
    printf '  tunnel DOWN (%d) -- reopening\n' "$DOWN"
    if [ -z "$(devices)" ]; then
      printf '  headset gone from the bus. Waiting for it to come back.\n'
      while [ -z "$(devices)" ]; do sleep "$POLL"; done
      DEV="$(devices | head -1)"
      printf '  headset back: %s\n' "$DEV"
    fi
    open_tunnel
    tunnel_ok && printf '  tunnel restored\n'
  fi
done
