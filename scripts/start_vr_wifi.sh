#!/usr/bin/env bash
# start_vr_wifi.sh -- the VR path over WIFI and WebXR. NO adb, NO USB, NO Unity.
#
#   bash scripts/start_vr_wifi.sh            # everything
#   bash scripts/start_vr_wifi.sh --page     # bridge only, no mapper/gripper
#   bash scripts/start_vr_wifi.sh --check    # print the diagnosis and exit
#
# ROUTE B. The default route in vr_bringup.md is a Unity client reaching
# ws://127.0.0.1:8766 through `adb reverse` over USB. That route needs adb and
# needs the app sideloaded onto the headset, so it is unavailable on a headset
# you may not modify. This route needs neither: WebXR runs in the headset's
# own browser and everything arrives over wifi.
#
# WHAT WIFI COSTS, stated up front so it is not discovered inside a headset:
#   * WebXR only starts in a SECURE CONTEXT, so the page must be https and the
#     certificate must carry this machine's IP in subjectAltName.
#   * The certificate is self-signed, so the operator accepts one warning.
#   * The page and the WebSocket are served from ONE port on purpose. A cert
#     exception is per-origin and a WebSocket cannot prompt -- split them and
#     the page works while the socket dies silently.
#   * Inbound TCP must survive the Windows firewall.
#   * The headset shares a network with everything else on it. Under mirrored
#     WSL networking this box's LAN address IS the Windows LAN address.
set -uo pipefail

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# ONE BUS. This script never set the domain, so it inherited whatever shell
# launched it -- and on 2026-08-26 that meant the whole VR stack came up on
# domain 0 while the arms and their bridges ran on domain 7. RViz followed
# the hand beautifully; the metal heard nothing, because nothing it needed
# was on its bus. env.sh joins the RUNNING rig's domain (explicit
# ROS_DOMAIN_ID still wins) and sets the mandatory transports.
# shellcheck disable=SC1091
source "$WS_DIR/scripts/env.sh"
PORT="${VR_PORT:-8765}"
CERT_DIR="${VR_CERT_DIR:-$HOME/.srl_vr_cert}"
CRT="$CERT_DIR/vr.crt"
KEY="$CERT_DIR/vr.key"
WEB="$WS_DIR/src/srl_vr_teleop/web"
PS='/mnt/c/windows/System32/WindowsPowerShell/v1.0/powershell.exe'

MODE=full
[ "${1:-}" = "--page" ]  && MODE=page
[ "${1:-}" = "--check" ] && MODE=check

bold()  { printf '\033[1m%s\033[0m\n' "$*"; }
warn()  { printf '\033[33m%s\033[0m\n' "$*"; }
bad()   { printf '\033[31m%s\033[0m\n' "$*"; }
good()  { printf '\033[32m%s\033[0m\n' "$*"; }

IP="$(hostname -I | tr ' ' '\n' | grep -E '^[0-9]' | grep -v '^127\.' | head -1)"
[ -n "$IP" ] || { bad "no LAN address on this host. Nothing can reach it."; exit 1; }

bold "[1/5] address"
CIDR="$(ip -o -4 addr show | awk -v ip="$IP" '$4 ~ "^"ip"/" {print $4}' | head -1)"
echo "  WSL LAN address : $IP"
echo "  subnet          : ${CIDR:-unknown}"
echo "  gateway         : $(ip route | awk '/^default/{print $3; exit}')"
echo "  networking mode : $(grep -h networkingMode /mnt/c/Users/*/.wslconfig 2>/dev/null | tr -d ' \r' || echo 'unknown')"
echo "  (mirrored means this IS the Windows LAN address -- no portproxy needed)"

bold "[2/5] certificate"
NEED_CERT=0
if [ ! -f "$CRT" ] || [ ! -f "$KEY" ]; then
  NEED_CERT=1
elif ! openssl x509 -in "$CRT" -noout -ext subjectAltName 2>/dev/null | grep -q "IP Address:${IP}\b"; then
  warn "  the existing cert does NOT carry $IP -- the DHCP lease moved."
  warn "  In the headset that shows as ERR_CERT_COMMON_NAME_INVALID, which"
  warn "  reads like a broken cert and is a stale one. Regenerating."
  NEED_CERT=1
fi
if [ "$NEED_CERT" = "1" ]; then
  bash "$WS_DIR/scripts/make_vr_cert.sh" >/dev/null || { bad "  cert generation failed"; exit 1; }
fi
good "  $CRT"
openssl x509 -in "$CRT" -noout -ext subjectAltName | tail -1 | sed 's/^/  /'

bold "[3/5] Windows firewall (inbound)"
FW_RULE="SRL VR bridge (WSL) TCP $PORT"
if [ -x "$PS" ]; then
  HIT="$("$PS" -NoProfile -Command \
    "(Get-NetFirewallRule -DisplayName '$FW_RULE' -ErrorAction SilentlyContinue | Measure-Object).Count" \
    2>/dev/null | tr -d '\r ')"
  if [ "${HIT:-0}" = "0" ]; then
    warn "  no inbound rule named '$FW_RULE'."
    warn "  If step 4's page does not load in the headset, open an ADMIN"
    warn "  PowerShell on Windows and run BOTH of these:"
    echo
    echo "      New-NetFirewallRule -DisplayName '$FW_RULE' \\"
    echo "        -Direction Inbound -Action Allow -Protocol TCP \\"
    echo "        -LocalPort $PORT,8080,8081 -Profile Any"
    echo
    echo "      # mirrored WSL has a SECOND firewall in front of the first."
    echo "      # Without this the rule above appears to do nothing."
    echo "      Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' \\"
    echo "        -DefaultInboundAction Allow"
    echo
  else
    good "  inbound rule present: $FW_RULE"
  fi
else
  warn "  no Windows PowerShell at $PS (interop needs the LOWERCASE /mnt/c/windows)"
fi

bold "[4/5] observer e-stop"
set +u; source /opt/ros/jazzy/setup.bash; source "$WS_DIR/install/setup.bash"; set -u
export FASTDDS_BUILTIN_TRANSPORTS=SHM
if timeout 5 ros2 topic list 2>/dev/null | grep -q observer_estop_present; then
  good "  /vr/observer_estop_present has a publisher"
else
  warn "  /vr/observer_estop_present has no publisher."
  warn "  vr_safety_node will REFUSE real-arm control. That is correct and is"
  warn "  not a problem for a SIM session -- nothing here drives a real arm."
fi

if [ "$MODE" = "check" ]; then
  bold "[5/5] check only -- not starting anything"
  exit 0
fi

bold "[5/5] starting"

# REFUSE IF THE PORT IS ALREADY BOUND. Without this the bridge dies on
# EADDRINUSE, the four other nodes come up fine, and the banner below still
# tells the operator to open a URL that nobody is serving -- so the failure is
# discovered inside a headset, on a stack that looks healthy from ROS.
if ss -ltn "sport = :$PORT" 2>/dev/null | grep -q ":$PORT"; then
  bad "  port $PORT is ALREADY BOUND. Refusing rather than starting a bridge"
  bad "  that will die while the rest of the stack looks healthy."
  ss -ltnp "sport = :$PORT" 2>/dev/null | sed 's/^/    /'
  bad "  Kill that PID (and its process GROUP) and re-run."
  exit 1
fi

PIDS=()
start() { local n="${!#}"; "$@" >/dev/null 2>&1 & PIDS+=($!); echo "  $n (pid ${PIDS[-1]})"; }

# THE LINK WATCHDOG MUST OUTLAST THE LINK'S OWN MEASURED STALLS. The real
# wifi session ran 81 Hz with a median gap of 9 ms -- and ONE stall of
# 1.14 s (VR_REAL_ARM_AS_RUN.md). The default stale_timeout_s of 0.2 turned
# every such stall into /vr/tracking_ok=false, which drops the operator's
# clutch mid-motion for a reason they cannot see inside a headset. 0.6 is
# the value the as-run doc prescribes; the arm is still protected -- a
# genuinely dead link freezes 0.6 s in, and the mapper commands
# hold-in-place from the first frozen tick.
ros2 run srl_vr_teleop quest_bridge_node --ros-args \
  -p port:=$PORT -p certfile:="$CRT" -p keyfile:="$KEY" -p web_dir:="$WEB" \
  -p stale_timeout_s:="${VR_LINK_TIMEOUT:-0.6}" &
PIDS+=($!)
echo "  quest_bridge_node (pid ${PIDS[-1]})  https+wss on :$PORT"

if [ "$MODE" = "full" ]; then
  sleep 2
  start ros2 run srl_vr_teleop vr_pose_mapper
  start ros2 run srl_vr_teleop vr_gripper_node
  # THE SAFETY NODE'S GATES, SET AT LAUNCH RATHER THAN LEFT AT THE DEFAULTS.
  #
  # They default SHUT and that is right: `require_observer_estop` means a
  # second person publishes a 5 Hz heartbeat confirming they are stood beside
  # the wearer holding the physical e-stop, and an operator in a headset
  # CANNOT SEE THE ARM. But this script started the node with the defaults
  # and no way to change them, so on a rig with the arms connected the
  # sequence was: press the button, everything launches, and the mapper is
  # frozen for a reason no control in the window can clear. The operator's
  # only route was to kill the node and relaunch it by hand -- which is how
  # a safety interlock stops being one, because the workaround is to remove
  # it entirely rather than to record that it was bypassed.
  #
  # So the three gates come from the environment, still shut by default, and
  # the GUI sets them from the "I am working alone" tick it already logs to
  # recordings/observer_bypass_log.jsonl. Bypassing is now a thing the window
  # does on the record, not a thing you do around the window.
  # tracking_timeout_s matches the bridge's stale_timeout_s above and for
  # the same reason: two watchdogs on one link, and the tighter one decides.
  # watch_tracking_reference guards a headset STANDING ON A SHELF (desk
  # operation, the shipped setup): a knock announces nothing, so the first
  # HMD pose is latched and 20 mm / 2 deg of drift freezes everything. On a
  # WORN headset that same gate freezes the moment the operator turns their
  # head -- stickily, clearable only by /vr/rebase_reference -- so the GUI's
  # "headset is worn" tick sets VR_WATCH_REFERENCE=false before launch.
  start ros2 run srl_vr_teleop vr_safety_node --ros-args \
    -p require_observer_estop:="${VR_REQUIRE_OBSERVER:-true}" \
    -p allow_real_arm:="${VR_ALLOW_REAL_ARM:-false}" \
    -p allow_real_arm_without_observer:="${VR_ALLOW_REAL_NO_OBSERVER:-false}" \
    -p watch_tracking_reference:="${VR_WATCH_REFERENCE:-true}" \
    -p tracking_timeout_s:="${VR_TRACKING_TIMEOUT:-0.6}"
  start ros2 run srl_vr_teleop vr_feedback_node
fi

# Kill the process GROUP, not the pid: `ros2 run` is a wrapper and killing it
# leaves the node behind holding the port. HARD CONSTRAINT 9.
cleanup() {
  printf '\n  stopping\n'
  for p in "${PIDS[@]}"; do kill -TERM -- "-$p" 2>/dev/null || kill -TERM "$p" 2>/dev/null; done
  sleep 1
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null || kill -KILL "$p" 2>/dev/null; done
  exit 0
}
trap cleanup INT TERM

sleep 3
if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
  bad "  the bridge EXITED during start-up. Run it in the foreground to see why:"
  bad "    ros2 run srl_vr_teleop quest_bridge_node --ros-args -p port:=$PORT \\"
  bad "      -p certfile:=$CRT -p keyfile:=$KEY -p web_dir:=$WEB"
  cleanup
fi
echo
bold "================================================================"
bold "  IN THE HEADSET, open the Meta Quest Browser and go to:"
echo
bold "      https://$IP:$PORT/"
echo
bold "================================================================"
echo "  You WILL get 'Your connection is not private'"
echo "  (NET::ERR_CERT_AUTHORITY_INVALID). That is the self-signed cert and"
echo "  it is expected. Press 'Advanced', then 'Proceed to $IP (unsafe)'."
echo "  An accepted exception still counts as a secure context, so WebXR works."
echo
echo "  Then press ENTER VR on the page and put the controllers in your hands."
echo "  Ctrl-C here to stop."
echo
wait
