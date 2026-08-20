#!/usr/bin/env bash
# scripts/attach_scene_camera.sh -- get the laptop camera into WSL, or say
# exactly why it is not there.
#
#     bash scripts/attach_scene_camera.sh          # check, and print the fix
#     bash scripts/attach_scene_camera.sh --attach # also try the attach step
#
# WHY THIS EXISTS. There is no USB in WSL. A USB camera has to be handed over
# from Windows with usbipd, exactly as the Teensy is, and the failure when it
# has not been is silent in the worst way: /dev/video* simply does not exist,
# the camera node finds nothing, and the body tracker reports no wearer --
# which looks identical to a room with nobody in it.
#
# THE ADMIN STEP IS REAL AND CANNOT BE WORKED AROUND. `usbipd bind` requires
# an elevated PowerShell; measured on this machine it answers
#     usbipd: error: Access denied; this operation requires administrator
#                    privileges.
# `bind` is needed once per device and survives reboots. `attach` is needed
# after every reboot and every unplug, and does NOT need admin.
set -euo pipefail

SRL_WS="${SRL_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

echo "=== 1. is a camera visible to Linux? ==="
shopt -s nullglob
vids=(/dev/video*)
shopt -u nullglob
if [ ${#vids[@]} -gt 0 ]; then
  echo "  yes: ${vids[*]}"
  for v in "${vids[@]}"; do
    if command -v v4l2-ctl >/dev/null 2>&1; then
      printf '  %s  %s\n' "$v" "$(v4l2-ctl -d "$v" --info 2>/dev/null | awk -F': ' '/Card type/{print $2}')"
    fi
  done
  echo
  echo "Nothing more to do. Start the camera with:"
  echo "  ros2 run srl_perception scene_camera_node"
  exit 0
fi
echo "  no /dev/video* -- the camera has not been handed over yet."
echo

# usbipd runs on the WINDOWS side. Interop lets us call it, but not elevate.
if ! command -v powershell.exe >/dev/null 2>&1; then
  echo "Windows interop is not available from this shell, so the device list"
  echo "cannot be read from here. Run 'usbipd list' in Windows PowerShell."
  exit 1
fi

echo "=== 2. what does Windows have? ==="
powershell.exe -NoProfile -Command "usbipd list" 2>/dev/null | sed 's/^/  /' || {
  echo "  usbipd is not installed on the Windows side."
  echo "  Install it once, from an elevated PowerShell:"
  echo "      winget install usbipd"
  exit 1
}

# Find a camera row. Matched on the WORD 'camera' in the device description,
# which is what usbipd prints; the busid is the first field.
BUSID="$(powershell.exe -NoProfile -Command "usbipd list" 2>/dev/null \
         | tr -d '\r' | awk 'tolower($0) ~ /camera/ && $1 ~ /^[0-9]+-[0-9]+$/ {print $1; exit}')"

if [ -z "${BUSID:-}" ]; then
  echo
  echo "No device with 'camera' in its name is connected to Windows."
  echo "Plug a USB camera in, or use the laptop's built-in one, and re-run."
  exit 1
fi

STATE="$(powershell.exe -NoProfile -Command "usbipd list" 2>/dev/null \
         | tr -d '\r' | awk -v b="$BUSID" '$1==b {print tolower($0)}')"

echo
echo "=== 3. what to run ==="
echo "  camera busid: $BUSID"
echo
if echo "$STATE" | grep -q "not shared"; then
  cat <<TXT
  It is not shared yet, so BOTH steps are needed.

  (a) In an ADMINISTRATOR PowerShell -- once, survives reboots:

          usbipd bind --busid $BUSID

  (b) In a normal PowerShell -- after every reboot or unplug:

          usbipd attach --wsl --busid $BUSID

  Windows loses the camera while it is attached. To give it back:

          usbipd detach --busid $BUSID
TXT
else
  cat <<TXT
  Already shared. Only the attach step is needed, in a normal PowerShell:

          usbipd attach --wsl --busid $BUSID
TXT
fi

if [ "${1:-}" = "--attach" ]; then
  echo
  echo "=== 4. trying the attach step now ==="
  powershell.exe -NoProfile -Command "usbipd attach --wsl --busid $BUSID" 2>&1 | sed 's/^/  /' || true
  sleep 4
  shopt -s nullglob
  vids=(/dev/video*)
  shopt -u nullglob
  if [ ${#vids[@]} -gt 0 ]; then
    echo "  attached: ${vids[*]}"
  else
    echo "  still no /dev/video* -- the bind step (a) has probably not been run."
  fi
fi
