#!/usr/bin/env bash
# scripts/start_gui.sh -- ONE COMMAND. Everything else is reachable from the
# window it opens.
#
#     bash scripts/start_gui.sh
#
# WHY THIS FILE EXISTS. The GUI was already one command -- once you had
# sourced two setup files in the right order, exported the one transport
# variable HARD CONSTRAINT 5 calls mandatory, and remembered that ROS's
# setup.bash cannot run under `set -u`. Three things to remember before the
# tool that exists so nothing has to be remembered. Now there are none.
#
# It also refuses rather than opening a second window: two of these is not a
# second stack, but it is two sets of launch buttons over one graph, and the
# operator cannot tell which window launched what.
set -euo pipefail

SRL_WS="${SRL_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
export SRL_WS

# HARD CONSTRAINT 10: ROS's setup.bash reads unbound variables and dies under
# `set -u`. env.sh handles that internally; this guard is for the source line
# itself.
set +u
# shellcheck disable=SC1091
source "$SRL_WS/scripts/env.sh"
set -u

# A SECOND WINDOW IS REFUSED BY NAME, not silently. procscan's rule applies:
# match on the script path, and never on a pattern this shell's own command
# line contains -- `pgrep -f srl_gui` matches the pgrep.
existing="$(pgrep -f "python3? .*scripts/srl_gui\.py" 2>/dev/null | grep -v "^$$\$" || true)"
if [ -n "$existing" ] && [ "${1:-}" != "--anyway" ]; then
  echo "An operations window is already open (pid $(echo "$existing" | tr '\n' ' '))."
  echo "Use that one, or close it first. Pass --anyway to open a second."
  exit 1
fi
[ "${1:-}" = "--anyway" ] && shift || true

# THE SCRATCH DIRECTORY IS WHERE A LAUNCHED JOB'S OUTPUT GOES. Without it the
# GUI falls back to /tmp, which works and makes the logs hard to find; with it
# every launch_<key>.log for a session is in one place.
export SRL_SCRATCH="${SRL_SCRATCH:-$SRL_WS/.scratch}"
mkdir -p "$SRL_SCRATCH"

echo "workspace   $SRL_WS"
echo "domain      ${ROS_DOMAIN_ID}   transport ${FASTDDS_BUILTIN_TRANSPORTS}"
echo "job logs    $SRL_SCRATCH"
echo
exec python3 "$SRL_WS/scripts/srl_gui.py" "$@"
