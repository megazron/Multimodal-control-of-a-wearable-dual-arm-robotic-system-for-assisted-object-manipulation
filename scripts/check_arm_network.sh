#!/usr/bin/env bash
# check_arm_network.sh — run this when the Kinova arms are connected.
#
# Verdict is about JITTER as much as reachability. The Kortex cyclic loop on
# UDP 10001 runs near 1 kHz; if a cyclic packet arrives late the arm's
# watchdog faults and the session drops mid-motion. WSL2 NAT adds a userspace
# hop, so "ping works" is NOT sufficient evidence that teleop will be stable.
#
#   LEFT  192.168.1.10   RIGHT 192.168.1.9
#   TCP 10000 Kortex session, TCP 443 web UI, UDP 10001 cyclic real-time
set +u

# Shared ROS environment, for consistency with the other scripts. This one is
# a pure network diagnostic and makes no ROS calls, so a workspace that is not
# built must NOT stop it from running -- that is exactly when you need it.
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/env.sh" 2>/dev/null \
  || echo "  (scripts/env.sh unavailable -- continuing, no ROS calls needed here)"

LEFT=${LEFT:-192.168.1.10}
RIGHT=${RIGHT:-192.168.1.9}
COUNT=${COUNT:-500}

# Thresholds. avg RTT above 1 ms, or mdev above 0.5 ms, means the cyclic loop
# is at risk even if everything "works".
RTT_WARN_MS=1.0
JITTER_WARN_MS=0.5

fail=0
warn=0

echo "=================================================================="
echo " Kinova arm network check"
echo "=================================================================="

echo
echo "--- WSL networking mode ---"
ip -4 addr show eth0 2>/dev/null | grep -q 'inet 172\.' \
  && echo "  eth0 is on a 172.x WSL NAT subnet -> NAT MODE" \
  || echo "  eth0 is not on the usual WSL NAT subnet (mirrored mode?)"
ip -4 addr show 2>/dev/null | grep inet | sed 's/^/    /'
echo "  route to $LEFT:"; ip route get "$LEFT" 2>&1 | sed 's/^/    /'
echo
echo "  NAT mode adds a userspace hop and materially increases jitter."
echo "  If the verdict below is marginal, switch to mirrored networking:"
echo "    in Windows  %USERPROFILE%\\.wslconfig   add"
echo "        [wsl2]"
echo "        networkingMode=mirrored"
echo "    then from Windows:  wsl --shutdown"
echo "  Requires Windows 11 22H2+ and WSL 2.0+."

for pair in "LEFT $LEFT" "RIGHT $RIGHT"; do
  set -- $pair; name=$1; ip=$2
  echo
  echo "=================================================================="
  echo " $name  $ip"
  echo "=================================================================="

  echo "--- ping x$COUNT at 10 ms ---"
  if out=$(ping -i 0.01 -c "$COUNT" -W 1 "$ip" 2>&1); then
    echo "$out" | tail -3 | sed 's/^/  /'
    stats=$(echo "$out" | grep -oE 'min/avg/max/mdev = [0-9./]+' | awk -F'= ' '{print $2}')
    loss=$(echo "$out" | grep -oE '[0-9.]+% packet loss' | grep -oE '^[0-9.]+')
    if [ -n "$stats" ]; then
      avg=$(echo "$stats" | cut -d/ -f2); mdev=$(echo "$stats" | cut -d/ -f4)
      echo "  avg ${avg} ms, jitter(mdev) ${mdev} ms, loss ${loss}%"
      awk -v a="$avg" -v t="$RTT_WARN_MS" 'BEGIN{exit !(a>t)}' && {
        echo "  WARN: avg RTT ${avg} ms > ${RTT_WARN_MS} ms"; warn=1; }
      awk -v m="$mdev" -v t="$JITTER_WARN_MS" 'BEGIN{exit !(m>t)}' && {
        echo "  WARN: jitter ${mdev} ms > ${JITTER_WARN_MS} ms"; warn=1; }
      awk -v l="${loss:-0}" 'BEGIN{exit !(l>0)}' && {
        echo "  FAIL: ${loss}% packet loss - unacceptable for a 1 kHz loop"; fail=1; }
    fi
  else
    echo "  FAIL: no reply from $ip"; fail=1; continue
  fi

  echo "--- TCP ports ---"
  for port in 10000 443; do
    if timeout 3 bash -c "echo > /dev/tcp/$ip/$port" 2>/dev/null; then
      echo "  TCP $port  OPEN"
    else
      echo "  TCP $port  CLOSED/filtered"
      [ "$port" = 10000 ] && { echo "    FAIL: 10000 is the Kortex session port"; fail=1; }
    fi
  done

  echo "--- UDP 10001 (cyclic) ---"
  if command -v nc >/dev/null 2>&1; then
    nc -u -z -w 2 "$ip" 10001 >/dev/null 2>&1 \
      && echo "  UDP 10001 appears open" \
      || echo "  UDP 10001 no response (normal - UDP is connectionless;"
    echo "    only the real Kortex session proves the cyclic loop works)"
  else
    echo "  nc not installed; skipping (apt install netcat-openbsd)"
  fi
done

echo
echo "=================================================================="
if [ "$fail" = 1 ]; then
  echo " VERDICT: FAIL - do not proceed to bring-up step 1."
elif [ "$warn" = 1 ]; then
  echo " VERDICT: MARGINAL - reachable, but latency/jitter risks Kortex"
  echo "          watchdog faults. Switch to mirrored networking and re-run."
else
  echo " VERDICT: PASS - proceed to bring-up step 1."
fi
echo "=================================================================="
exit $fail
