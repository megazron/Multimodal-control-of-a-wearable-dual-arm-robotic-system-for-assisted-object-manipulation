#!/usr/bin/env bash
# make_vr_cert.sh -- self-signed TLS cert for the wifi/WebXR VR route.
#
# WHY A CERT AT ALL. WebXR refuses to start a session outside a SECURE CONTEXT.
# Over USB (`adb reverse`) the origin is 127.0.0.1, which is secure by
# definition and needs no cert. Over wifi the origin is an IP address, so it
# must be https:// and there must be a certificate that CARRIES THAT IP.
#
# THE PART THAT CATCHES PEOPLE: a browser will not accept a bare CN any more.
# The address must appear in subjectAltName, as an IP: entry, not a DNS: one.
# A cert with the IP only in the CN produces ERR_CERT_COMMON_NAME_INVALID and
# no "proceed" link on some builds, which reads as "the cert is broken" when
# what is broken is the SAN list.
#
#   bash scripts/make_vr_cert.sh              # every LAN IP this host holds
#   bash scripts/make_vr_cert.sh --force      # replace one that already works
#   bash scripts/make_vr_cert.sh 192.168.3.146 srl-box.local
#
# IDEMPOTENT BY DEFAULT: if the cert on disk already carries this address and
# is not near expiry it is KEPT, because reissuing voids the exception the
# operator accepted in the headset and the resulting socket failure is
# completely silent.
set -euo pipefail

OUT="${VR_CERT_DIR:-$HOME/.srl_vr_cert}"
mkdir -p "$OUT"
CRT="$OUT/vr.crt"
KEY="$OUT/vr.key"

ARGS=()
for a in "$@"; do [ "$a" = "--force" ] || ARGS+=("$a"); done
if [ "${#ARGS[@]}" -gt 0 ]; then
  TARGETS=("${ARGS[@]}")
else
  mapfile -t TARGETS < <(hostname -I | tr ' ' '\n' | grep -E '^[0-9]' | grep -v '^127\.')
  TARGETS+=("$(hostname)")
fi

SAN=""
PRIMARY=""
for t in "${TARGETS[@]}"; do
  [ -z "$t" ] && continue
  if [[ "$t" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    SAN="${SAN}IP:${t},"
    [ -z "$PRIMARY" ] && PRIMARY="$t"
  else
    SAN="${SAN}DNS:${t},"
  fi
done
SAN="${SAN}IP:127.0.0.1,DNS:localhost"
[ -z "$PRIMARY" ] && PRIMARY="localhost"

# ---------------------------------------------------------------- IDEMPOTENT
# DO NOT REGENERATE A CERT THAT ALREADY WORKS.
#
# A self-signed cert is trusted by FINGERPRINT, not by address. Issuing a new
# one for the same IP produces a DIFFERENT certificate, which silently voids
# the exception the operator accepted inside the headset -- and a WebSocket
# cannot prompt for a new one, so the page may even load while the socket
# fails with no reason shown anywhere.
#
# Measured 2026-08-19: start_vr_wifi.sh called this on every restart, the
# operator set the headset down expecting to stream, and the bridge logged
# ZERO connection attempts for fifteen minutes. Nothing in the system said
# why, because from ROS's side nobody had connected.
#
# So: keep a cert that still covers this address and is not near expiry.
# --force to replace it deliberately.
FORCE=0
[ "${VR_CERT_FORCE:-0}" = "1" ] && FORCE=1
for a in "$@"; do [ "$a" = "--force" ] && FORCE=1; done

if [ "$FORCE" = "0" ] && [ -f "$CRT" ] && [ -f "$KEY" ]; then
  reuse=1
  for t in "${TARGETS[@]}"; do
    [[ "$t" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || continue
    openssl x509 -in "$CRT" -noout -ext subjectAltName 2>/dev/null \
      | grep -q "IP Address:${t}\b" || reuse=0
  done
  # 30 days of life left, so a session cannot expire mid-experiment
  openssl x509 -in "$CRT" -noout -checkend 2592000 >/dev/null 2>&1 || reuse=0
  if [ "$reuse" = "1" ]; then
    echo "KEEPING the existing certificate -- it already covers this address."
    echo "  $CRT"
    openssl x509 -in "$CRT" -noout -ext subjectAltName | tail -1 | sed 's/^/  /'
    echo "  fingerprint: $(openssl x509 -in "$CRT" -noout -fingerprint -sha256 | cut -d= -f2)"
    echo
    echo "  Reissuing would void the exception already accepted in the headset,"
    echo "  and a WebSocket cannot ask for a new one. Use --force if you really"
    echo "  mean to replace it -- then re-accept the warning in the headset."
    exit 0
  fi
fi

echo "subjectAltName = ${SAN}"

openssl req -x509 -newkey rsa:2048 -sha256 -nodes -days 825 \
  -keyout "$KEY" -out "$CRT" \
  -subj "/CN=${PRIMARY}/O=SRL wearable teleoperation/OU=sim only" \
  -addext "subjectAltName = ${SAN}" \
  -addext "basicConstraints = critical,CA:FALSE" \
  -addext "keyUsage = critical,digitalSignature,keyEncipherment" \
  -addext "extendedKeyUsage = serverAuth" 2>/dev/null

chmod 600 "$KEY"

echo
echo "cert: $CRT"
echo "key : $KEY"
echo
echo "--- what the browser will actually check ---"
openssl x509 -in "$CRT" -noout -subject -dates -ext subjectAltName

# The check that matters, and it CAN fail: is the address we intend to serve
# on actually in the SAN list? A cert built for yesterday's DHCP lease looks
# perfectly valid and is rejected in the headset for a reason nothing prints.
MISSING=0
for t in "${TARGETS[@]}"; do
  [[ "$t" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || continue
  if openssl x509 -in "$CRT" -noout -ext subjectAltName | grep -q "IP Address:${t}\b"; then
    echo "  OK      ${t} is in subjectAltName"
  else
    echo "  MISSING ${t} is NOT in subjectAltName"
    MISSING=1
  fi
done
[ "$MISSING" = "0" ] || { echo; echo "REFUSING: the cert does not carry an address you asked for."; exit 1; }
