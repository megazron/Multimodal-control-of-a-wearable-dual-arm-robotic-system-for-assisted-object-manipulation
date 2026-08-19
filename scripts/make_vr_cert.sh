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
#   bash scripts/make_vr_cert.sh 192.168.3.146 srl-box.local
set -euo pipefail

OUT="${VR_CERT_DIR:-$HOME/.srl_vr_cert}"
mkdir -p "$OUT"
CRT="$OUT/vr.crt"
KEY="$OUT/vr.key"

if [ "$#" -gt 0 ]; then
  TARGETS=("$@")
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
