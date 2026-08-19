# VR bring-up — numbered checklist

**TWO ROUTES. `adb` is a prerequisite of ROUTE A ONLY.** Route B needs no adb,
no USB and no sideloading, and works on a headset you are not allowed to
modify. Pick the route first; the numbered steps below are Route A's.

| | **A — USB / Unity** | **B — wifi / WebXR** |
| --- | --- | --- |
| client | the vendor **Unity app**, sideloaded | the headset's **own browser** |
| transport | `adb reverse tcp:8766` over USB | wifi to this host's LAN address |
| needs `adb` | **YES, and it is a hard blocker** | **NO** |
| needs Developer Mode / an accepted USB-debugging prompt | yes | **no** |
| needs the app installed on the headset | yes | **no** |
| certificate | none — `127.0.0.1` is a secure origin | **yes**, self-signed, IP in `subjectAltName` |
| firewall | none | inbound TCP must be allowed |
| bridge | `quest_vendor_bridge`, port 8766 | `quest_bridge_node`, port 8765 |
| start with | `bash scripts/start_vr.sh` | `bash scripts/start_vr_wifi.sh` |
| shares wifi with the arms | no | yes |
| verified | mock only; the Unity leg has never run | **end to end, 2026-08-19**, `scripts/verify_vr_wifi_route.py` |

**Route A is preferred when you can have it** — no cert, no firewall, no
warning to accept inside a headset, and it does not share the wireless network
the arms use. **Route B is the one that works on a BORROWED headset**, because
it changes nothing on the device: WebXR runs in the Quest's own browser and
`adb` is only ever needed for sideloading and for the USB tunnel. An `adb`
that reports `unauthorized` is Route A being unavailable. It is not a Route B
problem and never was.

---

# ROUTE B — WIFI AND WebXR. NO adb.

    bash scripts/vr_reachability.py     # step 1: can the headset see us AT ALL
    bash scripts/start_vr_wifi.sh       # steps 2-4: cert, firewall, bridge, page

## B0. What it costs, said before you put the headset on

WebXR refuses to start a session outside a **secure context**. Over USB the
origin is `127.0.0.1`, which is secure by definition. Over wifi the origin is
an IP address, so:

* the page must be **https**, with a certificate carrying **this machine's IP
  in `subjectAltName`** — an IP in the CN alone is rejected;
* the certificate is self-signed, so the operator accepts **one warning**;
* **the page and the WebSocket are served from ONE port on purpose.** A
  certificate exception is granted per ORIGIN, and a WebSocket handshake
  cannot show a prompt — it fails with no reason at all. Page on 8443 and
  socket on 8765 gives you an accepted warning, a page that looks fine, and a
  socket that silently never connects. `quest_bridge_node` serves both.

## B1. Prove the network FIRST, before any certificate

    python3 scripts/vr_reachability.py          # plain HTTP, on purpose

Open `http://<this host>:8080/` in the headset's browser. **Plain HTTP with no
TLS is the point**: lab wifi very often has AP client isolation, where two
clients on one SSID each reach the internet and neither reaches the other, and
from inside a headset that is INDISTINGUISHABLE from a certificate problem.
Skipping this step means debugging the wrong layer. Every hit prints in the
terminal, so nothing has to be read inside the headset.

It also runs a WebSocket echo on 8081, because some networks pass HTTP through
a proxy that eats the `Upgrade`. HTTP alone is not proof; the VR path is a
socket.

`--selftest` probes a live port and a dead one and requires different answers,
so a NO from it means something.

**If nothing reaches it:** same SSID (not a guest or 2.4/5 GHz split), ask for
AP isolation off, or use a phone hotspot — a hotspot never isolates. Only then
look at the firewall.

## B2. Certificate

    bash scripts/make_vr_cert.sh                # every LAN IP this host holds

Written to `~/.srl_vr_cert/`. It **refuses** if the address you asked for is
not in the SAN list. `start_vr_wifi.sh` regenerates automatically when the
DHCP lease has moved — a cert built for yesterday's address is perfectly valid
and is rejected in the headset as `ERR_CERT_COMMON_NAME_INVALID`, which reads
like a broken certificate and is a stale one.

## B3. Windows firewall

Under `networkingMode=mirrored` this box's LAN address **is** the Windows LAN
address, so no `netsh portproxy` is needed — but Windows Firewall still
applies, and **mirrored WSL has a second firewall in front of the first**.
`start_vr_wifi.sh` checks for the rule and prints both commands if it is
missing. In an **admin** PowerShell on Windows:

    New-NetFirewallRule -DisplayName 'SRL VR bridge (WSL) TCP 8765' `
      -Direction Inbound -Action Allow -Protocol TCP `
      -LocalPort 8765,8080,8081 -Profile Any

    Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' `
      -DefaultInboundAction Allow

Without the second one the first appears to do nothing.

## B4. Start, and what to do inside the headset

    bash scripts/start_vr_wifi.sh          # --check to diagnose and stop

1. Put the headset on. Open the **Meta Quest Browser** (the built-in one — it
   is the WebXR runtime; Firefox Reality is discontinued and Wolvic is not
   installed on a stock headset).
2. Go to **`https://<the address the script prints>:8765/`**. Type it into the
   address bar; there is no QR step.
3. You get **"Your connection is not private"**, `NET::ERR_CERT_AUTHORITY_INVALID`.
   That is the self-signed certificate and it is expected.
   Press **Advanced**, then **Proceed to \<ip\> (unsafe)**.
   An accepted exception still counts as a secure context, so WebXR works.
4. The page shows `secure context: yes`, `immersive-vr supported`, and
   `socket: OPEN`. If the page loads and the socket does not, the certificate
   was accepted for a DIFFERENT origin than the socket — check the port.
5. Press **ENTER VR** and pick up the controllers. The overlay panel appears
   below your line of sight.

## B5. Verify the whole route without a headset

    python3 scripts/verify_vr_wifi_route.py
    python3 scripts/verify_vr_wifi_route.py --break-cert    # must FAIL
    python3 scripts/verify_vr_wifi_route.py --break-frame   # must FAIL

Starts the real bridge with the real certificate, fetches the real page over
HTTPS with hostname checking ON, opens a real `wss://`, speaks the protocol
and checks the poses land on the ROS topics in the world frame. The two
`--break` flags are negative controls and the script exits non-zero if either
of them PASSES.

**Measured 2026-08-19, all seven checks pass.**

## B6. Two defects found by running it for the first time

* **`quest_bridge_node` had never started.** `self.clients = set()` in
  `__init__` collides with `rclpy.node.Node.clients`, a read-only property, so
  construction raised `AttributeError` every time. The node has been in the
  tree as "the WebXR fallback" since 2026-08-07 and one attempt to run it
  would have found this. Renamed to `ws_clients`.
* **Its axis mapping was for a different frame.** It shipped `[-z, -x, y]`,
  which lands in the ROS x-forward / y-left convention — not this repo's world
  frame (section 6). `vr_pose_mapper` adds the controller displacement
  **straight** onto the robot's world pose with no aligning rotation, so a
  hand pushed 3 m forward and 1 m right commanded 3 m to the RIGHT and 1 m
  BACKWARD. WebXR is right-handed (+z toward the viewer), Unity is left-handed
  (+z forward), so the two transports need different conversions and must not
  share one. Fixed to `[x, -z, y]` with `w` untouched, and pinned by
  `test_webxr_frames.py`, whose first tests fail against the shipped mapping.

---

# ROUTE A — USB, Unity and adb

Everything below except steps 2–4 and 11 has been **run and measured against
the desktop mock**. Steps 2–4 and 11 need the physical Quest **and** an
authorised `adb`.

---

## 0. What this route is

The Quest streams controller poses over a **WebSocket tunnelled by
`adb reverse` over USB**, to `ws://127.0.0.1:8766`. The client is the user's
**Unity** app, not a browser.

This removes the entire HTTPS problem — `127.0.0.1` is a secure origin by
definition, so no certificate, no LAN address, no firewall rule, no warning.
It is also USB, so it does not share wifi with the arms.

**The measurement that makes a ROS node the right place for the server:**
under `networkingMode=mirrored`, Windows can connect to a socket bound on
`127.0.0.1` **inside WSL** (verified: bound a listener in WSL, connected from
Windows PowerShell, `CONNECTED`, accepted from `127.0.0.1:61830`). So
`adb reverse` run on Windows lands directly on the bridge. No relay process.

---

## 1. Prerequisites (one time)

- [ ] **Android Platform-Tools installed on WINDOWS.** `adb` is currently
      installed *neither* in WSL nor on Windows — `Get-Command adb` returns
      nothing. **This blocks ROUTE A ONLY.** Route B above needs none of it.
- [ ] `pip install --user websockets` in WSL (done; 17.0.1). **Both routes.**
- [ ] WSL interop reachable. **Use the lowercase path**
      `/mnt/c/windows/System32/...`; `/mnt/c/Windows/...` returns
      `Invalid argument` and an empty listing, which reads exactly like the
      dead-9p-mount failure and is not one.

## 2. Headset (needs the Quest AND permission to change it)

- [ ] Developer Mode enabled on the headset — needs the **owner's** Meta
      account and an organisation. **Not available on a borrowed headset.**
- [ ] USB-C to the host, **Allow USB debugging** accepted on the headset.
- [ ] `adb devices` on Windows lists the Quest as `device`.

      `unauthorized` means the in-headset prompt has not been accepted. If it
      never appears, Developer Mode is off, and turning it on requires
      settings you may not have. **That is the point to switch to Route B**,
      not the point to keep fighting adb.

## 3. Start (needs the Quest)

    bash scripts/start_vr.sh

The script runs `adb reverse tcp:8766 tcp:8766` on the Windows side, then
starts `quest_vendor_bridge` and `wearer_view`. It **refuses with a named
reason** if adb is missing, if no device is visible, or if `adb reverse`
fails.

## 4. Launch the Unity client (needs the Quest)

- [ ] It connects to `ws://127.0.0.1:8766`.
- [ ] The bridge logs `Quest client connected`.

---

## 5. Verify WITHOUT the headset — this is fully runnable now

    bash scripts/start_vr.sh --mock

**Measured:** 822 packets in 12.0 s (**68.5 Hz**), 242 status frames pushed
back, **round-trip latency median 8.66 ms, p95 16.5 ms**.

Round trip is measured by the CLIENT, because only the client has one clock
at both ends. The bridge echoes the client's `timestamp` untouched. Measuring
it in the bridge would require the two clocks to agree, which they do not,
and would produce a confident wrong number.

## 6. Frames — stated explicitly

**The two routes do NOT share a conversion, and must not.**

| | Unity (Route A) | WebXR (Route B) |
| --- | --- | --- |
| handedness | **left** | **right** |
| +z is | forward | **backward**, toward the viewer |
| position | `(x, z, y)` | `(x, -z, y)` |
| quaternion | `(x, z, y, -w)` — the flip is carried by `-w` | `(x, -z, y, w)` — **no flip; w untouched** |
| lives in | `quest_vendor_bridge.quest_to_world_*` | `quest_bridge_node.webxr_to_world_*` |
| pinned by | `test_quest_frames.py` | `test_webxr_frames.py` |

Both land in the same place: this repo's world frame. Copying one into the
other inverts every rotation, and `test_webxr_frames.py` has a test whose only
job is to fail if someone "unifies" them.

Unity/Quest is **left-handed: x right, y UP, z forward**.
This repo's world frame is **x = wearer's RIGHT, y = FORWARD, z = UP**
(*not* the ROS x-forward convention).

    p_world = (x_q, z_q, y_q)
    q_world = (qx, qz, qy, -qw)

Swapping two axes flips handedness, which is exactly the conversion needed;
the rotation carries the flip in the negated scalar part. Seven tests in
`test_quest_frames.py` pin it, including that the result is always a proper
rotation (det = +1) and that **position and orientation end up in the same
frame** — if they disagree, IK is asked for a pose that does not exist.

A yaw about Unity's up axis comes out **negated** about world z, and that is
correct: +θ in a left-handed frame is the same physical turn as −θ in a
right-handed one. A conversion that preserved the sign would turn the
operator's wrist the wrong way.

## 7. Controls

| input | action |
| --- | --- |
| **grip** (rising edge) | clutch toggle. Disengage freezes the arm where it is; engage latches the reference at the controller's CURRENT pose, so the jump is zero by construction |
| **trigger** | gripper, proportional |
| `scale` parameter | live, applied about the engage point |

Verified: `clutch_cycle` scenario logs `DISENGAGED` then
`ENGAGED -- reference latched at the controller's CURRENT pose (...)`.

## 8. Safety — all three verified against the mock

| guard | limit | measured |
| --- | --- | --- |
| tracking loss | 0.2 s | fired at **0.24 s** |
| link loss | 0.3 s | fired at **0.32–0.38 s** |
| **observer e-stop absent** | — | **refuses to drive at all** |

    [VR] FROZEN: observer e-stop NOT confirmed -- refusing to drive the arm
    while the operator cannot see it

`require_observer_estop` defaults **true** and must stay true for any session
with a real arm. The operator is blind inside the headset; a second person
holding a stop is not optional, and the system refuses rather than assuming
one exists. Publish **`/vr/observer_estop_present`** from the observer's station — that is the topic `vr_safety_node` subscribes to. (`start_vr.sh` greps for the unprefixed substring, which matches either, so a publisher on the wrong name passes the script's check and never reaches the node.)

Freezing stops commands. It does **not** re-home and does **not** release the
gripper — dropping a held object because a tracker blinked is worse than
holding still.

## 9. First-person view

    ros2 run srl_vr_teleop wearer_view

Publishes TF `head -> wearer_eyes` (+90 mm forward, +60 mm up, looking along
the wearer's **+y**) plus a CameraInfo. Verified with `tf2_echo`.

This is not cosmetic. These are limbs bolted to a person's back, and whether
they feel *attached* or *external* is what the experiments measure. A
third-person orbit camera answers that question the wrong way before the
study starts.

## 10. In-headset overlay — the DATA channel is built and running

The bridge pushes this to the client at `status_hz` (20 Hz) on the same
socket, and republishes it on `/vr_state`:

```json
{"type":"status","seq":822,"rx_hz":68.5,"echo":<client ts>,
 "frozen":false,"freeze_reason":"","estop":false,"observer_estop":true,
 "scale":1.0,
 "arms":{"left":{"clutch":true,"tracked":true,"gripper":0.2,"cmd":[x,y,z]},
         "right":{...}}}
```

The **vendor server cannot do this** — it is receive-only, with no `send()`
anywhere. That is fine for a visualiser and fatal for teleoperation.

**The Unity-side rendering (Route A) is still NOT built.** Writing a Unity
scene that cannot be compiled or tested here would be worse than saying so.

**The WebXR rendering (Route B) IS built**, in
`src/srl_vr_teleop/web/vr_client.html`: a head-locked WebGL panel showing
link state, pose rate, round trip, per-arm clutch / scale / gripper /
tracking, workspace-edge margins, and — first, largest, on a red field —
`FROZEN` with `freeze_reason`.

`freeze_reason` was **not reaching the client**: `/vr/safety` carries it and
`vr_feedback_node` forwarded only the boolean, so a panel could say FROZEN and
never say why — and "tracking lost", "observer e-stop withdrawn" and "network
dropout" need three different reactions from an operator who cannot see the
arm. Now forwarded.

## 11. What needs the physical Quest

**Run `python3 scripts/vr_headset_check.py`** — it walks the operator through
all seven and writes the numbers. `--selftest` validates every analyser
against constructed truth AND against a deliberately broken trace, with no
headset, and it has already caught one wrong analyser in this file's own code.

1. **Actual headset frame rate and tracking quality.** The mock's 68.5 Hz is
   the mock's loop rate, not the Quest's 72 Hz display rate.
2. **End-to-end latency.** The mock runs on this host, so 8.66 ms is a
   **floor**, not a prediction. Over wifi the leg being added is real.
3. **Tracking loss freezing the arm inside 0.2 s** — from an induced,
   *real* dropout. The mock's clean on/off is not what a covered controller
   does; real dropouts are lighting- and pose-dependent.
4. **Clutch re-engage jump, in mm**, measured from the COMMAND topic rather
   than from the mapper's own self-report.
5. **Live scaling from the thumbstick**, including that the command does not
   STEP when the scale changes — an un-rebased change is worst exactly when
   the operator is far from the engage point, so it is measured there.
6. **The gripper on the trigger**: proportional in, latch, no sag while held,
   release only after the hold.
7. **The in-headset overlay**, whose data channel is measurable from ROS and
   whose rendering only a person wearing it can confirm.

Route A additionally needs: whether `adb reverse` survives sleep/wake and
cable re-seating, and the Unity overlay rendering.

## 12. The scientific difference, for the write-up

On the **mannequin**, 7 of 14 channels are incoherent and the wrist rolls are
dead, so autonomy supplies **degrees of freedom the input cannot measure** —
capability recovery, a categorical result.

On **VR** the input is fully capable: 6-DOF pose at high rate with no dead
channels. Autonomy there cannot add capability, so it supplies **precision
and reduced workload** instead.

Different claims, different measurements, and they must not be pooled. The
same autonomy code producing "the task became possible" on one input and "the
task became easier" on the other is the interesting result, not a
confound — provided the two are reported separately.
