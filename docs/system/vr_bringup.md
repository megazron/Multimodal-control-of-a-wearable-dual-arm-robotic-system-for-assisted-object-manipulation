# VR bring-up — numbered checklist

Everything below except steps 2–4 and 11 has been **run and measured against
the desktop mock**. Steps 2–4 and 11 need the physical Quest.

---

## 0. What this path is, and why it is not WebXR

The Quest streams controller poses over a **WebSocket tunnelled by
`adb reverse` over USB**, to `ws://127.0.0.1:8766`. The client is the user's
**Unity** app, not a browser.

This removes the entire HTTPS problem. WebXR needs a secure context, which on
a LAN means a self-signed certificate with the IP in `subjectAltName`, a
Windows firewall rule, and the operator accepting a certificate warning
*inside the headset*. `127.0.0.1` is a secure origin by definition — no
certificate, no LAN address, no firewall rule, no warning. It is also USB, so
it does not share wifi with the arms.

**The measurement that makes a ROS node the right place for the server:**
under `networkingMode=mirrored`, Windows can connect to a socket bound on
`127.0.0.1` **inside WSL** (verified: bound a listener in WSL, connected from
Windows PowerShell, `CONNECTED`, accepted from `127.0.0.1:61830`). So
`adb reverse` run on Windows lands directly on the bridge. No relay process.

The WebXR bridge (`quest_bridge_node`, port 8765) is kept as a fallback for a
headset without USB access.

---

## 1. Prerequisites (one time)

- [ ] **Android Platform-Tools installed on WINDOWS.** `adb` is currently
      installed *neither* in WSL nor on Windows — `Get-Command adb` returns
      nothing. This is the one hard blocker.
- [ ] `pip install --user websockets` in WSL (done; 17.0.1).
- [ ] WSL interop reachable. **Use the lowercase path**
      `/mnt/c/windows/System32/...`; `/mnt/c/Windows/...` returns
      `Invalid argument` and an empty listing, which reads exactly like the
      dead-9p-mount failure and is not one.

## 2. Headset (needs the Quest)

- [ ] USB-C to the host, **Allow USB debugging** accepted on the headset.
- [ ] `adb devices` on Windows lists the Quest as `device`.

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
one exists. Publish `/observer_estop_present` from the observer's station.

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

**The Unity-side rendering of this is NOT built.** Writing a Unity scene I
cannot compile or test here would be worse than saying so: the panel to build
shows connection + `rx_hz` + round-trip ms, per-arm clutch and gripper,
motion scale, e-stop and observer-e-stop state, and — largest and
unmissable — `freeze_reason` whenever `frozen` is true.

## 11. What needs the physical Quest

- Actual headset frame rate and tracking quality (the mock's 68.5 Hz is the
  mock's rate, not the Quest's 72 Hz display rate).
- Whether `adb reverse` survives sleep/wake and cable re-seating.
- Real tracking dropouts, which are lighting- and pose-dependent and are not
  modelled by the mock's clean on/off.
- Latency over USB with a real Unity client (the mock runs on the same host,
  so 8.66 ms is a **floor**, not a prediction).
- The Unity overlay rendering.

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
