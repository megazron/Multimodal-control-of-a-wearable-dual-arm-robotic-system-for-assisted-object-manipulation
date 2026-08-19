# VR teleoperation — architecture and design

## Why it is a separate system

The Quest path and the mannequin path are **two conditions in one study**, so
they must be runnable, citable and comparable independently. They share the
ROBOT side and nothing on the input side:

```
  mannequin: Teensy -> master_pose_node ─┐
                                          ├─> /master_arm_pose_<arm> -> ik_follower_node -> arm
  VR:        Quest  -> vr_pose_mapper   ─┘                              (+ e-stop, collision,
                                                                          srl_autonomy)
```

**Exactly one input may run at a time.** Two publishers on
`/master_arm_pose_<arm>` would fight. That is why they are separate launches
and are never merged.

`srl_vr_teleop` does not import `srl_teleop`, and `srl_teleop` does not import
`srl_vr_teleop`. Verified, not assumed.

## What VR gives that the mannequin cannot

Full 6-DOF **including yaw**, at 72 Hz, with no dead channels. Gravity gives
the mannequin roll and pitch but never yaw — a rank deficiency, not a
calibration problem — and on that rig j7 is railed and j3/j5 are dead. So the
mannequin runs `orientation_mode: fixed`; VR runs full Cartesian teleoperation
with orientation.

Every difference between `vr_pose_mapper` and `master_pose_node` traces back
to that one fact.

## Transport: two routes, one protocol shape

**Route A (default): Unity over `adb reverse` on USB, port 8766.**
**Route B (no adb): WebXR in the Quest's own browser over wifi, port 8765.**

Route B exists because Route A needs Developer Mode, an accepted USB-debugging
prompt and a sideloaded app — three things you do not have on a BORROWED
headset. It costs a self-signed certificate and a firewall rule and gains
nothing else; the client is a single HTML file the bridge serves itself, from
the same TLS origin as the socket so one accepted warning covers both.
Verified end to end on 2026-08-19 (`scripts/verify_vr_wifi_route.py`).

WebXR is **right-handed** and Unity is **left-handed**, so the two bridges
carry different conversions on purpose. See `vr_bringup.md` section 6.

## Why WebXR over a WebSocket is a reasonable transport

Chosen on latency and iteration cost. The dominant term is the headset's own
frame period — poses arrive in `requestAnimationFrame`, so pose age is bounded
below by 13.9 ms at 72 Hz **whatever the transport**, Unity included. The
WebSocket adds single-digit milliseconds on a LAN. Unity would buy a few ms of
a ~20 ms budget in exchange for a licence, adb sideloading, and a rebuild per
iteration.

rosbridge was rejected as the default: it wraps every message in a typed JSON
envelope and goes through a generic serialiser, where this bridge parses a
fixed 14-float protocol.

Measured on the mock: **controller 72.0 Hz, robot command 100 Hz, ROS-side lag
below the 5 ms measurement resolution.** The headset-to-bridge leg is
unmeasured until a Quest is connected — `scripts/vr_headset_check.py` is the
instrument, and its analysers are validated against constructed truth.

## Assistance differs from the mannequin, deliberately

| | mannequin | VR |
| --- | --- | --- |
| what the autonomy supplies | wrist ORIENTATION the input cannot express | PRECISION on the final approach |
| why | the pose is otherwise unreachable | the operator can already reach it |
| benefit | capability recovered | fewer failed attempts, lower workload |
| result shape | **categorical** (impossible → possible) | **continuous** improvement |

`vr_handover_arbiter` implements a **funnel**: inside the engage distance the
operator's pose is blended toward the validated grasp with a weight that grows
as they approach, reaching full authority only over the last 3 cm. The state
is still discrete and visible, entry still needs the P-threshold and
proximity, and it is cancellable at any moment. And it **yields** if the
operator starts rotating the controller — they are expressing an orientation
intention, and on this device they can.

## Safety, and the one that is unique to VR

**The operator cannot see the arm.** An independent observer e-stop, held by
someone not wearing the headset, is mandatory; `vr_safety_node` refuses to
enable real-arm control until `/vr/observer_estop_present` has been published.

Tracking loss, network dropout, a sleeping headset and a backgrounded app all
look identical from ROS, and all freeze the arm through the same 0.2 s
watchdog. **Freeze means publish nothing** — the follower holds its last
command. Publishing a "safe" pose would be commanding a motion nobody asked
for.

Workspace boundary warnings are rendered in-headset before the arm reaches its
limit, because the operator cannot see it approaching.
