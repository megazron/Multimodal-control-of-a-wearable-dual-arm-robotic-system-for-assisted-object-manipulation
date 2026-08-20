# 17. Seeing the environment from inside the headset

**DESIGN ONLY. Nothing in this document is built.**

Question: the operator must see the environment while wearing the headset.
Two routes -- the Quest's own passthrough cameras, or an external camera
streamed into the VR interface.

---

## 0. THE ANSWER TO "ADDITION OR ALTERNATIVE" FIRST, BECAUSE IT CHANGES THE REST

**Both routes are an ADDITION, and neither is on the critical path.**

The shipped setup does not require wearing the headset at all. The operator
sits across the room facing the wearer, holds the controllers as a 6-DOF input
device, and watches the real robot with their own eyes; the headset stands on
a shelf as the tracking reference with its proximity sensor taped
(`docs/system/vr_desk_operation.md`). The in-headset panel is dead weight in
that configuration, and the status the operator needs goes on a monitor beside
them via `scripts/vr_desk_monitor.py`.

So the honest framing is: **direct line of sight is the current design and it
is better than either route below.** Passthrough and a streamed camera both
exist to serve a DIFFERENT configuration -- one where the operator is not in
the room -- and that configuration has not been chosen. A remote operator is a
plausible future for a supernumerary limb system and is worth the design work;
it is not worth building before line-of-sight operation has been run once.

One caveat on the current claim. The brief describes it as "transient-pointer
gives controller poses with it on a table". What the client actually requests
is `immersive-vr` with `local-floor` (`web/vr_client.html:443`), reading
`gripSpace` per controller. The observed behaviour -- poses at 89.85 Hz with
the headset on a shelf -- is what matters and is measured, but it depends on
the headset staying awake, which is what the taped proximity sensor is for. It
is not a separate session mode.

---

## 1. ROUTE (a) -- THE QUEST'S OWN PASSTHROUGH CAMERAS

### What it needs

| | |
| --- | --- |
| session mode | `immersive-ar` instead of `immersive-vr`. On Quest 3 / Quest Pro / Quest 2, the browser composites passthrough as the AR background |
| client change | one line in `requestSession`, plus not clearing to an opaque colour -- the WebGL layer must be transparent (`alpha:true` is already set, `web/vr_client.html:440`), and `blendMode` must be checked rather than assumed |
| sideloading | **NONE.** This is the Quest browser's own WebXR AR mode |
| account | **NONE** |
| settings changed on the headset | **NONE** on Quest 3. On Quest 2 passthrough is greyscale and lower resolution but is still available to `immersive-ar` |
| raw camera pixels | **NOT AVAILABLE.** The Passthrough Camera API that exposes actual frames is a native Android permission on a signed app -- i.e. sideloading and an account. WebXR gets passthrough as a COMPOSITED BACKGROUND only: the page can draw over it, never read it |

### Works on a borrowed headset?

**Yes** -- and that is the significant fact. It is the only route here that
needs nothing installed, no account, and no settings changed, which are
exactly the constraints that ruled out adb and Quest Link in the first place.

The one thing to verify before promising it: `navigator.xr.isSessionSupported('immersive-ar')`
on the specific borrowed headset and browser version. The client already
performs and displays this check for `immersive-vr` (line 150) and gaining the
AR line is a two-line change.

### Latency added

**Effectively zero, and structurally it cannot be otherwise.** Passthrough is
composited by the headset's own compositor at display rate; it does not
traverse the network, the WebSocket, or this repository at all. The pose path
is unchanged, so the measured 9.88 ms one-way estimate and 89.85 Hz stand.

What it DOES cost is GPU and battery on the headset, and on Quest 2 the
passthrough image is greyscale, ~640x480 per eye, and noticeably delayed
relative to the world (tens of milliseconds) -- fine for "where is the table",
not fine for judging a 30 mm grasp gate.

### What it is actually good for

Situational awareness: not walking into the table, seeing the wearer, seeing
your own hands and the controllers. It is NOT a robot-side view -- it shows
the operator's own surroundings, which in desk operation they can already see.

---

## 2. ROUTE (b) -- STREAMING AN EXTERNAL CAMERA INTO THE VR INTERFACE

### What already exists

More than half of it. `srl_teleop/camera_vr_publisher.py` subscribes to the
wrist cameras, re-encodes to JPEG at a capped 15 Hz and publishes
`/vr_camera_<arm>/compressed` plus `/vr_camera_state`, with the SAME staleness
rule the desktop GUI uses (`camera_relay.ChannelState`) so the two cannot
drift apart. It never republishes an old frame. The transport to the headset
also exists: `quest_bridge_node` already serves the client page AND the `wss`
socket from ONE TLS origin, so one certificate exception covers both.

What does not exist is the last hop: those JPEGs are not sent to the client,
and the client does not draw them.

### What it needs

| | |
| --- | --- |
| bridge | a second WebSocket channel (or a binary frame type on the existing one) carrying JPEG bytes. Deliberately NOT the 14-float state message -- a 30 KB frame must never delay a pose |
| client | a textured quad in the XR scene per camera, drawn from an `ImageBitmap`. In `immersive-vr` it floats in a black world; in `immersive-ar` it floats over passthrough |
| sideloading | **NONE** |
| account | **NONE** |
| certificate | already solved -- same origin as the page |
| source camera | any: the two wrist cameras (exist, with a mock), or the scene camera of doc 16 (does not exist yet) |

### Works on a borrowed headset?

**Yes.** It is a web page drawing a texture. Nothing is installed.

### Latency added

This is the route where latency is a real number, and it is the sum of five
terms:

| term | estimate | note |
| --- | --- | --- |
| camera exposure + driver | 10-30 ms | the vendor driver's own pipeline; unmeasured here |
| ROS transport to the relay | ~1 ms | same host, SHM |
| JPEG encode | 5-15 ms | at 640x480, quality 60, on this laptop's CPU |
| rate cap | up to 67 ms | `max_hz` is 15, so a frame can wait most of a period. **This is the largest single term and it is a parameter, not a law** |
| WebSocket transit on LAN | 2-4 ms | measured for the pose path |
| decode + composite on the headset | 15-30 ms | `createImageBitmap` plus one frame of `requestAnimationFrame` at 72 Hz |

**Total: roughly 35-145 ms**, dominated by the rate cap and the encode. That
is one to two orders of magnitude worse than passthrough and it is worth
saying plainly: **this is not a view you can servo a grasp against.** It is a
view you can inspect a scene with.

Raising `max_hz` to 30 and dropping to 320x240 would bring the typical case to
~40-60 ms, which is usable for coarse teleoperation and still not for
fine manipulation.

### The failure mode that has to be designed in from the start

In a headset the operator has NO other window onto the workspace. A frozen
frame of a still scene and a live frame of a still scene are the same picture,
and the consequence of confusing them is worse here than on the desktop --
which is precisely why `camera_vr_publisher` already refuses to republish an
old frame and publishes the CHANNEL STATE separately. The client must draw the
state, not just the picture: past the staleness limit the quad goes to a flat
"NO SIGNAL" panel, never a dimmed last frame.

---

## 3. RECOMMENDATION

1. **Neither, for now.** Line of sight is better than both and is what the
   current design uses. Run a session that way first.
2. **If a headset-worn session is wanted: route (a), passthrough.** It is a
   two-line client change, costs no latency, needs nothing installed and works
   on a borrowed headset. It buys situational awareness, which is the thing a
   head-worn operator actually lacks.
3. **Route (b) only when the operator is not in the room**, and then from the
   SCENE camera rather than the wrist cameras -- a wrist camera 100 mm from a
   cube is not a view of a workspace. Budget the latency honestly and label
   the panel with it.
4. The two compose: `immersive-ar` with a camera quad floating in it gives the
   operator their real surroundings AND the robot's view, which is the
   configuration worth building if remote operation is ever chosen.

---

## 4. COST

| item | cost | notes |
| --- | --- | --- |
| (a) passthrough: `immersive-ar` + support check + transparent clear | **0.5 session** | plus one lab check on the borrowed headset |
| (a) verify the pose path is untouched by the mode change | **0.5 session** | rate and latency must re-measure at 89-90 Hz and ~10 ms, or the change is reverted |
| (b) binary frame channel in `quest_bridge_node`, separate from the state message | **1 session** | must not delay poses; needs a backpressure rule (drop, never queue) |
| (b) client-side textured quad + staleness panel | **1 session** | |
| (b) end-to-end latency measurement, stamped at the camera | **1 session** | without this the number above stays an estimate, and an unmeasured latency in a view someone steers by is not acceptable |
| (b) from the scene camera instead of the wrist cameras | **+0** | it is another topic; the relay is already generic |
| **total (a)** | **~1 session** | |
| **total (b)** | **~3 sessions** | on top of (a) if both |
