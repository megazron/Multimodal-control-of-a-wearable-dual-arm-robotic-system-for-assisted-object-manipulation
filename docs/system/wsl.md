# WSL, networking and screen capture

Mirrored networking, the Kortex cyclic path, /mnt/c, /dev/shm, the Quest transport and x11grab under WSLg. Split out of CLAUDE.md on 2026-08-12. Nothing deleted.

Read this BEFORE debugging anything that looks like a driver fault on this machine. Several entries here cost a day each.

## WSL networking — MIRRORED, and the arms are reachable

- `eth0 192.168.1.20/24` → **mirrored mode** is active
  (`networkingMode=mirrored` in the Windows-side `%USERPROFILE%\.wslconfig`).
  Both arms answer: 192.168.1.10 (left) at ~1.35–1.7 ms RTT, 0% loss.
- Mirrored mode shares **all** Windows adapters, so WSL now sees `eth0`,
  `eth3` and loopback rather than the single NAT interface. That is normal.
  It does **not** partition ROS 2 discovery — see the ros2 daemon note below.
- `scripts/check_arm_network.sh`: 500-ping latency/jitter/loss, TCP 10000 and
  443, UDP 10001 probe, PASS/MARGINAL/FAIL verdict.

### "No ROS 2 nodes at all" is usually a STALE ros2 DAEMON

After the switch to mirrored mode, terminal 3 reported *"No ROS 2 nodes at
all"* while terminal 1 ran a healthy stack. There was **no discovery
partition**: `ros2 node list --no-daemon` saw all 28 nodes the whole time.
The **ros2 daemon** caches network state, did not survive the interface
change, and *hung* rather than failing — so `timeout 20 ros2 node list`
returned an empty string, which read as "nothing is running".

Fix and prevention:

```
ros2 daemon stop && ros2 daemon start
ros2 node list --no-daemon      # ground truth, bypasses the daemon
```

With a healthy daemon the default settings give 28/28 nodes on 5/5 runs
across eth0 + eth3 + lo. **Do not pin discovery to loopback** to "fix" this —
`ROS_LOCALHOST_ONLY` is deprecated in Jazzy and, applied to only some
processes, causes the very partition it is meant to prevent (the daemon caches
the env of whichever shell spawned it first). `scripts/env.sh` is the single
source of environment truth: it pins `ROS_DOMAIN_ID=0`, actively *unsets* any
inherited `ROS_LOCALHOST_ONLY`, and provides daemon-aware graph helpers.
Every script sources it, and `teleop.launch.py` applies the same settings
in-process because terminal 1 is launched by hand and never sources it.

`start_real.sh`'s preflight distinguishes the three states rather than
conflating them: nothing running / running but not discoverable (names the
daemon) / healthy.

## THE CYCLIC PATH IS UNUSABLE OVER WSL — use the high-level API

**This is the single most important operational fact about running the real
arm from this machine.** Do not spend another day on the ros2_control driver.

The Kortex **cyclic** path (what `ros2_control`'s
`KortexMultiInterfaceHardware` commands through) is built for a **1 kHz** loop
on a dedicated link. Over WSL every cyclic write is a **network round trip
costing ~10 ms**. Measured, in a read-only launch with no controller even
capable of commanding:

```
Read time: 217 us, Update: 282 us, Write time: 10365 us
```

At 100 Hz the whole cycle budget is 10 ms, so the write alone consumes it.
The controller manager overran permanently, **no command ever reached the
arm**, and the arm sat still while serving perfectly live feedback. Lowering
the CM rate does not help: the write is a round trip at any rate.

**How this misleads you.** The symptom is a frozen-looking arm:
`/real/joint_states` flat to 16 decimal places, homing errors identical for
15 s, then `HOMING STALLED`. Everything points at a dead driver or a faulted
arm. Both are wrong. Checks that ruled them out, worth repeating before
believing any new theory:

- `left_arm_controller` was **active**, with `Subscription count: 1`. Not a
  spawner-lock problem.
- No leaked session — a fresh single driver connected cleanly
  (`Session created`, no lock retry), and read a *different* angle than the
  previous session, so the feedback was genuine and not a cache.
- Arm not faulted: `ARMSTATE_SERVOING_READY`, all 7 actuators
  `fault=0x0/0x0`, 24.3 V, `SINGLE_LEVEL_SERVOING`.
- The feedback was flat **because nothing was commanding it**, not the
  reverse.

**The supported route here is the HIGH-LEVEL API**:
`SendJointSpeedsCommand` over the TCP session. Same ~10 ms round trip, but it
is a *velocity* command — it holds a motion between sends, so it does not need
1 kHz. `srl_teleop/kortex_highlevel_bridge.py` implements it and
`real_arms_highlevel.launch.py` is the launch that uses it. `start_real.sh`
points at that launch. Measured end to end, homing the real arm to within
tolerance on all 7 joints:

| Metric | Value |
|---|---|
| Loop rate | **18.4–18.7 Hz** (target 30) |
| Send latency | min ~17 ms, **avg ~26 ms**, max 33 ms |
| Tracking error | **0.01–0.15 deg** (setpoint vs actual) |
| Homing result | all 7 joints ≤ 0.98 deg, `WITHIN` tolerance |

30 Hz is not achievable: a cycle is **two** sequential round trips (read, then
write) and back-to-back RPCs cost ~26 ms each rather than the ~10 ms an
isolated call costs. ~18 Hz is still far more than the motion needs — at
vmax 0.05 rad/s a joint moves 2.8 mrad per cycle against a 1 deg (17 mrad)
deadband. **Always read the logged achieved rate; never assume `rate_hz`.**

### Two traps in that node, both already paid for

1. **Do not apply the velocity law twice.** `real_homing_node` and
   `sim_to_real_bridge` both publish an *incremental* setpoint (`cur + v*dt`,
   ~1.7 mrad ahead of measured). A second position law on top sees an error an
   order of magnitude *inside* the 1 deg deadband and commands exactly zero —
   the arm never moves, the setpoint never advances because it tracks measured
   position, and homing stalls at its starting error. The bridge therefore uses
   **feedforward + feedback**: `speed = clip(v_ff + kp*err, ±vmax)`, taking
   `v_ff` from the trajectory's `velocities[]` when present (homing fills it)
   and differentiating the setpoint stream otherwise. The deadband suppresses
   the **correction only**.
2. **Never use `time.time()` for intervals in WSL.** The wall clock steps
   backwards on host resync; it produced a send latency of **−2321 ms**. Use
   `time.monotonic()`. This same stepping is what makes
   `real.robot_state_publisher` log *"Moved backwards in time, re-publishing
   joint transforms"* every ~30 s — a clock artefact, **not** a driver fault.

### kortex_api must stay isolated

The Kinova wheel pins `protobuf==3.5.1`, which is **broken on Python 3.12**
(`collections.MutableMapping`) and, installed to user site, **shadows the
protobuf ROS needs**. It lives in `~/kortex_ws/.kortex_venv`, built with
`--system-site-packages` so one interpreter has both `rclpy` and `kortex_api`:

```
python3 -m venv --system-site-packages ~/kortex_ws/.kortex_venv
~/kortex_ws/.kortex_venv/bin/pip install protobuf==3.20.3
~/kortex_ws/.kortex_venv/bin/pip install --no-deps \
  https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl
```

`real_arms_highlevel.launch.py` runs the bridge through that interpreter via
`ExecuteProcess`. Do **not** `pip install kortex_api` into user site.

### One session, and closing it

The arm permits exactly **one** session and a leaked one blocks the next run.
The bridge shares a single session between read and write and closes it on
every exit path (`zero speeds → Stop() → CloseSession`), logging
`kortex session closed cleanly`. Grep for that line after any run. SIGINT the
bridge process directly if you have to kill things by hand — SIGKILL leaks
the session.

## /mnt/c is broken and CANNOT be fixed from inside WSL

`/proc/mounts` still shows the mount — `C:\134 /mnt/c 9p rw,aname=drvfs;path=C:\;...trans=fd,rfd=6,wfd=6` — but every access returns **EIO**. The
9p transport behind those file descriptors is dead; the mount entry is a
corpse. `sudo` on this machine **requires a password**, so the remount cannot
even be attempted unattended, and a remount would not help anyway: the 9p
channel is established by the WSL host process, not by the guest.

**Fix (must be done by a human, from Windows):**
```
wsl --shutdown          # in Windows PowerShell
```
then reopen the WSL terminal.

**Everything downstream stays PENDING until then:** Windows host LAN address,
`wsl --version`, the Windows build, whether this machine qualifies for
mirrored networking, and any `usbipd` check. Note the Teensy still works —
`usbipd` was attached earlier in the session, and `/run/usbipd-win` is a
*separate* 9p mount that is still alive.

## /real controller_manager rate, and the REAL blocker (2026-07-31)

`srl_teleop/config/real_controllers.yaml` now configures the /real CM
separately from the sim (keyed `/real/controller_manager`, 20 Hz). The sim's
`srl_moveit_config/config/ros2_controllers.yaml` is UNCHANGED at 100 Hz.
Spawner timeout raised to 120 s; `real_arms.launch.py` gained `home:=false`
and `bridge:=false` so the driver can be brought up without motion.

**The rate was a red herring.** Measured at both 30 Hz and 20 Hz:

    Read time  :      50 - 160 us       <- fast
    Write time : 3000000 - 6008589 us   <- 3 to 6 SECONDS, then
                 "timeout detected: BaseCyclicClient::RefreshFeedback"
    22 write timeouts, 47 overruns, achieved joint_states 4.0 Hz,
    feedback FROZEN (1 distinct value/joint, std 0.000e+00)

No update_rate absorbs a 3-second blocking call. From the driver banner:

    Port used          '10000'   TCP config  -> session created, 7 actuators
    Realtime port used '10001'   UDP cyclic  -> times out

TCP 10000 opens fine and ping is 1.5 ms. It is the **UDP realtime channel on
10001** that WSL2 NAT breaks. `/etc/wsl.conf` has no `networkingMode`, so this
box is on NAT.

**MIRRORED NETWORKING IS MANDATORY.** In Windows `%UserProfile%\.wslconfig`:

    [wsl2]
    networkingMode=mirrored

then `wsl --shutdown`. Until then the real arm CANNOT be commanded, and
`ros2 control list_controllers` showing "active" is misleading -- the
controllers activate and then the hardware deactivates underneath them.

**Trap:** the "fast" 50-160 us read is NOT an improvement over the original
11479 us. Once a write error deactivates the component, read() returns a
cache. A fast read here is the dead-channel signature, the same zero-variance
pattern as the frozen `/real/joint_states` and the dead j7 pot.

---

# QUEST TRANSPORT — the vendor servers, and the decision (2026-08-07)

Two RAR5 archives from `/mnt/c/Users/Gausms/Downloads`. No `unrar` and `sudo`
needs a password, but **`libarchive.so.13` is already present**, so
`libarchive-c` in a throwaway venv extracts RAR5 with no install and no root.

## What they are

| | hand pose | controller signal |
| --- | --- | --- |
| port | `ws://127.0.0.1:8765` | `ws://127.0.0.1:8766` |
| protocol tag | `wen.quest.handpose.v1` (validated) | none; requires `left`+`right` |
| transport | **`adb reverse tcp:8765 tcp:8765` over USB** | same, 8766 |
| payload | `left/right`: `tracked`, `confidence`, `position{xyz}`, `rotation{xyzw}`, plus `space`, `sequence` | `left/right`: `connected`, `positionTracked`, `rotationTracked`, `position`, `orientation`, `trigger`, `grip`, `thumbstick{x,y,pressed,touched}`, `buttons{x,y|a,b}`, plus `schemaVersion`, `sequence`, `timestamp` |
| rate | not set by the server — it MEASURES it (`packet_rate_hz`) | measures `receive_hz` |
| frame | **Unity: x right, y UP, z forward** (`unity_to_plot` maps Unity XYZ to plot X,Z,Y "so Unity Y is vertical") | same |
| direction | **receive-only — no `send()` anywhere** | receive-only |
| host | Windows: conda + `.bat`, `adb` from Android Platform-Tools | same |

The client is a **Unity app**, not a browser.

## THE TRANSPORT DECISION: adopt the vendor PROTOCOL, drop WebXR

**`adb reverse` over USB removes the entire HTTPS problem.** WebXR needs a
secure context, which on a LAN means a self-signed cert with the IP in
subjectAltName, a Windows firewall rule, and the operator accepting a warning
in the headset. With `adb reverse` the Quest connects to `ws://127.0.0.1`,
which is a secure origin by definition. No cert, no LAN IP, no firewall rule,
no warning. It is also USB, so it does not share the wifi with the arms.

**MEASURED, and it is what makes this work:** under `networkingMode=mirrored`,
**Windows can connect to a socket bound on 127.0.0.1 INSIDE WSL** — verified
by binding a listener in WSL and connecting from Windows PowerShell
(`WINDOWS->WSL localhost: CONNECTED`, accepted from 127.0.0.1:61830). So
`adb reverse` run on Windows lands directly on a ROS node in WSL. No relay
process, no second hop.

So: **speak the vendor wire protocol from a ROS node in WSL**, on the same
ports, with the same JSON. The user's existing Unity client then works
unmodified. What is NOT reused is the vendor process itself — it binds
127.0.0.1 in a Tkinter/matplotlib GUI on Windows under conda, and it is
receive-only, so it cannot carry the haptics or the state feedback the
in-headset overlay needs. The 31 KB file is mostly GUI; the protocol part is
~50 lines.

WebXR is kept as a fallback for a headset without USB access.

**REVISITED 2026-08-19, and the fallback is now the working route.** On a
BORROWED headset the adb route is not merely inconvenient, it is unavailable:
`adb` reports `unauthorized` and the in-headset prompt never appears because
turning on Developer Mode needs the device owner's account. None of the HTTPS
cost listed above went away — the cert, the SAN, the firewall rule and the
in-headset warning are all still real — but they are all one-time, and they
are on THIS side of the glass where they can be automated. They now are:
`scripts/make_vr_cert.sh`, `scripts/start_vr_wifi.sh`,
`scripts/vr_reachability.py`. The route is verified end to end by
`scripts/verify_vr_wifi_route.py`, which has two negative controls.

The one point the original decision got exactly right and is worth keeping:
**one origin.** The page and the socket are served by the same node on the
same TLS port, because a cert exception is per-origin and a WebSocket cannot
prompt for one.

## Environment facts found while doing this

- **WSL interop works only via the lowercase path** `/mnt/c/windows/...`.
  `/mnt/c/Windows/...` returns `Invalid argument` and an empty listing, which
  reads exactly like the dead-9p-mount failure and is not one.
- **`adb` is installed neither in WSL nor on Windows** (`Get-Command adb`
  returns nothing). Android Platform-Tools is a prerequisite.
- **The machine is no longer on the lab network.** `eth0` is DOWN and `eth3`
  holds `192.168.3.146/22`; `ping 192.168.1.10` is 100% loss. Nothing that
  needs a real arm can run here.

---

# SCREEN-RECORDING RVIZ UNDER WSLg (2026-08-08) — READ THIS BEFORE TRYING

## x11grab ON `:0` RECORDS BLACK. This will cost you a day.

`ffmpeg -f x11grab -i :0` with RViz plainly visible on screen produces a
**black video**. Measured, not inferred:

| grab | mean pixel value |
| --- | --- |
| full screen 1920x1200 on `:0`, RViz visible | **0.0** |
| the RViz window region on `:0` | **0.0** |
| the same RViz on **Xvfb `:99`** | **126.8** (std 110.4, 40742 distinct colours) |

**Why.** WSLg runs a Wayland compositor with XWayland. Window contents are
composited by Wayland and **never land in the X root window** that x11grab
reads. There is nothing wrong with the ffmpeg command; the pixels are not
there to be read. Nothing on the X side can fix it — not window ids, not
`-window_id`, not region offsets. All were tried.

## THE FIX: a virtual display with its own RViz

```bash
Xvfb :99 -screen 0 1600x1000x24 &
DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb \
  rviz2 -d src/srl_experiments/config/verification_capture.rviz &
ffmpeg -f x11grab -video_size 1600x1000 -framerate 12 -i :99.0 out.mp4
```

Xvfb has **no compositor**, so its root window really does hold the rendered
pixels. `LIBGL_ALWAYS_SOFTWARE=1` is required — llvmpipe renders RViz's
OpenGL fine and reports GL 4.5. This is a **second** RViz instance subscribing
to the same ROS graph; it is not a second stack and does not violate the
one-stack rule (no `master_pose_node` involved).

`scripts/record_rviz.py` does all of this itself and is idempotent about it.

**ffmpeg is not installed system-wide.** A static build was obtained without
root via `pip install --target <dir> imageio-ffmpeg` and copied to
`~/.local/bin/ffmpeg`. **`ffprobe` was NOT** — a verifier that shells out to
`ffprobe` silently reports "0 frames" for every clip, which is what the first
version of `verify_rviz_clips.py` did. Sample frames by TIME (`-ss`) instead
of by frame index; it needs no frame count.

## THE CLIP VERIFIER MUST BE CALIBRATED ON *RENDERED* COLOUR

`verify_rviz_clips.py` proves an object is on screen by counting pixels of its
colour. **Matching against the RGB set on the marker fails**, because RViz
lights and shades every surface:

| object | requested RGB | actually rendered |
| --- | --- | --- |
| ball | 242, 191, 26 | **189, 165, 74** |
| sling | 140, 89, 46 | 136, 111, 63 |
| target green | 26, 230, 51 | **68, 151, 63** |
| container teal | 13, 191, 179 | **45, 141, 140** |
| block orange | 255, 115, 0 | **207, 138, 35** |

The first version failed **11 clips whose objects I had just looked at**.
Detection now uses RELATIONS between channels (`R - B > 65`, `G - R > 45`),
which survive shading, with thresholds taken from pixel counts measured on
frames confirmed by eye. Validated both ways: every confirmed-good frame
detects its object (38-860 px), a black frame detects none.

**FOUR more calibration traps in the same file, all found by checking the
instrument against clips I had already looked at:**

- **The overlay text is detected as the object.** The HUD is white and
  orange-yellow — the same channel relations the ball and sling detectors
  match. Measured on one T6 frame: **587 "ball" pixels, of which 58 were the
  ball and 529 were the letters.** Because the text never moves it dragged
  every centroid toward a fixed point and made carried objects read as
  static. Every detector now ignores the top 24% of the frame.
- **Four temporal samples is UNDERSAMPLING.** Every clip carries ~1.5 s of
  approach and ~1.4 s of hold, so on a short scenario the moving part is a
  thin slice and four probes land mostly in the static hold. T3 S1 (a 50 mm
  lift) read 7.1 px and was called static; with nine samples it reads
  25.9 px. T7 S1 went 0.0 -> 27.5 px the same way.
- **Two targets share one centroid.** T7 and T9 draw a target per arm; their
  COMBINED centroid sits between them and barely moves even when both are
  orbiting. Measure each half of the frame separately.
- **A target sphere gets OCCLUDED BY THE GRIPPER** once the arm arrives on
  it, so the colour vanishes from the later frames of a perfectly good clip.
  Pursuit and reach tasks (T7/T8/T9) have no carried object at all and are
  judged by the arm's own travel and tracking error instead. The target
  marker was also enlarged to 100 mm and made translucent so it reads as a
  halo around the gripper rather than disappearing inside it.

**Two structural traps:**

- **"Nothing moved" is not a failure.** T9's zero-sway scenario is a
  stationary arm holding a world-fixed point ON PURPOSE. The check must fail
  only a genuinely FROZEN capture (frame delta < 0.05), not a still one.
- **Objects must be gated to the phase that owns them.** Drawing the sling
  during the approach — when the grippers are still ~1.46 m apart, because
  they start at home — makes a 350 mm sling read as taut and the ball "falls"
  before the task begins. S4 reported a drop on a scenario whose 58 mm sag
  clears the 40 mm ball comfortably.
- **MarkerArray must begin with DELETEALL every frame.** The topic is
  TRANSIENT_LOCAL and ids are reassigned per frame, so without it the previous
  run's container and blocks stay on screen underneath the next task's
  objects — two scenes in one picture.

## Two recordings per run, and they answer different questions

| file | what it is |
| --- | --- |
| `rviz.mp4` | **real screen capture.** What you would see at the machine. Watch this |
| `clip.mp4` | TF-rendered 3-D + front view. Ugly, but drawn from exactly the samples that produced `summary.json`, so picture and numbers cannot disagree |


---
