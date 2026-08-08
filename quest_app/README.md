# quest_app — the Quest-side client

**WebXR in the Quest browser.** No Unity, no licence, no sideloading, no build
step: edit `webxr/index.html`, reload the page on the headset.

## Run it

    # on the ROS host
    python3 -m http.server 8080 --directory quest_app/webxr
    ros2 launch srl_vr_teleop vr_teleop.launch.py

    # on the Quest, in the browser
    http://<ros-host-ip>:8080
    # set the ROS host box to ws://<ros-host-ip>:8765 and press Enter VR

WebXR requires a secure context for immersive sessions. `http://` works for
`localhost` only, so for a real headset either serve over HTTPS with a
self-signed certificate, or enable
`chrome://flags/#unsafely-treat-insecure-origin-as-secure` in the Quest
browser for your host. **This is the one setup step that will bite you**, and
it is the price of not using Unity.

## Why not Unity

| | latency | effort | risk |
| --- | --- | --- | --- |
| Unity + ROS-TCP-Connector | best in principle | licence, adb sideload, rebuild per iteration | a second build system nobody else here can run |
| **WebXR + WebSocket** | one headset frame + a few ms | open a URL | low |
| OpenXR on a tethered PC | lowest | needs Windows + Link; WSL cannot host it | tethering defeats a standalone headset |

The dominant latency term is the headset's own frame period (13.9 ms at 72 Hz),
which is identical for all three. See `quest_bridge_node.py` for the full
argument.

## Protocol

One JSON object per `requestAnimationFrame`, and the bridge replies with robot
state for rendering. The client stamps `t`; the bridge echoes it; the client
computes the round trip and sends it back as `rtt`. So latency is measured
end to end rather than estimated.
