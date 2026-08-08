#!/usr/bin/env python3
"""Desktop mock of the Quest Unity client, speaking the VENDOR schema.

    ros2 run srl_vr_teleop quest_vendor_mock
    ros2 run srl_vr_teleop quest_vendor_mock --ros-args -p scenario:=tracking_loss

It connects to the bridge exactly as the headset does, emits the vendor's
JSON at `rate_hz`, and consumes the status frames the bridge pushes back --
so the whole path, including the overlay channel and the round-trip latency
measurement, is exercised with no headset present.

ROUND-TRIP LATENCY is measured HERE, by the client, because only the client
has one clock on both ends. The bridge echoes `timestamp` back untouched; the
client subtracts. Measuring it in the bridge would require the two clocks to
agree, which they do not, and would produce a confident wrong number.

Scenarios exist so the safety paths are exercised, not just described:

    normal          both controllers tracked, a slow circle
    tracking_loss   right controller drops tracking after 6 s
    link_loss       the client stops sending after 8 s (socket stays open)
    disconnect      the client closes the socket after 8 s
    clutch_cycle    grip pressed every 4 s, to index repeatedly
"""
import argparse
import asyncio
import json
import math
import statistics
import sys
import time


def controller(side, t, tracked=True, grip=0.0, trigger=0.0):
    r = 0.15
    return {
        "connected": True,
        "positionTracked": tracked,
        "rotationTracked": tracked,
        # Unity axes: x right, y UP, z forward.
        "position": {"x": (0.3 if side == "right" else -0.3)
                     + r * math.cos(0.6 * t),
                     "y": 1.1 + 0.05 * math.sin(0.4 * t),
                     "z": 0.35 + r * math.sin(0.6 * t)},
        "orientation": {"x": 0.0, "y": math.sin(0.1 * t),
                        "z": 0.0, "w": math.cos(0.1 * t)},
        "trigger": trigger,
        "grip": grip,
        "thumbstick": {"x": 0.0, "y": 0.0, "pressed": False, "touched": False},
        "buttons": ({"x": False, "y": False} if side == "left"
                    else {"a": False, "b": False}),
    }


async def run(uri, rate, dur, scenario):
    import websockets
    rtts = []
    n_status = 0
    last_status = {}
    print("mock client -> %s  scenario=%s" % (uri, scenario))
    async with websockets.connect(uri, ping_interval=None) as ws:

        async def reader():
            nonlocal n_status, last_status
            try:
                async for raw in ws:
                    d = json.loads(raw)
                    n_status += 1
                    last_status = d
                    e = d.get("echo", 0)
                    if e:
                        rtts.append((time.monotonic() - e) * 1000.0)
            except Exception:                                # noqa: BLE001
                pass

        task = asyncio.ensure_future(reader())
        t0 = time.monotonic()
        seq = 0
        while time.monotonic() - t0 < dur:
            t = time.monotonic() - t0
            if scenario == "link_loss" and t > 8.0:
                await asyncio.sleep(0.05)
                continue
            if scenario == "disconnect" and t > 8.0:
                break
            grip = 0.0
            if scenario == "clutch_cycle" and int(t) % 4 == 0 and t > 1:
                grip = 1.0
            right_tracked = not (scenario == "tracking_loss" and t > 6.0)
            seq += 1
            pkt = {
                "schemaVersion": 1,
                "sequence": seq,
                "timestamp": time.monotonic(),
                "left": controller("left", t, True, grip, 0.2),
                "right": controller("right", t, right_tracked, grip, 0.5),
            }
            await ws.send(json.dumps(pkt))
            await asyncio.sleep(1.0 / rate)
        await asyncio.sleep(0.4)
        task.cancel()

    print("  sent %d packets over %.1f s (%.1f Hz)"
          % (seq, dur, seq / max(dur, 1e-9)))
    print("  status frames received: %d" % n_status)
    if rtts:
        print("  ROUND-TRIP latency  n=%d  median %.2f ms  p95 %.2f ms  "
              "max %.2f ms" % (len(rtts), statistics.median(rtts),
                               sorted(rtts)[int(0.95 * len(rtts))], max(rtts)))
    else:
        print("  NO echo received -- the bridge is not pushing status back")
    if last_status:
        print("  last overlay state: frozen=%s reason=%r observer=%s"
              % (last_status.get("frozen"),
                 last_status.get("freeze_reason"),
                 last_status.get("observer_estop")))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--uri", default="ws://127.0.0.1:8766")
    ap.add_argument("--rate", type=float, default=72.0)
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--scenario", default="normal",
                    choices=["normal", "tracking_loss", "link_loss",
                             "disconnect", "clutch_cycle"])
    a, _ = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    return asyncio.run(run(a.uri, a.rate, a.duration, a.scenario))


if __name__ == "__main__":
    sys.exit(main())
