#!/usr/bin/env python3
"""Microphone -> wake word -> transcript -> /voice_transcript.

    ros2 run srl_autonomy voice_listener                      # real STT
    ros2 run srl_autonomy voice_listener --ros-args -p source:=text

AUDIO UNDER WSL IS THE HARD PART, AND A DEVICE PASSTHROUGH IS THE WRONG FIX
---------------------------------------------------------------------------
MEASURED on this machine: `/dev/snd` contains only `timer`. There is no
capture device, so ALSA and PulseAudio inside WSL have nothing to open.
`usbipd` CAN forward a USB microphone, but it takes exclusive ownership away
from Windows, and this rig already relies on `usbipd` for the Teensy and will
need it for the Quest -- competing for the same mechanism to get audio is a
poor trade for a stream that is one-way and low bandwidth.

So the shipped path is NETWORK AUDIO: a tiny sender runs on Windows, captures
the default microphone, and posts 16 kHz mono PCM to this node over UDP.
Under `networkingMode=mirrored` Windows reaches WSL's own 127.0.0.1 --
verified separately for the Quest bridge -- so there is no address to
configure and no firewall rule. `scripts/win_mic_sender.py` is that sender.

`source:=text` bypasses audio entirely and reads utterances from
`/voice_text_in`, which is how every autonomy test in this repo runs: it
makes voice-to-motion latency measurable without a microphone, and it is the
only way the refusal matrix can be exercised deterministically.

STT MODEL -- faster-whisper `small`, int8, CTranslate2
------------------------------------------------------
Chosen against a 4 GB VRAM ceiling (RTX A500 Laptop). See CLAUDE.md for the
full comparison. `small` int8 is ~1 GB and 3.4% WER; `large-v3-turbo` would
be better English but needs ~6 GB and would leave nothing for the detector,
which has to be resident at the same time.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, "/home/gausms/kortex_ws/src/srl_autonomy")
from srl_autonomy import voice_intent as vi          # noqa: E402


class VoiceListener(Node):
    def __init__(self):
        super().__init__("voice_listener")
        self.declare_parameter("source", "text")      # text | udp
        self.declare_parameter("udp_port", 5599)
        self.declare_parameter("model", "small")
        self.declare_parameter("compute_type", "int8")
        self.declare_parameter("wake_word", vi.WAKE_DEFAULT)
        self.declare_parameter("require_wake", True)

        self.source = str(self.get_parameter("source").value)
        self.wake = str(self.get_parameter("wake_word").value)
        self.require_wake = bool(self.get_parameter("require_wake").value)

        self.pub = self.create_publisher(String, "/voice_transcript", 10)
        self.status = self.create_publisher(String, "/voice_status", 10)
        self.create_subscription(String, "/voice_text_in", self._on_text, 10)
        self.stt = None

        if self.source == "udp":
            self._start_stt()
            self._start_udp()
        else:
            self.get_logger().warn(
                "source:=text -- reading utterances from /voice_text_in. No "
                "microphone is opened. This is the mode every autonomy test "
                "uses, because it makes voice-to-motion latency measurable "
                "without audio hardware.")
        self.get_logger().info("wake word: %r (STOP bypasses it)" % self.wake)

    # ------------------------------------------------------------- input
    def _on_text(self, m):
        """A typed utterance is treated EXACTLY like a transcribed one, so
        the parse path under test is the shipped one."""
        self._emit(m.data, latency_ms=0.0, src="text")

    def _emit(self, text, latency_ms, src):
        self.pub.publish(String(data=text))
        self.status.publish(String(data=json.dumps(
            dict(text=text, stt_ms=round(latency_ms, 1), source=src,
                 wake=self.wake))))
        self.get_logger().info("[HEARD:%s %.0f ms] %r" % (src, latency_ms,
                                                          text))

    # --------------------------------------------------------------- STT
    def _start_stt(self):
        try:
            from faster_whisper import WhisperModel
        except ImportError:
            self.get_logger().error(
                "faster-whisper is NOT installed, so no audio can be "
                "transcribed. Install it, or run with source:=text. "
                "Refusing to pretend: an STT that silently produces nothing "
                "looks identical to an operator who said nothing.")
            return
        m = str(self.get_parameter("model").value)
        ct = str(self.get_parameter("compute_type").value)
        t0 = time.monotonic()
        self.stt = WhisperModel(m, device="cuda", compute_type=ct)
        self.get_logger().info(
            "faster-whisper %s/%s loaded in %.1f s" % (m, ct,
                                                       time.monotonic() - t0))

    def _start_udp(self):
        import socket
        import threading
        port = int(self.get_parameter("udp_port").value)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", port))
        self.get_logger().warn(
            "listening for 16 kHz mono PCM on udp://127.0.0.1:%d -- run "
            "scripts/win_mic_sender.py on WINDOWS. /dev/snd here holds only "
            "`timer`, so there is no capture device to open inside WSL."
            % port)

        def loop():
            import numpy as np
            buf = bytearray()
            while rclpy.ok():
                try:
                    chunk, _ = s.recvfrom(65535)
                except OSError:
                    return
                buf += chunk
                # 1.5 s of 16 kHz int16 mono
                if len(buf) < 2 * 16000 * 3 // 2:
                    continue
                audio = np.frombuffer(bytes(buf), dtype="<i2")
                buf = bytearray()
                if self.stt is None:
                    continue
                t0 = time.monotonic()
                segs, _ = self.stt.transcribe(
                    audio.astype("float32") / 32768.0, language="en",
                    vad_filter=True)
                text = " ".join(x.text for x in segs).strip()
                if not text:
                    continue
                dt = (time.monotonic() - t0) * 1000.0
                heard, _ = vi.strip_wake(text, self.wake)
                is_stop = vi.parse(text, wake=self.wake).verb == "stop"
                if self.require_wake and not heard and not is_stop:
                    self.get_logger().debug("no wake word in %r" % text)
                    continue
                self._emit(text, dt, "udp")

        threading.Thread(target=loop, daemon=True).start()


def main(argv=None):
    rclpy.init(args=argv)
    n = VoiceListener()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
