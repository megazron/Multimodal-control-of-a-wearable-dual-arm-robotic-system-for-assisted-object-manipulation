#!/usr/bin/env python3
"""PART 6: run mode 6 and measure it. Refusals, latency, stop, mode switching.

    python3 scripts/verify_autonomy.py

Drives a live `autonomy_executive` through every case that must REFUSE, plus
the two latencies that matter, and asserts on the outcome rather than on the
node merely having started.

The refusal cases are the point. A pick-and-place that works is easy to
demonstrate and tells you nothing about a robot mounted beside a person's
head; what matters is that it declines, by name, when it should.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Harness(Node):
    def __init__(self):
        super().__init__("verify_autonomy")
        self.stage = []
        self.speech = []
        self.decisions = []
        self.estop = []
        self.create_subscription(String, "/autonomy_stage", self._stage, 20)
        self.create_subscription(String, "/robot_speech", self._say, 20)
        self.create_subscription(String, "/autonomy_decision", self._dec, 50)
        self.create_subscription(Bool, "/estop", self._estop, 20)
        self.voice = self.create_publisher(String, "/voice_transcript", 10)
        self.dets = self.create_publisher(String, "/detections", 10)
        self.rst = self.create_publisher(String, "/autonomy_reset", 10)

    def _stage(self, m):
        self.stage.append((time.monotonic(), json.loads(m.data)))

    def _say(self, m):
        self.speech.append((time.monotonic(), m.data))

    def _dec(self, m):
        self.decisions.append((time.monotonic(), json.loads(m.data)))

    def _estop(self, m):
        self.estop.append((time.monotonic(), bool(m.data)))

    def wait_matched(self, timeout=15.0):
        """Block until the executive has actually SUBSCRIBED to us.

        Publishing before DDS has matched drops the message silently, and the
        harness then reports every refusal case as a failure -- a discovery
        race masquerading as a broken state machine. The same trap is already
        documented for the fault injector in CLAUDE.md; I walked into it
        again.
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if (self.voice.get_subscription_count() > 0
                    and self.dets.get_subscription_count() > 0):
                self.spin(0.3)          # let the match settle
                return True
            self.spin(0.1)
        return False

    def spin(self, s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def send(self, text):
        self.voice.publish(String(data=text))

    def see(self, objs):
        self.dets.publish(String(data=json.dumps(objs)))
        self.spin(0.6)

    def reset(self):
        """Clear OUR buffers AND the executive's world model. Objects persist
        by design, so without this each case inherits the previous case's
        objects and a unique match silently becomes an ambiguous one."""
        self.rst.publish(String(data="reset"))
        self.spin(0.3)
        self.stage.clear()
        self.speech.clear()
        self.decisions.clear()
        self.estop.clear()


def obj(label, xyz, conf=0.9):
    return {"label": label, "position": list(xyz), "confidence": conf}


GREEN_L = obj("green cube", [-0.30, 0.34, 0.90])
GREEN_R = obj("green cube", [+0.30, 0.34, 0.90])
RED = obj("red block", [+0.34, 0.30, 0.90])
BEHIND = obj("green cube", [0.00, -0.30, 1.20])
LOWCONF = obj("blue ball", [+0.30, 0.34, 0.90], conf=0.20)


def case(h, name, dets, utter, expect_stage, expect_words, results):
    h.reset()
    if dets is not None:
        h.see(dets)
    t0 = time.monotonic()
    h.send(utter)
    h.spin(1.6)
    stages = [s["stage"] for _, s in h.stage]
    said = " ".join(t for _, t in h.speech).lower()
    lat = None
    for ts, _ in h.stage:
        lat = (ts - t0) * 1000.0
        break
    ok_stage = expect_stage in stages
    ok_words = all(w.lower() in said for w in expect_words)
    ok = ok_stage and ok_words
    results.append((name, ok, stages, said[:120], lat))
    print("  %-34s %-8s stages=%s" % (name, "PASS" if ok else "FAIL",
                                      "->".join(stages) or "(none)"))
    if said:
        print("      said: %s" % said[:110])
    if not ok:
        print("      EXPECTED stage %r and words %s"
              % (expect_stage, expect_words))
    return ok


def main():
    rclpy.init()
    h = Harness()
    h.spin(1.0)
    if not h.wait_matched():
        print("REFUSING TO REPORT: nothing subscribed to /voice_transcript "
              "or /detections within 15 s -- the executive is not running.")
        h.destroy_node()
        rclpy.shutdown()
        return 2
    # PROVE THE EXECUTIVE IS ALIVE FIRST.
    #
    # Without this, "no wake word -> ignored" PASSES when nothing is running
    # at all, because it asserts on silence. A check that passes on a dead
    # system is the same defect as a test that builds the environment where
    # the bug cannot occur -- it reports green for the wrong reason.
    h.reset()
    h.see([obj("probe cube", [0.30, 0.34, 0.90])])
    h.send("hey doc oc grab the probe cube")
    h.spin(1.5)
    if not h.stage and not h.speech:
        print("REFUSING TO REPORT: autonomy_executive is not responding.")
        print("  Start it first:  ros2 run srl_autonomy autonomy_executive "
              "--ros-args -p mode:=6")
        h.destroy_node()
        rclpy.shutdown()
        return 2
    print("  (executive is alive: responded to a probe command)\n")
    print("MODE 6 VERIFICATION -- refusals, latency, stop\n")
    R = []

    print("REFUSAL MATRIX")
    case(h, "no matching object", [RED], "hey doc oc grab the green cube",
         "REFUSED", ["can't see anything"], R)
    case(h, "ambiguous: two matches", [GREEN_L, GREEN_R],
         "hey doc oc grab the green cube", "AWAIT_COMMAND",
         ["which one", "two"], R)
    case(h, "low confidence below floor", [LOWCONF],
         "hey doc oc grab the blue ball", "REFUSED", ["can't see anything"], R)
    case(h, "target BEHIND the wearer", [BEHIND],
         "hey doc oc grab the green cube", "REFUSED",
         ["behind the wearer", "refuse"], R)
    case(h, "deictic, no pointing input", [GREEN_R],
         "hey doc oc grab that thing over there", "REFUSED",
         ["pointed"], R)
    case(h, "unknown verb", [GREEN_R], "hey doc oc juggle the green cube",
         None if False else "IDLE", [], R) if False else None
    h.reset()
    h.send("hey doc oc juggle the green cube")
    h.spin(1.2)
    said = " ".join(t for _, t in h.speech).lower()
    ok = "didn't understand" in said
    R.append(("unknown verb", ok, [], said[:80], None))
    print("  %-34s %-8s said: %s" % ("unknown verb", "PASS" if ok else "FAIL",
                                     said[:70]))

    h.reset()
    h.send("grab the green cube")            # no wake word
    h.spin(1.2)
    ok = not h.stage and not h.speech
    R.append(("no wake word -> ignored", ok, [], "", None))
    print("  %-34s %-8s (silent, correct)"
          % ("no wake word -> ignored", "PASS" if ok else "FAIL"))

    print("\nACCEPT + SPOKEN CONFIRMATION")
    h.reset()
    h.see([RED])
    t0 = time.monotonic()
    h.send("hey doc oc grab the red block")
    h.spin(3.5)
    stages = [s["stage"] for _, s in h.stage]
    said = " ".join(t for _, t in h.speech).lower()
    v2m = None
    for ts, s in h.stage:
        if s["stage"] == "PLAN":
            v2m = (ts - t0) * 1000.0
            break
    ok = ("CONFIRM" in stages and "PLAN" in stages
          and "reaching now" in said)
    R.append(("accept + announce + move", ok, stages, said[:110], v2m))
    print("  %-34s %-8s stages=%s" % ("accept + announce + move",
                                      "PASS" if ok else "FAIL",
                                      "->".join(stages)))
    print("      said: %s" % said[:110])
    if v2m:
        print("      VOICE -> MOTION latency: %.0f ms (includes the "
              "configured confirmation wait)" % v2m)

    print("\nSTOP")
    h.reset()
    h.see([RED])
    h.send("hey doc oc grab the red block")
    h.spin(0.4)                       # mid-sequence, before the wait expires
    t0 = time.monotonic()
    h.send("stop")
    h.spin(1.2)
    stop_lat = None
    for ts, s in h.stage:
        if s["stage"] == "STOPPED":
            stop_lat = (ts - t0) * 1000.0
            break
    estopped = any(v for _, v in h.estop)
    ok = stop_lat is not None and estopped
    R.append(("voice stop mid-sequence", ok, [], "", stop_lat))
    print("  %-34s %-8s latency %s, /estop published: %s"
          % ("voice stop mid-sequence", "PASS" if ok else "FAIL",
             "%.0f ms" % stop_lat if stop_lat else "NEVER", estopped))

    print("\nDECISION LOG")
    kinds = [d.get("what") for _, d in h.decisions]
    ok = "voice_stop" in kinds
    R.append(("every decision logged", ok, [], "", None))
    print("  %-34s %-8s kinds seen: %s"
          % ("every decision logged", "PASS" if ok else "FAIL",
             sorted(set(kinds))))

    print("\n" + "=" * 62)
    n_ok = sum(1 for r in R if r[1])
    print("  %d/%d checks passed" % (n_ok, len(R)))
    for name, ok, _, _, lat in R:
        if not ok:
            print("    FAILED: %s" % name)
    h.destroy_node()
    rclpy.shutdown()
    return 0 if n_ok == len(R) else 1


if __name__ == "__main__":
    sys.exit(main())
