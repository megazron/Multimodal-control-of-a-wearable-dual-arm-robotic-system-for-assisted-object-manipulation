#!/usr/bin/env python3
"""The observer's station: a heartbeat that says somebody is standing there.

    python3 scripts/observer_estop.py

RUN THIS ON A SECOND MACHINE OR A SECOND TERMINAL, NEXT TO THE WEARER, BY THE
PERSON HOLDING THE PHYSICAL E-STOP. Not by the operator. That is the whole
point of the requirement: the operator is across the room with two controllers
in their hands and cannot reach the wearer.

WHAT IT IS FOR. `vr_safety_node` refuses to enable real-arm control until
`/vr/observer_estop_present` is seen -- and NOTHING IN THIS REPOSITORY EVER
PUBLISHED IT. `start_vr_wifi.sh` checked for a publisher and printed a warning
when there was none, which is a warning about a topic that could not exist.
So the interlock was, in practice, a permanent refusal that somebody would
eventually route around by setting `require_observer_estop:=false` on a lab
day. This is the thing that makes it satisfiable honestly.

IT IS A HEARTBEAT, NOT A CHECKBOX. It publishes at 5 Hz for as long as this
process is alive and the observer keeps confirming. Close the terminal, sleep
the laptop, kill the process, or walk away with it, and the beats stop --
`vr_safety_node` treats silence past 2 s as withdrawal and freezes the arm.
A latched "yes" would mean the observer could leave the building and the
system would still believe they were standing there.

    ENTER      re-confirm you are here and holding it (every 30 s)
    e + ENTER  E-STOP NOW
    q + ENTER  stand down: publishes False, then exits

THE E-STOP HERE IS THE SOFTWARE HALF AND IT IS THE SECOND LINE, NOT THE FIRST.
The observer's job is the PHYSICAL button. This exists so that the person who
can see the wearer can also halt the software path without walking round to
the operator's keyboard.
"""
import argparse
import select
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

CONFIRM_EVERY_S = 30.0


class Observer(Node):

    def __init__(self, hz=5.0, confirm_every=CONFIRM_EVERY_S):
        super().__init__("observer_estop")
        self.pub = self.create_publisher(Bool, "/vr/observer_estop_present", 10)
        # BOTH NAMES. `vr_safety_node` reads /vr/observer_estop_present and
        # `quest_vendor_bridge` reads /observer_estop_present; publishing one
        # would satisfy one interlock and silently leave the other refusing.
        self.pub2 = self.create_publisher(Bool, "/observer_estop_present", 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)
        self.note_pub = self.create_publisher(String, "/vr/observer_note", 10)
        self.cli = self.create_client(Trigger, "/estop")
        self.halt = self.create_client(Trigger, "/estop_real_halt")
        self.present = False
        self.last_confirm = 0.0
        self.confirm_every = confirm_every
        self.create_timer(1.0 / hz, self.beat)

    def beat(self):
        stale = (time.monotonic() - self.last_confirm) > self.confirm_every
        ok = self.present and not stale
        self.pub.publish(Bool(data=ok))
        self.pub2.publish(Bool(data=ok))
        if self.present and stale:
            # NOT SILENT. The beats become False rather than stopping, so the
            # operator's monitor shows WITHDRAWN rather than a dead topic --
            # two different problems with two different fixes.
            self.get_logger().warn(
                "observer has not re-confirmed for %.0f s -- reporting "
                "ABSENT until they do" % self.confirm_every,
                throttle_duration_sec=5.0)

    def confirm(self):
        self.last_confirm = time.monotonic()
        if not self.present:
            self.present = True
            self.get_logger().info("observer PRESENT")

    def stand_down(self):
        self.present = False
        for _ in range(6):
            self.pub.publish(Bool(data=False))
            self.pub2.publish(Bool(data=False))
            time.sleep(0.05)
        self.get_logger().warn("observer STOOD DOWN -- the arm will freeze")

    def fire(self):
        """Halt every way this repository knows, in the safe order.

        The TOPIC first, because it needs nobody to answer. HARD CONSTRAINT 6
        is the reason the service call is not first: `wait_for_service` in the
        e-stop path once blocked the e-stop for 4.0 s, so nothing here waits
        for anything before the halt is already on the wire.
        """
        self.estop_pub.publish(Bool(data=True))
        self.get_logger().error("E-STOP published by the OBSERVER")
        for cli, name in ((self.cli, "/estop"),
                          (self.halt, "/estop_real_halt")):
            if cli.service_is_ready():
                cli.call_async(Trigger.Request())
                self.get_logger().error("called %s" % name)
        self.note_pub.publish(String(data="observer fired the e-stop"))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--confirm-every", type=float, default=CONFIRM_EVERY_S)
    ap.add_argument("--who", default="",
                    help="the observer's role, for the log. Not a name -- "
                         "write_manifest refuses those and so does this.")
    a = ap.parse_args(argv)
    for banned in ("name", "@"):
        if banned in a.who.lower():
            print("--who is a ROLE, not a person. Anonymity is enforced in "
                  "code here (HARD CONSTRAINT 12).")
            return 2

    rclpy.init()
    n = Observer(a.hz, a.confirm_every)
    threading.Thread(target=lambda: rclpy.spin(n), daemon=True).start()

    print("""
=========================================================================
 OBSERVER STATION
=========================================================================
 You are the person NOT wearing the headset and NOT holding the
 controllers. You stand where you can see the wearer and reach the
 physical e-stop.

 Confirm below ONLY if all three are true:
   1. you are holding the physical e-stop, or it is within your reach;
   2. you can see the wearer and both arms;
   3. you can stop the run without asking anyone.

 ENTER      = yes, I am here (re-confirm every %.0f s)
 e + ENTER  = E-STOP NOW
 q + ENTER  = stand down and exit
=========================================================================
""" % a.confirm_every)
    try:
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 1.0)
            if not ready:
                left = a.confirm_every - (time.monotonic() - n.last_confirm)
                if n.present and left < 10:
                    print("  re-confirm within %.0f s (ENTER)" % max(0, left))
                continue
            line = sys.stdin.readline().strip().lower()
            if line == "q":
                n.stand_down()
                break
            if line == "e":
                n.fire()
                print("  E-STOP FIRED. The arm is halted and LATCHED. "
                      "/estop_reset is the only way out, and it is a "
                      "deliberate act.")
                continue
            n.confirm()
            print("  present, holding. next re-confirm in %.0fs"
                  % a.confirm_every)
    except KeyboardInterrupt:
        pass
    finally:
        # LEAVING IS WITHDRAWING. Ctrl-C is not a way to keep the interlock
        # satisfied on the way out.
        n.stand_down()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
