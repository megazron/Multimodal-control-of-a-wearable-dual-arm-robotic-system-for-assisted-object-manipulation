#!/usr/bin/env python3
"""Ask the arm, directly, why its gripper is not moving.

    ~/kortex_ws/.kortex_venv/bin/python scripts/diagnose_gripper.py --arm left

STOP THE BRIDGE FIRST. The arm permits exactly ONE Kortex session (HARD
CONSTRAINT 2) and the bridge holds it. This script opens its own, so running
it against a live bridge will simply be refused -- and it says so rather than
hanging.

WHAT IT ESTABLISHES, in the order that narrows the problem fastest:

  1. is a gripper CONFIGURED on this arm at all
  2. what does the arm say the gripper's position is
  3. are there FAULTS on the gripper's own banks
  4. does GRIPPER_POSITION move it
  5. does GRIPPER_SPEED move it

4 and 5 are separate questions and the difference matters. A gripper that
ignores POSITION and obeys SPEED is a working gripper being asked the wrong
way; one that ignores both is not being reached at all.

Every step prints what it saw. Nothing here infers a cause from a silence.
"""
from __future__ import annotations

import argparse
import sys
import time

IPS = {"left": "192.168.1.10", "right": "192.168.1.9"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=("left", "right"))
    ap.add_argument("--ip", default=None)
    ap.add_argument("--move", action="store_true",
                    help="actually try to move the fingers (steps 4 and 5)")
    a = ap.parse_args()
    ip = a.ip or IPS[a.arm]

    import kortex_api.autogen.client_stubs.BaseClientRpc as BaseClientRpc
    import kortex_api.autogen.client_stubs.BaseCyclicClientRpc as CyclicRpc
    import kortex_api.autogen.messages.Base_pb2 as Base_pb2
    import kortex_api.autogen.messages.Session_pb2 as Session_pb2
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.RouterClient import RouterClient
    from kortex_api.SessionManager import SessionManager

    print("connecting to %s (%s) ..." % (a.arm, ip))
    tr = TCPTransport()
    tr.connect(ip, 10000)
    rt = RouterClient(tr, lambda e: print("  [router] %s" % e))
    si = Session_pb2.CreateSessionInfo()
    si.username, si.password = "admin", "admin"
    si.session_inactivity_timeout = 60000
    si.connection_inactivity_timeout = 2000
    sm = SessionManager(rt)
    sm.CreateSession(si)
    base = BaseClientRpc.BaseClient(rt)
    cyc = CyclicRpc.BaseCyclicClient(rt)
    print("  session created")

    try:
        _report(base, cyc, Base_pb2, a.move)
    finally:
        try:
            sm.CloseSession()
        except Exception:                                      # noqa: BLE001
            pass
        tr.disconnect()
        print("\nsession closed")
    return 0


def _report(base, cyc, Base_pb2, do_move):
    # ---- 1. is a gripper configured -----------------------------------
    print("\n[1] what the arm says it HAS")
    try:
        # The product configuration names the end effector the arm was
        # commissioned with. A gripper missing HERE is not a software fault
        # and no amount of commanding will move it.
        import kortex_api.autogen.client_stubs.ProductConfigurationClientRpc \
            as PCRpc
        pc = PCRpc.ProductConfigurationClient(base.router)
        cfg = pc.GetProductConfiguration()
        print("    end effector type : %s" % cfg.end_effector_type)
        print("    (0 normally means NO end effector configured)")
    except Exception as e:                                     # noqa: BLE001
        print("    could not read product configuration: %s" % e)

    # ---- 2. measured position ------------------------------------------
    print("\n[2] measured gripper position")
    pos = _measured(base, Base_pb2)
    print("    GetMeasuredGripperMovement -> %s" %
          ("(no fingers reported)" if pos is None else "%.4f" % pos))

    fb = cyc.RefreshFeedback()
    motors = list(fb.interconnect.gripper_feedback.motor)
    print("    cyclic gripper motors reported: %d" % len(motors))
    for m in motors:
        print("      motor %d: position %.2f%%  velocity %.2f  current %.3f A"
              "  voltage %.1f V  temp %.1f C"
              % (m.motor_id, m.position, m.velocity, m.current_motor,
                 m.voltage, m.temperature_motor))
    if not motors:
        print("    NO GRIPPER MOTOR IN THE FEEDBACK AT ALL. The interconnect")
        print("    is not reporting a gripper, so nothing downstream can be")
        print("    a software problem -- the hand is not on the bus.")

    # ---- 3. faults ------------------------------------------------------
    print("\n[3] faults and status")
    gfb = fb.interconnect.gripper_feedback
    print("    gripper status_flags  : 0x%08X" % gfb.status_flags)
    print("    gripper fault_bank_a  : 0x%08X" % gfb.fault_bank_a)
    print("    gripper fault_bank_b  : 0x%08X" % gfb.fault_bank_b)
    print("    gripper warning_bank_a: 0x%08X" % gfb.warning_bank_a)
    print("    interconnect faults   : a 0x%08X  b 0x%08X"
          % (fb.interconnect.fault_bank_a, fb.interconnect.fault_bank_b))
    try:
        print("    arm servoing mode     : %s"
              % base.GetServoingMode().servoing_mode)
    except Exception as e:                                     # noqa: BLE001
        print("    servoing mode unreadable: %s" % e)

    if not do_move:
        print("\n(no --move, so nothing was commanded)")
        return

    # ---- 4. POSITION ----------------------------------------------------
    print("\n[4] GRIPPER_POSITION: commanding 1.0 (closed), then 0.0")
    for want in (1.0, 0.0):
        _cmd_position(base, Base_pb2, want)
        _watch(base, cyc, Base_pb2, "position -> %.1f" % want)

    # ---- 5. SPEED -------------------------------------------------------
    print("\n[5] GRIPPER_SPEED: closing at +0.5, then opening at -0.5")
    for v, what in ((0.5, "close"), (-0.5, "open")):
        _cmd_speed(base, Base_pb2, v)
        _watch(base, cyc, Base_pb2, "speed %+.1f (%s)" % (v, what))
        _cmd_speed(base, Base_pb2, 0.0)


def _measured(base, Base_pb2):
    try:
        req = Base_pb2.GripperRequest()
        req.mode = Base_pb2.GRIPPER_POSITION
        m = base.GetMeasuredGripperMovement(req)
        if not m.finger:
            return None
        return float(m.finger[0].value)
    except Exception as e:                                     # noqa: BLE001
        print("      GetMeasuredGripperMovement failed: %s" % e)
        return None


def _cmd_position(base, Base_pb2, value):
    cmd = Base_pb2.GripperCommand()
    cmd.mode = Base_pb2.GRIPPER_POSITION
    f = cmd.gripper.finger.add()
    f.finger_identifier = 1
    f.value = float(value)
    try:
        base.SendGripperCommand(cmd)
        print("    sent POSITION %.2f -- returned cleanly" % value)
    except Exception as e:                                     # noqa: BLE001
        print("    sent POSITION %.2f -- RAISED: %s" % (value, e))


def _cmd_speed(base, Base_pb2, value):
    cmd = Base_pb2.GripperCommand()
    cmd.mode = Base_pb2.GRIPPER_SPEED
    f = cmd.gripper.finger.add()
    f.finger_identifier = 1
    f.value = float(value)
    try:
        base.SendGripperCommand(cmd)
        if value:
            print("    sent SPEED %+.2f -- returned cleanly" % value)
    except Exception as e:                                     # noqa: BLE001
        print("    sent SPEED %+.2f -- RAISED: %s" % (value, e))


def _watch(base, cyc, Base_pb2, label, secs=3.0):
    """Print the measured position while it should be moving."""
    t0 = time.time()
    seen = []
    while time.time() - t0 < secs:
        fb = cyc.RefreshFeedback()
        mot = list(fb.interconnect.gripper_feedback.motor)
        p = mot[0].position if mot else float("nan")
        seen.append(p)
        time.sleep(0.25)
    if not seen:
        return
    print("      %-24s %.1f%% -> %.1f%%  (min %.1f max %.1f)"
          % (label, seen[0], seen[-1], min(seen), max(seen)))
    if abs(seen[-1] - seen[0]) < 1.0 and (max(seen) - min(seen)) < 1.0:
        print("      DID NOT MOVE.")


if __name__ == "__main__":
    sys.exit(main())
