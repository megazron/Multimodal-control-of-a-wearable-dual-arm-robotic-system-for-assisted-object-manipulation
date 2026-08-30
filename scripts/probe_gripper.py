#!/usr/bin/env python3
"""Ask the ARM directly whether it has a gripper, and try to move it.

    # the bridge holds the arm's single Kortex session, so stop it first
    python3 scripts/probe_gripper.py --ip 192.168.1.10

WHY
---
Commanding the gripper through the bridge produces NO error and NO motion:
`gripper_enabled` is true, the topic has a subscriber, `SendGripperCommand`
returns cleanly, and the hand does not move.  That is the signature of a
command the arm accepts and discards, which happens when the arm has no tool
configured -- but it is also what a dead gripper looks like from outside.

This asks the arm the questions the bridge never asks:
  * what devices does it actually have?
  * does it report gripper feedback at all?
  * does a POSITION command change that feedback?
  * does a SPEED command, which some firmware needs instead?

It says which, rather than leaving us to guess.
"""
import argparse
import sys
import time

sys.path.insert(0, "/home/gausms/kortex_ws/src/srl_teleop")

import kortex_api.autogen.client_stubs.BaseClientRpc as BaseClient  # noqa: E402
import kortex_api.autogen.client_stubs.BaseCyclicClientRpc as BaseCyclicClient  # noqa: E402
import kortex_api.autogen.client_stubs.DeviceManagerClientRpc as DevMgr  # noqa: E402
from kortex_api.autogen.messages import Base_pb2, Session_pb2  # noqa: E402
from kortex_api.RouterClient import RouterClient  # noqa: E402
from kortex_api.SessionManager import SessionManager  # noqa: E402
from kortex_api.TCPTransport import TCPTransport  # noqa: E402


def gripper_position(base):
    """Measured gripper position (0..1), or None if the arm reports none."""
    try:
        req = Base_pb2.GripperRequest()
        req.mode = Base_pb2.GRIPPER_POSITION
        m = base.GetMeasuredGripperMovement(req)
        if len(m.finger) == 0:
            return None
        return m.finger[0].value
    except Exception as e:
        return ("error", str(e)[:80])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--user", default="admin")
    ap.add_argument("--password", default="admin")
    args = ap.parse_args()

    tr = TCPTransport()
    tr.connect(args.ip, 10000)
    rt = RouterClient(tr, lambda ex: print("  router error:", ex))
    ss = SessionManager(rt)
    info = Session_pb2.CreateSessionInfo()
    info.username = args.user
    info.password = args.password
    info.session_inactivity_timeout = 60000
    info.connection_inactivity_timeout = 2000
    ss.CreateSession(info)
    base = BaseClient.BaseClient(rt)
    print("connected to %s -- arm state %s"
          % (args.ip, Base_pb2.ArmState.Name(base.GetArmState().active_state)))

    print("\n1. what devices does the arm report?")
    try:
        dm = DevMgr.DeviceManagerClient(rt)
        for d in dm.ReadAllDevices().device_handle:
            print("   id=%-3d type=%-28s name=%s"
                  % (d.device_identifier,
                     Base_pb2.DeviceTypes.Name(d.device_type)
                     if hasattr(Base_pb2, "DeviceTypes") else str(d.device_type),
                     d.name if hasattr(d, "name") else "-"))
    except Exception as e:
        print("   device manager unavailable:", str(e)[:100])

    print("\n2. does the arm report a GRIPPER at all?")
    p = gripper_position(base)
    if p is None:
        print("   NO GRIPPER REPORTED. The arm has no tool configured, so")
        print("   gripper commands are accepted and discarded -- exactly what")
        print("   we see. Fix in the web UI at http://%s :" % args.ip)
        print("     Configurations -> Robot -> Tool/End-effector -> Robotiq 2F-85")
        return 1
    if isinstance(p, tuple):
        print("   query failed:", p[1])
        return 1
    print("   gripper reports position %.3f (0=open, 1=closed)" % p)

    print("\n3. does a POSITION command move it?")
    for target in (0.10, 0.80, 0.10):
        cmd = Base_pb2.GripperCommand()
        cmd.mode = Base_pb2.GRIPPER_POSITION
        f = cmd.gripper.finger.add()
        f.finger_identifier = 1
        f.value = target
        base.SendGripperCommand(cmd)
        time.sleep(2.5)
        now = gripper_position(base)
        print("   commanded %.2f -> measured %.3f" % (target, now))
    moved_pos = abs(gripper_position(base) - p) > 0.02

    print("\n4. does a SPEED command move it? (some firmware needs this)")
    cmd = Base_pb2.GripperCommand()
    cmd.mode = Base_pb2.GRIPPER_SPEED
    f = cmd.gripper.finger.add()
    f.finger_identifier = 1
    f.value = 0.3                      # positive = close
    base.SendGripperCommand(cmd)
    time.sleep(2.0)
    mid = gripper_position(base)
    f.value = 0.0
    base.SendGripperCommand(cmd)
    print("   after a 2 s close at speed 0.3 -> measured %.3f" % mid)

    print("\nVERDICT")
    if moved_pos:
        print("  the gripper DOES respond to POSITION commands. The bridge's")
        print("  path must be at fault, not the hardware.")
    else:
        print("  the gripper is REPORTED but does not move to position")
        print("  commands. Check that the fingers are not jammed and that the")
        print("  tool is powered; then try the SPEED result above.")
    ss.CloseSession()
    rt.SetActivationStatus(False)
    tr.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
