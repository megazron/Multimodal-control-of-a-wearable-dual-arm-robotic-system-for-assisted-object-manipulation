#!/usr/bin/env bash
# Open and close the gripper 3 times. Watch the hand and tell me what it does.
#
#   bash scripts/test_gripper.sh            # left arm
#   bash scripts/test_gripper.sh right      # right arm
#
# This talks STRAIGHT to the arm over the Kortex API -- no ROS, no domain, no
# bridge -- so it cannot fail for any of the reasons the ROS path can.  It
# stops the bridge first (the arm allows exactly ONE session) and restarts it
# afterwards.
set -uo pipefail
ARM="${1:-left}"
[ "$ARM" = "right" ] && IP=192.168.1.9 || IP=192.168.1.10

P=$(pgrep -f "kortex_highlevel_bridge_$ARM" | head -1)
if [ -n "$P" ]; then
    echo "stopping the $ARM bridge (pid $P) so this can take the session"
    kill -INT "$P"
    for _ in $(seq 1 30); do kill -0 "$P" 2>/dev/null || break; sleep 1; done
fi
sleep 2

"$HOME/kortex_ws/.kortex_venv/bin/python" - "$IP" <<'PY'
import sys, time
import kortex_api.autogen.client_stubs.BaseClientRpc as BaseClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.TCPTransport import TCPTransport

ip = sys.argv[1]
tr = TCPTransport(); tr.connect(ip, 10000)
rt = RouterClient(tr, lambda ex: None)
ss = SessionManager(rt)
i = Session_pb2.CreateSessionInfo()
i.username, i.password = "admin", "admin"
i.session_inactivity_timeout, i.connection_inactivity_timeout = 60000, 2000
ss.CreateSession(i)
base = BaseClient.BaseClient(rt)
print("connected to %s\n" % ip)


def pos():
    r = Base_pb2.GripperRequest(); r.mode = Base_pb2.GRIPPER_POSITION
    m = base.GetMeasuredGripperMovement(r)
    return m.finger[0].value if len(m.finger) else None


def go(v, label):
    c = Base_pb2.GripperCommand(); c.mode = Base_pb2.GRIPPER_POSITION
    f = c.gripper.finger.add(); f.finger_identifier = 1; f.value = v
    base.SendGripperCommand(c)
    for _ in range(15):
        time.sleep(0.2)
    print("  %-6s commanded %.2f -> gripper reports %s"
          % (label, v, "%.3f" % pos() if pos() is not None else "NOTHING"))


start = pos()
print("gripper reports position: %s"
      % ("%.3f" % start if start is not None else "NO GRIPPER ON THIS ARM"))
print("(0.000 = fully open, 1.000 = fully closed)\n")

for n in (1, 2, 3):
    print("cycle %d" % n)
    go(0.90, "CLOSE")
    go(0.00, "OPEN")

end = pos()
print("\nstart %s, end %s" % (start, end))
if start is not None and abs((end or 0) - (start or 0)) < 0.01 and (end or 0) == 0.0:
    print("The reported position never changed from 0.000, which is what an")
    print("arm with NO TOOL CONFIGURED reports. If the hand also did not move,")
    print("set the end-effector to Robotiq 2F-85 at http://%s and power-cycle."
          % ip)
ss.CloseSession(); rt.SetActivationStatus(False); tr.disconnect()
PY

echo
echo "restarting the $ARM bridge"
bash "$HOME/kortex_ws/scripts/bringup_arm.sh" "$ARM" >/dev/null 2>&1
echo "done"
