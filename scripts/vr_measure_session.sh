#!/usr/bin/env bash
# vr_measure_session.sh -- the lab-day VR measurements, ONE AT A TIME.
#
#   bash scripts/vr_measure_session.sh              # what to run, in order
#   bash scripts/vr_measure_session.sh yaw          # ALWAYS FIRST
#   bash scripts/vr_measure_session.sh rate
#   bash scripts/vr_measure_session.sh freeze
#   bash scripts/vr_measure_session.sh clutch
#   bash scripts/vr_measure_session.sh scale
#   bash scripts/vr_measure_session.sh gripper
#   bash scripts/vr_measure_session.sh reset
#   bash scripts/vr_measure_session.sh report       # everything measured so far
#
# WHY ONE AT A TIME. These need a person doing a physical thing at a known
# moment, and every earlier attempt at them missed the action -- the recorder
# started AFTER the operator was told to move, so three separate steps came
# back "0 commands" on a system that was working perfectly. The operator is
# the only one who knows when the action began and ended, so the operator ends
# the step, and only one step runs per invocation. Each writes into the same
# report file, so stopping half way leaves the half you did.
#
# DESK OPERATION. Nobody wears the headset. `vr_guided_session.py` renders its
# instructions on an in-headset panel and ends each step on the A button --
# which, with the headset on a shelf, is a panel nobody can see and a button
# nobody is looking at. `--desk` moves both to the terminal.
#
# YAW IS FIRST AND IT IS NOT OPTIONAL. The operator sits FACING the wearer, so
# "away from me" is the robot's "toward its own front". Until that angle is
# measured the mapping adds the displacement raw, and a heading error shows up
# as the arm moving the wrong way -- which is invisible in a still frame and
# obvious the moment something is near a person.
set -uo pipefail

WS_DIR="${SRL_WS:-$(cd "$(dirname "$0")/.." && pwd)}"
export SRL_WS="$WS_DIR"
OUT="$WS_DIR/recordings/baselines/vr_headset_session.json"
G="python3 $WS_DIR/scripts/vr_guided_session.py --desk --append --out $OUT"

set +u
# shellcheck disable=SC1091
source "$WS_DIR/scripts/env.sh"
set -u

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
bad()  { printf '\033[31m%s\033[0m\n' "$*"; }
ok()   { printf '\033[32m%s\033[0m\n' "$*"; }

need_topic() {
  # A STEP THAT RUNS WITH NOTHING PUBLISHING RECORDS NOTHING AND SAYS
  # "0 commands", which reads as a broken robot. Check first, refuse loudly.
  local t="$1"
  if ! timeout 8 ros2 topic list 2>/dev/null | grep -qx "$t"; then
    bad "REFUSING: $t is not being published."
    echo "  The VR chain is not up. Start it with:"
    echo "      bash scripts/start_vr_wifi.sh"
    echo "  and make sure the headset's browser is on the page with the"
    echo "  session ENTERED -- poses only flow inside an XR session."
    exit 1
  fi
}

case "${1:-help}" in

yaw)
  bold "STEP 1 -- THE OPERATOR'S HEADING. Do this before anything else."
  need_topic /vr/controller_pose_right
  cat <<'TXT'

  You sit facing the wearer. The robot's "forward" and your "forward" are
  opposed, and the correction is ONE ANGLE -- never a mirror, because a
  mirror flips every orientation while the positions still look right.

  The script will ask you for two motions:
    1. move your hand about 40 cm STRAIGHT TOWARDS THE WEARER;
    2. move your hand about 40 cm TO YOUR OWN RIGHT.

  The second is a CROSS-CHECK, not an input. If the two do not come out
  perpendicular and horizontal, the script REFUSES to write a number rather
  than writing a confident wrong one.

  Nothing moves during this. It reads the controller pose directly and never
  engages the clutch.

TXT
  python3 "$WS_DIR/scripts/calibrate_operator_yaw.py" --apply
  ;;

rate)     bold "STEP 2 -- POSE RATE AND LATENCY";     need_topic /vr/controller_pose_right; $G --only m1 ;;
freeze)   bold "STEP 3 -- TRACKING LOSS FREEZES";     need_topic /vr/controller_pose_right; $G --only m2 ;;
clutch)   bold "STEP 4 -- CLUTCH RE-ENGAGE JUMP";     need_topic /vr/controller_pose_right; $G --only m3 ;;
scale)    bold "STEP 5 -- THUMBSTICK SCALING";        need_topic /vr/controller_pose_right; $G --only m4 ;;
gripper)  bold "STEP 6 -- TRIGGER ON THE GRIPPER";    need_topic /vr/controller_pose_right; $G --only m5 ;;

axes)
  bold "OPTIONAL -- WHICH WAY THE ARM GOES for right / forward / up"
  need_topic /vr/controller_pose_right
  $G --only axis_right axis_forward axis_up
  ;;

reset)
  bold "STEP 7 -- THE MAPPER RESETS BETWEEN RUNS"
  echo
  echo "  This one needs no motion. It calls the reset the sweep calls and"
  echo "  checks the mapper actually forgot: references, anchors, the filter"
  echo "  and the thumbstick scale."
  echo
  python3 - <<'PY'
import json, subprocess, sys, time
TOPIC = '/vr/mapper_right'


def state(timeout=8):
    """The mapper's own report, or ''. Readable while DISENGAGED now: the
    node publishes an idle heartbeat at 2 Hz, because before that the thing
    reset() clears was only observable when it was not cleared."""
    try:
        p = subprocess.run(['ros2', 'topic', 'echo', TOPIC, '--once',
                            '--field', 'data'], capture_output=True,
                           text=True, timeout=timeout)
        return (p.stdout or '').strip().splitlines()[0] if p.stdout else ''
    except Exception:
        return ''


before = state()
print('  before:', (before or '(nothing on %s)' % TOPIC)[:110])
if not before:
    print()
    print('  NOT MEASURED: the mapper is not publishing. "It reset" and "it')
    print('  was never running" are the same observation from here.')
    print('  Start the VR chain: bash scripts/start_vr_wifi.sh')
    sys.exit(1)
r = subprocess.run(['ros2', 'service', 'call', '/vr/reset',
                    'std_srvs/srv/Trigger', '{}'], capture_output=True,
                   text=True, timeout=25)
answered = 'success=True' in r.stdout.replace(' ', '')
print('  /vr/reset ->', 'ANSWERED' if answered else
      'NO ANSWER -- the service is gone or the mapper is wedged')
if not answered:
    sys.exit(1)
time.sleep(1.2)
after = state()
print('  after :', (after or '(nothing)')[:110])
try:
    b, a = json.loads(before), json.loads(after)
except Exception as e:
    print('  could not parse: %r' % (e,))
    sys.exit(1)
for k in ('engaged', 'has_reference', 'has_anchor', 'filter_primed',
          'scale', 'resets'):
    print('    %-16s %s -> %s' % (k, b.get(k), a.get(k)))
fails = [k for k in ('engaged', 'has_reference', 'has_anchor',
                     'filter_primed') if a.get(k)]
if abs(float(a.get('scale', 1.0)) - 1.0) > 1e-6:
    fails.append('scale is %s not 1.0' % a.get('scale'))
if int(a.get('resets', 0)) <= int(b.get('resets', -1)):
    fails.append('the reset counter did not advance -- reset() never ran')
if fails:
    print()
    print('  FAIL: %s' % '; '.join(fails))
    sys.exit(1)
print()
print('  PASS: references, anchors and the filter cleared, scale back to')
print('  1.0, and the reset counter advanced %s -> %s.'
      % (b.get('resets'), a.get('resets')))
print('  align_yaw_deg is deliberately NOT cleared: it is where you are')
print('  sitting, not per-run state. It reads %s deg.'
      % a.get('align_yaw_deg'))
PY
  ;;

report)
  bold "WHAT HAS BEEN MEASURED SO FAR"
  if [ ! -f "$OUT" ]; then bad "  nothing yet -- $OUT does not exist"; exit 1; fi
  python3 - "$OUT" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
order = [('m1','pose rate and latency'), ('m2','tracking loss freezes'),
         ('m3','clutch re-engage jump'), ('m4','thumbstick scaling'),
         ('m5','trigger on the gripper')]
for k, label in order:
    v = d.get(k)
    print('\n%-28s %s' % (label, '' if v else 'NOT RUN'))
    if v:
        for kk, vv in v.items():
            print('    %-26s %s' % (kk, str(vv)[:90]))
print('\nfile: %s' % sys.argv[1])
PY
  ;;

*)
  cat <<TXT
$(bold "VR MEASUREMENTS -- run these IN ORDER, one per command.")

  1. bash scripts/vr_measure_session.sh yaw       <- FIRST. Not optional.
  2. bash scripts/vr_measure_session.sh rate
  3. bash scripts/vr_measure_session.sh freeze
  4. bash scripts/vr_measure_session.sh clutch
  5. bash scripts/vr_measure_session.sh scale
  6. bash scripts/vr_measure_session.sh gripper
  7. bash scripts/vr_measure_session.sh reset
     bash scripts/vr_measure_session.sh report

  Optional:
     bash scripts/vr_measure_session.sh axes      which way the arm goes

  Everything lands in
      $OUT
  and each step MERGES into it, so stopping half way keeps the half you did.

  ALL OF THIS RUNS AGAINST THE SIM. Nothing here needs the real arms, and
  none of it should be done with them powered -- see
  docs/system/VR_REAL_ARM_RUN.md, which puts the real arms AFTER these.
TXT
  ;;
esac
