"""
capture_zero.py — capture the master arm's rest reference
=============================================================================
Records, over the same N frames:
  * the pot ZEROS for j1..j7, and
  * a_hat, the arm's long axis in the wrist IMU sensor frame.

THE ARM MUST HANG STRAIGHT DOWN. At rest the accelerometer reads +1g along
the axis pointing UP, and the arm points DOWN, so the arm's long axis in the
sensor frame is a_hat = -normalize(accel_rest). That vector is what makes
position_mode:=spherical mount-independent: elevation comes from the angle
between a_hat and measured gravity, with no mount rotation to estimate and
no integration anywhere.

(The old prompt here said STRAIGHT OUT IN FRONT. That was stale -- see
CLAUDE.md -- and the hanging-down posture is now REQUIRED, not merely
preferred, because a_hat is only meaningful when the arm axis lies along
gravity at capture time.)

Run:
  python3 capture_zero.py left  [/dev/ttyACM0]
  python3 capture_zero.py right [/dev/ttyACM0]
"""
import glob, math, re, sys, time
import serial
import numpy as np

try:                                    # works once install/setup.bash is sourced
    from srl_teleop.master_calibration import PotCalibration, check_rest_elevation
    from srl_teleop.serial_port import find_port as _find_port, PortNotFound
except ImportError:                     # also works run directly from this dir
    from master_calibration import PotCalibration, check_rest_elevation
    from serial_port import find_port as _find_port, PortNotFound

BAUD = 115200
# 200 frames: the pot zeros only need a few dozen, but the gyro bias is an
# average of a noisy rate signal and wants a longer window.
N    = 200

# Spread above which a channel is too noisy to trust as a zero reference.
MAX_POT_SPREAD_DEG = 2.0
# Rest sample must be still: gravity only, and pointing consistently.
MAX_ACCEL_SPREAD_G = 0.08
ACCEL_MAG_TOL_G    = 0.15


class _Say:
    """find_port() logs through a .info(); print is the CLI equivalent."""
    @staticmethod
    def info(msg):
        print(msg)


def find_port(explicit=None):
    """Delegates to the shared detector so this and the nodes cannot drift."""
    try:
        return _find_port(explicit, BAUD, logger=_Say)
    except PortNotFound as e:
        sys.exit(str(e))


def sample(port, arm, n=N):
    """Collect n frames carrying BOTH all 7 pots and the wrist IMU.

    A frame missing either is skipped rather than partly used -- the zeros
    and a_hat must describe the same instant of the same rest pose.
    """
    pat = re.compile(r"%sj([1-7])$" % ("k1" if arm == "left" else "k2"))
    imu_pat = re.compile(
        r"%s:(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),"
        r"(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*)"
        % ("K1IMU" if arm == "left" else "K2IMU"))

    ser = serial.Serial(port, BAUD, timeout=1.0)
    time.sleep(2.0)
    ser.reset_input_buffer()
    acc = [[] for _ in range(7)]
    accel = []
    gyro = []
    print("Hold the %s arm HANGING STRAIGHT DOWN at rest. Sampling %d frames..."
          % (arm, n))
    end = time.monotonic() + 30.0
    while len(acc[0]) < n:
        if time.monotonic() > end:
            ser.close()
            sys.exit("Timed out after %d/%d frames on %s (need pots AND %s IMU "
                     "in the same frame)"
                     % (len(acc[0]), n, port, "K1" if arm == "left" else "K2"))
        line = ser.readline().decode(errors="ignore").strip()
        got = {}
        for tok in line.split(","):
            if ":" not in tok:
                continue
            k, v = tok.split(":", 1)
            m = pat.match(k.strip())
            if m:
                try:
                    got[int(m.group(1))] = float(v)
                except ValueError:
                    pass
        imu_m = imu_pat.search(line)
        if len(got) == 7 and imu_m:
            for j in range(1, 8):
                acc[j-1].append(got[j])
            accel.append([float(imu_m.group(i)) for i in (1, 2, 3)])
            gyro.append([float(imu_m.group(i)) for i in (4, 5, 6)])
    ser.close()
    return acc, np.array(accel), np.array(gyro)


def main(argv=None):
    """Entry point. Also runs as a plain script."""
    argv = sys.argv if argv is None else argv
    arm = argv[1] if len(argv) > 1 else "left"
    if arm not in ("left", "right"):
        sys.exit("Arm must be left or right")
    port = find_port(argv[2] if len(argv) > 2 else None)
    acc, accel, gyro = sample(port, arm)

    zeros = [sum(a)/len(a) for a in acc]
    accel_rest = accel.mean(axis=0)

    cal = (PotCalibration(arm).capture(zeros).capture_a_hat(accel_rest)
           .capture_gyro_bias(gyro))

    print("\n--- pot zeros ---")
    for i, z in enumerate(zeros):
        spread = max(acc[i]) - min(acc[i])
        flag = "   <-- NOISY" if spread > MAX_POT_SPREAD_DEG else ""
        print("  joint_%d: %7.1f deg  (spread %.1f)%s" % (i+1, z, spread, flag))

    print("\n--- wrist IMU rest sample ---")
    mag = float(np.linalg.norm(accel_rest))
    spread = float(np.max(accel.max(axis=0) - accel.min(axis=0)))
    print("  accel_rest : [%+.4f %+.4f %+.4f] g" % tuple(accel_rest))
    print("  |accel|    : %.4f g" % mag)
    print("  spread     : %.4f g" % spread)
    print("  a_hat      : [%+.6f %+.6f %+.6f]" % tuple(cal.a_hat))

    problems = []
    if abs(mag - 1.0) > ACCEL_MAG_TOL_G:
        problems.append("|accel| = %.3f g is outside 1 +/- %.2f g -- the arm was "
                        "moving, or the IMU is mis-scaled." % (mag, ACCEL_MAG_TOL_G))
    if spread > MAX_ACCEL_SPREAD_G:
        problems.append("accel spread %.3f g > %.2f g -- the arm was not still."
                        % (spread, MAX_ACCEL_SPREAD_G))

    print("\n--- gyro zero-rate bias ---")
    print("  bias       : [%+.4f %+.4f %+.4f] deg/s" % tuple(cal.gyro_bias))
    resid = float(np.max(np.abs(gyro - cal.gyro_bias)))
    print("  residual   : %.4f deg/s peak after bias removal" % resid)
    if resid > 1.0:
        problems.append("gyro residual %.2f deg/s at rest exceeds 1 deg/s -- the "
                        "arm was moving, or the bias has drifted with "
                        "temperature." % resid)

    rest_elev = check_rest_elevation(cal.a_hat, accel_rest)
    print("\n  rest elevation self-check: %+.2f deg (must be -90.00)" % rest_elev)
    if abs(rest_elev + 90.0) > 0.5:
        problems.append("rest elevation is %+.2f deg, not -90 deg. a_hat and the "
                        "elevation convention disagree." % rest_elev)

    for w in cal.health(zeros):
        print("  WARNING: %s" % w)

    if problems:
        print("\nNOT WRITTEN -- fix these and re-run:")
        for p in problems:
            print("  * %s" % p)
        sys.exit(1)

    print("\nwrote %s" % cal.save())


if __name__ == "__main__":
    main()
