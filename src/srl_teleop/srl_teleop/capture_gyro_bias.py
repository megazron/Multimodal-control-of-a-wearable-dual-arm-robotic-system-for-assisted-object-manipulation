#!/usr/bin/env python3
"""
capture_gyro_bias.py — measure the gyro zero-rate offset only.

Deliberately separate from capture_zero.py: the pot zeros and a_hat are
already captured and verified (rest elevation exactly -90.00 deg on both
arms), and re-capturing them requires the arm to be hanging straight down.
The gyro bias needs only that the arm is STILL, in any posture, so this
loads the existing calibration, updates gyro_bias, and writes it back with
everything else preserved.

Run (needs the serial port free -- stop master_pose_node first):
  python3 -m srl_teleop.capture_gyro_bias left  [/dev/ttyACM0]
  python3 -m srl_teleop.capture_gyro_bias both
"""
import re
import sys
import time

import numpy as np
import serial

from srl_teleop.master_calibration import (PotCalibration, GYRO_REST_WARN_DPS)
from srl_teleop.serial_port import find_port, PortNotFound, BAUD

N = 200


def sample(port, arms, n=N):
    pats = {a: re.compile(
        r"%s:(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),"
        r"(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*)"
        % ("K1IMU" if a == "left" else "K2IMU")) for a in arms}
    out = {a: [] for a in arms}
    ser = serial.Serial(port, BAUD, timeout=1.0)
    time.sleep(2.0)
    ser.reset_input_buffer()
    print("Hold both arms STILL. Sampling %d frames..." % n)
    end = time.monotonic() + 40.0
    while min(len(v) for v in out.values()) < n:
        if time.monotonic() > end:
            ser.close()
            sys.exit("Timed out: got %s frames"
                     % {a: len(v) for a, v in out.items()})
        line = ser.readline().decode(errors="ignore").strip()
        for a in arms:
            m = pats[a].search(line)
            if m and len(out[a]) < n:
                out[a].append([float(m.group(i)) for i in (4, 5, 6)])
    ser.close()
    return {a: np.array(v) for a, v in out.items()}


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    which = argv[0] if argv else "both"
    arms = ["left", "right"] if which == "both" else [which]
    try:
        port = find_port(argv[1] if len(argv) > 1 else None)
    except PortNotFound as e:
        sys.exit(str(e))

    data = sample(port, arms)
    bad = False
    for a in arms:
        g = data[a]
        bias = g.mean(axis=0)
        resid = float(np.max(np.abs(g - bias)))
        std = float(np.max(g.std(axis=0)))
        print("\n--- %s ---" % a)
        print("  bias     : [%+.4f %+.4f %+.4f] deg/s" % tuple(bias))
        print("  |bias|   : %.4f deg/s" % float(np.linalg.norm(bias)))
        print("  peak resid: %.4f deg/s   noise std: %.4f deg/s" % (resid, std))
        if std > GYRO_REST_WARN_DPS:
            print("  WARNING: noise std %.2f deg/s exceeds %.1f deg/s -- the arm "
                  "was moving, or the gyro is unusually noisy." % (std, GYRO_REST_WARN_DPS))
            bad = True
        cal = PotCalibration.load(a)
        cal.gyro_bias = bias
        print("  wrote %s" % cal.save())
    if bad:
        print("\nBias written, but at least one arm looked non-stationary.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
