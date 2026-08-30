#!/usr/bin/env python3
"""One arm, for full autonomy: joint states, interruptible motion, RGB-D.

    python3 scripts/srl_fa/fa_arm.py --arm left --report

WHAT IT ADDS TO THE PROVEN Executor / Eye
-----------------------------------------
`servo_pick_left.Eye` (which is `execute_pick_left.Executor` plus the camera
subscriptions) already does the hard parts: preflight that names its own
failure, `fresh_q()` that cannot be starved by a publish loop, a proportional
bridge whose deadband is lowered for the run, a force-jump collision guard,
and arrival waited for rather than timed.  It is SUBCLASSED here, not copied
and not edited.

Two things a window needs that a script did not:

1.  **A move you can STOP.**  `Executor.goto` runs to completion.  Behind a
    GUI that is unacceptable -- the operator has to be able to hit STOP while
    the arm is moving, and the honest way to stop a bridge-driven arm is to
    stop publishing targets and let its 0.5 s watchdog zero the speed.  So
    `goto` and `hold_gripper` are OVERRIDDEN with abort-aware versions.  They
    keep the original's ramp rate, force guard, arrival tolerance and stall
    detection -- imported from `execute_pick_left` rather than restated, so
    the two cannot drift apart.

2.  **`look()`**, which turns the latest RGB-D pair into a
    `fa_perception.Scene` with colour attached through the depth->colour
    baseline that FK gives for free.
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
sys.path.insert(0, "/home/gausms/kortex_ws/scripts/srl_fa")

import execute_pick_left as X  # noqa: E402
from servo_pick_left import Eye  # noqa: E402
from srl_fk import FK  # noqa: E402

import fa_kin as K  # noqa: E402
import fa_perception as PC  # noqa: E402

GRIP_OPEN = 0.0
GRIP_MAX = 0.72          # fully closed on nothing
# Knuckle angle that holds an object of a given width.  Measured from FK in
# this repository: finger-tip separation minus 50.7 mm is the real gap, giving
# 84.8 mm at 0.0 rad and 40 mm at 0.447 rad.
_GRIP_A, _GRIP_B = 84.8, (84.8 - 40.0) / 0.447


def grip_for_width(width_mm):
    """Knuckle angle at which the pads just touch an object `width_mm` wide."""
    return float(np.clip((_GRIP_A - width_mm) / _GRIP_B, 0.0, GRIP_MAX))


class Aborted(Exception):
    """The operator pressed STOP, or a guard fired. The arm is holding."""


class Arm(Eye):
    """One arm plus its wrist camera, driveable from a GUI thread."""

    def __init__(self, arm="left", fk=None, log=None, abort=None):
        super().__init__(arm)
        self.fk = fk or FK()
        self.log = log or (lambda s: print(s, flush=True))
        self.abort = abort or threading.Event()
        self._last_scene = None

    # ------------------------------------------------------------- lifecycle
    def start(self, wait=25.0):
        if not self.preflight(wait=wait):
            return False
        self.set_deadband(0.10)
        self._hold = self.fresh_q().copy()
        return True

    def _check_abort(self):
        if self.abort.is_set():
            # STOP means STOP PUBLISHING. The bridge watchdog zeroes the speed
            # 0.5 s later. Commanding a "stop pose" instead would be one more
            # motion at the moment somebody asked for none.
            raise Aborted("stopped by the operator")

    # ---------------------------------------------------------------- motion
    def goto(self, q_target, label, on_progress=None):
        """Ramp to q_target and wait for arrival. Interruptible.

        Returns the final worst-joint error in degrees, or None if a guard
        fired.  Raises Aborted if the operator pressed STOP.
        """
        q_start = self.fresh_q().copy()
        d = X.ang_wrap(np.asarray(q_target, float) - q_start)
        span = math.degrees(np.abs(d).max())
        if span < 0.05:
            return 0.0
        secs = max(1.0, span / X.DEG_PER_S)
        steps = max(2, int(secs * X.RATE))
        self.log("  %s: worst joint %.2f deg over %.1f s" % (label, span, secs))
        t_begin = time.time()
        checked = False
        for k in range(1, steps + 1):
            self._check_abort()
            a = k / steps
            a = a * a * (3 - 2 * a)
            self._hold = q_start + d * a
            self.send_arm(self._hold)
            import rclpy
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(1.0 / X.RATE)
            if on_progress:
                on_progress(a)
            j = self.force_jump()
            if j > X.FORCE_JUMP_N:
                self.log("     COLLISION GUARD: tool force rose %.1f N in 1.5 s"
                         " (limit %.1f). STOPPING." % (j, X.FORCE_JUMP_N))
                self._hold = self.fresh_q().copy()
                return None
            if not checked and time.time() - t_begin > X.MOVE_CHECK_S:
                checked = True
                moved = math.degrees(
                    np.abs(X.ang_wrap(self.fresh_q() - q_start)).max())
                if moved < X.MOVE_MIN_DEG and span > 1.0:
                    self.log("     ARM IS NOT RESPONDING to targets (moved "
                             "%.3f deg). Suspect a latched e-stop, the arm not"
                             " in SERVOING_READY, or a fault bank." % moved)
                    return None
        self._hold = np.asarray(q_target, float)
        t0 = time.time()
        best, t_improve, err = float("inf"), time.time(), float("inf")
        import rclpy
        while time.time() - t0 < X.CONVERGE_MAX_S:
            self._check_abort()
            self.send_arm(self._hold)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(1.0 / X.RATE)
            err = math.degrees(np.abs(X.ang_wrap(self.fresh_q() - self._hold)).max())
            if err < X.ARRIVE_TOL_DEG:
                self.log("     arrived in %.1f s (%.3f deg)" % (time.time() - t0, err))
                return err
            if err < best - X.STALL_MIN_DEG:
                best, t_improve = err, time.time()
            elif time.time() - t_improve > X.STALL_S:
                self.log("     stalled at %.3f deg" % err)
                return err
        self.log("     did not converge (%.3f deg)" % err)
        return err

    def hold_gripper(self, rad, secs, label):
        self.log("  %s (knuckle %.2f rad) for %.1f s" % (label, rad, secs))
        import rclpy
        t0 = time.time()
        while time.time() - t0 < secs:
            self._check_abort()
            self.send_grip(rad)
            if self._hold is not None:
                self.send_arm(self._hold)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(1.0 / X.RATE)

    def hold_still(self, secs):
        """Keep publishing the current target so the watchdog stays fed."""
        import rclpy
        t0 = time.time()
        while time.time() - t0 < secs:
            self._check_abort()
            if self._hold is not None:
                self.send_arm(self._hold)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(1.0 / X.RATE)

    # ------------------------------------------------------------- kinematics
    def pads_cam(self, grip):
        return K.pads_in_cam(self.fk, self.arm_name, self.fresh_q(), grip)

    def pads_w(self, grip, q=None):
        return K.pads_world(self.fk, self.arm_name,
                            self.fresh_q() if q is None else q, grip)

    def T_cam(self, q=None):
        T, = self.fk.poses(self.arm_name,
                           self.fresh_q() if q is None else q, [K.CAM_LINK])
        return T

    # -------------------------------------------------------------- percept
    def look(self, wait=6.0, z_max=2.0, exclude_hand=True, grip=0.0):
        """Latest RGB-D pair -> a Scene. None if the camera delivered nothing.

        `exclude_hand` masks the gripper out of the OBJECT segmentation (never
        out of the plane fit) using FK, because the fingers otherwise list as
        an object on the table.
        """
        import cv2
        self.img.clear()
        t0 = time.time()
        import rclpy
        while time.time() - t0 < wait and len(self.img) < 4:
            self._check_abort()
            rclpy.spin_once(self, timeout_sec=0.02)
        if len(self.img) < 4:
            return None
        c, ck = self.img["c"], self.img["ck"]
        dmsg, dk = self.img["d"], self.img["dk"]
        depth = (np.frombuffer(dmsg.data, np.uint16)
                 .reshape(dmsg.height, dmsg.width).astype(np.float32) * 0.001)
        col = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, -1)
        if c.encoding == "rgb8":
            col = cv2.cvtColor(col, cv2.COLOR_RGB2BGR)
        q = self.fresh_q()
        ex = K.hand_spheres(self.fk, self.arm_name, q, grip) if exclude_hand \
            else ()
        sc = PC.understand(depth, list(dk.k), colour_bgr=col,
                           colour_K=list(ck.k),
                           T_col_dep=K.T_colour_from_depth(self.fk,
                                                           self.arm_name, q),
                           q=np.array(q), arm=self.arm_name,
                           stamp=time.time(), z_max=z_max, exclude=ex)
        if sc is not None:
            sc.colour_image = col
            sc.depth_image = depth
            sc.depth_K = list(dk.k)
            sc.colour_K = list(ck.k)
        self._last_scene = sc
        return sc


def _report(arm_name):
    import rclpy
    rclpy.init()
    a = Arm(arm_name)
    if not a.start():
        return 2
    sc = a.look()
    if sc is None:
        print("no camera frames on /%s_camera" % arm_name)
        return 1
    print(sc.describe())
    lp, rp, mid = a.pads_cam(0.447)
    print("pad midpoint %.1f mm above the fitted plane"
          % (PC.pad_height_above(mid, sc.n, sc.d) * 1000))
    floor, seen = PC.descent_floor(sc.P, sc.n, sc.d, [lp, rp])
    print("descent floor under the pads: %s (%d pts)"
          % ("UNSEEN -- would refuse" if floor is None
             else "%.1f mm" % (floor * 1000), seen))
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left")
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args()
    raise SystemExit(_report(a.arm) if a.report else 0)
