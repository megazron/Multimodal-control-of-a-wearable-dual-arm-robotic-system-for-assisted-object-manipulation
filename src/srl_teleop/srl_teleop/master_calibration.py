import math, os
from pathlib import Path
import numpy as np

LINK_LENGTHS = [0.043,0.037,0.043,0.037,0.043,0.036,0.033]
IS_ROLL      = [True,False,True,False,True,False,True]
FULL_EXT     = sum(LINK_LENGTHS)
CONFIG_DIR   = Path(os.path.expanduser("~/kortex_ws/config"))

# MEASURED from the 2026-07-31 button-gated recording (see CLAUDE.md).
# The old ("+x","-y","-z") was derived for the FK path. In SPHERICAL mode the
# vertical sign is already carried by the elevation convention (-90 hanging,
# +90 up), so a "-z" entry DOUBLE-negates it: the measured gain was
# cmd_z = -1.011 * phys_z, i.e. raising the master drove the robot down.
# Likewise cmd_y = -0.993 * phys_y inverted fore/aft.
# Identity fixes both. The lateral axis is NOT fixed by this and cannot be --
# see the AZIMUTH LIMITATION note in CLAUDE.md.
AXIS_MAP        = ("+x","+y","+z")
WORKSPACE_SCALE = 1.0
# MEASURED workspace anchors: world -> {arm}_end_effector_link with the sim
# at its home pose (tf2_echo, both arms verified at home to 4 dp). These
# replace the old unverified guess of (0.3,0,1.2)/(-0.3,0,1.2) -- that guess
# was NOT reachable for the left arm (/compute_ik returned -31 for it, while
# the measured value returns SUCCESS).
# RE-DERIVED 2026-08-05 (second pass) after the mount was DERIVED rather than
# searched. Read from the offline model and cross-checked against tf2 on the
# live stack. offset = P_HOME, so a mount change MUST change these.
#
# The home JOINT ANGLES did not change and must not: they are ground truth
# from the real arm. Only the mount moved.
#
#   mount, backpack-relative xyz (+-0.20, -0.20, 0.18) -> world (+-0.20, -0.32, 1.25)
#   left  rpy  ( 1.247666407, -0.232161321,  1.508064663)
#   right rpy  (-1.256457931, -0.033197491,  1.699631975)
#   NOT exact rpy mirrors, deliberately: the plate NORMAL is mirrored, the
#   per-arm CLOCKING is not. See CLAUDE.md.
#
#   at home: ZERO colliding link pairs (MoveIt valid=True, contacts=0 both arms)
#            min clearance 0.1664 m, every arm link vs every wearer link
#            both EEs in front of the wearer at working height
#   BUT IK over a frontal task volume is 18.8% / 18.8% (live /compute_ik).
#   Clearance and reach are in DIRECT CONFLICT at this scale - see CLAUDE.md.
WORKSPACE_CENTRE= {"left":(0.6966,0.2481,1.1463), "right":(-0.7627,0.2979,1.1788)}
WORKSPACE_ORIENT= {"left":(-0.0896,0.4860,0.8693,0.0032),
                   "right":(0.1335,0.5423,0.8288,0.0345)}
DEFAULT_SIGNS   = [1,1,1,1,1,1,1]

def _Rz(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1.]])

def _Ry(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1.]])

def _Tz(d):
    T=np.eye(4); T[2,3]=d; return T

def fk_matrix(q):
    """Full 4x4 homogeneous transform of the master tip in the base frame.

    Same chain fk() has always walked -- fk() now reads its translation
    column out of this, so the two can never drift apart."""
    T=np.eye(4)
    for i in range(7):
        T = T @ (_Rz(q[i]) if IS_ROLL[i] else _Ry(q[i]))
        T = T @ _Tz(LINK_LENGTHS[i])
    return T

def fk_rotation(q):
    """3x3 orientation of the tip frame w.r.t. the master base frame."""
    return fk_matrix(q)[:3,:3]

def fk(q):
    return fk_matrix(q)[:3,3]

_AX = {"x":0,"y":1,"z":2}

def _axis_matrix(am=AXIS_MAP):
    R=np.zeros((3,3))
    for r,spec in enumerate(am):
        R[r,_AX[spec[-1]]] = -1.0 if spec[0]=="-" else 1.0
    return R

# Legal permutations for SPHERICAL mode. All keep a POSITIVE vertical: in
# spherical mode the elevation convention already carries the vertical sign
# (-90 hanging, +90 up), so a "-z" entry double-negates it and no such map
# can produce correct up/down. (The old four all ended in "-z" -- they were
# derived for the FK path, where +z runs along the hanging arm, and they do
# not hold here.) Each of these is a proper rotation; anything else is a
# reflection, which mirrors exactly one axis and looks plausible on the
# other two.
LEGAL_AXIS_MAPS = (("+x", "+y", "+z"), ("-x", "-y", "+z"),
                   ("+y", "-x", "+z"), ("-y", "+x", "+z"))


def validate_axis_map(am=AXIS_MAP, strict=True):
    """Refuse a reflection. Called at import, so a bad edit fails loudly."""
    am = tuple(am)
    if strict and am not in LEGAL_AXIS_MAPS:
        raise ValueError(
            "AXIS_MAP %r is not one of the four legal hanging-down maps %r"
            % (am, LEGAL_AXIS_MAPS))
    det = float(np.linalg.det(_axis_matrix(am)))
    if abs(det - 1.0) > 1e-9:
        raise ValueError(
            "AXIS_MAP %r has det = %+.6f, not +1.0. That is a reflection and "
            "will silently mirror one axis of the workspace mapping." % (am, det))
    return True


def to_robot_frame(p, am=AXIS_MAP):
    return _axis_matrix(am) @ np.asarray(p,dtype=float)

def map_to_workspace(p, arm, scale=WORKSPACE_SCALE, am=AXIS_MAP):
    neutral = to_robot_frame((0.0,0.0,FULL_EXT), am)
    d = to_robot_frame(p, am) - neutral
    return np.asarray(WORKSPACE_CENTRE[arm],dtype=float) + scale*d

# Tolerance on |a_hat| when loading. It is written normalised to 9 dp, so
# anything meaningfully off unit is a hand-edit or a bad capture.
A_HAT_TOL = 1e-3


def elevation_from_accel(a_hat, accel):
    """Arm elevation above horizontal, in radians, from gravity alone.

    a_hat is the arm's long axis in the SENSOR frame, captured once with the
    arm hanging straight down (see PotCalibration.capture_a_hat). It is a
    fixed property of the glue joint, so it needs no mount rotation.

    normalize(accel) is world-UP expressed in the sensor frame -- an
    accelerometer at rest reports specific force, which points up. The
    component of the arm axis along world-up is therefore
    dot(a_hat, up_sensor), and its arcsine is the angle above horizontal.

    Sign check, and the reason this is not written with a leading minus:
    hanging straight down, up_sensor == -a_hat, so dot == -1 and elevation
    == -90 deg. Straight up gives +90, horizontal gives 0. See
    ELEVATION_SIGN_NOTE.
    """
    u = _unit(accel)
    return math.asin(float(np.clip(np.dot(np.asarray(a_hat, dtype=float), u),
                                   -1.0, 1.0)))


ELEVATION_SIGN_NOTE = """\
The brief specified BOTH a_hat = -normalize(accel_rest) AND
elevation = asin(-dot(a_hat, normalize(accel_now))). Those two negations
compound: together they put the hanging-down rest pose at +90 deg
elevation, i.e. straight UP. Since cos() is even and only sin() carries the
sign, that flips the z axis alone -- the exact up/down mirror this rebuild
exists to remove ("down came out HIGHER than up").

Implemented here: a_hat = -normalize(accel_rest) as specified, and
elevation = asin(+dot(...)), so hanging down reads -90 deg. Verified by
check_rest_elevation()."""


def check_rest_elevation(a_hat, accel_rest):
    """Rest-pose self-check: hanging straight down must read -90 deg."""
    return math.degrees(elevation_from_accel(a_hat, accel_rest))


def spherical_position(reach, elev, azim):
    """Master tip position from reach magnitude + IMU elevation + j1 azimuth."""
    return np.array([reach * math.cos(elev) * math.cos(azim),
                     reach * math.cos(elev) * math.sin(azim),
                     reach * math.sin(elev)], dtype=float)


# Residual rest rate above which the stored gyro bias is stale (temperature
# drift). 1 deg/s integrates to 60 deg over a minute -- useless.
GYRO_REST_WARN_DPS = 1.0


def yaw_rate_from_gyro(gyro_dps, gyro_bias_dps, u_hat):
    """Rate of rotation about the WORLD VERTICAL, in rad/s.

    u_hat is the world-up direction expressed in the sensor frame, i.e.
    normalize(accel) while quasi-static. The component of angular velocity
    about that axis is exactly the azimuth rate:

        yaw_rate = dot(omega - bias, u_hat)

    Two properties make this the right sensor for azimuth:
      * it needs NO mount rotation -- both vectors are already in the sensor
        frame, so the failed IMU mount calibration is irrelevant here;
      * it observes rotation about vertical DIRECTLY, so it is immune to the
        lever-arm coupling that makes azimuth-from-j1 scale with the arm's
        bend, and to the chain-error accumulation that sank FK azimuth.
    Teensy reports deg/s; the return is rad/s.
    """
    omega = np.asarray(gyro_dps, dtype=float)
    if gyro_bias_dps is not None:
        omega = omega - np.asarray(gyro_bias_dps, dtype=float)
    return math.radians(float(np.dot(omega, _unit(u_hat))))


class PotCalibration:
    def __init__(self, arm, zeros=None, signs=None, a_hat=None, gyro_bias=None):
        self.arm=arm
        self.zeros=list(zeros) if zeros else [0.0]*7
        self.signs=list(signs) if signs else list(DEFAULT_SIGNS)
        # Arm long axis in the wrist IMU sensor frame. None until captured;
        # only the spherical position mode needs it, so a missing value must
        # not break the existing FK path.
        self.a_hat=None if a_hat is None else np.asarray(a_hat, dtype=float)
        # Gyro zero-rate offset, deg/s. None until captured; only the
        # gyro-aided azimuth path needs it.
        self.gyro_bias=None if gyro_bias is None else np.asarray(gyro_bias, dtype=float)

    @property
    def path(self):
        return CONFIG_DIR / ("master_zero_%s.txt" % self.arm)

    @classmethod
    def load(cls, arm):
        c=cls(arm)
        if not c.path.exists():
            raise FileNotFoundError("No zero reference at %s" % c.path)
        for line in c.path.read_text().splitlines():
            line=line.split("#")[0].strip()
            if ":" not in line: continue
            k,v=(s.strip() for s in line.split(":",1))
            if k.startswith("joint_"): c.zeros[int(k[6:])-1]=float(v)
            elif k.startswith("sign_"): c.signs[int(k[5:])-1]=int(float(v))
            elif k=="gyro_bias":
                parts=v.split()
                if len(parts)!=3:
                    raise ValueError("%s: gyro_bias needs 3 numbers, got %r"
                                     % (c.path, v))
                c.gyro_bias=np.array([float(x) for x in parts])
            elif k=="a_hat":
                parts=v.split()
                if len(parts)!=3:
                    raise ValueError("%s: a_hat needs 3 numbers, got %r"
                                     % (c.path, v))
                c.a_hat=np.array([float(x) for x in parts])
        if c.a_hat is not None:
            n=float(np.linalg.norm(c.a_hat))
            if abs(n-1.0) > A_HAT_TOL:
                raise ValueError(
                    "%s: |a_hat| = %.6f, not 1.0 within %g. It is a direction, "
                    "so it must be a unit vector -- re-run capture_zero."
                    % (c.path, n, A_HAT_TOL))
        return c

    def capture_gyro_bias(self, gyro_samples):
        """Mean gyro reading while stationary, in deg/s, per axis.

        Every rate gyro has a non-zero output at rest. Integrating that raw
        offset is what makes gyro yaw drift without bound, so it is measured
        once and subtracted every frame. It drifts with temperature, so
        master_pose_node warns if the residual rate at rest exceeds
        GYRO_REST_WARN_DPS -- that means re-capture.
        """
        a = np.asarray(gyro_samples, dtype=float)
        self.gyro_bias = a.mean(axis=0)
        return self

    def capture_a_hat(self, accel_rest):
        """Store the arm's long axis from a hanging-straight-down rest sample.

        At rest the accelerometer reads +1g along UP, and the arm points
        DOWN, so the axis is the negated gravity reading.
        """
        self.a_hat = -_unit(accel_rest)
        return self

    def save(self):
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        L=["# %s master arm reference, captured with the arm HANGING" % self.arm,
           "# STRAIGHT DOWN at rest. (The old header here said STRAIGHT OUT IN",
           "# FRONT; that was stale and wrong -- see CLAUDE.md. The hanging-down",
           "# posture is now REQUIRED, because a_hat below is only meaningful",
           "# when the arm axis is along gravity at capture time.)",
           ""]
        L+= ["joint_%d: %.1f" % (i+1,z) for i,z in enumerate(self.zeros)]
        L+= ["","# flip to -1 if a joint drives the robot backwards",""]
        L+= ["sign_%d: %d" % (i+1,s) for i,s in enumerate(self.signs)]
        if self.a_hat is not None:
            L+= ["",
                 "# Arm long axis in the wrist IMU sensor frame, unit vector.",
                 "#   a_hat = -normalize(accel at rest, arm hanging down)",
                 "# Used by position_mode:=spherical for elevation, which is",
                 "# mount-independent and drift-free. Must stay unit length;",
                 "# load() rejects it otherwise.",
                 "",
                 "a_hat: %+.9f %+.9f %+.9f" % tuple(self.a_hat)]
        if self.gyro_bias is not None:
            L+= ["",
                 "# Gyro zero-rate offset in deg/s, measured stationary.",
                 "# Subtracted every frame; integrating the raw offset is what",
                 "# makes gyro yaw drift. Drifts with temperature -- re-capture",
                 "# if the node warns about residual rest rate.",
                 "",
                 "gyro_bias: %+.6f %+.6f %+.6f" % tuple(self.gyro_bias)]
        self.path.write_text("\n".join(L)+"\n")
        return self.path

    def capture(self, raw):
        self.zeros=[float(v) for v in raw]; return self

    def apply(self, raw):
        out=[]
        for i,r in enumerate(raw):
            d=(float(r)-self.zeros[i]+180.0) % 360.0 - 180.0
            out.append(math.radians(self.signs[i]*d))
        return out

    def health(self, raw):
        w=[]
        for i,r in enumerate(raw):
            if abs(float(r)) < 1e-9: w.append("joint_%d reads exactly 0.0" % (i+1))
            elif float(r) > 359.5:   w.append("joint_%d railed at %.1f" % (i+1,float(r)))
        return w

# ===========================================================================
# IMU-AIDED POT REPAIR -- gravity model and wrist-IMU mount calibration
# ===========================================================================
# The wrist accelerometer measures the direction of gravity in the SENSOR
# frame. FK over the 7 pot angles predicts the tip's orientation, hence the
# same direction, in the TIP frame. The two differ by a fixed unknown
# rotation R_mount (the glue joint), so once R_mount is known:
#
#     normalize(accel)  ~=  R_mount @ fk_rotation(q).T @ G_HAT
#
# That is TWO independent scalar constraints on q every frame (a unit vector
# has 2 DOF), drift-free, with no integration anywhere. It can therefore
# pin down at most 2 unknown joint angles -- see recover_joints().
#
# NOTHING here integrates acceleration to get position. Position stays with
# fk(); double-integrating MPU6050 bias drifts ~0.05 m in 1 s against a
# 0.272 m total reach, which would be worse than useless.

# Load-bearing pot channels: an exact 0.0 here is a firmware dropout clamp,
# not a real reading. j6 legitimately clamps to 0 (mounted at the bottom of
# its travel) and j7 is a terminal roll that moves the tip 0.000000 m, so
# neither is checked. Canonical copy of the policy documented in CLAUDE.md.
LOAD_BEARING_IDX = (0, 1, 2, 3, 4)   # j1..j5

# Gravity direction in the MASTER FK BASE frame, as a unit vector.
# Sign convention is resolved empirically by the mount calibration rather
# than assumed -- solve_imu_mount() fits both +z and -z and keeps whichever
# actually matches the recorded data. See CLAUDE.md: the pot zeros were
# captured with the arms HANGING DOWN, and fk() puts the tip along +z at
# q=0, so base +z is expected to point downward -- but "expected" is not
# "measured", and an accelerometer at rest reports specific force (up),
# not gravity (down), which flips it again.
G_HAT_CANDIDATES = {"+z": np.array([0.0, 0.0, 1.0]),
                    "-z": np.array([0.0, 0.0, -1.0])}

# Accelerometer magnitude gate. Outside |1g| +/- this, the arm is
# accelerating and gravity cannot be isolated from the measurement.
ACCEL_GATE_G = 0.15


def zero_dropouts(raw):
    """Indices of load-bearing channels reading a firmware-clamped 0.0."""
    return [i for i in LOAD_BEARING_IDX if float(raw[i]) == 0.0]


def accel_magnitude_ok(accel, gate=ACCEL_GATE_G):
    """True when the accelerometer is close enough to pure 1g that the
    reading can be treated as the gravity direction."""
    return abs(float(np.linalg.norm(accel)) - 1.0) < gate


def _unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("cannot normalise a zero-length vector")
    return v / n


def predicted_gravity_tip(q, g_hat):
    """Gravity direction expressed in the TIP frame, from pot angles alone."""
    return fk_rotation(q).T @ np.asarray(g_hat, dtype=float)


def kabsch(src, dst):
    """Rotation R minimising sum ||R @ src_i - dst_i||^2 over unit vectors.

    Wahba's problem, solved in closed form by SVD -- no iteration, no
    initial guess, no local minima. src/dst are (N,3) arrays of unit
    vectors. The det correction keeps the result a proper rotation
    (det = +1); without it a reflection can score better numerically and
    would silently mirror the whole mapping.
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
        raise ValueError("src and dst must both be (N,3)")
    if src.shape[0] < 2:
        raise ValueError("need at least 2 vector pairs to fix a rotation")
    H = src.T @ dst                      # 3x3 correlation
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T
    d = np.sign(np.linalg.det(V @ U.T))
    if d == 0.0:
        d = 1.0
    return V @ np.diag([1.0, 1.0, d]) @ U.T


def _residuals_deg(R, src, dst):
    out = []
    for s, d in zip(src, dst):
        c = float(np.clip(np.dot(R @ s, d), -1.0, 1.0))
        out.append(math.degrees(math.acos(c)))
    return out


def solve_imu_mount(samples, g_hat=None):
    """Estimate the wrist IMU mount rotation from static postures.

    samples: iterable of (label, q_radians, accel_xyz).
    Returns dict with R, g_hat_name, per-posture residuals in degrees,
    and the rms/max. When g_hat is None both sign conventions are fitted
    and the better one is returned, with the loser reported alongside so
    the choice is visible rather than silent.
    """
    samples = list(samples)
    if len(samples) < 2:
        raise ValueError("need at least 2 postures to fit a rotation")

    names = [g_hat] if g_hat is not None else list(G_HAT_CANDIDATES)
    fits = {}
    for name in names:
        gv = G_HAT_CANDIDATES[name]
        src = np.array([predicted_gravity_tip(q, gv) for _, q, _ in samples])
        dst = np.array([_unit(a) for _, _, a in samples])
        R = kabsch(src, dst)
        res = _residuals_deg(R, src, dst)
        fits[name] = {
            "g_hat_name": name,
            "R": R,
            "residuals_deg": res,
            "labels": [lb for lb, _, _ in samples],
            "rms_deg": float(np.sqrt(np.mean(np.square(res)))),
            "max_deg": float(np.max(res)),
            "n": len(samples),
        }

    best = min(fits.values(), key=lambda f: f["rms_deg"])
    best["alternatives"] = {k: v["rms_deg"] for k, v in fits.items()}
    return best


class IMUMount:
    """Fixed rotation from the FK tip frame to the wrist IMU sensor frame.

    Stored as plain editable text, same style as the pot zero files.
    """

    def __init__(self, arm, R=None, g_hat_name="+z", meta=None):
        self.arm = arm
        self.R = np.eye(3) if R is None else np.asarray(R, dtype=float)
        self.g_hat_name = g_hat_name
        self.meta = dict(meta or {})

    @property
    def path(self):
        return CONFIG_DIR / ("imu_mount_%s.txt" % self.arm)

    @property
    def g_hat(self):
        return G_HAT_CANDIDATES[self.g_hat_name]

    @classmethod
    def load(cls, arm):
        c = cls(arm)
        if not c.path.exists():
            raise FileNotFoundError("No IMU mount calibration at %s" % c.path)
        rows = {}
        for line in c.path.read_text().splitlines():
            line = line.split("#")[0].strip()
            if ":" not in line:
                continue
            k, v = (s.strip() for s in line.split(":", 1))
            if k.startswith("row_"):
                rows[int(k[4:]) - 1] = [float(x) for x in v.split()]
            elif k == "g_hat":
                c.g_hat_name = v
        if len(rows) != 3:
            raise ValueError("%s must define row_1..row_3" % c.path)
        c.R = np.array([rows[i] for i in range(3)], dtype=float)
        if c.g_hat_name not in G_HAT_CANDIDATES:
            raise ValueError("g_hat must be one of %s, got %r"
                             % (list(G_HAT_CANDIDATES), c.g_hat_name))
        c.validate()
        return c

    def validate(self, tol=1e-3):
        """A hand-edited file is easy to break. Reject anything that is not
        a proper rotation rather than letting it quietly skew every frame."""
        should_be_I = self.R.T @ self.R
        if not np.allclose(should_be_I, np.eye(3), atol=tol):
            raise ValueError("%s is not orthonormal (R^T R != I)" % self.path)
        det = float(np.linalg.det(self.R))
        if abs(det - 1.0) > tol:
            raise ValueError(
                "%s has det = %+.6f, not +1 -- a reflection, not a rotation"
                % (self.path, det))
        return self

    def save(self):
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        m = self.meta
        L = [
            "# %s wrist IMU mount rotation, tip frame -> sensor frame." % self.arm,
            "#",
            "#   normalize(accel)  ~=  R_mount @ fk_rotation(q).T @ g_hat",
            "#",
            "# Estimated by Kabsch/SVD over static postures (Wahba's problem).",
            "# Must stay a proper rotation: rows orthonormal, det = +1. It is",
            "# loaded with that check enforced, so a bad hand-edit fails loudly",
            "# instead of skewing every frame.",
            "",
        ]
        L += ["row_%d: %+.9f %+.9f %+.9f" % (i + 1, *self.R[i]) for i in range(3)]
        L += [
            "",
            "# Gravity direction in the master FK base frame this was fitted",
            "# with. Resolved from the data, not assumed. One of: +z, -z.",
            "",
            "g_hat: %s" % self.g_hat_name,
            "",
            "# ---- fit provenance (informational, not read back) ----",
        ]
        if m:
            L += ["# postures fitted : %s" % m.get("n", "?"),
                  "# rms residual    : %.3f deg" % m["rms_deg"]
                  if "rms_deg" in m else "",
                  "# max residual    : %.3f deg" % m["max_deg"]
                  if "max_deg" in m else "",
                  "# source          : %s" % m.get("source", "?")]
            for lb, r in zip(m.get("labels", []), m.get("residuals_deg", [])):
                L.append("#   %-22s %6.2f deg" % (lb, r))
        self.path.write_text("\n".join(x for x in L if x != "") + "\n")
        return self.path


def pose_from_raw(raw, calib, arm):
    return map_to_workspace(fk(calib.apply(raw)), arm)

if __name__ == "__main__":
    z=np.zeros(7)
    print("straight arm     ", fk(z).round(4), "|p| = %.4f" % np.linalg.norm(fk(z)))
    q=np.zeros(7); q[1]=math.pi/2
    print("90 bend at J2    ", fk(q).round(4))
    q=np.zeros(7); q[6]=math.pi/2
    print("90 roll at J7    moves tip %.6f" % np.linalg.norm(fk(q)-fk(z)))
    q=np.zeros(7); q[0]=math.pi/2; q[1]=math.pi/2
    print("J1 roll + J2 bend", fk(q).round(4), " must leave the x-z plane")
    print("neutral -> robot ", map_to_workspace(fk(z),"left").round(3), " centre (0.3, 0.0, 1.2)")

# Master-frame rest position, in the master's own frame, per position mode.
# fk mode: the straight-out neutral the FK path has always used (+z).
# spherical mode: the arm HANGING DOWN (-z), which is the posture the pot
# zeros and a_hat are captured in, so that at rest the workspace offset is
# reached exactly and the clutch anchor starts from zero displacement.
NEUTRAL = to_robot_frame((0.0, 0.0, FULL_EXT))
NEUTRAL_SPHERICAL = to_robot_frame((0.0, 0.0, -FULL_EXT))

# Fail at import rather than mirroring an axis in silence.
validate_axis_map()
