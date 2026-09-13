#!/usr/bin/env python3
"""
THE PHYSICS GATE.

One question: can the Robotiq 2F-85, as this workspace actually describes it,
close on a 40 mm cube in Gazebo and HOLD it -- without jitter -- and drop it
when released?

The answer decides the simulation architecture.  It is a physics question, so
the rig deliberately contains as little non-physics as possible: no
ros2_control, no MoveIt, no IK, no ROS at all.  The arm hangs upside down from
a fixed post with every joint at zero, so the approach is top-down by
construction; the lift is a fold of the three parallel bend joints, which
preserves the tool orientation exactly.

  python3 gate.py --probe            measure where the fingertips actually are
  python3 gate.py                    run the gate
  python3 gate.py --tip-mu 1.0       again without the shipped mu=1e5
  python3 gate.py --engine dartsim   again on the engine with no mimic support

Everything measured is written to the run directory as JSON, so a later run is
diffed against it rather than remembered.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parent
SCRATCH = Path(os.environ.get(
    "GATE_SCRATCH",
    "/tmp/scratch/"
    "39ddc0a7-8b69-4984-92ae-fe1f20d56237/scratchpad/gate"))
WORLD_NAME = "gate"

CUBE_SIZE = 0.040          # m, the gate's object
CUBE_MASS = 0.050          # kg
TABLE_TOP_Z = 0.600        # m, world
PEDESTAL_XY = 0.020        # m; slim enough for the fingers to pass either side

PAD_L = "robotiq_85_left_finger_tip_link"
PAD_R = "robotiq_85_right_finger_tip_link"
# THE MODEL ENTRY, NOT THE LINK.  dynamic_pose/info reports a LINK's pose
# RELATIVE TO ITS MODEL, and gz's JSON omits zero-valued fields, so
# `cube_link` reads as exactly (0, 0, 0) at every sample for ever.  Parsed
# with a 0.0 default that is a plausible-looking measurement, not a gap: one
# full gate run scored the cube as lying on the floor from t=0 while the
# gripper was in fact holding it.  Absence read as a value, again.
# The arm's links are safe only because ITS model sits at the world origin.
CUBE = "cube"
ARM_JOINTS = ["joint_%d" % i for i in range(1, 8)]
KNUCKLE = "robotiq_85_left_knuckle_joint"
# Multipliers straight out of the URDF's own <mimic> tags, so the software
# coupling and the description cannot drift apart.
GRIPPER_CHAIN = {
    "robotiq_85_left_knuckle_joint": +1.0,
    "robotiq_85_right_knuckle_joint": -1.0,
    "robotiq_85_left_inner_knuckle_joint": +1.0,
    "robotiq_85_right_inner_knuckle_joint": -1.0,
    "robotiq_85_left_finger_tip_joint": -1.0,
    "robotiq_85_right_finger_tip_joint": +1.0,
}
MODEL = "gate_arm"
# The plugin's DEFAULT command topic.  It ignores <topic> silently.
CMD_TOPIC = "/model/" + MODEL + "/joint/%s/0/cmd_pos"
JS_TOPIC = "/world/" + WORLD_NAME + "/model/" + MODEL + "/joint_state"


# --------------------------------------------------------------------------
# environment
# --------------------------------------------------------------------------
def gz_env() -> dict:
    env = dict(os.environ)
    shares = [WS / "install" / p / "share"
              for p in ("kortex_description", "robotiq_description")]
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(
        [str(s) for s in shares] + [env.get("GZ_SIM_RESOURCE_PATH", "")]
    ).strip(":")
    env.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    return env


def sh(cmd, **kw):
    """Always through bash -- `source` is not a /bin/sh builtin, and the ROS
    setup scripts are the only thing that puts these tools on PATH."""
    if isinstance(cmd, str):
        cmd = ["bash", "-c", cmd]
    return subprocess.run(cmd, env=gz_env(), capture_output=True, text=True,
                          **kw)


SRC = "source /opt/ros/jazzy/setup.bash"


# --------------------------------------------------------------------------
# description
# --------------------------------------------------------------------------
def build_model_sdf(out: Path, tip_mu, post_z: float) -> Path:
    urdf, sdf = out / "gate_arm.urdf", out / "gate_arm.sdf"
    src = SRC + " && source %s/install/setup.bash" % WS
    r = sh("%s && xacro %s/gate_arm.urdf.xacro post_z:=%.5f" % (src, HERE, post_z))
    if r.returncode != 0:
        sys.exit("xacro failed:\n" + r.stderr)
    urdf.write_text(r.stdout)

    r = sh("%s && gz sdf -p %s" % (src, urdf))
    if r.returncode != 0 or not r.stdout.strip():
        sys.exit("gz sdf failed:\n" + r.stderr)
    text = r.stdout

    if tip_mu is not None:
        # The shipped description gives the pads mu=100000, which is not a
        # friction coefficient -- it is a way of writing "never slip".  A gate
        # that only passes there has not answered the question, so it is a
        # knob and both settings are run.
        text = text.replace("<mu>100000</mu>", "<mu>%g</mu>" % tip_mu)
        text = text.replace("<mu2>100000</mu2>", "<mu2>%g</mu2>" % tip_mu)
    sdf.write_text(text)
    return sdf


def model_body(sdf_path: Path) -> str:
    t = sdf_path.read_text()
    return t[t.index("<model"): t.rindex("</model>") + len("</model>")]


def build_world(out: Path, model_xml: str, cube_mu: float, engine: str,
                step: float, cube_pose: str, pedestal_top: float | None) -> Path:
    ixx = CUBE_MASS * CUBE_SIZE ** 2 / 6.0
    ped = ""
    if pedestal_top is not None:
        h = pedestal_top - TABLE_TOP_Z
        ped = f"""
    <model name="pedestal">
      <static>true</static>
      <pose>__PED_X__ __PED_Y__ {TABLE_TOP_Z + h / 2:.5f} 0 0 0</pose>
      <link name="pedestal_link">
        <collision name="c">
          <geometry><box><size>{PEDESTAL_XY} {PEDESTAL_XY} {h:.5f}</size></box></geometry>
          <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><box><size>{PEDESTAL_XY} {PEDESTAL_XY} {h:.5f}</size></box></geometry>
        </visual>
      </link>
    </model>"""

    # ENGINE IS NOT A DETAIL HERE.  gz-physics 7.6.0's dartsim plugin -- the
    # DEFAULT -- exports no mimic-constraint symbols at all, and says so at
    # runtime.  The 2F-85's four follower joints then float free and the hand
    # stops being a parallel gripper.  Only bullet-featherstone supports it.
    world = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics name="step" type="ignored">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
      <engine><filename>gz-physics-{engine}-plugin</filename></engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <gravity>0 0 -9.81</gravity>

    <model name="ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </visual>
      </link>
    </model>

    <model name="table">
      <static>true</static>
      <pose>0 0 {TABLE_TOP_Z / 2:.5f} 0 0 0</pose>
      <link name="table_link">
        <collision name="c">
          <geometry><box><size>1.4 1.0 {TABLE_TOP_Z:.5f}</size></box></geometry>
          <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><box><size>1.4 1.0 {TABLE_TOP_Z:.5f}</size></box></geometry>
        </visual>
      </link>
    </model>
{ped}

    <model name="cube">
      <pose>{cube_pose}</pose>
      <link name="cube_link">
        <inertial>
          <mass>{CUBE_MASS}</mass>
          <inertia><ixx>{ixx:.6e}</ixx><iyy>{ixx:.6e}</iyy><izz>{ixx:.6e}</izz>
                   <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">
          <geometry><box><size>{CUBE_SIZE} {CUBE_SIZE} {CUBE_SIZE}</size></box></geometry>
          <surface>
            <friction><ode><mu>{cube_mu}</mu><mu2>{cube_mu}</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="v">
          <geometry><box><size>{CUBE_SIZE} {CUBE_SIZE} {CUBE_SIZE}</size></box></geometry>
          <material><ambient>0.9 0.3 0.1 1</ambient><diffuse>0.9 0.3 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>

{model_xml}
  </world>
</sdf>
"""
    p = out / "gate_world.sdf"
    p.write_text(world)
    return p


# --------------------------------------------------------------------------
# running
# --------------------------------------------------------------------------
class Sim:
    def __init__(self, world: Path, log: Path):
        self.log = open(log, "w")
        self.proc = subprocess.Popen(
            # NO -r.  Starting PAUSED and only running once every setpoint is
            # loaded removes the startup race: gz's JointPositionController is
            # inert until its first message, so during the seconds it takes to
            # publish eight setpoints the arm is FREE.  Measured: it collapses,
            # joint_4 and joint_6 end past their limits, and the continuous
            # rolls spin at 17-27 rad/s.  A controller then fighting a 2.5 rad
            # error is not the experiment anyone meant to run.
            ["bash", "-lc", "%s && exec gz sim -s -v 2 %s" % (SRC, world)],
            env=gz_env(), stdout=self.log, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.recorders = []

    def wait_ready(self, timeout=120.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = sh("%s && gz topic -l" % SRC)
            if "/world/%s/dynamic_pose/info" % WORLD_NAME in r.stdout:
                return True
            if self.proc.poll() is not None:
                return False
            time.sleep(1.0)
        return False

    def record(self, topic: str, path: Path):
        f = open(path, "w")
        p = subprocess.Popen(
            ["bash", "-lc",
             "%s && exec gz topic -e -t %s --json-output" % (SRC, topic)],
            env=gz_env(), stdout=f, stderr=subprocess.DEVNULL,
            start_new_session=True)
        self.recorders.append((p, f))

    def cmd(self, joint: str, value: float, repeats: int = 2):
        """Publish REPEATEDLY.  `gz topic -p` publishes once from a process
        that exits immediately, so a setpoint sent before the plugin's
        subscription is discovered is dropped with no error anywhere.  That
        is what made the arm free-spin at "zero command" on the first probe."""
        for _ in range(repeats):
            sh("%s && gz topic -t %s -m gz.msgs.Double -p 'data: %.6f'"
               % (SRC, CMD_TOPIC % joint, value))
            time.sleep(0.15)

    def play(self):
        sh("%s && gz service -s /world/%s/control "
           "--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
           "--timeout 5000 --req 'pause: false'" % (SRC, WORLD_NAME))

    def read_joints(self, tries: int = 4) -> dict:
        """One joint_state sample, as {name: (position, velocity)}.

        RETRIED, because `gz topic -e -n 1` occasionally returns nothing at
        all, and an empty read is indistinguishable from a dead model unless
        you insist.  Returning {} on the first miss once turned a working
        gain sweep into a traceback."""
        msg = None
        for _ in range(tries):
            r = sh("%s && timeout 12 gz topic -e -t %s -n 1 --json-output"
                   % (SRC, JS_TOPIC))
            try:
                msg = json.loads(r.stdout)
                break
            except Exception:
                time.sleep(1.0)
        if msg is None:
            return {}
        out = {}
        for j in msg.get("joint", []):
            ax = j.get("axis1") or {}
            out[j.get("name", "?")] = (float(ax.get("position", 0.0)),
                                       float(ax.get("velocity", 0.0)))
        return out

    def cmd_verified(self, joint: str, value: float, tol: float = 0.05,
                     settle: float = 3.0):
        """Command, then CHECK the joint moved there.  A setpoint that lands
        on a topic nobody subscribes to is silent, and this rig has already
        produced a full run of confident numbers that way."""
        self.cmd(joint, value)
        time.sleep(settle)
        js = self.read_joints()
        got = js.get(joint, (float("nan"), 0.0))[0]
        return {"joint": joint, "commanded": value, "reached": got,
                "error": got - value, "ok": abs(got - value) <= tol}

    def grip(self, q: float):
        """Drive the whole finger chain together.  On dartsim there is no
        mimic constraint, so if only the knuckle were commanded the other
        five joints would simply droop -- which is exactly what the first
        jaw map measured and mistook for tracking."""
        for j, mult in GRIPPER_CHAIN.items():
            self.cmd(j, mult * q)

    def hold_all(self):
        for j in ARM_JOINTS:
            self.cmd(j, 0.0)
        self.grip(0.0)

    def fold(self, a: float):
        """joint_2=+a, joint_4=-2a, joint_6=+a.  The three bend axes are
        parallel while the rolls are at zero, and tool orientation is their
        SUM, which is zero for every a.  So this raises the hand without
        rotating it at all."""
        self.cmd("joint_2", a)
        self.cmd("joint_4", -2.0 * a)
        self.cmd("joint_6", a)

    def stop(self):
        for p, f in self.recorders:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
                p.wait(timeout=5)
            except Exception:
                pass
            f.close()
        for sig in (signal.SIGINT, signal.SIGKILL):
            if self.proc.poll() is not None:
                break
            try:
                os.killpg(os.getpgid(self.proc.pid), sig)
                self.proc.wait(timeout=10)
            except Exception:
                pass
        self.log.close()


# --------------------------------------------------------------------------
# reading what came back
# --------------------------------------------------------------------------
def parse_json_stream(path: Path):
    """gz topic --json-output pretty-prints one object per message across many
    lines, so this decodes a stream rather than reading line by line."""
    dec, out = json.JSONDecoder(), []
    text = path.read_text()
    i, n = 0, len(text)
    while i < n:
        while i < n and text[i] in " \t\r\n":
            i += 1
        if i >= n:
            break
        try:
            obj, i = dec.raw_decode(text, i)
        except json.JSONDecodeError:
            break
        out.append(obj)
    return out


def stamp_s(msg) -> float:
    h = msg.get("header", {}).get("stamp", {})
    return float(h.get("sec", 0)) + float(h.get("nsec", 0)) * 1e-9


def poses_of(msg) -> dict:
    d = {}
    for p in msg.get("pose", []):
        pos = p.get("position", {})
        d[p.get("name", "?")] = (float(pos.get("x", 0.0)),
                                 float(pos.get("y", 0.0)),
                                 float(pos.get("z", 0.0)))
    return d


def series(msgs):
    return [(stamp_s(m), poses_of(m)) for m in msgs]


def window(ser, t0, t1, name):
    return [(t, p[name]) for t, p in ser if t0 <= t <= t1 and name in p]


def spread(vals):
    if not vals:
        return None
    m = sum(vals) / len(vals)
    var = sum((v - m) ** 2 for v in vals) / len(vals)
    return {"mean": m, "rms_about_mean": math.sqrt(var),
            "p2p": max(vals) - min(vals)}


# --------------------------------------------------------------------------
# probe: measure the rig instead of assuming it
# --------------------------------------------------------------------------
def probe(out: Path, tip_mu, cube_mu, engine, step, post_z, fold_probe):
    """Three things nothing downstream may guess at:
       * where the open pads sit with every arm joint at zero;
       * how far a given fold raises them (the lift calibration);
       * the JAW MAP -- pad separation against knuckle angle, in free air.
    The jaw map exists because without it a failed gate is ambiguous between
    "the contact is unstable" and "the fingers never reached the cube", and
    those two have opposite conclusions.
    """
    sdf = build_model_sdf(out, tip_mu, post_z)
    world = build_world(out, model_body(sdf), cube_mu, engine, step,
                        cube_pose="5 5 0.02 0 0 0", pedestal_top=None)
    sim = Sim(world, out / "gz.log")
    if not sim.wait_ready():
        sim.stop()
        sys.exit("gz sim never came up; see %s" % (out / "gz.log"))
    sim.record("/world/%s/dynamic_pose/info" % WORLD_NAME, out / "poses.json")
    sim.record("/gate/joint_state", out / "joints.json")

    marks = []

    def dwell(label, secs, action=None):
        if action:
            action()
        t0 = time.time()
        time.sleep(secs)
        marks.append({"label": label, "w0": t0, "w1": time.time()})

    sim.hold_all()
    sim.play()
    t_start = time.time()
    dwell("zero", 8.0)
    dwell("folded", 8.0, lambda: sim.fold(fold_probe))
    dwell("unfolded", 5.0, lambda: sim.fold(0.0))
    jaw_q = [0.0, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80]
    for q in jaw_q:
        dwell("jaw_%.2f" % q, 4.0, lambda q=q: sim.grip(q))
    t_wall_end = time.time()
    sim.stop()

    ser = series(parse_json_stream(out / "poses.json"))
    if not ser:
        sys.exit("probe recorded nothing; see %s" % (out / "gz.log"))
    have = [(t, p) for t, p in ser if PAD_L in p and PAD_R in p]
    if not have:
        sys.exit("probe never saw the fingertip links.  Seen: %s"
                 % sorted(ser[-1][1])[:25])

    # Wall clock -> sim time, through the ACHIEVED real-time factor.  Every
    # window below is read this way; assuming 1:1 would sample the wrong
    # dwell the moment the sim ran slow.
    rtf = have[-1][0] / max(1e-6, t_wall_end - t_start)

    def tail(mark, frac=0.4):
        """the settled tail of a dwell, in sim time"""
        lo = (mark["w0"] - t_start) * rtf
        hi = (mark["w1"] - t_start) * rtf
        return [(t, p) for t, p in have if lo + (hi - lo) * (1 - frac) <= t <= hi]

    def mid_of(rows):
        pts = [tuple((a + b) / 2 for a, b in zip(p[PAD_L], p[PAD_R]))
               for _, p in rows]
        return (tuple(sum(c[k] for c in pts) / len(pts) for k in range(3))
                if pts else None)

    def sep_of(rows):
        v = [math.dist(p[PAD_L], p[PAD_R]) for _, p in rows]
        return sum(v) / len(v) if v else None

    by = {m["label"]: m for m in marks}
    mid0 = mid_of(tail(by["zero"]))
    mid1 = mid_of(tail(by["folded"]))
    if mid0 is None or mid1 is None:
        sys.exit("probe could not read the zero/folded dwells "
                 "(rtf=%.3f, sim_end=%.2f)" % (rtf, have[-1][0]))

    jaw_map = [{"knuckle_rad": q,
                "pad_origin_separation_m": sep_of(tail(by["jaw_%.2f" % q]))}
               for q in jaw_q]

    res = {
        "engine": engine,
        "achieved_rtf": rtf,
        "pad_mid_zero": mid0,
        "pad_mid_folded": mid1,
        "fold_probe_rad": fold_probe,
        "rise_at_probe_fold_m": mid1[2] - mid0[2],
        "rise_per_fold_m_per_rad": (mid1[2] - mid0[2]) / fold_probe,
        "pad_separation_open_m": sep_of(tail(by["zero"])),
        "jaw_map_free_air": jaw_map,
        "n_messages": len(ser),
        "sim_time_s": have[-1][0],
    }
    (out / "probe.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))
    return res


# --------------------------------------------------------------------------
# the gate itself
# --------------------------------------------------------------------------
def run_gate(out: Path, a, probe_res):
    sdf = build_model_sdf(out, a.tip_mu, a.post_z)

    # Cube directly between the open jaws, resting on a slim pedestal so the
    # fingers can reach below its centre without fouling the bench.
    cx, cy, cz = probe_res["pad_mid_zero"]
    cz += a.pad_z_off
    world = build_world(out, model_body(sdf), a.cube_mu, a.engine, a.step,
                        cube_pose="%.5f %.5f %.5f 0 0 0" % (cx, cy, cz),
                        pedestal_top=cz - CUBE_SIZE / 2.0)
    world.write_text(world.read_text()
                     .replace("__PED_X__", "%.5f" % cx)
                     .replace("__PED_Y__", "%.5f" % cy))

    fold = a.lift_m / probe_res["rise_per_fold_m_per_rad"]

    sim = Sim(world, out / "gz.log")
    if not sim.wait_ready():
        sim.stop()
        sys.exit("gz sim never came up; see %s" % (out / "gz.log"))
    sim.record("/world/%s/dynamic_pose/info" % WORLD_NAME, out / "poses.json")
    sim.record("/gate/joint_state", out / "joints.json")
    sim.hold_all()
    sim.play()

    phases = []

    def phase(name, secs, action=None):
        if action:
            action()
        t0 = time.time()
        phases.append({"name": name, "wall_start": t0})
        time.sleep(secs)
        phases[-1]["wall_end"] = time.time()

    phase("settle", 4.0)
    phase("close", 4.0, lambda: sim.grip(a.close_rad))
    phase("lift", 5.0, lambda: sim.fold(fold))
    phase("hold", 6.0)
    phase("rotate", 5.0, lambda: sim.cmd("joint_1", a.rotate_rad))
    phase("hold2", 5.0)
    phase("release", 5.0, lambda: sim.grip(0.0))
    sim.stop()

    return {"cube_start": (cx, cy, cz), "fold_rad": fold, "phases": phases}


def evaluate(out: Path, meta: dict, a) -> dict:
    ser = series(parse_json_stream(out / "poses.json"))
    if not ser:
        return {"verdict": "NO DATA",
                "reason": "pose recorder captured nothing"}
    # Guard for the failure above: a cube that is EXACTLY at the origin in
    # every sample was never reported, it did not sit there.
    seen = [p[CUBE] for _, p in ser if CUBE in p]
    if not seen:
        return {"verdict": "NO DATA",
                "reason": "no entity named %r in the pose stream; present: %s"
                          % (CUBE, sorted(ser[-1][1])[:20])}
    if all(v == (0.0, 0.0, 0.0) for v in seen):
        return {"verdict": "NO DATA",
                "reason": "the cube reads exactly (0,0,0) in all %d samples "
                          "-- that is an unreported pose, not a measurement"
                          % len(seen)}
    t_end = ser[-1][0]

    # Phases were driven on the wall clock, so map them into SIM time through
    # the ACHIEVED real-time factor.  Assuming 1:1 would score the wrong
    # window whenever the sim ran slow, and the verdict would be an artefact.
    ph = meta["phases"]
    wall0 = ph[0]["wall_start"]
    rtf = t_end / max(1e-6, ph[-1]["wall_end"] - wall0)
    idx = {w["name"]: i for i, w in enumerate(ph)}
    s = lambda n: (ph[idx[n]]["wall_start"] - wall0) * rtf   # noqa: E731
    e = lambda n: (ph[idx[n]]["wall_end"] - wall0) * rtf     # noqa: E731

    cz0 = meta["cube_start"][2]
    res = {"achieved_rtf": rtf, "sim_duration_s": t_end,
           "n_pose_messages": len(ser)}

    def rel(t0, t1):
        """Cube position IN THE HAND: cube minus pad midpoint.  Absolute cube
        motion during a lift is dominated by the lift, so it cannot show
        whether the grasp is steady.  The gap is what must not move."""
        o = []
        for t, p in ser:
            if t0 <= t <= t1 and CUBE in p and PAD_L in p and PAD_R in p:
                mid = tuple((x + y) / 2 for x, y in zip(p[PAD_L], p[PAD_R]))
                o.append(tuple(c - m for c, m in zip(p[CUBE], mid)))
        return o

    # --- 1. did it come up with the hand? ------------------------------
    hz = [z for _, (_, _, z) in window(ser, s("hold") + 1.0, e("hold"), CUBE)]
    if not hz:
        return dict(res, verdict="NO DATA", reason="no cube samples in HOLD")
    res["cube_rise_m"] = sum(hz) / len(hz) - cz0
    res["lift_commanded_m"] = a.lift_m
    lifted = res["cube_rise_m"] > 0.6 * a.lift_m

    # --- 2. is it steady in the hand? ----------------------------------
    r1 = rel(s("hold") + 1.0, e("hold"))
    res["hold_samples"] = len(r1)
    for k, ax in enumerate("xyz"):
        res["hold_rel_%s_mm" % ax] = (
            {kk: vv * 1000.0 for kk, vv in spread([v[k] for v in r1]).items()}
            if r1 else None)
    steady = bool(r1) and all(res["hold_rel_%s_mm" % ax]["p2p"] < a.jitter_mm
                              for ax in "xyz")

    # --- 3. does it survive the arm swinging it? -----------------------
    r2 = rel(s("hold2") + 1.0, e("hold2"))
    res["rot_samples"] = len(r2)
    for k, ax in enumerate("xyz"):
        res["rot_rel_%s_mm" % ax] = (
            {kk: vv * 1000.0 for kk, vv in spread([v[k] for v in r2]).items()}
            if r2 else None)
    held_rot = bool(r2) and all(res["rot_rel_%s_mm" % ax]["p2p"]
                                < a.jitter_mm * 2 for ax in "xyz")

    # --- 4. does it fall when released? --------------------------------
    rw = window(ser, e("release") - 2.0, e("release"), CUBE)
    if rw:
        z_after = sum(z for _, (_, _, z) in rw) / len(rw)
        res["cube_z_after_release_m"] = z_after
        res["cube_z_at_start_m"] = cz0
        # It must end up on something below, not still in the hand.
        res["cube_drop_from_hold_m"] = (sum(hz) / len(hz)) - z_after
        dropped = res["cube_drop_from_hold_m"] > 0.6 * res["cube_rise_m"]
    else:
        dropped, res["cube_z_after_release_m"] = False, None

    res["checks"] = {"lifted": lifted, "steady_in_hand": steady,
                     "held_through_arm_rotation": held_rot,
                     "fell_on_release": dropped}
    res["verdict"] = "PASS" if all(res["checks"].values()) else "FAIL"
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--tip-mu", type=float, default=None,
                    help="override the shipped fingertip mu (100000)")
    ap.add_argument("--cube-mu", type=float, default=1.0)
    ap.add_argument("--close-rad", type=float, default=0.60)
    ap.add_argument("--lift-m", type=float, default=0.100)
    ap.add_argument("--rotate-rad", type=float, default=math.pi / 2)
    ap.add_argument("--jitter-mm", type=float, default=3.0)
    ap.add_argument("--pad-z-off", type=float, default=0.0)
    ap.add_argument("--post-z", type=float, default=2.076)
    ap.add_argument("--fold-probe", type=float, default=0.30)
    ap.add_argument("--engine", default="dartsim",
                    choices=["bullet-featherstone", "dartsim", "bullet", "tpe"])
    ap.add_argument("--step", type=float, default=0.001)
    ap.add_argument("--name", default=None)
    a = ap.parse_args()

    tag = a.name or ("probe_%s" % a.engine if a.probe else
                     "%s_mu%s_close%.2f" % (a.engine,
                                            a.tip_mu if a.tip_mu is not None
                                            else "ship", a.close_rad))
    out = SCRATCH / tag
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    if a.probe:
        probe(out, a.tip_mu, a.cube_mu, a.engine, a.step, a.post_z,
              a.fold_probe)
        return

    pj = SCRATCH / ("probe_%s" % a.engine) / "probe.json"
    if pj.exists():
        pr = json.loads(pj.read_text())
    else:
        print("no probe for %s yet -- running one first" % a.engine)
        pout = SCRATCH / ("probe_%s" % a.engine)
        if pout.exists():
            shutil.rmtree(pout)
        pout.mkdir(parents=True)
        pr = probe(pout, a.tip_mu, a.cube_mu, a.engine, a.step, a.post_z,
                   a.fold_probe)

    meta = run_gate(out, a, pr)
    res = evaluate(out, meta, a)
    res["settings"] = {
        "engine": a.engine, "step_s": a.step,
        "tip_mu": a.tip_mu if a.tip_mu is not None else "as shipped (100000)",
        "cube_mu": a.cube_mu, "close_rad": a.close_rad, "lift_m": a.lift_m,
        "rotate_rad": a.rotate_rad, "jitter_limit_mm": a.jitter_mm,
        "cube_mass_kg": CUBE_MASS, "cube_size_m": CUBE_SIZE,
        "fold_rad": meta["fold_rad"]}
    (out / "result.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))
    print("\nartifacts: %s" % out)


if __name__ == "__main__":
    main()
