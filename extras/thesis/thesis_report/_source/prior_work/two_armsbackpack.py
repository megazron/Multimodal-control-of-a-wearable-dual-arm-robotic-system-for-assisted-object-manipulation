import mujoco
import mujoco.viewer
import numpy as np
import serial
import threading
import copy
import xml.etree.ElementTree as ET
import os

SERIAL_PORT = "COM3"
BAUD_RATE   = 115200
GEN3_XML    = "mujoco_menagerie/kinova_gen3/gen3.xml"

HUMAN_XML = """
  <geom name="head" type="sphere" size="0.1"
        pos="0 0 0.35" rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="neck" type="cylinder" size="0.03 0.05"
        pos="0 0 0.22" rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="torso" type="box" size="0.14 0.07 0.22"
        pos="0 0 0" rgba="0.25 0.35 0.55 1" contype="0" conaffinity="0"/>
  <geom name="hips" type="box" size="0.10 0.07 0.06"
        pos="0 0 -0.26" rgba="0.20 0.28 0.45 1" contype="0" conaffinity="0"/>
  <geom name="l_thigh" type="cylinder" size="0.04 0.18"
        pos="0.06 0 -0.48" rgba="0.25 0.35 0.55 1" contype="0" conaffinity="0"/>
  <geom name="l_shin" type="cylinder" size="0.03 0.17"
        pos="0.06 0 -0.83" rgba="0.25 0.35 0.55 1" contype="0" conaffinity="0"/>
  <geom name="l_foot" type="box" size="0.04 0.10 0.03"
        pos="0.06 0.04 -1.02" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
  <geom name="r_thigh" type="cylinder" size="0.04 0.18"
        pos="-0.06 0 -0.48" rgba="0.25 0.35 0.55 1" contype="0" conaffinity="0"/>
  <geom name="r_shin" type="cylinder" size="0.03 0.17"
        pos="-0.06 0 -0.83" rgba="0.25 0.35 0.55 1" contype="0" conaffinity="0"/>
  <geom name="r_foot" type="box" size="0.04 0.10 0.03"
        pos="-0.06 0.04 -1.02" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
  <geom name="l_upper_arm" type="cylinder" size="0.025 0.10"
        pos="0.17 0 0.08" euler="0 0 20" rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="l_fore_arm" type="cylinder" size="0.02 0.09"
        pos="0.19 0 -0.10" euler="0 0 10" rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="r_upper_arm" type="cylinder" size="0.025 0.10"
        pos="-0.17 0 0.08" euler="0 0 -20" rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="r_fore_arm" type="cylinder" size="0.02 0.09"
        pos="-0.19 0 -0.10" euler="0 0 -10" rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="backpack_plate" type="box" size="0.13 0.03 0.20"
        pos="0 -0.11 0.02" rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0"/>
"""

def prefix_element(el, prefix):
    NAME_ATTRS  = {'name', 'joint', 'body', 'body1', 'body2',
                   'site', 'tendon', 'actuator', 'sensor'}
    CLASS_ATTRS = {'class', 'childclass'}
    el = copy.deepcopy(el)
    def recurse(e):
        for attr in list(e.attrib):
            val = e.attrib[attr]
            if attr in NAME_ATTRS:
                if not val.startswith(prefix):
                    e.attrib[attr] = prefix + val
            elif attr in CLASS_ATTRS:
                builtins = {'free', 'slide', 'hinge', 'ball', 'weld'}
                if val not in builtins and not val.startswith(prefix):
                    e.attrib[attr] = prefix + val
        for child in e:
            recurse(child)
    recurse(el)
    return el

def section_xml(root, tag, prefix):
    el = root.find(tag)
    if el is None:
        return ''
    return ET.tostring(prefix_element(el, prefix), encoding='unicode')

def worldbody_xml(root, prefix):
    wb = root.find('worldbody')
    if wb is None:
        return ''
    return ''.join(ET.tostring(prefix_element(c, prefix), encoding='unicode') for c in wb)

def load_two_arm_model(gen3_path):
    tree = ET.parse(gen3_path)
    root = tree.getroot()

    gen3_dir    = os.path.dirname(os.path.abspath(gen3_path))
    compiler_el = root.find('compiler')
    rel_meshdir = compiler_el.get('meshdir', 'assets') if compiler_el is not None else 'assets'
    abs_meshdir = os.path.join(gen3_dir, rel_meshdir).replace('\\', '/')

    asset_el  = root.find('asset')
    asset_str = ET.tostring(asset_el, encoding='unicode') if asset_el is not None else '<asset/>'

    k1_default = section_xml(root, 'default',  'k1_')
    k2_default = section_xml(root, 'default',  'k2_')
    k1_wb      = worldbody_xml(root, 'k1_')
    k2_wb      = worldbody_xml(root, 'k2_')
    k1_act     = section_xml(root, 'actuator', 'k1_')
    k2_act     = section_xml(root, 'actuator', 'k2_')
    k1_con     = section_xml(root, 'contact',  'k1_')
    k2_con     = section_xml(root, 'contact',  'k2_')
    k1_eq      = section_xml(root, 'equality', 'k1_')
    k2_eq      = section_xml(root, 'equality', 'k2_')

    return f"""
<mujoco model="two_gen3">
  <compiler meshdir="{abs_meshdir}" autolimits="true"/>
  <option gravity="0 0 0" timestep="0.002"/>
  <visual>
    <headlight ambient="0.6 0.6 0.6" diffuse="0.8 0.8 0.8"/>
  </visual>

  {asset_str}
  {k1_default}
  {k2_default}

  <worldbody>
    <light pos="0 -2 4" dir="0 0.3 -1" diffuse="0.8 0.8 0.8" castshadow="true"/>
    <geom name="floor" type="plane" size="3 3 0.1" pos="0 0 0" rgba="0.4 0.4 0.4 1"/>

    <body name="human" pos="0 0 1.05">
      {HUMAN_XML}

      <body name="k1_root" pos="0.20 -0.06 0.15" euler="-90 20 0">
        {k1_wb}
      </body>

      <body name="k2_root" pos="-0.20 -0.06 0.15" euler="-90 -20 0">
        {k2_wb}
      </body>

    </body>
  </worldbody>

  {k1_act}{k2_act}
  {k1_con}{k2_con}
  {k1_eq}{k2_eq}
</mujoco>
"""

# ── Load model ────────────────────────────────────────────────────────────────
print("Building scene...")
xml_str = load_two_arm_model(GEN3_XML)
model = mujoco.MjModel.from_xml_string(xml_str)
data  = mujoco.MjData(model)
print("Model loaded OK")

# ── Build joint qpos maps ─────────────────────────────────────────────────────
k1_qpos = {}
k2_qpos = {}

for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or ''
    adr  = model.jnt_qposadr[i]
    for j in range(1, 8):
        if 'k1_' in name and f'joint_{j}' in name:
            k1_qpos[j-1] = adr
        elif 'k2_' in name and f'joint_{j}' in name:
            k2_qpos[j-1] = adr

print(f"K1 qpos: {k1_qpos}")
print(f"K2 qpos: {k2_qpos}")

# ── Control state ─────────────────────────────────────────────────────────────
NUM_JOINTS = 7
STEP = np.deg2rad(3.0)

LIMITS_RAD = [
    (np.deg2rad(-180), np.deg2rad(180)),
    (np.deg2rad(-128), np.deg2rad(128)),
    (np.deg2rad(-180), np.deg2rad(180)),
    (np.deg2rad(-147), np.deg2rad(147)),
    (np.deg2rad(-180), np.deg2rad(180)),
    (np.deg2rad(-120), np.deg2rad(120)),
    (np.deg2rad(-180), np.deg2rad(180)),
]

k1_target = np.zeros(NUM_JOINTS)
k2_target = np.zeros(NUM_JOINTS)

def apply_targets():
    for j in range(NUM_JOINTS):
        if j in k1_qpos: data.qpos[k1_qpos[j]] = k1_target[j]
        if j in k2_qpos: data.qpos[k2_qpos[j]] = k2_target[j]
    mujoco.mj_forward(model, data)

apply_targets()

lock = threading.Lock()
pending_update = threading.Event()

# ── Serial parser ─────────────────────────────────────────────────────────────
def apply_line(line):
    line = line.strip().lower()
    if not line:
        return
    arm_data = {}
    current_arm = None
    for token in line.split(','):
        token = token.strip()
        if token.startswith('k1:'):
            current_arm = 'k1'
            rest = token[3:]
            if rest: arm_data.setdefault('k1', []).append(rest)
        elif token.startswith('k2:'):
            current_arm = 'k2'
            rest = token[3:]
            if rest: arm_data.setdefault('k2', []).append(rest)
        elif current_arm and token:
            arm_data.setdefault(current_arm, []).append(token)

    with lock:
        for arm, tokens in arm_data.items():
            target = k1_target if arm == 'k1' else k2_target
            for token in tokens:
                if len(token) < 3 or token[0] != 'j':
                    continue
                try:
                    jidx = int(token[1]) - 1
                except ValueError:
                    continue
                direction = token[2]
                if direction not in ('p', 'n') or not (0 <= jidx < NUM_JOINTS):
                    continue
                delta = STEP if direction == 'p' else -STEP
                lo, hi = LIMITS_RAD[jidx]
                target[jidx] = float(np.clip(target[jidx] + delta, lo, hi))
                print(f"  {arm.upper()} J{jidx+1} "
                      f"{'↑' if direction=='p' else '↓'} "
                      f"→ {np.rad2deg(target[jidx]):+.1f}°")
    pending_update.set()

def serial_reader():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
    except Exception as e:
        print(f"Serial error: {e}")
        return
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            print(f"\nReceived: {line}")
            apply_line(line)
        except Exception as e:
            print(f"Read error: {e}")

def clean_visuals(viewer):
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_SELECT]       = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]        = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR]     = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_COM]          = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE]    = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTOBJ]      = False
    viewer.opt.frame = mujoco.mjtFrame.mjFRAME_NONE
    viewer.opt.label = mujoco.mjtLabel.mjLABEL_NONE

# ── Main ──────────────────────────────────────────────────────────────────────
t = threading.Thread(target=serial_reader, daemon=True)
t.start()

print("Ready — format: k1:j1p,j2n,j3p,k2:j1n,j2p,j3n ...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    clean_visuals(viewer)
    while viewer.is_running():
        if pending_update.is_set():
            with lock:
                apply_targets()
            pending_update.clear()
        clean_visuals(viewer)
        viewer.sync()