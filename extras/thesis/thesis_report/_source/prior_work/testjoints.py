import mujoco
import mujoco.viewer
import numpy as np
import copy
import xml.etree.ElementTree as ET
import os

GEN3_XML = "mujoco_menagerie/kinova_gen3/gen3.xml"

def prefix_element(el, prefix):
    NAME_ATTRS  = {'name','joint','body','body1','body2','site','tendon','actuator','sensor'}
    CLASS_ATTRS = {'class','childclass'}
    el = copy.deepcopy(el)
    def recurse(e):
        for attr in list(e.attrib):
            val = e.attrib[attr]
            if attr in NAME_ATTRS:
                if not val.startswith(prefix): e.attrib[attr] = prefix + val
            elif attr in CLASS_ATTRS:
                if val not in {'free','slide','hinge','ball','weld'} and not val.startswith(prefix):
                    e.attrib[attr] = prefix + val
        for child in e: recurse(child)
    recurse(el)
    return el

def section_xml(root, tag, prefix):
    el = root.find(tag)
    return '' if el is None else ET.tostring(prefix_element(el, prefix), encoding='unicode')

def worldbody_xml(root, prefix):
    wb = root.find('worldbody')
    return '' if wb is None else ''.join(
        ET.tostring(prefix_element(c, prefix), encoding='unicode') for c in wb)

tree = ET.parse(GEN3_XML)
root = tree.getroot()
gen3_dir    = os.path.dirname(os.path.abspath(GEN3_XML))
abs_meshdir = os.path.join(gen3_dir, root.find('compiler').get('meshdir','assets')).replace('\\','/')
asset_str   = ET.tostring(root.find('asset'), encoding='unicode')

xml_str = f"""
<mujoco model="two_gen3">
  <compiler meshdir="{abs_meshdir}" autolimits="true"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  {asset_str}
  {section_xml(root,'default','k1_')}
  {section_xml(root,'default','k2_')}
  <worldbody>
    <geom name="floor" type="plane" size="3 3 0.1"/>
    <body name="k1_root" pos="-0.5 0 0.5">
      {worldbody_xml(root,'k1_')}
    </body>
    <body name="k2_root" pos="0.5 0 0.5">
      {worldbody_xml(root,'k2_')}
    </body>
  </worldbody>
  {section_xml(root,'actuator','k1_')}
  {section_xml(root,'actuator','k2_')}
</mujoco>"""

model = mujoco.MjModel.from_xml_string(xml_str)
data  = mujoco.MjData(model)

# Get indices
k1_act = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'k1_joint_{i}') for i in range(1,8)]
k2_act = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'k2_joint_{i}') for i in range(1,8)]
k1_jnt = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'k1_joint_{i}')] for i in range(1,8)]
k2_jnt = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'k2_joint_{i}')] for i in range(1,8)]

print(f"k1_act: {k1_act}")
print(f"k2_act: {k2_act}")
print(f"k1_jnt: {k1_jnt}")
print(f"k2_jnt: {k2_jnt}")
print(f"model.nu (total actuators): {model.nu}")

# Set a target and check actuator type
home = [0.0, np.deg2rad(-90), 0.0, np.deg2rad(90), 0.0, 0.0, 0.0]
for i in range(7):
    data.qpos[k1_jnt[i]] = home[i]
    data.ctrl[k1_act[i]] = home[i]
    data.qpos[k2_jnt[i]] = home[i]
    data.ctrl[k2_act[i]] = home[i]

mujoco.mj_forward(model, data)

print(f"\ndata.ctrl = {data.ctrl}")
print(f"data.qpos[:14] = {data.qpos[:14]}")

# Print actuator types — this tells us if they are position controllers
print("\nActuator types (0=motor,1=servo,2=cylinder,3=muscle,4=general,5=position,6=velocity):")
for i in range(model.nu):
    print(f"  {i} '{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)}': "
          f"trntype={model.actuator_trntype[i]} dyntype={model.actuator_dyntype[i]} "
          f"gaintype={model.actuator_gaintype[i]} biastype={model.actuator_biastype[i]}")

# Run 1 second and see if arms hold
print("\nRunning 500 steps to see if arms hold position...")
for _ in range(500):
    for i in range(7):
        data.ctrl[k1_act[i]] = home[i]
        data.ctrl[k2_act[i]] = home[i]
    mujoco.mj_step(model, data)

print(f"qpos after 500 steps: {np.rad2deg(data.qpos[:14]).round(1)}")
print("If these match home pose the actuators work. If not, gains are wrong.")

# Launch viewer so you can see
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        for i in range(7):
            data.ctrl[k1_act[i]] = home[i]
            data.ctrl[k2_act[i]] = home[i]
        mujoco.mj_step(model, data)
        viewer.sync()