from glob import glob
from setuptools import find_packages, setup

package_name = 'srl_vr_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gausms',
    maintainer_email='gmsayyadsvc4@gmail.com',
    description='Meta Quest teleoperation: a separate input modality from the '
                'mannequin master. Full 6-DOF including yaw.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'quest_bridge_node = srl_vr_teleop.quest_bridge_node:main',
        'quest_vendor_mock = srl_vr_teleop.quest_vendor_mock:main',
        'quest_vendor_bridge = srl_vr_teleop.quest_vendor_bridge:main',
        'wearer_view = srl_vr_teleop.wearer_view:main',
        'vr_pose_mapper = srl_vr_teleop.vr_pose_mapper:main',
        'vr_gripper_node = srl_vr_teleop.vr_gripper_node:main',
        'vr_feedback_node = srl_vr_teleop.vr_feedback_node:main',
        'vr_safety_node = srl_vr_teleop.vr_safety_node:main',
        'vr_mock_publisher = srl_vr_teleop.vr_mock_publisher:main',
    ]},
)
