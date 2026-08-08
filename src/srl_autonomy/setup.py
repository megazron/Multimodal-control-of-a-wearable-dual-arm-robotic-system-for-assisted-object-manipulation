from glob import glob

from setuptools import find_packages, setup

package_name = 'srl_autonomy'

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
    description='Shared autonomy: grasp generation, intent inference, handover.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # Offers only grasps that /compute_ik has already accepted.
            'grasp_generator = srl_autonomy.grasp_generator:main',
            # P(goal) over detected objects, driven by IMU pointing.
            'intent_inference = srl_autonomy.intent_inference:main',
            # DIRECT / ASSIST / GRASPED. Discrete, operator-confirmed.
            'handover_arbiter = srl_autonomy.handover_arbiter:main',
            # Legacy scripted demos, not part of the study pipeline.
            'scripted_pick_place = srl_autonomy.scripted_pick_place:main',
            # Mode 5/6 state machine: the ONE place mode is set.
            'autonomy_executive = srl_autonomy.autonomy_executive:main',
            # Wake word + STT -> /voice_transcript.
            'voice_listener = srl_autonomy.voice_listener:main',
            'move_to_pose = srl_autonomy.move_to_pose:main',
        ],
    },
)
