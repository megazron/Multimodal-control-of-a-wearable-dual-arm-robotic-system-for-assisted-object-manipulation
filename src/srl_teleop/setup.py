from glob import glob

from setuptools import find_packages, setup

package_name = 'srl_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # glob, not a hand-maintained list: a new launch file that silently
        # fails to install looks exactly like a missing feature at runtime.
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gausms',
    maintainer_email='gmsayyadsvc4@gmail.com',
    description='Teleoperation only: master sensing, IK follower, clutch, '
                'scaling, sim-to-real bridge and e-stop. No autonomy, and no '
                'dependency on srl_autonomy or srl_perception.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
         'console_scripts': [
            # --- master arm sensing ---
            'master_pose_node = srl_teleop.master_pose_node:main',
            'check_buttons = srl_teleop.check_buttons:main',
            'launcher = srl_teleop.launcher:main',
            'console = srl_teleop.console:main',
            'pot_bridge = srl_teleop.pot_bridge:main',
            'fake_pot = srl_teleop.fake_pot:main',
            # NEW (Part 3): IMU-primary sensing.
            'master_imu_node = srl_teleop.master_imu_node:main',
            'pointing_direction_node = srl_teleop.pointing_direction_node:main',
            'channel_manager = srl_teleop.channel_manager:main',
            # --- following ---
            'ik_follower_node = srl_teleop.ik_follower_node:main',
            'fsr_gripper_node = srl_teleop.fsr_gripper_node:main',
            'leader_follower_node = srl_teleop.leader_follower_node:main',
            'srl_teleop_node = srl_teleop.srl_teleop_node:main',
            # --- safety ---
            'estop_node = srl_teleop.estop_node:main',
            'selftest_node = srl_teleop.selftest_node:main',
            'mount_guard_node = srl_teleop.mount_guard_node:main',
            'blocking_aggregator = srl_teleop.blocking_aggregator:main',
            'teleop_gui = srl_teleop.teleop_gui:main',
            'recovery_manager = srl_teleop.recovery_manager:main',
            'arm_link_monitor = srl_teleop.arm_link_monitor:main',
            'fault_injector = srl_teleop.fault_injector:main',
            'preflight = srl_teleop.preflight:main',
            'real_arm_gate = srl_teleop.real_arm_gate:main',
            # NEW (Part 8): participant-session limits, separate from dev.
            'participant_safety_node = srl_teleop.participant_safety_node:main',
            # --- real hardware ---
            'real_homing_node = srl_teleop.real_homing_node:main',
            'sim_to_real_bridge = srl_teleop.sim_to_real_bridge:main',
            'mock_real_stack = srl_teleop.mock_real_stack:main',
            'real_robot_state_display = srl_teleop.real_robot_state_display:main',
            # NEW (Part 6): Kinova payload configuration.
            'payload_manager = srl_teleop.payload_manager:main',
            # --- monitoring ---
            'dashboard = srl_teleop.dashboard:main',
            'live_monitor = srl_teleop.live_monitor:main',
            # --- calibration and capture ---
            'capture_zero = srl_teleop.capture_zero:main',
            'capture_gyro_bias = srl_teleop.capture_gyro_bias:main',
            'imu_mount_calibration = srl_teleop.imu_mount_calibration:main',
            'max_position_recorder = srl_teleop.max_position_recorder:main',
            'teleop_recorder = srl_teleop.teleop_recorder:main',
            'full_state_recorder = srl_teleop.full_state_recorder:main',
            'compare_master_sim = srl_teleop.compare_master_sim:main',
            'analyse_teleop = srl_teleop.analyse_teleop:main',
        ],
    },
)
