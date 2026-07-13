from setuptools import find_packages, setup

package_name = 'srl_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/move_to_pose.launch.py', 'launch/pick_place.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gausms',
    maintainer_email='gausms@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
         'console_scripts': [
             'pot_bridge = srl_teleop.pot_bridge:main',
            'fake_pot = srl_teleop.fake_pot:main',
            'srl_teleop_node = srl_teleop.srl_teleop_node:main',
            'vlm_locate_node = srl_teleop.vlm_locate_node:main',
             'move_to_pose_node = srl_teleop.move_to_pose_node:main',
            'spawn_cubes = srl_teleop.spawn_cubes:main',
            'pick_place_node = srl_teleop.pick_place_node:main',
        ],
    },
)
