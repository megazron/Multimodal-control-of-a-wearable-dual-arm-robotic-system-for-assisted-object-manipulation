from glob import glob

from setuptools import find_packages, setup

package_name = 'srl_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # glob, not a hand-maintained list: a launch file that silently fails
        # to install looks exactly like a missing feature at runtime.
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gausms',
    maintainer_email='gmsayyadsvc4@gmail.com',
    description='Object detection and 6-DOF pose estimation for the SRL rig.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mock_rgbd_camera = srl_perception.mock_rgbd_camera:main',
            # PRIMARY detector. Named for what it does, not for the library.
            'apriltag_detector = srl_perception.apriltag_detector:main',
            # Clearly-secondary fallback; off unless explicitly enabled.
            'colour_shape_detector = srl_perception.colour_shape_detector:main',
            # Turns detections into stable, world-frame object poses.
            'object_pose_tracker = srl_perception.object_pose_tracker:main',
            # Scene fingerprinting: calibrate only when the bed has changed.
            'scene_fingerprint_node = srl_perception.scene_fingerprint_node:main',
            # Measures the work surface from depth and is the ONLY producer
            # for srl_experiments.work_surface.set_measured(), which had none
            # until now -- the height was declared and never measured.
            'work_surface_node = srl_perception.work_surface_node:main',
            # Offline characterisation: detection rate, accuracy, latency.
            'measure_detector = srl_perception.measure_detector:main',
            # Legacy VLM locator, kept but not part of the study pipeline.
            'vlm_object_locator = srl_perception.vlm_object_locator:main',
        ],
    },
)
