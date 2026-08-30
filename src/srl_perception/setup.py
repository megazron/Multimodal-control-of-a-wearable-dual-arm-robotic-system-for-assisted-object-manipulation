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
            # THE SCENE CAMERA: the front-of-table view of the table, the
            # objects and the WEARER. Its job is to make the wearer's body a
            # measured quantity rather than a declared one.
            'scene_camera_node = srl_perception.scene_camera_node:main',
            # The body tracker is NOT registered here on purpose. It needs
            # MediaPipe, which lives in .venv_pose and must not be installed
            # system-wide -- `ros2 run` would start it under the system
            # interpreter, where it cannot import its model and would refuse
            # every frame. scripts/run_wearer_tracker.sh starts it correctly.
            # The end-to-end check is scripts/verify_scene_camera.py, a
            # script rather than a node: it must be runnable with no stack.
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
            # ALL FOUR CAMERAS through one vision pipeline, publishing what
            # each one saw WITH provenance -- and refusing, by name, for the
            # ones that cannot answer (no frames, zero K, no plane).
            'scene_understanding_node = srl_perception.scene_understanding_node:main',
            # The seam from measured objects to MoveIt: mapped_* ids only,
            # add/grow only; removal is an explicit Trigger, never a timeout.
            'map_obstacles_node = srl_perception.map_obstacles_node:main',
        ],
    },
)
