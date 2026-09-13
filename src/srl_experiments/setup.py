from glob import glob

from setuptools import find_packages, setup

package_name = 'srl_experiments'

# Each experiment lives in its own folder under experiments/, exactly as the
# protocol documents describe it. run_*.py / analyse_*.py are installed as
# executables (`ros2 run srl_experiments run_fitts.py`) rather than as module
# entry points, so the folder on disk and the thing you run are the same file.
# TWO levels, not one. The bimanual set nests one folder deeper
# (experiments/bimanual/t3_coordinated_carry/analyse_t3.py), so a single-level
# glob installed run_bimanual.py and NONE of the seven analysers -- they have
# never been packaged, and `ros2 run srl_experiments analyse_t3.py` has always
# failed while looking like an installation problem.
EXP_SCRIPTS = sorted(
    glob('experiments/*/run_*.py') + glob('experiments/*/analyse_*.py')
    + glob('experiments/*/*/run_*.py') + glob('experiments/*/*/analyse_*.py')
    # THREE levels now, because the superseded sets live under
    # experiments/_archive/<set>/<task>/. They are ARCHIVED, NOT DELETED --
    # docs/ENGINEERING_LOG.md cites several of them by name -- so they must still install,
    # and a stale two-level glob broke the whole package build after the move.
    + glob('experiments/_archive/*/*/run_*.py')
    + glob('experiments/_archive/*/*/analyse_*.py'))

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
]
# protocol.md, config.yaml and scenarios/ travel with the package so a run is
# reproducible from the install tree alone.
for d in sorted(glob('experiments/*/') + glob('experiments/*/*/')
                + glob('experiments/_archive/*/*/')):
    rel = d.rstrip('/')[len('experiments/'):]
    files = [f for f in glob(d + '*') if f.endswith(('.md', '.yaml'))]
    if files:
        data_files.append(('share/%s/experiments/%s' % (package_name, rel), files))
    scen = glob(d + 'scenarios/*.yaml')
    if scen:
        data_files.append(('share/%s/experiments/%s/scenarios' % (package_name, rel), scen))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    scripts=EXP_SCRIPTS,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gausms',
    maintainer_email='gmsayyadsvc4@gmail.com',
    description='Experiment runner, conditions, logging and analysis. Current set: T2/T3/T5/T6/T7 (run_bimanual.py); E1-E6 superseded but kept as analysis backends.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'record_trajectories = trajectory_capture.record_trajectories:main',
            'scene_spawner = srl_experiments.scene_spawner:main',
            'task_scene = srl_experiments.task_scene:main',
            'scripted_operator = srl_experiments.scripted_operator:main',
            'session_manager = srl_experiments.session_manager:main',
        ],
    },
)
