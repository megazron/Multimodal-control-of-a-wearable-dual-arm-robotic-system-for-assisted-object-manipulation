from glob import glob
from setuptools import find_packages, setup

package_name = 'srl_vr_autonomy'

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
    description='Shared autonomy for the VR path: assistance supplies PRECISION, not missing DOFs.',

    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'vr_intent_source = srl_vr_autonomy.vr_intent_source:main',
        'vr_handover_arbiter = srl_vr_autonomy.vr_handover_arbiter:main',
    ]},
)
