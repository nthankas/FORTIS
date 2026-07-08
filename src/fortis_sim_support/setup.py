import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fortis_sim_support'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikhil Thankasala',
    maintainer_email='nikhilthankasala@gmail.com',
    description='Hardware-free synthetic data sources for FORTIS: a '
                'procedural RGBD scene renderer and an OAK camera replayer '
                'publishing the exact depthai v3 topic contract.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'oak_replayer_node = '
            'fortis_sim_support.oak_replayer_node:main',
        ],
    },
)
