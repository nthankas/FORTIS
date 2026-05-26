import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fortis_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FORTIS',
    maintainer_email='nikhilthankasala@gmail.com',
    description=(
        'ros2_control configuration for the FORTIS X-drive chassis, '
        'binding four ODrive S1s via odrive_ros2_control to a velocity '
        'group controller.'
    ),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)
