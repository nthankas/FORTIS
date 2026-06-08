import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fortis_localization'

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
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikhil Thankasala',
    maintainer_email='nikhilthankasala@gmail.com',
    description='Wheel odometry + robot_localization EKF (wheel odom + IMU) '
                'for the FORTIS X-drive base.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = '
            'fortis_localization.wheel_odometry_node:main',
            'imu_gyro_debias_node = '
            'fortis_localization.imu_gyro_debias_node:main',
        ],
    },
)
