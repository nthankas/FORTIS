from setuptools import find_packages, setup

package_name = 'fortis_comms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        # EKF default noise covariances. Loaded via importlib.resources
        # from fortis_comms.ekf when no override is passed to EKF().
        'fortis_comms.cfgs': ['*.json'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikhil Thankasala',
    maintainer_email='nikhilthankasala@gmail.com',
    description=(
        'Motor abstractions, X-drive kinematics, and EKF for FORTIS '
        '(ament_python library).'
    ),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # Library only -- no console scripts. ROS nodes that consume this
    # library (fortis_drive today, fortis_arm tomorrow) own their own
    # entry points.
    entry_points={
        'console_scripts': [],
    },
)
