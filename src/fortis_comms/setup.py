from setuptools import find_packages, setup

package_name = 'fortis_comms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikhil Thankasala',
    maintainer_email='nikhilthankasala@gmail.com',
    description='X-drive kinematics library for FORTIS (ament_python).',
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
