from setuptools import find_packages, setup

package_name = 'fortis_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/arm_calibration.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikhil Thankasala',
    maintainer_email='nikhilthankasala@gmail.com',
    description=(
        'Arm control for FORTIS: mission-gated gripper seam, the Teensy '
        'serial bridge, analytic 4-DOF kinematics and the MoveToPose '
        'motion node.'
    ),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_controller = fortis_arm.arm_controller_node:main',
            'arm_motion = fortis_arm.arm_motion_node:main',
            'teensy_bridge = fortis_arm.teensy_bridge_node:main',
        ],
    },
)
