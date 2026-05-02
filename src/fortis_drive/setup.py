from setuptools import find_packages, setup

package_name = 'fortis_drive'

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
    description='X-drive ROS 2 wrapper for FORTIS, gated by mission state.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_node = fortis_drive.drive_node:main',
        ],
    },
)
