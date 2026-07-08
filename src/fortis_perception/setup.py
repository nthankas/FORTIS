from setuptools import find_packages, setup

package_name = 'fortis_perception'

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
    description='FORTIS perception stack: RGBD point clouds, multi-camera '
                'fusion, voxel mapping with cross-run diff, RGBD visual '
                'odometry, object detection, targeting, and system health.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_to_cloud_node = fortis_perception.depth_to_cloud_node:main',
            'cloud_fusion_node = fortis_perception.cloud_fusion_node:main',
            'voxel_map_node = fortis_perception.voxel_map_node:main',
            'map_diff_node = fortis_perception.map_diff_node:main',
            'rgbd_vo_node = fortis_perception.rgbd_vo_node:main',
            'detection_node = fortis_perception.detection_node:main',
            'target_selector_node = fortis_perception.target_selector_node:main',
            'system_health_node = fortis_perception.system_health_node:main',
            'download_models = fortis_perception.download_models:main',
        ],
    },
)
