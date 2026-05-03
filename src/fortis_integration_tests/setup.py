from setuptools import find_packages, setup

package_name = 'fortis_integration_tests'

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
    description=(
        'Cross-package integration tests for FORTIS. Test-only; '
        'no runtime entry points.'
    ),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # Test-only package -- no runtime nodes shipped here.
    entry_points={
        'console_scripts': [],
    },
)
