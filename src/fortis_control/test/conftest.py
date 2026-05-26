"""
Per-process pytest setup for fortis_control.

Mirror of src/fortis_drive/test/conftest.py: gives each test process a
distinct ROS_DOMAIN_ID so the launch_testing-driven controller_manager
spin-up in test_bench_launch.py does not cross-talk with other packages'
test processes running concurrently under colcon test.
"""

import os


def _isolate_ros_domain():
    # Always override -- the dev container ships with a default
    # ROS_DOMAIN_ID baked into the shell environment.
    os.environ['ROS_DOMAIN_ID'] = str((os.getpid() % 100) + 50)


_isolate_ros_domain()
