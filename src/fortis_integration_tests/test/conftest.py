"""
Per-process pytest setup for fortis_integration_tests.

Sets ROS_DOMAIN_ID to a per-PID value before any test imports rclpy or
launches child nodes. Without this isolation, colcon test runs the
launch_testing-based integration tests in parallel with
fortis_arm and fortis_drive unit tests on the default
ROS_DOMAIN_ID=0; the test-side mission_state publishers cross-talk
with the launched mission_state_node, producing intermittent
state-flip flakes that pass when each package is tested alone.

Range chosen to stay well clear of values commonly used by hand (0-9)
or by ros2cli defaults; PID modulo gives enough variety across
concurrent test processes without coordination.
"""

import os


def _isolate_ros_domain():
    # Always override -- the dev container ships with a default
    # ROS_DOMAIN_ID baked into the shell environment, which would
    # short-circuit any "respect existing value" check and re-introduce
    # the very cross-talk we are trying to prevent. Subprocess nodes
    # launched by launch_testing inherit this env var.
    os.environ['ROS_DOMAIN_ID'] = str((os.getpid() % 100) + 50)


_isolate_ros_domain()
