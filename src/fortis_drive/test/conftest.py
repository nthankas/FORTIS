"""
Per-process pytest setup for fortis_drive.

Sets ROS_DOMAIN_ID to a per-PID value before any test imports rclpy or
launches child nodes. Without this isolation, colcon test runs
fortis_drive and fortis_arm tests in parallel processes that share the
default ROS_DOMAIN_ID=0, and the test-side mission_state publishers in
each cross-talk with the other's drive node / arm controller -- the
result is intermittent state-flip flakes that pass when each package
is tested alone.

Range chosen to stay well clear of values commonly used by hand (0-9)
or by ros2cli defaults; PID modulo gives enough variety across
concurrent test processes without coordination.
"""

import os


def _isolate_ros_domain():
    # Always override -- the dev container ships with a default
    # ROS_DOMAIN_ID baked into the shell environment, which would
    # short-circuit any "respect existing value" check and re-introduce
    # the very cross-talk we are trying to prevent.
    os.environ['ROS_DOMAIN_ID'] = str((os.getpid() % 100) + 50)


_isolate_ros_domain()
