"""
Per-process pytest setup for fortis_arm.

Sets ROS_DOMAIN_ID to a per-PID value before any test imports rclpy or
launches child nodes. Without this isolation, colcon test runs
fortis_arm and fortis_drive (and the safety+drive integration test) in
parallel processes that share the default ROS_DOMAIN_ID=0; the
test-side publisher in fortis_arm publishing /fortis/mission_state
then cross-talks with the drive integration test's real
mission_state_node, producing intermittent state-flip flakes.

Picking from a wide upper-band range keeps the chosen domain away from
common defaults developers run by hand (0-9), which would otherwise
collide if a developer was inspecting topics while CI ran.
"""

import os


def _isolate_ros_domain():
    # Always override -- the dev container ships with a default
    # ROS_DOMAIN_ID baked into the shell environment, which would
    # short-circuit any "respect existing value" check and re-introduce
    # the very cross-talk we are trying to prevent. Range chosen to
    # stay clear of values commonly used by hand (0-9) and by ros2cli
    # defaults; PID modulo gives enough variety across concurrent test
    # processes without coordination.
    os.environ['ROS_DOMAIN_ID'] = str((os.getpid() % 100) + 50)


_isolate_ros_domain()
