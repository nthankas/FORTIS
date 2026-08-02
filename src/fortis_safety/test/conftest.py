"""
Per-process pytest setup for fortis_safety.

Pins this package's tests to a FIXED, unique ROS_DOMAIN_ID before any
test imports rclpy or launches child nodes, so the bridge round-trip
test cannot cross-talk with sibling test packages running in parallel
under colcon test. Same rationale as fortis_drive's conftest.

ROS_DOMAIN_ID registry -- keep in sync across ALL test conftests:
    fortis_drive ............. 91
    fortis_arm ............... 92
    fortis_control ........... 93
    fortis_integration_tests . 94
    fortis_localization ...... 95
    fortis_perception ........ 96
    fortis_sim_support ....... 97
    fortis_safety ............ 99
IDs are in the safe 0-101 range (avoids the ephemeral-port band > 101),
clear of the container/CI default (42) and hand-used values (0-9). A new
ROS test package takes the next unused ID here (98 is earmarked for the
planned fortis_gz package).
"""

import os

#: This package's dedicated test domain. See the registry in the module
#: docstring before changing it.
ROS_DOMAIN_ID = "99"


def _isolate_ros_domain():
    # Always override -- the dev container ships with a default
    # ROS_DOMAIN_ID baked into the shell environment, which would
    # otherwise re-introduce the cross-talk this isolation prevents.
    os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID


_isolate_ros_domain()
