"""
Per-process pytest setup for fortis_integration_tests.

Pins this package's tests to a FIXED, unique ROS_DOMAIN_ID before any
test imports rclpy or launches child nodes. colcon test runs each
package's tests in a separate process, and on the default
ROS_DOMAIN_ID those processes share one DDS domain: the
launch_testing-launched mission_state_node here latches IDLE
(TRANSIENT_LOCAL) onto /fortis/mission_state, which then flips a
colliding drive/arm unit test's gate shut -- the intermittent
state-flip flake that passes when each package is tested alone.
Subprocess nodes launched by launch_testing inherit this env var.

A fixed per-package ID (not a pid-derived one) makes isolation
deterministic: pid % N silently collides when two concurrent test
processes share a residue, which is exactly the flake we are removing.

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
ROS test package takes the next unused ID here.
"""

import os

#: This package's dedicated test domain. See the registry in the module
#: docstring before changing it.
ROS_DOMAIN_ID = "94"


def _isolate_ros_domain():
    # Always override -- the dev container ships with a default
    # ROS_DOMAIN_ID baked into the shell environment, which would
    # otherwise re-introduce the cross-talk this isolation prevents.
    os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID


_isolate_ros_domain()
