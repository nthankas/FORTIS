# Snapshot of all four ODrive S1 axes for the X-drive. Published by a
# bridge node that translates the upstream odrive_ros2_control /odrive_status
# (or per-axis equivalent) into this FORTIS-internal contract.
#
# Why this exists: the ros2_control plugin can close the loop on the four
# S1s but does not surface per-axis disarm / undervoltage / overcurrent
# events into ros2_control's hardware-interface state. The mission FSM in
# fortis_safety needs that visibility to transition to FAULT on motor
# faults. See ADR 0002, follow-up "error feedback from the S1".
#
# Consumer: fortis_safety/odrive_health_monitor_node, which aggregates
# the four-axis view into a single /fortis/context/drive_healthy Bool and
# emits /fortis/events/fault Empty on a True->False healthy transition.
#
# Wheel field naming follows the canonical FL/FR/RL/RR convention shared
# with WheelVelocities.msg, fortis_comms.xdrive_kinematics, and the
# ros2_control YAML's joints list.

OdriveAxisHealth fl
OdriveAxisHealth fr
OdriveAxisHealth rl
OdriveAxisHealth rr
builtin_interfaces/Time stamp
