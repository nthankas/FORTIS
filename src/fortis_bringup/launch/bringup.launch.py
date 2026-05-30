"""
Top-level FORTIS bringup launch file.

Composes mission_state_node (FSM), drive_node (X-drive ROS interface),
drive_enable_node (UI arm/disarm of the wheel controller), and the
odrive health monitor into a single launch entry point. Arm controller,
perception, localization, and diagnostics will be added as those
packages come online.

Loads config/bringup_params.yaml so any future declare_parameter() the
nodes adopt picks up the documented defaults automatically. See that
file's header for why most values there are inert today.
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_params = PathJoinSubstitution([
        FindPackageShare("fortis_bringup"),
        "config",
        "bringup_params.yaml",
    ])

    mission_state_node = Node(
        package='fortis_safety',
        executable='mission_state_node',
        name='mission_state_node',
        output='screen',
        parameters=[bringup_params],
    )

    drive_node = Node(
        package='fortis_drive',
        executable='drive_node',
        name='drive_node',
        output='screen',
        parameters=[bringup_params],
    )

    # Wraps the controller_manager switch_controller service behind a
    # plain Bool topic (/fortis/commands/drive_enable) so the operator
    # can arm/disarm the wheel drive from the UI with a button, without
    # touching ros2_control internals. See fortis_drive/drive_enable_node.py.
    drive_enable_node = Node(
        package='fortis_drive',
        executable='drive_enable_node',
        name='drive_enable_node',
        output='screen',
        parameters=[bringup_params],
    )

    # Aggregates per-axis ODrive health snapshots into a single
    # /fortis/context/drive_healthy Bool and emits /fortis/events/fault
    # on the True->False edge. The mission_state_node consumes
    # drive_healthy as a context field; the FAULT event drives the FSM
    # into State.FAULT. See fortis_safety/odrive_health_monitor_node.py
    # for the design rationale.
    odrive_health_monitor_node = Node(
        package='fortis_safety',
        executable='odrive_health_monitor_node',
        name='odrive_health_monitor_node',
        output='screen',
        parameters=[bringup_params],
    )

    return LaunchDescription([
        mission_state_node,
        drive_node,
        drive_enable_node,
        odrive_health_monitor_node,
    ])
