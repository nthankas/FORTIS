"""
Top-level FORTIS bringup launch file.

Composes mission_state_node (FSM) and drive_node (X-drive ROS
interface) into a single launch entry point. Arm controller,
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

    return LaunchDescription([
        mission_state_node,
        drive_node,
    ])
