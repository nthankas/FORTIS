"""
Top-level FORTIS bringup launch file.

Composes mission_state_node (FSM) and drive_node (X-drive ROS interface)
into a single launch entry point. Arm controller, perception, localization,
and diagnostics will be added as those packages come online.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_state_node = Node(
        package='fortis_safety',
        executable='mission_state_node',
        name='mission_state_node',
        output='screen',
    )

    drive_node = Node(
        package='fortis_drive',
        executable='drive_node',
        name='drive_node',
        output='screen',
    )

    return LaunchDescription([
        mission_state_node,
        drive_node,
    ])
