"""
Synthetic OAK bring-up: one oak_replayer_node standing in for a real camera.

Minimal skeleton -- it will grow scene/trajectory arguments once the
renderer lands. The camera_name arg mirrors the depthai v3 namespace so
downstream perception launch files can consume either source unchanged.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")

    declare_camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="oak_chassis_front",
        description=(
            "Camera namespace to replay under; matches the depthai v3 "
            "namespace of the real camera this replayer stands in for."
        ),
    )

    oak_replayer_node = Node(
        package="fortis_sim_support",
        executable="oak_replayer_node",
        name="oak_replayer",
        output="screen",
        parameters=[{"camera_name": camera_name}],
    )

    return LaunchDescription([
        declare_camera_name,
        oak_replayer_node,
    ])
