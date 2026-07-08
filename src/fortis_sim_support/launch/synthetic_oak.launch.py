"""
Synthetic OAK bring-up: one oak_replayer standing in for a real chassis camera.

Publishes the depthai v3 topic contract under /<camera_name>/ plus
/fortis/sim/ground_truth, so downstream perception launch files consume
either the real camera or this replayer unchanged. Defaults mirror the
real capture config (640x400 @ 15 fps) on the default 1 m orbit.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare the replayer arguments and start one oak_replayer node."""
    args = [
        DeclareLaunchArgument(
            "camera_name",
            default_value="oak_chassis_front",
            description=(
                "Camera namespace to replay under; matches the depthai v3 "
                "namespace of the real camera this replayer stands in for."
            ),
        ),
        DeclareLaunchArgument(
            "trajectory",
            default_value="orbit",
            description="Base trajectory: orbit | line | hold.",
        ),
        DeclareLaunchArgument(
            "scene",
            default_value="baseline",
            description="Scene variant: baseline | modified (one box added, one removed).",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="Broadcast odom->base_link plus the static camera mount TF.",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="15.0",
            description="Render/publish rate; default matches the real capture config.",
        ),
    ]

    replayer = Node(
        package="fortis_sim_support",
        executable="oak_replayer_node",
        name="oak_replayer",
        output="screen",
        parameters=[{
            "camera_name": LaunchConfiguration("camera_name"),
            "trajectory": LaunchConfiguration("trajectory"),
            "scene": LaunchConfiguration("scene"),
            "publish_tf": ParameterValue(
                LaunchConfiguration("publish_tf"), value_type=bool),
            "fps": ParameterValue(LaunchConfiguration("fps"), value_type=float),
        }],
    )

    return LaunchDescription(args + [replayer])
