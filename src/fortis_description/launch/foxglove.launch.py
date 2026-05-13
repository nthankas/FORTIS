"""Publish FORTIS to Foxglove Studio over a WebSocket bridge.

Usage:
    ros2 launch fortis_description foxglove.launch.py

Then in Foxglove Studio (desktop or web):
    Open connection -> Foxglove WebSocket -> ws://localhost:8765
    Add a "3D" panel, set the "Frame" display to URDF + TF, fixed frame = base_link.

Optional args:
    use_gui:=false   # disable joint slider GUI (publishes zero joint angles)
    port:=8765       # foxglove_bridge WebSocket port
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("fortis_description")
    xacro_file = PathJoinSubstitution(
        [pkg_share, "urdf", "fortis_robot.urdf.xacro"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file]),
            value_type=str,
        )
    }

    use_gui = LaunchConfiguration("use_gui")
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Run joint_state_publisher_gui (sliders).",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="8765",
            description="WebSocket port for foxglove_bridge.",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(use_gui),
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": port,
                "address": "0.0.0.0",
                "tls": False,
                "use_compression": False,
                "capabilities": [
                    "clientPublish",
                    "parameters",
                    "parametersSubscribe",
                    "services",
                    "connectionGraph",
                    "assets",
                ],
            }],
        ),
    ])
