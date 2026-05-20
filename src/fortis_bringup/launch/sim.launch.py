"""
Simulation-side ROS launch for FORTIS.

What this brings up
-------------------
- robot_state_publisher  serves the xacro-expanded URDF on
                         /robot_description and publishes /tf for the
                         fixed link tree. Required for any URDF-aware
                         consumer (Foxglove, RViz, Isaac Sim's URDF
                         importer when reading over the ROS bridge).
- joint_state_publisher  publishes a zero-pose /joint_states so the
                         URDF renders even when no external joint
                         source is connected. Disable with
                         publish_joint_states:=false if Isaac Sim (or
                         another sim) is streaming /joint_states.
- foxglove_bridge        WebSocket bridge so Foxglove Studio can
                         visualise the running graph.

What this does NOT do
---------------------
This launch file does NOT spawn Isaac Sim. Isaac Sim runs on the
Windows host outside the dev container, launched via the canonical
scripts at sim/isaac/xdrive/canonical/ (see sim/README.md). The
container side just publishes the URDF and the foxglove bridge so the
external sim has a ROS graph to talk to.

Pairs with
----------
- bringup.launch.py     production node graph (mission_state_node,
                        drive_node, ...). Run this in another terminal
                        when you want the FSM + drive layered on top
                        of the sim view.
- foxglove.launch.py    fortis_description's standalone URDF +
                        Foxglove launch. Use that for headless URDF
                        viewing without the sim framing; use this
                        file when you specifically want the sim
                        configuration's defaults (no GUI sliders, an
                        argument to disable joint_state_publisher).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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

    publish_joint_states = LaunchConfiguration("publish_joint_states")
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_joint_states",
            default_value="true",
            description=(
                "Run joint_state_publisher so the URDF renders even "
                "without an external joint source. Set false when "
                "Isaac Sim (or another simulator) is streaming "
                "/joint_states; running both produces a fight on the "
                "topic."
            ),
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
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            condition=IfCondition(publish_joint_states),
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
