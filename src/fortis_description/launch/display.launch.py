"""Display FORTIS in RViz with joint_state_publisher_gui sliders.

Usage:
    ros2 launch fortis_description display.launch.py

Optional args:
    use_gui:=false       # disable joint slider GUI (publishes zeros)
    rviz_config:=...     # override default rviz config
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
    default_rviz_config = PathJoinSubstitution(
        [pkg_share, "rviz", "fortis.rviz"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file]),
            value_type=str,
        )
    }

    use_gui = LaunchConfiguration("use_gui")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Run joint_state_publisher_gui (sliders).",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Path to RViz config file.",
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
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
