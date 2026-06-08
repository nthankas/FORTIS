"""
Top-level FORTIS bringup launch file.

Composes mission_state_node (FSM), drive_node (X-drive ROS interface),
drive_enable_node (UI arm/disarm of the wheel controller), and the
odrive health monitor into a single launch entry point. Arm controller,
perception, and diagnostics will be added as those packages come online.

Localization (wheel odometry + robot_localization EKF) is available as an
OPT-IN include, gated by the `localization` launch arg (default false). It
is off by default so this entry point's behaviour is unchanged until the
estimation layer is bench-verified; bring it up with `localization:=true`.

Loads config/bringup_params.yaml so any future declare_parameter() the
nodes adopt picks up the documented defaults automatically. See that
file's header for why most values there are inert today.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_params = PathJoinSubstitution([
        FindPackageShare("fortis_bringup"),
        "config",
        "bringup_params.yaml",
    ])

    # Opt-in estimation layer. Off by default so existing bring-up is
    # byte-for-byte unchanged; set localization:=true to add wheel odometry
    # + the robot_localization EKF (and the odom->base_link TF it owns).
    localization = LaunchConfiguration("localization")
    localization_launch = PathJoinSubstitution([
        FindPackageShare("fortis_localization"),
        "launch",
        "localization.launch.py",
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

    localization_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        condition=IfCondition(localization),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            description=(
                "Bring up wheel odometry + the robot_localization EKF "
                "(fortis_localization). Off by default; the existing node "
                "graph is unchanged unless this is true."
            ),
        ),
        mission_state_node,
        drive_node,
        drive_enable_node,
        odrive_health_monitor_node,
        localization_include,
    ])
