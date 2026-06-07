"""
One-shot FORTIS X-drive hardware-test launch.

Brings up the entire drive stack for the end-to-end X-drive test in a
single command, so the only variables left at the bench are hardware.

What it composes
----------------
  * fortis_control/drive_hw.launch.py
        controller_manager + odrive_ros2_control plugin on all four
        wheels (CAN node_ids 0..3), robot_state_publisher,
        joint_state_broadcaster, and the wheel_velocity_controller
        loaded INACTIVE.
  * fortis_bringup/bringup.launch.py
        mission_state_node (FSM), drive_node (cmd_vel -> wheels),
        odrive_health_monitor_node.
  * foxglove_bridge (node only)
        WebSocket telemetry/teleop bridge on :8765.

Why only the foxglove *node* and not foxglove.launch.py
-------------------------------------------------------
foxglove.launch.py and drive_hw.launch.py would BOTH start a
robot_state_publisher, and foxglove.launch.py also starts a
joint_state_publisher that publishes a fake zero /joint_states. On real
hardware we want exactly one robot_state_publisher (from drive_hw) and
the REAL /joint_states from joint_state_broadcaster. So we include
drive_hw for those and add only the foxglove_bridge node here.

Safety: nothing arms on launch
------------------------------
The wheel_velocity_controller comes up INACTIVE on purpose (no motor is
energised by launching this file). Arming and opening the motion gate
are deliberate, separate manual steps:

    # 0. SocketCAN already up + 4 heartbeats seen (see tools/xdrive_bringup.md)
    ros2 launch fortis_bringup drive_test.launch.py        # default can_interface:=can1

    # 1. arm the wheels (sends CLOSED_LOOP_CONTROL to all four S1s):
    ros2 control switch_controllers --activate wheel_velocity_controller

    # 2. open the motion gate (mission FSM IDLE -> ORBIT):
    ros2 topic pub --once /fortis/events/start_orbit std_msgs/msg/Empty "{}"

    # 3. drive from Foxglove Teleop (/cmd_vel), or from the CLI:
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

Args
----
    can_interface:=can1        SocketCAN interface for the ODrive bus (gs_usb adapter)
    use_mock_hardware:=false   swap the ODrive plugin for a mock (no CAN)
    port:=8765                 foxglove_bridge WebSocket port
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    port = LaunchConfiguration("port")

    drive_hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("fortis_control"),
            "launch",
            "drive_hw.launch.py",
        ])),
        launch_arguments={
            "can_interface": can_interface,
            "use_mock_hardware": use_mock_hardware,
        }.items(),
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("fortis_bringup"),
            "launch",
            "bringup.launch.py",
        ])),
    )

    foxglove_bridge = Node(
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
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "can_interface",
            default_value="can1",
            description=(
                "SocketCAN interface for the ODrive bus. Default can1 is the "
                "gs_usb USB-CAN adapter on the Jetson; can0 is the onboard "
                "Tegra mttcan and is NOT wired to the ODrive chain."
            ),
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock_components/GenericSystem instead of the ODrive plugin.",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="8765",
            description="WebSocket port for foxglove_bridge.",
        ),
        drive_hw,
        bringup,
        foxglove_bridge,
    ])
