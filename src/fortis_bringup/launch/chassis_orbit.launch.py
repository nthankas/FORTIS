"""
One-command FORTIS orbit-demo bring-up: cameras + drive stack + orbit.

Composes, in a single launch:
  * drive_test.launch.py with orbit:=true
        the full X-drive stack -- controller_manager + odrive_ros2_control on
        all four wheels, mission FSM, drive_node, drive_enable_node, the ODrive
        health monitor, ONE foxglove_bridge (:8765), AND orbit_node (the
        hold-to-run face-the-center orbit generator).
  * oak_chassis_cameras.launch.py
        every connected OAK-D Lite (serial-pinned front/rear/left/right) as an
        independent depthai-ros v3 driver: on-device MJPEG RGB + 16UC1 depth,
        no RGBD cloud, IMU on.

So one command brings up everything for the orbit run on the Jetson. The
operator drives it from Foxglove: ENABLE DRIVE, START ORBIT (open the mission
gate), then HOLD an ORBIT CCW / CW button to circle. Releasing the button
stops (orbit_node's hold dead-man + drive_node's watchdog).

Heading hold is OFF by default (heading_hold:=false): the orbit runs open-loop
("rough orbit"). It drifts -- expected at this stage, which is why the orbit
speed defaults to 0.1 m/s. Flip heading_hold:=true (with localization:=true) to
try closing the yaw loop on the IMU-dominated EKF.

One bridge, not two
-------------------
Foxglove is a CLIENT of the single :8765 bridge -- you do NOT run a second
bridge. Open the teleop layout and the camera layout in two Studio windows (or
browser tabs) against the same ws://<jetson>:8765. Foxglove only subscribes to
a topic when a VISIBLE panel uses it, so the camera Tab panel (one camera tab
at a time) keeps the raw-depth bandwidth survivable over Wi-Fi.

Args
----
    can_interface:=can1        SocketCAN interface for the ODrive bus (gs_usb adapter)
    use_mock_hardware:=false   swap the ODrive plugin for a mock (no CAN)
    port:=8765                 foxglove_bridge WebSocket port
    localization:=false        wheel odometry + EKF (needed for heading_hold)
    heading_hold:=false        closed-loop yaw on the EKF (needs localization:=true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    port = LaunchConfiguration("port")
    localization = LaunchConfiguration("localization")
    heading_hold = LaunchConfiguration("heading_hold")

    bringup_share = FindPackageShare("fortis_bringup")

    # The whole drive stack + bridge, with the orbit generator switched on. All
    # the hardware-test args pass straight through; only orbit is pinned true.
    drive_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            bringup_share, "launch", "drive_test.launch.py",
        ])),
        launch_arguments={
            "can_interface": can_interface,
            "use_mock_hardware": use_mock_hardware,
            "port": port,
            "localization": localization,
            "heading_hold": heading_hold,
            "orbit": "true",
        }.items(),
    )

    # Every connected OAK. Discovery happens at launch time (on the Jetson where
    # the cameras are attached); no devices found => it logs and starts nothing,
    # so this is harmless on the dev PC.
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            bringup_share, "launch", "oak_chassis_cameras.launch.py",
        ])),
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
            description="WebSocket port for the single foxglove_bridge.",
        ),
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            description="Wheel odometry + EKF (needed for heading_hold).",
        ),
        DeclareLaunchArgument(
            "heading_hold",
            default_value="false",
            description=(
                "Closed-loop yaw on the IMU-dominated EKF (needs "
                "localization:=true). Off => open-loop 'rough' orbit."
            ),
        ),
        drive_stack,
        cameras,
    ])
