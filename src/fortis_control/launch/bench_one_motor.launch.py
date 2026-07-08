r"""
FORTIS bench bring-up launch: one motor (FL) over CAN.

What this brings up
-------------------
  * robot_state_publisher with the chassis URDF expanded for
    ros2_control on a single wheel.
  * controller_manager loaded with the odrive_ros2_control hardware
    plugin against a single S1 at CAN node_id 0.
  * joint_state_broadcaster
  * wheel_velocity_controller (velocity_controllers/JointGroupVelocityController)
    with `joints: [fl_wheel_joint]`.

What this does NOT do
---------------------
  * Does not arm the axis. wheel_velocity_controller is spawned
    --inactive, so the S1 stays in IDLE until the operator explicitly
    activates the controller (which is what arms it -- see below).
    This is intentional: bench launch should never auto-spin a motor.
  * Does not bring up fortis_safety or fortis_drive. This is a hardware
    transport test, not a mission test. Once it is green, run the full
    stack via fortis_bringup.

Pre-requisites
--------------
  1. The single S1 on the bench is calibrated per tools/odrive_calibrate.md
     and its node_id is set to 0.
  2. The USB-CAN adapter is plugged into the host. Bring up SocketCAN
     before launching:

         sudo ip link set can0 up type can bitrate 250000
         sudo ip link set can0 txqueuelen 1000

     Verify with `candump can0` -- the S1 heartbeat should appear at
     ~10 Hz.

  3. If the adapter does not enumerate as native SocketCAN (no can0 in
     `ip link`), it is a slcan-style dongle. Use:

         sudo modprobe slcan
         sudo slcand -o -c -s5 /dev/ttyACM0 can0
         sudo ip link set can0 up
         sudo ip link set can0 txqueuelen 1000

     (-s5 = 250 kbps, matches odrive_calibrate.md.)

How to use after launch
-----------------------
  # Activate the controller. odrive_ros2_control auto-arms the axis
  # (sends AXIS_STATE_CLOSED_LOOP_CONTROL over CAN). No separate
  # /request_axis_state call is needed; that service only exists on the
  # standalone odrive_node, not on the ros2_control plugin.
  ros2 control switch_controllers --activate wheel_velocity_controller

  # Spin at 2 rad/s
  ros2 topic pub --once /wheel_velocity_controller/commands \\
      std_msgs/msg/Float64MultiArray "{data: [2.0]}"

  # Stop and disarm. Deactivating the controller auto-IDLEs the axis.
  ros2 topic pub --once /wheel_velocity_controller/commands \\
      std_msgs/msg/Float64MultiArray "{data: [0.0]}"
  ros2 control switch_controllers --deactivate wheel_velocity_controller

  # Watch joint state stream
  ros2 topic echo /joint_states
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Thin wrapper over drive_hw.launch.py. The bench is the same control
    # stack -- controller_manager + odrive plugin + joint_state_broadcaster
    # + an --inactive wheel_velocity_controller, brought up in the same
    # order -- but with only the FL wheel enabled and the single-wheel
    # controllers YAML. Delegating to drive_hw keeps ONE launch
    # implementation, so the bench and production bring-ups can never drift
    # in spawner ordering or plugin wiring; only the wheel set and the
    # controllers file differ.
    declared_arguments = [
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="SocketCAN interface name, passed through to drive_hw.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description=(
                "If true, use mock_components/GenericSystem instead of the "
                "ODrive plugin. Useful for testing the launch + controller "
                "wiring without a CAN bus."
            ),
        ),
    ]

    drive_hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fortis_control"),
                "launch",
                "drive_hw.launch.py",
            ])
        ),
        launch_arguments={
            "wheels": "fl",
            "controllers_file": "fortis_drive_controllers_bench.yaml",
            "can_interface": LaunchConfiguration("can_interface"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [drive_hw])
