"""
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
  * Does not arm the axis. The S1 starts in IDLE. Explicit
    `/request_axis_state` call required to enter CLOSED_LOOP_CONTROL.
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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
    # ----- Args ---------------------------------------------------------
    declared_arguments = [
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description=(
                "SocketCAN interface name passed to "
                "odrive_ros2_control_plugin/ODriveHardwareInterface."
            ),
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description=(
                "If true, use mock_components/GenericSystem instead of "
                "the ODrive plugin. Useful for testing the launch + "
                "controller wiring without a CAN bus."
            ),
        ),
    ]

    can_interface = LaunchConfiguration("can_interface")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # ----- robot_description --------------------------------------------
    # Expand the chassis xacro with ros2_control enabled, only FL wired,
    # and the can interface threaded through. The wheels arg "fl" matches
    # the joints list in fortis_drive_controllers_bench.yaml.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("fortis_description"),
            "urdf",
            "fortis_robot.urdf.xacro",
        ]),
        " enable_ros2_control:=true",
        " wheels:=fl",
        " can_interface:=", can_interface,
        " use_mock_hardware:=", use_mock_hardware,
    ])
    # ParameterValue(..., value_type=str) prevents launch_ros from trying
    # to YAML-parse the xacro output (a long XML string starts with `<`,
    # which yaml interprets as a flow-sequence error). Required for any
    # robot_description that comes from a Command substitution.
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("fortis_control"),
        "config",
        "fortis_drive_controllers_bench.yaml",
    ])

    # ----- Nodes --------------------------------------------------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager", "/controller_manager",
            # --inactive: load the controller but do not activate it on
            # spawn. The bench operator activates it explicitly once the
            # ODrive axis is in CLOSED_LOOP_CONTROL. Prevents the
            # controller from claiming the velocity command interface
            # against an axis that is still IDLE, which on some
            # combinations of odrive_ros2_control + ros2_control will log
            # an error every update cycle.
            "--inactive",
        ],
    )

    # joint_state_broadcaster must come up before any other controller is
    # spawned so DDS discovery for /joint_states is established. The
    # botwheel_explorer example uses the same OnProcessExit pattern.
    delay_wheel_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[wheel_velocity_controller_spawner],
        )
    )

    return LaunchDescription(declared_arguments + [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_wheel_controller_after_joint_state_broadcaster,
    ])
