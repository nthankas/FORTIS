"""
FORTIS production bring-up launch: all four wheels over CAN.

What this brings up
-------------------
  * robot_state_publisher with the full chassis URDF expanded for
    ros2_control on all four wheels.
  * controller_manager loaded with the odrive_ros2_control hardware
    plugin against four ODrive S1s at CAN node_ids 0..3.
  * joint_state_broadcaster
  * wheel_velocity_controller (JointGroupVelocityController) with
    [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint].

Pre-requisites
--------------
  1. All four S1s calibrated per tools/odrive_calibrate.md with node_ids
     0/1/2/3 for FL/FR/RR/RL respectively. Sharpie labels match.
  2. CAN harness wired FL->FR->RR->RL (chain position = node_id), USB-CAN
     adapter at the FL end with built-in 120 Ohm termination, RL's on-board
     termination jumper closed at the other end.
  3. SocketCAN brought up:

         sudo ip link set can0 up type can bitrate 250000
         sudo ip link set can0 txqueuelen 1000

     `candump can0` should show heartbeats from all four node_ids.

Composition with fortis_drive / fortis_safety
---------------------------------------------
This launch only stands up the controller_manager + hardware plugin
+ the velocity controller. It does NOT start drive_node or
mission_state_node. For the full mission stack, launch this and
fortis_bringup/bringup.launch.py side by side, or include this from a
top-level bringup once fortis_bringup is wired to ros2_control.
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
                "the ODrive plugin."
            ),
        ),
        DeclareLaunchArgument(
            "wheels",
            default_value="fl,fr,rl,rr",
            description=(
                "Comma-separated wheels to enable in the ros2_control "
                "xacro. Production is all four (fl,fr,rl,rr); the bench "
                "launch overrides this to a single wheel."
            ),
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="fortis_drive_controllers.yaml",
            description=(
                "Filename (under fortis_control/config) of the "
                "controller_manager YAML. Production uses the four-wheel "
                "config; the bench launch overrides to the single-wheel one."
            ),
        ),
    ]

    can_interface = LaunchConfiguration("can_interface")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    wheels = LaunchConfiguration("wheels")
    controllers_file = LaunchConfiguration("controllers_file")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("fortis_description"),
            "urdf",
            "fortis_robot.urdf.xacro",
        ]),
        " enable_ros2_control:=true",
        " wheels:=", wheels,
        " can_interface:=", can_interface,
        " use_mock_hardware:=", use_mock_hardware,
    ])
    # See bench_one_motor.launch.py for why ParameterValue is required.
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("fortis_control"),
        "config",
        controllers_file,
    ])

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
            "--inactive",
        ],
    )

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
