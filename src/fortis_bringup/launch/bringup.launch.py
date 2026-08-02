"""
Top-level FORTIS bringup launch file.

Composes mission_state_node (FSM), drive_node (X-drive ROS interface),
drive_enable_node (UI arm/disarm of the wheel controller), and the
odrive health monitor into a single launch entry point. The perception
chain and the arm seam are available as OPT-IN includes (see below);
diagnostics ride along with perception's system_health.

Localization (wheel odometry + robot_localization EKF) is available as an
OPT-IN include, gated by the `localization` launch arg (default false). It
is off by default so this entry point's behaviour is unchanged until the
estimation layer is bench-verified; bring it up with `localization:=true`.

Heading hold (closed-loop yaw on the EKF estimate) is a second OPT-IN, gated
by the `heading_hold` arg (default false). When true it launches
heading_hold_node AND remaps drive_node's /cmd_vel input to the controller's
/cmd_vel_heading output, so operator turns are arbitrated against the
IMU-dominated EKF yaw. When false drive_node reads /cmd_vel directly and the
node graph is byte-for-byte the existing one. heading_hold needs the EKF
running, so it implies localization:=true in practice (enable both).

The perception chain is a third OPT-IN, gated by the `perception` arg
(default false). When true it includes perception.launch.py: camera source
(synthetic replayer by default), per-camera point clouds, cloud fusion,
voxel map (+ optional cross-run diff), detection, click-to-target, system
health, optional RGBD VO, and that file's default-on foxglove_bridge --
bringup runs no bridge of its own, so it stays the only one. Pass
foxglove:=false when a composed launch (e.g. drive_test.launch.py) already
owns a bridge. Pair with localization:=true vio:=true to fuse the VO
stream into the EKF.

The arm seam is a fourth OPT-IN, gated by the `arm` arg (default false):
teensy_bridge (serial link to the arm Teensy; port from `serial_port`,
default /dev/ttyACM0 -- point it at tools/mock_teensy.py's pty for
hardware-free runs), arm_controller (mission-gated gripper services), and
arm_motion (analytic IK: the latched ik_ok context flag plus the
MoveToPose action server that emits ARM_AT_VIEW_POSE).

Loads config/bringup_params.yaml so any future declare_parameter() the
nodes adopt picks up the documented defaults automatically (heading_hold_node
already reads its gains from there). See that file's header for which values
are live vs. documentation-only.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
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

    # Opt-in heading hold. Off by default. When true, drive_node's /cmd_vel
    # subscription is remapped to /cmd_vel_heading (the controller output) so
    # heading_hold_node sits in front of the drive. When false the remap
    # target is /cmd_vel itself -- an identity remap, so drive_node reads
    # /cmd_vel exactly as before and the existing behaviour is unchanged.
    heading_hold = LaunchConfiguration("heading_hold")
    drive_cmd_vel_topic = PythonExpression([
        "'/cmd_vel_heading' if '", heading_hold, "' == 'true' else '/cmd_vel'",
    ])

    # Opt-in orbit generator. Off by default. When true, orbit_node runs and a
    # held Foxglove ORBIT button streams a face-the-center orbit on /cmd_vel --
    # which flows through heading_hold (if enabled) then drive_node, exactly
    # like a teleop command.
    orbit = LaunchConfiguration("orbit")

    # Opt-in perception chain and arm seam. Both off by default.
    perception = LaunchConfiguration("perception")
    perception_launch = PathJoinSubstitution([
        FindPackageShare("fortis_bringup"),
        "launch",
        "perception.launch.py",
    ])
    arm = LaunchConfiguration("arm")

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
        # Identity remap when heading_hold is false (reads /cmd_vel); points at
        # /cmd_vel_heading when true so the heading controller is in the loop.
        remappings=[('/cmd_vel', drive_cmd_vel_topic)],
    )

    # Closed-loop heading hold. Only launched when heading_hold:=true. Reads
    # the operator /cmd_vel and the EKF /odometry/filtered, republishes the
    # corrected command on /cmd_vel_heading (which drive_node consumes via the
    # remap above). Gains come from bringup_params.yaml (heading_hold_node).
    heading_hold_node = Node(
        package='fortis_drive',
        executable='heading_hold_node',
        name='heading_hold_node',
        output='screen',
        parameters=[bringup_params],
        condition=IfCondition(heading_hold),
    )

    # Hold-to-run, face-the-center orbit generator. Only launched when
    # orbit:=true. A held Foxglove ORBIT button streams /fortis/commands/orbit_dir,
    # which orbit_node converts into a continuous face-center /cmd_vel (strafe +
    # yaw=v/R). drive_node gates and sign-corrects it like any teleop command.
    # Speed / radius / omega_sign come from bringup_params.yaml (orbit_node).
    orbit_node = Node(
        package='fortis_drive',
        executable='orbit_node',
        name='orbit_node',
        output='screen',
        parameters=[bringup_params],
        condition=IfCondition(orbit),
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

    # Serial bridge to the arm Teensy 4.1 + the mission-gated arm controller.
    # Only launched when arm:=true. serial_port overrides the bridge's
    # declared default so a mock pty can be injected from the CLI.
    teensy_bridge_node = Node(
        package='fortis_arm',
        executable='teensy_bridge',
        name='teensy_bridge',
        output='screen',
        parameters=[bringup_params,
                    {'serial_port': LaunchConfiguration('serial_port')}],
        condition=IfCondition(arm),
    )

    arm_controller_node = Node(
        package='fortis_arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[bringup_params],
        condition=IfCondition(arm),
    )

    # IK-backed ik_ok context flag + the MoveToPose action server. Rides the
    # same `arm` gate: without it the FSM can never take TARGETING ->
    # ARM_AT_VIEW, so the seam is incomplete with bridge+controller alone.
    arm_motion_node = Node(
        package='fortis_arm',
        executable='arm_motion',
        name='arm_motion',
        output='screen',
        parameters=[bringup_params],
        condition=IfCondition(arm),
    )

    localization_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        condition=IfCondition(localization),
    )

    # Scoped group on purpose: the include pins the child's `arm` arg false
    # (bringup owns the arm nodes above) and include launch_arguments are
    # otherwise GLOBAL -- unscoped, that pin would also flip this file's
    # IfCondition(arm) off for any entity visited after it.
    perception_group = GroupAction(
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource([perception_launch]),
            launch_arguments={"arm": "false"}.items(),
        )],
        condition=IfCondition(perception),
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
        DeclareLaunchArgument(
            "heading_hold",
            default_value="false",
            description=(
                "Run heading_hold_node and feed drive_node from its "
                "/cmd_vel_heading output (closed-loop yaw on the EKF "
                "estimate). Off by default; needs the EKF, so enable "
                "alongside localization:=true. When false, drive_node reads "
                "/cmd_vel directly and behaviour is unchanged."
            ),
        ),
        DeclareLaunchArgument(
            "orbit",
            default_value="false",
            description=(
                "Run orbit_node: a held Foxglove ORBIT button streams a "
                "face-the-center orbit /cmd_vel (open-loop; radius/speed from "
                "bringup_params.yaml). Off by default; the chassis_orbit "
                "launch sets it true."
            ),
        ),
        DeclareLaunchArgument(
            "perception",
            default_value="false",
            description=(
                "Include perception.launch.py: cameras (synthetic by "
                "default) -> point clouds -> voxel map -> detection -> "
                "click-to-target + system health, plus its default-on "
                "foxglove_bridge (bringup has none of its own). Off by "
                "default; the existing node graph is unchanged unless true."
            ),
        ),
        DeclareLaunchArgument(
            "arm",
            default_value="false",
            description=(
                "Run teensy_bridge + arm_controller + arm_motion "
                "(fortis_arm). Off by default. See serial_port for the "
                "Teensy device."
            ),
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description=(
                "Teensy USB-CDC port for teensy_bridge (only used with "
                "arm:=true); use the pty printed by tools/mock_teensy.py "
                "for hardware-free runs."
            ),
        ),
        mission_state_node,
        drive_node,
        drive_enable_node,
        odrive_health_monitor_node,
        heading_hold_node,
        orbit_node,
        teensy_bridge_node,
        arm_controller_node,
        arm_motion_node,
        localization_include,
        perception_group,
    ])
