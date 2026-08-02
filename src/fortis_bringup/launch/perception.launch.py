"""
FORTIS perception bring-up: camera source -> clouds -> map -> detection -> target.

The sprint's demo entry point. One command composes the perception chain
against either the synthetic OAK replayers (synthetic:=true, the
default: one per name in `cameras`; runs anywhere, no hardware) or the
real chassis cameras (synthetic:=false includes
oak_chassis_cameras.launch.py).

Always launched: one depth_to_cloud per name in `cameras`, cloud_fusion
(its input_topics built from the same list), voxel_map, detection,
target_selector, and system_health. Options:

  * vio:=true (default) adds rgbd_vo publishing /fortis/vo. The EKF that
    fuses it is NOT launched here: bringup.launch.py owns localization
    (localization:=true vio:=true selects ekf_vio.yaml).
  * reference_map:=<path.npz> adds map_diff against that saved run.
  * foxglove:=true (default) runs ONE foxglove_bridge on :8765. Pass
    foxglove:=false when composing with a launch that already owns a
    bridge (drive_test.launch.py / chassis_orbit.launch.py).
  * arm:=true adds teensy_bridge (see serial_port) + arm_controller +
    arm_motion (ik_ok flag and the MoveToPose action server).

The mission FSM is NOT started here: bringup.launch.py owns
mission_state_node (run `bringup.launch.py perception:=true` for the full
click-to-TARGETING flow). target_selector still publishes its pose, flags,
and chassis_cam_click event standalone; without the FSM they simply have
no consumer.

The camera list is a comma-separated launch argument, so the node set is
assembled at launch time inside an OpaqueFunction (same pattern as
oak_chassis_cameras.launch.py).
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#: Fixed bridge port; matches drive_test.launch.py's default so Foxglove
#: always connects to ws://<host>:8765 whichever launch owns the bridge.
FOXGLOVE_PORT = 8765


def _truthy(context, name):
    """Resolve a boolean launch configuration to a Python bool."""
    return LaunchConfiguration(name).perform(context).strip().lower() in ("true", "1")


def _camera_source(context, cameras):
    """Build the camera source: synthetic replayers or the real 4-cam launch."""
    if not _truthy(context, "synthetic"):
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("fortis_bringup"),
                "launch",
                "oak_chassis_cameras.launch.py",
            ])),
        )]
    # One replayer per camera name; the mount is the name's suffix
    # (oak_chassis_front -> front). Every instance broadcasts its own
    # static mount TF; only the first owns odom->base_link + ground truth.
    nodes = []
    for index, camera in enumerate(cameras):
        mount = camera.removeprefix("oak_chassis_")
        nodes.append(Node(
            package="fortis_sim_support",
            executable="oak_replayer_node",
            name=f"oak_replayer_{mount}",
            output="screen",
            parameters=[{
                "mount": mount,
                "camera_name": camera,
                "trajectory": "orbit",
                "scene": LaunchConfiguration("scene").perform(context),
                "publish_tf": True,
                "publish_base_tf": index == 0,
            }],
        ))
    return nodes


def _cloud_chain(cameras):
    """Build the per-camera cloud nodes plus fusion and the voxel map."""
    actions = [
        Node(
            package="fortis_perception",
            executable="depth_to_cloud_node",
            name=f"depth_to_cloud_{camera}",
            output="screen",
            parameters=[{"camera_name": camera}],
        )
        for camera in cameras
    ]
    actions.append(Node(
        package="fortis_perception",
        executable="cloud_fusion_node",
        name="cloud_fusion",
        output="screen",
        parameters=[{
            "input_topics": [
                f"/fortis/perception/{camera}/points" for camera in cameras
            ],
        }],
    ))
    actions.append(Node(
        package="fortis_perception",
        executable="voxel_map_node",
        name="voxel_map",
        output="screen",
    ))
    return actions


def _map_diff(context):
    """Build map_diff only when a reference map path is configured."""
    reference = LaunchConfiguration("reference_map").perform(context).strip()
    if not reference:
        return []
    return [Node(
        package="fortis_perception",
        executable="map_diff_node",
        name="map_diff",
        output="screen",
        parameters=[{"reference_map": reference}],
    )]


def _detection_and_targeting(context, cameras):
    """Build detection (first camera), target_selector, and system_health."""
    return [
        Node(
            package="fortis_perception",
            executable="detection_node",
            name="detection",
            output="screen",
            parameters=[{
                "camera_name": cameras[0],
                "detector": LaunchConfiguration("detector").perform(context),
            }],
        ),
        Node(
            package="fortis_perception",
            executable="target_selector_node",
            name="target_selector",
            output="screen",
        ),
        Node(
            package="fortis_perception",
            executable="system_health_node",
            name="system_health",
            output="screen",
            parameters=[{"cameras": cameras}],
        ),
    ]


def _vio(context, cameras):
    """Build rgbd_vo when vio:=true (the consuming EKF lives in bringup)."""
    if not _truthy(context, "vio"):
        return []
    return [Node(
        package="fortis_perception",
        executable="rgbd_vo_node",
        name="rgbd_vo",
        output="screen",
        parameters=[{"camera_name": cameras[0]}],
    )]


def _foxglove(context):
    """Build the single foxglove_bridge when foxglove:=true."""
    if not _truthy(context, "foxglove"):
        return []
    return [Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": FOXGLOVE_PORT,
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
    )]


def _arm(context):
    """Build teensy_bridge + arm_controller + arm_motion when arm:=true."""
    if not _truthy(context, "arm"):
        return []
    return [
        Node(
            package="fortis_arm",
            executable="teensy_bridge",
            name="teensy_bridge",
            output="screen",
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port").perform(context),
            }],
        ),
        Node(
            package="fortis_arm",
            executable="arm_controller",
            name="arm_controller",
            output="screen",
        ),
        Node(
            package="fortis_arm",
            executable="arm_motion",
            name="arm_motion",
            output="screen",
        ),
    ]


def _compose(context):
    """Assemble the launch actions from the resolved arguments."""
    cameras = [
        c.strip()
        for c in LaunchConfiguration("cameras").perform(context).split(",")
        if c.strip()
    ]
    if not cameras:
        raise RuntimeError("cameras must name at least one camera")
    return (
        _camera_source(context, cameras)
        + _cloud_chain(cameras)
        + _map_diff(context)
        + _detection_and_targeting(context, cameras)
        + _vio(context, cameras)
        + _foxglove(context)
        + _arm(context)
    )


def generate_launch_description():
    """Declare the arguments and defer node assembly to launch time."""
    args = [
        DeclareLaunchArgument(
            "synthetic",
            default_value="true",
            description=(
                "true: one oak_replayer per name in `cameras` (orbit "
                "trajectory, TF on, ground truth owned by the first) "
                "stands in for the camera rig. false: include the real "
                "oak_chassis_cameras.launch.py bring-up."
            ),
        ),
        DeclareLaunchArgument(
            "scene",
            default_value="baseline",
            description="Synthetic scene variant: baseline | modified.",
        ),
        DeclareLaunchArgument(
            "cameras",
            default_value="oak_chassis_front",
            description=(
                "Comma-separated camera names: one depth_to_cloud each and "
                "the cloud_fusion input set. The first name is the "
                "detection / VO camera. With synthetic:=true each name "
                "must be oak_chassis_<mount> (front|rear|left|right) and "
                "gets its own replayer."
            ),
        ),
        DeclareLaunchArgument(
            "detector",
            default_value="blob",
            description=(
                "Detection backend: blob (HSV, no weights) | yolo (needs "
                "`ros2 run fortis_perception download_models`)."
            ),
        ),
        DeclareLaunchArgument(
            "vio",
            default_value="true",
            description=(
                "Run rgbd_vo (/fortis/vo). The EKF that fuses it is owned "
                "by bringup.launch.py (localization:=true vio:=true)."
            ),
        ),
        DeclareLaunchArgument(
            "reference_map",
            default_value="",
            description=(
                "Path to a saved .npz voxel map; non-empty starts map_diff "
                "against it (save one via voxel_map's ~/save_map)."
            ),
        ),
        DeclareLaunchArgument(
            "foxglove",
            default_value="true",
            description=(
                "Run the single foxglove_bridge on :8765. Set false when a "
                "composed launch (drive_test / chassis_orbit) already owns "
                "a bridge."
            ),
        ),
        DeclareLaunchArgument(
            "arm",
            default_value="false",
            description=(
                "Run teensy_bridge + arm_controller + arm_motion "
                "(fortis_arm)."
            ),
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description=(
                "Teensy USB-CDC port for teensy_bridge; use the pty printed "
                "by tools/mock_teensy.py for hardware-free runs."
            ),
        ),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_compose)])
