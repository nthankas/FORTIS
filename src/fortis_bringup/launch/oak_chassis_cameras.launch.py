"""
Multi-camera bring-up for all connected OAK-D Lite chassis cameras.

Discovers every connected OAK and starts each one as an independent depthai-ros
v3 ``Driver`` (on-device MJPEG RGB + regular depth, no RGBD cloud), each in its
own ComposableNodeContainer.

Naming: each camera is pinned by serial to its chassis position via CAMERA_ROSTER
below (oak_chassis_front / _rear / _left / _right), so topics are stable across
runs regardless of USB enumeration order. Discovery only spawns the cameras that
are actually plugged in. Each roster camera also gets a static identity TF
``<pos>_camera_link -> oak_chassis_<pos>`` attaching the driver's calibration TF
tree to the FORTIS URDF (same bridge as oak_chassis_front.launch.py -- see its
docstring for the full why). Unknown-serial cameras have no URDF mount link, so
their frames stay a detached TF tree.

Params come from config/oak_chassis_cameras.yaml (the tested capture config),
loaded once via fortis_bringup.camera_params.load_camera_params() and re-applied
to every node as a node-scoped dict so the values are a single source of truth
across the single-cam and multi-cam launches.

Pairs with foxglove_bridge; load foxglove/fortis_chassis_cams.json (4-tab toggle,
one camera per tab -> only the active camera streams).
"""

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from fortis_bringup.camera_params import load_camera_params


def _discover_device_ids():
    """Return the serials of all connected OAK devices.

    Returns an empty list if depthai or the devices are unavailable (e.g. on the
    dev PC where the cameras aren't attached).
    """
    try:
        import depthai as dai

        return [dev.getDeviceId() for dev in dai.Device.getAllAvailableDevices()]
    except Exception as exc:  # noqa: BLE001 - discovery is best-effort at launch time
        print(f"[oak_chassis_cameras] device discovery unavailable: {exc}")
        return []


def build_camera_node(camera_name, device_id, camera_params):
    """Build the ComposableNodeContainer that runs ONE camera's Driver.

    Same Driver as the single-cam front launch; the only per-camera differences
    are the node name and the i_device_id serial pin. i_tf_base_frame is rooted
    at the node name so each camera's calibration frames stay namespaced;
    build_mount_tf attaches that root to the robot URDF.
    """
    per_node_params = {
        "driver": {
            "i_device_id": device_id,
            "i_publish_tf_from_calibration": True,
            "i_tf_base_frame": camera_name,
        },
    }
    return ComposableNodeContainer(
        name=f"{camera_name}_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver_v3",
                plugin="depthai_ros_driver::Driver",
                name=camera_name,
                parameters=[camera_params, per_node_params],
            ),
        ],
        output="both",
    )


def build_mount_tf(camera_name):
    """Build the static identity TF attaching a camera's TF root to the URDF.

    Same bridge as oak_chassis_front.launch.py: the Driver roots its calibration
    frames at camera_name but never attaches them to the robot. The URDF mount
    link follows from the roster name (oak_chassis_<pos> -> <pos>_camera_link);
    identity because the URDF owns the mount pose.
    """
    parent_frame = camera_name.removeprefix("oak_chassis_") + "_camera_link"
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{camera_name}_mount_tf",
        arguments=["0", "0", "0", "0", "0", "0", parent_frame, camera_name],
        output="both",
    )


# Physical OAK serial -> chassis position. Confirmed live 2026-06-05 by bringing
# all four up and identifying each stream in Foxglove. Discovery only spawns the
# cameras that are actually connected; a serial not in this map gets a generic
# name so it still comes up (and is obvious in the logs) rather than being dropped.
CAMERA_ROSTER = {
    "19443010610BDE7D00": "oak_chassis_front",
    "19443010C1DF397E00": "oak_chassis_rear",
    "1944301011323C7E00": "oak_chassis_left",
    "1944301031353C7E00": "oak_chassis_right",
}


def _spawn_cameras(context):
    camera_params = load_camera_params()
    device_ids = _discover_device_ids()
    if not device_ids:
        return [LogInfo(msg="[oak_chassis_cameras] no OAK devices found -- nothing to start.")]

    actions = [LogInfo(msg=f"[oak_chassis_cameras] starting {len(device_ids)} camera(s):")]
    for device_id in device_ids:
        camera_name = CAMERA_ROSTER.get(device_id, f"oak_chassis_unknown_{device_id[:6]}")
        actions.append(LogInfo(msg=f"   {camera_name} -> {device_id}"))
        actions.append(build_camera_node(camera_name, device_id, camera_params))
        # Only roster cameras have a URDF mount link to attach to.
        if device_id in CAMERA_ROSTER:
            actions.append(build_mount_tf(camera_name))
    return actions


def generate_launch_description():
    # Discovery has to run at launch time (when the cameras are attached, i.e. on
    # the Jetson), so the node list is built inside an OpaqueFunction.
    return LaunchDescription([OpaqueFunction(function=_spawn_cameras)])
