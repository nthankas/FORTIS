"""
Front chassis camera bring-up (single OAK-D Lite) on depthai-ros v3.

Runs the depthai_ros_driver_v3 ``Driver`` component DIRECTLY (in a
ComposableNodeContainer) rather than including the driver's own
``driver.launch.py``. The upstream launch also starts a bundled
robot_state_publisher for the OAK's URDF (via depthai_descriptions_v3); that
second RSP subscribes to the global ``/joint_states`` and collides with the
FORTIS robot_state_publisher, spamming "Robot state publisher ignored an
invalid JointState message" at ~10 Hz. We don't need the OAK's visual URDF --
the FORTIS URDF already provides ``front_camera_link`` -- so we skip it.

The Driver publishes the camera-internal frames from factory calibration
(``driver.i_publish_tf_from_calibration``), but rooted at its OWN base frame
``oak_chassis_front`` (= driver.i_tf_base_frame) -- NOT attached to the robot.
``driver.i_tf_parent_frame`` is a xacro arg consumed by the OAK's RSP, which we
skip, so it does nothing here and the camera frames are left a disconnected TF
tree (confirmed live with ``tf2_echo``: "not part of the same tree"). We bridge
the gap with a static transform ``front_camera_link -> oak_chassis_front``
(identity; the URDF owns the mount pose). Full chain: ``base_link -> (FORTIS
URDF) -> front_camera_link -> (static) -> oak_chassis_front -> (calibration) ->
oak_chassis_front_*_optical_frame``. (The RGBD cloud is off in this mode -- RGB
and depth publish as independent streams -- so the TF chain only matters once we
re-enable fusion; the image panels don't need it.)

This is the single-camera (front) bring-up. To start ALL connected OAKs at once,
use oak_chassis_cameras.launch.py. VIO / localization are out of scope.
Pairs with robot_state_publisher (FORTIS URDF) + foxglove_bridge.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from fortis_bringup.camera_params import load_camera_params


# The node name for this single (front) camera. The shared capture config is
# loaded as a node-agnostic dict via load_camera_params(), so it no longer has
# to match a key in the yaml -- see fortis_bringup.camera_params.
CAMERA_NAME = "oak_chassis_front"

# The URDF link this camera is mounted on (fortis_description chassis xacro).
# The driver hangs its optical frames off this via the calibration TF.
PARENT_FRAME = "front_camera_link"


def generate_launch_description():
    camera_params = load_camera_params()

    # TF params. i_publish_tf_from_calibration makes the Driver publish the
    # camera-internal optical frames, rooted at i_tf_base_frame (CAMERA_NAME).
    # i_tf_parent_frame is ignored without the OAK RSP (which we skip) -- the
    # static_transform_publisher below does the parent attach.
    # (RGBD cloud is OFF: i_enable_rgbd comes from the params file, now false.)
    driver_params = {
        "driver": {
            "i_publish_tf_from_calibration": True,
            "i_tf_base_frame": CAMERA_NAME,
            "i_tf_parent_frame": PARENT_FRAME,
        },
    }

    container = ComposableNodeContainer(
        name=f"{CAMERA_NAME}_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver_v3",
                plugin="depthai_ros_driver::Driver",
                name=CAMERA_NAME,
                parameters=[camera_params, driver_params],
            ),
        ],
        output="both",
    )

    # Bridge the FORTIS URDF tree to the OAK's calibration TF tree. The driver
    # roots its frames at CAMERA_NAME but never attaches that to the robot (see
    # module docstring). Identity transform: front_camera_link is the mount
    # point; the camera-internal offsets come from calibration below it.
    mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{CAMERA_NAME}_mount_tf",
        arguments=["0", "0", "0", "0", "0", "0", PARENT_FRAME, CAMERA_NAME],
        output="both",
    )

    return LaunchDescription([
        mount_tf,
        container,
    ])
