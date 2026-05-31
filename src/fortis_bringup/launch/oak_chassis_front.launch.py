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

We still get the camera's optical-frame TF: the Driver publishes it from the
factory calibration (``driver.i_publish_tf_from_calibration``), parented to
``front_camera_link``. So the frame chain is
``base_link -> (FORTIS URDF) -> front_camera_link -> (driver calibration) ->
oak_chassis_front_*_optical_frame``, and the RGBD point cloud
(``/oak_chassis_front/rgbd/points``) resolves correctly in the 3D view.

The other three chassis cameras and any VIO / localization are out of scope.
Pairs with robot_state_publisher (FORTIS URDF) + foxglove_bridge; load
foxglove/fortis_chassis_cam.json.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# Must match the top-level key in config/oak_chassis_front.yaml, or the driver
# silently ignores the params file and runs on its own defaults.
CAMERA_NAME = "oak_chassis_front"

# The URDF link this camera is mounted on (fortis_description chassis xacro).
# The driver hangs its optical frames off this via the calibration TF.
PARENT_FRAME = "front_camera_link"


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("fortis_bringup"),
        "config",
        "oak_chassis_front.yaml",
    )

    # TF + RGBD params that driver.launch.py would normally inject. cam_pos_*
    # default to 0: the URDF owns the mounting pose; the driver only adds the
    # camera-internal optical frames below PARENT_FRAME.
    driver_params = {
        "driver": {
            "i_publish_tf_from_calibration": True,
            "i_tf_tf_prefix": CAMERA_NAME,
            "i_tf_base_frame": CAMERA_NAME,
            "i_tf_parent_frame": PARENT_FRAME,
        },
        # Publish the colored RGBD point cloud (driver.launch.py sets this when
        # pointcloud.enable:=true). The rest of the pipeline_gen / stereo / rgb
        # config comes from params_file.
        "pipeline_gen": {
            "i_enable_rgbd": True,
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
                parameters=[params_file, driver_params],
            ),
        ],
        output="both",
    )

    return LaunchDescription([
        container,
    ])
