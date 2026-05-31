"""
Front chassis camera bring-up (single OAK-D Lite) on depthai-ros v3.

Includes Luxonis's depthai_ros_driver_v3 ``driver.launch.py`` for the ONE
front-facing OAK-D Lite: RGB + stereo depth + a colored RGBD point cloud +
IMU. The other three chassis cameras (rear / left / right) and any VIO /
localization are intentionally out of scope here.

depthai-ros v3 vs the old v2 driver (what changed here)
-------------------------------------------------------
  * package ``depthai_ros_driver_v3`` (plugin depthai_ros_driver::Driver),
    launch ``driver.launch.py``  -- was depthai_ros_driver / camera.launch.py.
  * the device is pinned by ``driver.i_device_id`` in the params yaml -- was
    ``camera.i_mx_id``.
  * the colored point cloud publishes on ``<name>/rgbd/points`` when
    pointcloud.enable:=true -- was ``<name>/points``.
  * ``publish_tf_from_calibration`` (v3 default true) builds the camera TF
    tree under ``parent_frame``, so parent_frame:=front_camera_link still
    hangs the optical frames off the URDF link, same as v2.

Frame tie-in
------------
parent_frame is ``front_camera_link`` -- the link the URDF places on the
chassis (fortis_chassis.urdf.xacro, camera name "front"). cam_pos_* stay 0:
the URDF owns the mounting pose, the driver only adds the camera-internal
frames below it.

Pairs with robot_state_publisher (for the URDF /tf tree) + foxglove_bridge;
load foxglove/fortis_chassis_cam.json.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# Must match the top-level key in config/oak_chassis_front.yaml, or the
# driver silently ignores the params file and runs on its own defaults.
CAMERA_NAME = "oak_chassis_front"

# The URDF link this camera is mounted on (fortis_description chassis xacro).
PARENT_FRAME = "front_camera_link"


def generate_launch_description():
    driver_launch = os.path.join(
        get_package_share_directory("depthai_ros_driver_v3"),
        "launch",
        "driver.launch.py",
    )
    params_file = os.path.join(
        get_package_share_directory("fortis_bringup"),
        "config",
        "oak_chassis_front.yaml",
    )

    front_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={
            "name": CAMERA_NAME,
            "parent_frame": PARENT_FRAME,
            "params_file": params_file,
            "camera_model": "OAK-D-LITE",
            # Publish the colored RGBD point cloud (default off in the driver).
            "pointcloud.enable": "true",
            "use_rviz": "false",
        }.items(),
    )

    return LaunchDescription([
        front_camera,
    ])
