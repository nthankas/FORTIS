"""
Front chassis camera bring-up (single OAK-D Lite).

Launches Luxonis's official depthai_ros_driver ``camera.launch.py`` for the
ONE front-facing OAK-D Lite and nothing else. The other three chassis
cameras (rear / left / right) are intentionally out of scope here -- this
file exists to prove the front camera streams RGB, depth, an RGBD point
cloud, and raw IMU into Foxglove. EKF / localization / VIO are deliberately
NOT wired up.

Why include the driver's launch instead of spawning a Node
----------------------------------------------------------
depthai_ros_driver builds the on-device pipeline (color + stereo + optional
point cloud + optional NN) and publishes the camera_info / TF tree from the
device's factory calibration. Re-implementing that as a bare Node would mean
re-deriving all of it. We only override four things: the node name, the
parent TF frame, our params file, and turning the point cloud on.

Frame tie-in
------------
parent_frame is set to ``front_camera_link`` -- the link the URDF already
places on the chassis (see fortis_chassis.urdf.xacro, camera name "front").
The driver hangs its own optical frames off that link, so the point cloud
and images land in the right place relative to base_link in the 3D view.
cam_pos_* are left at 0 because the URDF, not the driver, owns the mounting
pose.

Pairs with
----------
- robot_state_publisher (for /robot_description + the URDF TF tree)
- foxglove_bridge (to view it; load foxglove/fortis_chassis_cam.json)
Run those via bringup or in separate terminals.
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
        get_package_share_directory("depthai_ros_driver"),
        "launch",
        "camera.launch.py",
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
            "rectify_rgb": "true",
            "use_rviz": "false",
        }.items(),
    )

    return LaunchDescription([
        front_camera,
    ])
