"""
cuVSLAM visual-inertial odometry scaffold for FORTIS.

SCAFFOLD. This launch file was written against the isaac_ros_visual_slam
source at release-3.2 (the Isaac ROS line pinned by docker/Dockerfile.dev-gpu)
and has not been run on the robot. It only runs when launched explicitly;
nothing in fortis_bringup includes it. It needs the fortis-dev-gpu image,
NGC access on the Jetson, and the front OAK-D Lite publishing its left and
right mono streams (the current chassis camera config publishes RGB and
aligned depth only).

What it does
------------
Runs nvidia::isaac_ros::visual_slam::VisualSlamNode in a multithreaded
component container as a stereo visual-inertial odometry source and remaps
its odometry output to /fortis/vo, the topic config/ekf_vio.yaml already
fuses (vx, vy only). The EKF stays the sole owner of odom -> base_link, so
both of the plugin's TF publishers are off and mapping is disabled.

Run this OR fortis_perception's rgbd_vo_node, never both: they publish the
same topic and the EKF would fuse two odometry sources as one.

VERIFY before first use
-----------------------
- The input topic defaults below are the depthai-ros v3 names for the
  front camera's mono pair and IMU; confirm them against the live driver.
- The IMU noise parameters are the plugin's own defaults, marked
  PLACEHOLDER: replace them with the BMI270 figures from the OAK datasheet
  or a calibration run.
- imu_frame must match the frame_id the IMU messages carry, and a TF path
  from it to base_link must exist (see localization.launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

#: Output topic. Matches odom1 in config/ekf_vio.yaml, so vio:=true on
#: localization.launch.py consumes cuVSLAM without any config change.
VO_TOPIC = "/fortis/vo"

#: PLACEHOLDER IMU noise model: the plugin's release-3.2 defaults, not
#: measured BMI270 values. Units follow the plugin (rad/s/sqrt(Hz),
#: rad/s^2/sqrt(Hz), m/s^2/sqrt(Hz), m/s^3/sqrt(Hz)).
IMU_NOISE_PLACEHOLDER = {
    "gyro_noise_density": 0.000244,
    "gyro_random_walk": 0.000019393,
    "accel_noise_density": 0.001862,
    "accel_random_walk": 0.003,
    "calibration_frequency": 200.0,
}


def generate_launch_description():
    left_image = LaunchConfiguration("left_image")
    left_camera_info = LaunchConfiguration("left_camera_info")
    right_image = LaunchConfiguration("right_image")
    right_camera_info = LaunchConfiguration("right_camera_info")
    imu_topic = LaunchConfiguration("imu_topic")
    imu_frame = LaunchConfiguration("imu_frame")

    # VERIFY: depthai-ros v3 topic names for the front OAK-D Lite mono pair.
    declare_args = [
        DeclareLaunchArgument(
            "left_image",
            default_value="/oak_chassis_front/left/image_raw",
            description="Left mono image of the front stereo pair (VERIFY)."),
        DeclareLaunchArgument(
            "left_camera_info",
            default_value="/oak_chassis_front/left/camera_info",
            description="Left camera info of the front stereo pair (VERIFY)."),
        DeclareLaunchArgument(
            "right_image",
            default_value="/oak_chassis_front/right/image_raw",
            description="Right mono image of the front stereo pair (VERIFY)."),
        DeclareLaunchArgument(
            "right_camera_info",
            default_value="/oak_chassis_front/right/camera_info",
            description="Right camera info of the front stereo pair (VERIFY)."),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/oak_chassis_front/imu/data",
            description="IMU stream fused with the stereo pair (VERIFY)."),
        DeclareLaunchArgument(
            "imu_frame",
            default_value="oak_chassis_front_imu_frame",
            description=(
                "frame_id the IMU messages carry; a TF path to base_link "
                "must exist (VERIFY against the live stream)."),
        ),
    ]

    visual_slam_node = ComposableNode(
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam",
        parameters=[{
            "num_cameras": 2,
            "min_num_images": 2,
            # The OAK mono streams are raw; the plugin rectifies internally
            # from the camera_info distortion model.
            "rectified_images": False,
            "enable_imu_fusion": True,
            "imu_frame": imu_frame,
            "base_frame": "base_link",
            "odom_frame": "odom",
            # Odometry only. The EKF owns odom -> base_link, so neither of
            # the plugin's TF publishers may run.
            "enable_localization_n_mapping": False,
            "publish_odom_to_base_tf": False,
            "publish_map_to_odom_tf": False,
            **IMU_NOISE_PLACEHOLDER,
        }],
        remappings=[
            ("visual_slam/image_0", left_image),
            ("visual_slam/camera_info_0", left_camera_info),
            ("visual_slam/image_1", right_image),
            ("visual_slam/camera_info_1", right_camera_info),
            ("visual_slam/imu", imu_topic),
            ("visual_slam/tracking/odometry", VO_TOPIC),
        ],
    )

    container = ComposableNodeContainer(
        name="visual_slam_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[visual_slam_node],
        output="screen",
    )

    return LaunchDescription([*declare_args, container])
