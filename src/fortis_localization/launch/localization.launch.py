"""
FORTIS localization bring-up: wheel odometry + robot_localization EKF.

Brings up
---------
- wheel_odometry_node   integrates /joint_states into /odom (body twist +
                        dead-reckoned pose). Does NOT broadcast TF.
- ekf_filter_node       robot_localization EKF; fuses /odom + /imu into
                        /odometry/filtered and OWNS the odom->base_link TF
                        (config/ekf.yaml, publish_tf: true).
- imu_to_base_tf        OPTIONAL static transform attaching the IMU frame to
                        base_link, so the EKF can transform the IMU yaw rate
                        into the base frame. See the imu_frame / publish_imu_tf
                        args below for when this is needed vs. a duplicate.

IMU topic remap
---------------
The front OAK-D Lite publishes its IMU on /oak_chassis_front/imu/data (depthai
v3 nests it under /imu/data); the EKF config subscribes to /imu. The
`imu_topic` arg drives a remap so the source
topic can change (e.g. a different camera) without editing ekf.yaml.

Why the static IMU TF is conditional
-------------------------------------
The EKF needs a TF path from the IMU message's frame_id to base_link. The
depthai driver roots its calibration frames at `oak_chassis_front`, which is
only attached to the robot (front_camera_link -> oak_chassis_front, then the
FORTIS URDF base_link -> front_camera_link) while oak_chassis_front.launch.py
is running. The FORTIS URDF itself does NOT model the camera's internal IMU
frame, so base_link -> <imu_frame> never exists from robot_state_publisher
alone. This launch therefore publishes a static IMU-frame -> base_link
transform by default so localization works standalone. Set
publish_imu_tf:=false when the camera launch is already supplying the full
base_link -> ... -> imu_frame chain, to avoid a duplicate transform.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_params = PathJoinSubstitution([
        FindPackageShare("fortis_localization"),
        "config",
        "ekf.yaml",
    ])

    imu_topic = LaunchConfiguration("imu_topic")
    imu_frame = LaunchConfiguration("imu_frame")
    publish_imu_tf = LaunchConfiguration("publish_imu_tf")

    declare_imu_topic = DeclareLaunchArgument(
        "imu_topic",
        default_value="/oak_chassis_front/imu/data",
        description=(
            "Source IMU topic, remapped to /imu for the EKF. Defaults to the "
            "front OAK-D Lite (depthai v3 nests it at .../imu/data)."
        ),
    )
    declare_imu_frame = DeclareLaunchArgument(
        "imu_frame",
        default_value="oak_chassis_front_imu_frame",
        description=(
            "frame_id the IMU messages carry. Used as the child of the "
            "static IMU->base_link transform. VERIFY against the live IMU: "
            "`ros2 topic echo <imu_topic> --field header.frame_id`."
        ),
    )
    declare_publish_imu_tf = DeclareLaunchArgument(
        "publish_imu_tf",
        default_value="true",
        description=(
            "Publish a static imu_frame->base_link transform. Set false when "
            "oak_chassis_front.launch.py already supplies the base_link->...->"
            "imu_frame chain, to avoid a duplicate TF."
        ),
    )

    wheel_odometry_node = Node(
        package="fortis_localization",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        output="screen",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
        remappings=[
            ("/imu", imu_topic),
            ("odometry/filtered", "/odometry/filtered"),
        ],
    )

    # Identity static transform: the OAK IMU sits at the chassis front, close
    # to base_link, and two_d_mode only consumes its yaw RATE -- a rate is
    # invariant to a pure translation, so the lever arm does not bias it. A
    # real rotational offset between the IMU axes and base_link WOULD matter;
    # set the rpy args here once the camera's mounting orientation is measured.
    imu_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_to_base_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", imu_frame],
        condition=IfCondition(publish_imu_tf),
        output="screen",
    )

    return LaunchDescription([
        declare_imu_topic,
        declare_imu_frame,
        declare_publish_imu_tf,
        wheel_odometry_node,
        ekf_node,
        imu_to_base_tf,
    ])
