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

Gyro debias (default on)
------------------------
The BMI270 gyro has a small zero-rate bias, so the IMU-owned yaw integrates a
slow drift and heading-hold creeps. With `debias_imu:=true` (default) an
imu_gyro_debias_node sits between the raw IMU and the EKF: it estimates the
gyro-Z bias whenever the drive is disarmed (robot physically still) and
subtracts it from every sample, publishing /imu/debiased. The EKF then reads
/imu/debiased instead of the raw `imu_topic`, so it sees a zero-mean yaw rate
at rest. Set `debias_imu:=false` to bypass the node entirely -- the EKF reads
`imu_topic` directly, byte-for-byte the pre-debias behaviour.

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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#: Fixed intermediate topic the debias node publishes and the EKF reads when
#: debias_imu is on. Not an arg: it is internal plumbing between the two nodes
#: in THIS launch file, so exposing it would only invite a mismatch.
DEBIASED_IMU_TOPIC = "/imu/debiased"


def generate_launch_description():
    ekf_params = PathJoinSubstitution([
        FindPackageShare("fortis_localization"),
        "config",
        "ekf.yaml",
    ])

    imu_topic = LaunchConfiguration("imu_topic")
    imu_frame = LaunchConfiguration("imu_frame")
    publish_imu_tf = LaunchConfiguration("publish_imu_tf")
    debias_imu = LaunchConfiguration("debias_imu")

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
    declare_debias_imu = DeclareLaunchArgument(
        "debias_imu",
        default_value="true",
        description=(
            "Insert imu_gyro_debias_node between the raw IMU and the EKF: it "
            "auto-estimates the gyro zero-rate bias while the drive is "
            "disarmed and subtracts it, so the EKF sees a zero-mean yaw rate "
            "at rest. true: EKF reads /imu/debiased. false: EKF reads "
            "imu_topic directly (pre-debias behaviour)."
        ),
    )

    wheel_odometry_node = Node(
        package="fortis_localization",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        output="screen",
    )

    # The bias estimator reads the RAW imu_topic and republishes the corrected
    # stream on DEBIASED_IMU_TOPIC. Only launched when debias_imu is true.
    imu_gyro_debias_node = Node(
        package="fortis_localization",
        executable="imu_gyro_debias_node",
        name="imu_gyro_debias_node",
        output="screen",
        parameters=[{
            "raw_imu_topic": imu_topic,
            "debiased_imu_topic": DEBIASED_IMU_TOPIC,
        }],
        condition=IfCondition(debias_imu),
    )

    # Two EKF variants guarded by opposite conditions on debias_imu: a launch
    # remap target cannot itself be conditional, so the only difference -- which
    # topic /imu maps to -- is expressed as which of these two nodes runs.
    # debias on -> EKF consumes the debiased stream; debias off -> EKF consumes
    # the raw imu_topic exactly as before. Both carry the same name/config so
    # the running system is identical apart from the IMU source.
    ekf_node_debiased = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
        remappings=[
            ("/imu", DEBIASED_IMU_TOPIC),
            ("odometry/filtered", "/odometry/filtered"),
        ],
        condition=IfCondition(debias_imu),
    )
    ekf_node_raw = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
        remappings=[
            ("/imu", imu_topic),
            ("odometry/filtered", "/odometry/filtered"),
        ],
        condition=UnlessCondition(debias_imu),
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
        declare_debias_imu,
        wheel_odometry_node,
        imu_gyro_debias_node,
        ekf_node_debiased,
        ekf_node_raw,
        imu_to_base_tf,
    ])
