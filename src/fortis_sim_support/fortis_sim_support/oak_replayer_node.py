"""
oak_replayer -- hardware-free synthetic OAK camera publisher.

Renders fortis_sim_support.synthetic_scene through the raycaster along
an analytic trajectory and publishes the EXACT topic contract of one
real chassis camera (depthai-ros v3 as configured by
fortis_bringup/config/oak_chassis_cameras.yaml: on-device MJPEG RGB +
rgb-aligned 16UC1 depth + IMU):

    /<camera_name>/rgb/image_raw/compressed  sensor_msgs/CompressedImage (jpeg)
    /<camera_name>/rgb/camera_info           sensor_msgs/CameraInfo
    /<camera_name>/stereo/image_raw          sensor_msgs/Image (16UC1, rgb-aligned)
    /<camera_name>/stereo/camera_info        sensor_msgs/CameraInfo (same K as rgb)
    /<camera_name>/imu/data                  sensor_msgs/Imu

plus exact ground truth for error-bound tests:

    /fortis/sim/ground_truth                 nav_msgs/Odometry (odom -> base_link)

The `mount` parameter picks which of the four chassis-camera mounts
(front | rear | left | right, URDF poses in MOUNTS) this instance
replays; camera_name defaults to oak_chassis_<mount>. With publish_tf
the node broadcasts, statically, base_link -> <camera_name> (its own
mount pose) and <camera_name> -> <camera_name>_rgb_camera_optical_frame,
so the TF chain matches the live bring-up (the intermediate
<mount>_camera_link is collapsed into the mount transform here).

Every instance samples the SAME base trajectory -- one rigid robot,
up to four cameras -- so exactly one instance per rig may own the
dynamic odom->base_link TF and the ground-truth topic: publish_base_tf
(default true) gates both, and a multi-camera launch enables it on the
first replayer only.

Trajectory time is n / rate (not wall clock): a run is deterministic
for a given seed, and every image shares its exact sample time with
the ground-truth message published beside it.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from fortis_sim_support import trajectory
from fortis_sim_support.raycaster import render
from fortis_sim_support.synthetic_scene import scene_baseline, scene_modified

# --- Constants ---------------------------------------------------------------

NODE_NAME = "oak_replayer"
GROUND_TRUTH_TOPIC = "/fortis/sim/ground_truth"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

#: Nominal OAK-D Lite RGB horizontal FOV; fx = fy is derived from it.
HFOV_RAD: float = math.radians(69.0)

#: Chassis-camera mount poses (xyz, rpy) in base_link, keyed by mount
#: face; hardcoded from fortis_description/urdf/fortis_chassis.urdf.xacro
#: + fortis_constants.xacro:
#:   cam_front_x = chassis_length/2 + cam_edge_to_housing + oak_lite_z/2
#:               = 0.332/2 + 0.01933 + 0.017/2 = 0.19383
#:   cam_side_y  = chassis_width/2  + cam_edge_to_housing + oak_lite_z/2
#:               = 0.217/2 + 0.01933 + 0.017/2 = 0.13633
#:   z = cam_height_z = 0.21514 (all four)
#: User "front" is base_link -X (chassis convention), so front sits at
#: -cam_front_x with yaw pi and is pitched 30 deg up (pitch -0.524);
#: rear/left/right are level, yawed to face outward from their face.
MOUNTS = {
    "front": ((-0.19383, 0.0, 0.21514), (0.0, -0.524, math.pi)),
    "rear": ((0.19383, 0.0, 0.21514), (0.0, 0.0, 0.0)),
    "left": ((0.0, -0.13633, 0.21514), (0.0, 0.0, -math.pi / 2.0)),
    "right": ((0.0, 0.13633, 0.21514), (0.0, 0.0, math.pi / 2.0)),
}

#: URDF optical-frame convention (fortis_chassis.urdf.xacro): Z fwd, X right, Y down.
OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)

#: Standard gravity for the synthetic accelerometer (m/s^2).
GRAVITY: float = 9.80665

#: Planar depth beyond this renders 0 ("no return"), like the real sensor.
MAX_RANGE_M: float = 8.0

#: Central-difference step for world acceleration (s); O(dt^2) error is
#: negligible against the closed-form trajectories.
_ACCEL_DT: float = 1e-3


# --- Pure helpers ------------------------------------------------------------


def _pinhole_k(width: int, height: int) -> np.ndarray:
    """Build the pinhole intrinsic matrix from the nominal horizontal FOV."""
    fx = (width / 2.0) / math.tan(HFOV_RAD / 2.0)
    return np.array([
        [fx, 0.0, width / 2.0],
        [0.0, fx, height / 2.0],
        [0.0, 0.0, 1.0],
    ])


def _mat_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build the rotation matrix for URDF-style fixed-axis rpy (R = Rz Ry Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return the (x, y, z, w) quaternion for URDF-style fixed-axis rpy."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _transform_from_rpy(xyz, rpy) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a translation and fixed-axis rpy."""
    t = np.eye(4)
    t[:3, :3] = _mat_from_rpy(*rpy)
    t[:3, 3] = xyz
    return t


def _transform_from_yaw(pos: np.ndarray, yaw: float) -> np.ndarray:
    """Build the 4x4 world<-body transform for a planar (yaw-only) pose."""
    c, s = math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[0, 0], t[0, 1] = c, -s
    t[1, 0], t[1, 1] = s, c
    t[:3, 3] = pos
    return t


def _tf_msg(parent: str, child: str, xyz, quat_xyzw, stamp) -> TransformStamped:
    """Build one stamped transform message."""
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(xyz[0])
    msg.transform.translation.y = float(xyz[1])
    msg.transform.translation.z = float(xyz[2])
    msg.transform.rotation.x = float(quat_xyzw[0])
    msg.transform.rotation.y = float(quat_xyzw[1])
    msg.transform.rotation.z = float(quat_xyzw[2])
    msg.transform.rotation.w = float(quat_xyzw[3])
    return msg


# --- Node --------------------------------------------------------------------


class OakReplayerNode(Node):
    """Render the synthetic scene along a trajectory and publish it as an OAK."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(NODE_NAME, parameter_overrides=parameter_overrides)

        self.declare_parameter("mount", "front")
        self.declare_parameter("camera_name", "")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 400)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("imu_rate_hz", 100.0)
        self.declare_parameter("trajectory", "orbit")
        self.declare_parameter("orbit_radius", 1.0)
        self.declare_parameter("orbit_omega", 0.3)
        self.declare_parameter("scene", "baseline")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_base_tf", True)
        self.declare_parameter("imu_gyro_bias_z", 0.0)
        self.declare_parameter("imu_noise_std", 0.0)
        self.declare_parameter("jpeg_quality", 60)
        self.declare_parameter("seed", 0)

        mount = str(self.get_parameter("mount").value)
        if mount not in MOUNTS:
            raise ValueError(f"unknown mount {mount!r} (expected {'|'.join(MOUNTS)})")
        self._mount_xyz, self._mount_rpy = MOUNTS[mount]
        # Empty camera_name (the default) derives the real rig's namespace.
        name = str(self.get_parameter("camera_name").value).strip()
        self._camera_name = name or f"oak_chassis_{mount}"
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._fps = float(self.get_parameter("fps").value)
        self._imu_rate_hz = float(self.get_parameter("imu_rate_hz").value)
        self._gyro_bias_z = float(self.get_parameter("imu_gyro_bias_z").value)
        self._imu_noise_std = float(self.get_parameter("imu_noise_std").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        if self._fps <= 0.0 or self._imu_rate_hz <= 0.0:
            raise ValueError("fps and imu_rate_hz must be > 0")

        scene_name = str(self.get_parameter("scene").value)
        self._scene = self._build_scene(scene_name)
        trajectory_name = str(self.get_parameter("trajectory").value)
        self._trajectory = self._build_trajectory(
            trajectory_name,
            float(self.get_parameter("orbit_radius").value),
            float(self.get_parameter("orbit_omega").value),
        )

        self._k = _pinhole_k(self._width, self._height)
        # Camera pose in the body: URDF mount, then the optical rotation.
        self._t_base_cam = (
            _transform_from_rpy(self._mount_xyz, self._mount_rpy)
            @ _transform_from_rpy((0.0, 0.0, 0.0), OPTICAL_RPY)
        )
        self._rgb_frame = f"{self._camera_name}_rgb_camera_optical_frame"
        self._imu_frame = f"{self._camera_name}_imu_frame"
        self._bridge = CvBridge()
        self._rng = np.random.default_rng(int(self.get_parameter("seed").value))
        self._camera_info = self._make_camera_info()
        self._frame_index = 0
        self._imu_index = 0

        base = f"/{self._camera_name}"
        self._rgb_pub = self.create_publisher(
            CompressedImage, f"{base}/rgb/image_raw/compressed", 10)
        self._rgb_info_pub = self.create_publisher(
            CameraInfo, f"{base}/rgb/camera_info", 10)
        self._depth_pub = self.create_publisher(
            Image, f"{base}/stereo/image_raw", 10)
        self._depth_info_pub = self.create_publisher(
            CameraInfo, f"{base}/stereo/camera_info", 10)
        self._imu_pub = self.create_publisher(Imu, f"{base}/imu/data", 10)
        # Exactly one instance per rig owns ground truth (and, with
        # publish_tf, the dynamic odom->base_link): every instance samples
        # the same base trajectory, so duplicates would only add traffic.
        self._publish_base_tf = bool(self.get_parameter("publish_base_tf").value)
        self._gt_pub = (self.create_publisher(Odometry, GROUND_TRUTH_TOPIC, 10)
                        if self._publish_base_tf else None)

        self._tf_broadcaster = None
        if bool(self.get_parameter("publish_tf").value):
            if self._publish_base_tf:
                self._tf_broadcaster = TransformBroadcaster(self)
            self._static_broadcaster = StaticTransformBroadcaster(self)
            stamp = self.get_clock().now().to_msg()
            self._static_broadcaster.sendTransform([
                _tf_msg(BASE_FRAME, self._camera_name,
                        self._mount_xyz, _quat_from_rpy(*self._mount_rpy), stamp),
                _tf_msg(self._camera_name, self._rgb_frame,
                        (0.0, 0.0, 0.0), _quat_from_rpy(*OPTICAL_RPY), stamp),
            ])

        self.create_timer(1.0 / self._fps, self._on_render_timer)
        self.create_timer(1.0 / self._imu_rate_hz, self._on_imu_timer)

        self.get_logger().info(
            f"oak_replayer up: {base}/* ({mount} mount) at "
            f"{self._width}x{self._height}@{self._fps:g} fps, "
            f"imu {self._imu_rate_hz:g} Hz, trajectory={trajectory_name} "
            f"scene={scene_name}, base TF/ground truth owner: "
            f"{self._publish_base_tf}."
        )

    @staticmethod
    def _build_scene(name: str):
        """Build the scene selected by the `scene` parameter."""
        if name == "baseline":
            return scene_baseline()
        if name == "modified":
            return scene_modified()
        raise ValueError(f"unknown scene {name!r} (expected baseline|modified)")

    @staticmethod
    def _build_trajectory(name: str, radius: float, omega: float):
        """Build the trajectory selected by the `trajectory` parameter.

        hold and line both start at the orbit's t = 0 pose (radius, 0, yaw 0)
        so the scene around the world origin stays in the camera's view; line
        moves along the orbit's t = 0 tangent.
        """
        if name == "orbit":
            return trajectory.orbit(center=(0.0, 0.0), radius=radius, omega=omega)
        if name == "hold":
            return trajectory.hold(xyz=(radius, 0.0, 0.0), yaw=0.0)
        if name == "line":
            return trajectory.line(start=(radius, 0.0, 0.0),
                                   velocity=(0.0, radius * omega, 0.0), yaw=0.0)
        raise ValueError(f"unknown trajectory {name!r} (expected orbit|line|hold)")

    def _make_camera_info(self) -> CameraInfo:
        """Build the shared pinhole CameraInfo (aligned depth reuses rgb's K)."""
        info = CameraInfo()
        info.header.frame_id = self._rgb_frame
        info.width = self._width
        info.height = self._height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [float(v) for v in self._k.reshape(-1)]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        fx, fy = float(self._k[0, 0]), float(self._k[1, 1])
        cx, cy = float(self._k[0, 2]), float(self._k[1, 2])
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    # --- Timers ---------------------------------------------------------------

    def _on_render_timer(self) -> None:
        """Render and publish one RGBD frame, camera infos, ground truth, TF."""
        t = self._frame_index / self._fps
        self._frame_index += 1
        stamp = self.get_clock().now().to_msg()

        pos, quat, lin_w, ang_w = self._trajectory.sample(t)
        yaw = 2.0 * math.atan2(quat[2], quat[3])
        t_world_cam = _transform_from_yaw(pos, yaw) @ self._t_base_cam
        rgb, depth = render(self._scene, self._k, t_world_cam,
                            self._width, self._height, max_range=MAX_RANGE_M)

        ok, jpeg = cv2.imencode(
            ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
        )
        if not ok:  # pragma: no cover - imencode only fails on invalid input
            self.get_logger().error("jpeg encode failed; dropping frame")
            return
        rgb_msg = CompressedImage()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self._rgb_frame
        rgb_msg.format = "jpeg"
        rgb_msg.data = jpeg.tobytes()
        self._rgb_pub.publish(rgb_msg)

        depth_msg = self._bridge.cv2_to_imgmsg(depth, encoding="16UC1")
        depth_msg.header.stamp = stamp
        # Aligned depth shares the rgb optical frame, like stereo.i_aligned.
        depth_msg.header.frame_id = self._rgb_frame
        self._depth_pub.publish(depth_msg)

        self._camera_info.header.stamp = stamp
        self._rgb_info_pub.publish(self._camera_info)
        self._depth_info_pub.publish(self._camera_info)

        if self._gt_pub is not None:
            self._publish_ground_truth(stamp, pos, quat, lin_w, ang_w, yaw)
        if self._tf_broadcaster is not None:
            self._tf_broadcaster.sendTransform(
                _tf_msg(ODOM_FRAME, BASE_FRAME, pos, quat, stamp))

    def _publish_ground_truth(self, stamp, pos, quat, lin_w, ang_w, yaw) -> None:
        """Publish the exact odom->base_link pose and body twist."""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])
        # Twist in child_frame_id (body frame), per the Odometry contract.
        c, s = math.cos(yaw), math.sin(yaw)
        odom.twist.twist.linear.x = float(c * lin_w[0] + s * lin_w[1])
        odom.twist.twist.linear.y = float(-s * lin_w[0] + c * lin_w[1])
        odom.twist.twist.angular.z = float(ang_w[2])
        # Covariances stay zero: exact ground truth, not an estimate.
        self._gt_pub.publish(odom)

    def _on_imu_timer(self) -> None:
        """Publish one synthetic IMU sample (body rates + specific force)."""
        t = self._imu_index / self._imu_rate_hz
        self._imu_index += 1
        stamp = self.get_clock().now().to_msg()

        _, quat, _, ang_w = self._trajectory.sample(t)
        yaw = 2.0 * math.atan2(quat[2], quat[3])
        _, _, v_plus, _ = self._trajectory.sample(t + _ACCEL_DT)
        _, _, v_minus, _ = self._trajectory.sample(t - _ACCEL_DT)
        a_world = (v_plus - v_minus) / (2.0 * _ACCEL_DT)

        gyro = np.array([0.0, 0.0, float(ang_w[2]) + self._gyro_bias_z])
        if self._imu_noise_std > 0.0:
            gyro = gyro + self._rng.normal(0.0, self._imu_noise_std, 3)

        # Specific force in the body frame: R_wb^T (a_world - g_world). The
        # trajectories are planar (yaw-only), so z passes straight through
        # and a stationary sample reads +g on z (includes centripetal on x/y).
        c, s = math.cos(yaw), math.sin(yaw)
        accel = (
            c * a_world[0] + s * a_world[1],
            -s * a_world[0] + c * a_world[1],
            a_world[2] + GRAVITY,
        )

        imu = Imu()
        imu.header.stamp = stamp
        # Simplification: the synthetic IMU frame is orientation-aligned with
        # base_link (a real OAK's imu frame comes from factory calibration);
        # consumers of this stream use the z gyro + gravity direction only.
        imu.header.frame_id = self._imu_frame
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = -1.0  # no orientation estimate
        imu.angular_velocity.x = float(gyro[0])
        imu.angular_velocity.y = float(gyro[1])
        imu.angular_velocity.z = float(gyro[2])
        imu.linear_acceleration.x = float(accel[0])
        imu.linear_acceleration.y = float(accel[1])
        imu.linear_acceleration.z = float(accel[2])
        var = self._imu_noise_std ** 2
        for i in (0, 4, 8):
            imu.angular_velocity_covariance[i] = var
        self._imu_pub.publish(imu)


def main(args=None):
    """Entry point registered as the `oak_replayer_node` console script."""
    rclpy.init(args=args)
    node = OakReplayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
