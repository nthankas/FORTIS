"""
RGBD visual odometry node: front OAK frames -> /fortis/vo (nav_msgs/Odometry).

Feeds the pure ``rgbd_vo`` core with the front chassis camera's
device-encoded JPEG RGB and aligned 16UC1 depth streams (the
oak_chassis_cameras.yaml contract: 640x400 @ 15 fps, shared K) and
publishes the integrated pose plus the frame-to-frame body twist.

Frame conventions
-----------------
- Camera OPTICAL frame: X right, Y down, Z forward.
- base_link: X forward, Y left, Z up (REP-103). The FORTIS chassis
  "front" -- where this camera points -- is base_link -X. The bolted
  front-mount extrinsic lives in fortis_perception.geometry.
- The VO world is the base_link pose at the instant tracking starts,
  labelled "odom" in the header. The label is nominal: the EKF variant
  consuming this topic (ekf_vio.yaml) fuses the TWIST only, so the
  absolute pose is advisory and never fused.

Depth pairing
-------------
Latest-depth caching instead of an ApproximateTimeSynchronizer: both
streams leave one device pipeline at a matched 15 fps, but the RGB path
adds variable on-device MJPEG encoding latency. A strict synchroniser
drops whole pairs whenever that latency wobbles; caching the newest
depth degrades gracefully to at most one frame of skew, bounded by an
explicit staleness check.

Tracking loss
-------------
On loss the node publishes NO Odometry (the EKF coasts on wheels + IMU),
counts the dropped frames, and keeps publishing the inlier count on
/fortis/vo/inliers (0.0 while lost) as the cheap health signal.
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Float32

from fortis_perception.geometry import T_BASE_CAM, T_CAM_BASE
from fortis_perception.rgbd_vo import RgbdVo, rotation_matrix_to_quaternion

#: Node name; matches the `rgbd_vo_node` console-script registration.
NODE_NAME = "rgbd_vo"
VO_TOPIC = "/fortis/vo"
INLIERS_TOPIC = "/fortis/vo/inliers"

#: Reject an RGB frame whose cached depth is further away in stamp than
#: this. Three frame periods at 15 fps: one period of legitimate encoder
#: skew plus margin, while still catching a stalled depth stream.
MAX_RGBD_SKEW_S = 0.2

#: Per-axis measurement variance at ONE inlier (0.01 * 36 translation,
#: 0.02 * 36 rotation); divided by the actual inlier count per message so
#: the EKF's trust in VO rises and falls with feature support.
TRANS_VAR_AT_ONE_INLIER = 0.36
ROT_VAR_AT_ONE_INLIER = 0.72


def _stamp_to_s(stamp) -> float:
    """Convert a builtin_interfaces Time to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class RgbdVoNode(Node):
    """Run the RgbdVo core on the front chassis camera and publish /fortis/vo."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        camera_name = (
            self.declare_parameter("camera_name", "oak_chassis_front")
            .get_parameter_value()
            .string_value
        )
        n_features = (
            self.declare_parameter("n_features", 800)
            .get_parameter_value()
            .integer_value
        )
        min_inliers = (
            self.declare_parameter("min_inliers", 12)
            .get_parameter_value()
            .integer_value
        )

        self._vo = RgbdVo(n_features=n_features, min_inliers=min_inliers)
        self._cam_k: np.ndarray | None = None
        self._depth_msg: Image | None = None
        #: Frames dropped since startup because tracking was lost -- the
        #: diagnostics-friendly counter (system_health_node relays it).
        self.lost_frames = 0
        self._prev_t_odom_base: np.ndarray | None = None
        self._prev_stamp_s: float | None = None

        self._odom_pub = self.create_publisher(Odometry, VO_TOPIC, 10)
        self._inliers_pub = self.create_publisher(Float32, INLIERS_TOPIC, 10)

        self.create_subscription(
            CompressedImage,
            f"/{camera_name}/rgb/image_raw/compressed",
            self._on_rgb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            f"/{camera_name}/stereo/image_raw",
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            f"/{camera_name}/rgb/camera_info",
            self._on_camera_info,
            qos_profile_sensor_data,
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Cache the pinhole K (rgb and aligned depth share it)."""
        self._cam_k = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_depth(self, msg: Image) -> None:
        """Cache the newest aligned depth frame; paired lazily in _on_rgb."""
        self._depth_msg = msg

    def _on_rgb(self, msg: CompressedImage) -> None:
        """Track one RGB frame against the cached depth and publish odometry."""
        if self._cam_k is None or self._depth_msg is None:
            self.get_logger().info(
                "waiting for camera_info and depth before starting VO",
                throttle_duration_sec=5.0,
            )
            return
        stamp_s = _stamp_to_s(msg.header.stamp)
        depth_msg = self._depth_msg
        if abs(stamp_s - _stamp_to_s(depth_msg.header.stamp)) > MAX_RGBD_SKEW_S:
            self.get_logger().warning(
                f"rgb/depth stamp skew exceeds {MAX_RGBD_SKEW_S}s; dropping frame",
                throttle_duration_sec=2.0,
            )
            return
        gray = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE
        )
        depth = self._depth_to_array(depth_msg)
        if gray is None or depth is None:
            return

        result = self._vo.process(gray, depth, self._cam_k)
        self._inliers_pub.publish(Float32(data=float(result.n_inliers)))
        if not result.tracking:
            self.lost_frames += 1
            # Publish nothing: robot_localization coasts on wheels + IMU.
            self.get_logger().warning(
                f"VO tracking lost ({result.n_inliers} inliers; "
                f"{self.lost_frames} frames dropped so far)",
                throttle_duration_sec=2.0,
            )
            # Break the twist chain so the first re-tracked frame does not
            # divide a near-zero pose delta by the whole outage duration.
            self._prev_t_odom_base = None
            self._prev_stamp_s = None
            return

        t_odom_base = T_BASE_CAM @ self._vo.pose @ T_CAM_BASE
        self._publish_odom(msg.header.stamp, stamp_s, t_odom_base, result.n_inliers)
        self._prev_t_odom_base = t_odom_base
        self._prev_stamp_s = stamp_s

    def _depth_to_array(self, msg: Image) -> np.ndarray | None:
        """Reinterpret a 16UC1 depth Image as an HxW uint16 array (no cv_bridge)."""
        if msg.encoding != "16UC1" or msg.is_bigendian:
            self.get_logger().error(
                f"unsupported depth encoding {msg.encoding!r} "
                f"(bigendian={msg.is_bigendian}); expected little-endian 16UC1",
                throttle_duration_sec=5.0,
            )
            return None
        data = np.frombuffer(msg.data, dtype=np.uint16)
        if data.size != msg.height * msg.width:
            self.get_logger().warning(
                "depth image has row padding; dropping frame",
                throttle_duration_sec=5.0,
            )
            return None
        return data.reshape(msg.height, msg.width)

    def _publish_odom(self, stamp, stamp_s, t_odom_base, n_inliers) -> None:
        """Publish one Odometry: integrated pose + frame-to-frame body twist."""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(t_odom_base[0, 3])
        odom.pose.pose.position.y = float(t_odom_base[1, 3])
        odom.pose.pose.position.z = float(t_odom_base[2, 3])
        qx, qy, qz, qw = rotation_matrix_to_quaternion(t_odom_base[:3, :3])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        if self._prev_t_odom_base is not None and self._prev_stamp_s is not None:
            dt = stamp_s - self._prev_stamp_s
            if dt > 1e-6:
                # Motion since the previous frame expressed in the PREVIOUS
                # base pose: the body-twist approximation the EKF reads in
                # child_frame_id coordinates.
                rel = np.linalg.inv(self._prev_t_odom_base) @ t_odom_base
                linear = rel[:3, 3] / dt
                angular = cv2.Rodrigues(rel[:3, :3])[0].ravel() / dt
                odom.twist.twist.linear.x = float(linear[0])
                odom.twist.twist.linear.y = float(linear[1])
                odom.twist.twist.linear.z = float(linear[2])
                odom.twist.twist.angular.x = float(angular[0])
                odom.twist.twist.angular.y = float(angular[1])
                odom.twist.twist.angular.z = float(angular[2])

        trans_var = TRANS_VAR_AT_ONE_INLIER / max(n_inliers, 1)
        rot_var = ROT_VAR_AT_ONE_INLIER / max(n_inliers, 1)
        cov = np.zeros(36)
        cov[[0, 7, 14]] = trans_var
        cov[[21, 28, 35]] = rot_var
        odom.pose.covariance = cov.tolist()
        odom.twist.covariance = cov.tolist()
        self._odom_pub.publish(odom)


def main(args=None):
    """Run the rgbd_vo node (console script entry point)."""
    rclpy.init(args=args)
    node = RgbdVoNode()
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
