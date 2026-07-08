"""
Depth-to-point-cloud node.

Back-projects one camera's aligned 16UC1 depth image through the
pinhole model, colours each point from the device-encoded JPEG RGB
stream, and publishes /fortis/perception/<cam>/points
(sensor_msgs/PointCloud2, XYZRGB) in the incoming depth frame. One
instance runs per camera. CameraInfo is cached from a plain
subscription; only rgb + depth go through the approximate-time sync.
The XYZRGB layout and rgb packing live in fortis_perception.cloud_utils.
"""

import threading

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2

from fortis_perception.cloud_utils import make_xyzrgb_cloud, pack_rgb

#: Node name registered with ROS.
NODE_NAME = "depth_to_cloud"


class DepthToCloudNode(Node):
    """Convert one OAK camera's aligned depth + JPEG RGB into a point cloud."""

    def __init__(self, **kwargs):
        super().__init__(NODE_NAME, **kwargs)
        self.declare_parameter("camera_name", "oak_chassis_front")
        self.declare_parameter("stride", 4)
        self.declare_parameter("min_range_m", 0.15)
        self.declare_parameter("max_range_m", 6.0)
        camera = self.get_parameter("camera_name").value
        self._stride = max(1, int(self.get_parameter("stride").value))
        self._min_range = float(self.get_parameter("min_range_m").value)
        self._max_range = float(self.get_parameter("max_range_m").value)

        self._camera_info = None
        self._grids = None  # (cache key, x-factor grid, y-factor grid)
        self._busy = threading.Lock()

        self._cloud_pub = self.create_publisher(
            PointCloud2, f"/fortis/perception/{camera}/points", 5)
        self.create_subscription(
            CameraInfo, f"/{camera}/stereo/camera_info",
            self._on_camera_info, qos_profile_sensor_data)
        self._sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, CompressedImage, f"/{camera}/rgb/image_raw/compressed",
                           qos_profile=qos_profile_sensor_data),
                Subscriber(self, Image, f"/{camera}/stereo/image_raw",
                           qos_profile=qos_profile_sensor_data),
            ],
            5,     # queue size
            0.1,   # slop (s)
        )
        self._sync.registerCallback(self._on_frames)

    def _on_camera_info(self, msg):
        """Cache the latest CameraInfo; intrinsics only change on reconfig."""
        self._camera_info = msg

    def _on_frames(self, rgb_msg, depth_msg):
        """Handle one synchronized rgb+depth pair, dropping frames while busy."""
        if not self._busy.acquire(blocking=False):
            return  # previous pair still reprojecting: drop, don't queue
        try:
            self._process(rgb_msg, depth_msg)
        finally:
            self._busy.release()

    def _process(self, rgb_msg, depth_msg):
        """Reproject a depth frame into a coloured cloud and publish it."""
        info = self._camera_info
        if info is None:
            self.get_logger().warning(
                "dropping frames: no camera_info received yet",
                throttle_duration_sec=5.0)
            return
        if info.k[0] <= 0.0 or info.k[4] <= 0.0:
            self.get_logger().warning(
                "dropping frames: camera_info has non-positive focal length",
                throttle_duration_sec=5.0)
            return
        depth = self._decode_depth(depth_msg)
        if depth is None:
            return
        stride = self._stride
        z = depth[::stride, ::stride].astype(np.float32) * 1e-3
        x_factor, y_factor = self._reprojection_grids(info, depth.shape)
        mask = (z >= self._min_range) & (z <= self._max_range)
        z_masked = z[mask]
        xyz = np.column_stack(
            (x_factor[mask] * z_masked, y_factor[mask] * z_masked, z_masked))
        color = self._decode_rgb(rgb_msg, depth.shape)
        if color is None:
            colors = np.full((z_masked.shape[0], 3), 200, dtype=np.uint8)
        else:
            colors = color[::stride, ::stride][mask]
        self._cloud_pub.publish(
            make_xyzrgb_cloud(depth_msg.header, xyz, pack_rgb(colors)))

    def _reprojection_grids(self, info, shape):
        """Return cached strided ((u-cx)/fx, (v-cy)/fy) grids for this K/shape."""
        key = (shape, self._stride, tuple(info.k))
        if self._grids is None or self._grids[0] != key:
            fx, fy = info.k[0], info.k[4]
            cx, cy = info.k[2], info.k[5]
            us = np.arange(0, shape[1], self._stride, dtype=np.float32)
            vs = np.arange(0, shape[0], self._stride, dtype=np.float32)
            uu, vv = np.meshgrid(us, vs)
            self._grids = (key, (uu - cx) / fx, (vv - cy) / fy)
        return self._grids[1], self._grids[2]

    def _decode_depth(self, msg):
        """Return the 16UC1 depth image as an HxW uint16 array, or None."""
        if msg.encoding != "16UC1":
            self.get_logger().warning(
                f"unsupported depth encoding {msg.encoding!r}",
                throttle_duration_sec=10.0)
            return None
        row_words = msg.step // 2
        data = np.frombuffer(msg.data, dtype=np.uint16)
        if row_words < msg.width or data.size < msg.height * row_words:
            self.get_logger().warning(
                "depth image data does not match its declared geometry",
                throttle_duration_sec=10.0)
            return None
        return data[:msg.height * row_words].reshape(
            msg.height, row_words)[:, :msg.width]

    def _decode_rgb(self, msg, depth_shape):
        """Decode the JPEG rgb frame to an RGB array matching the depth shape."""
        bgr = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warning(
                "failed to decode compressed rgb frame",
                throttle_duration_sec=10.0)
            return None
        if bgr.shape[:2] != depth_shape:
            bgr = cv2.resize(
                bgr, (depth_shape[1], depth_shape[0]),
                interpolation=cv2.INTER_NEAREST)
        return bgr[:, :, ::-1]


def main(args=None):
    """Entry point registered as the `depth_to_cloud_node` console script."""
    rclpy.init(args=args)
    node = DepthToCloudNode()
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
