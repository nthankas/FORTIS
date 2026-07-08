"""
Multi-camera cloud fusion node.

Buffers the latest cloud from every per-camera
/fortis/perception/<cam>/points topic, TF-transforms each into a common
target frame on a fixed-rate timer, concatenates them, voxel-downsamples
the result, and publishes /fortis/perception/points_fused
(sensor_msgs/PointCloud2, XYZRGB). A camera whose transform is missing
is skipped for that cycle instead of stalling the others.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from fortis_perception.cloud_utils import make_xyzrgb_cloud, transform_to_matrix

#: Node name registered with ROS.
NODE_NAME = "cloud_fusion"

#: Fused output topic (see the fortis_perception README topic registry).
FUSED_CLOUD_TOPIC = "/fortis/perception/points_fused"

#: Default per-camera inputs: the four chassis OAK-D Lites.
DEFAULT_INPUT_TOPICS = [
    "/fortis/perception/oak_chassis_front/points",
    "/fortis/perception/oak_chassis_rear/points",
    "/fortis/perception/oak_chassis_left/points",
    "/fortis/perception/oak_chassis_right/points",
]


def voxel_downsample(xyz, rgb, voxel_size):
    """Keep the first point in each occupied voxel; return filtered (xyz, rgb)."""
    if voxel_size <= 0.0 or xyz.shape[0] == 0:
        return xyz, rgb
    idx = np.ascontiguousarray(np.floor(xyz / voxel_size).astype(np.int64))
    keys = idx.view(np.dtype((np.void, idx.dtype.itemsize * 3))).ravel()
    _, keep = np.unique(keys, return_index=True)
    keep = np.sort(keep)
    return xyz[keep], rgb[keep]


class CloudFusionNode(Node):
    """Fuse the per-camera clouds into one downsampled target-frame cloud."""

    def __init__(self, **kwargs):
        super().__init__(NODE_NAME, **kwargs)
        self.declare_parameter("input_topics", DEFAULT_INPUT_TOPICS)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("publish_rate_hz", 5.0)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._voxel_size = float(self.get_parameter("voxel_size").value)

        self._clouds = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._fused_pub = self.create_publisher(PointCloud2, FUSED_CLOUD_TOPIC, 5)
        for topic in list(self.get_parameter("input_topics").value):
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, name=topic: self._buffer_cloud(name, msg),
                qos_profile_sensor_data)
        rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._on_timer)

    def _buffer_cloud(self, topic, msg):
        """Buffer the newest cloud from one input topic."""
        self._clouds[topic] = msg

    def _on_timer(self):
        """Transform, merge, downsample, and publish the buffered clouds."""
        xyz_parts = []
        rgb_parts = []
        for cloud in list(self._clouds.values()):
            part = self._to_target_frame(cloud)
            if part is not None:
                xyz_parts.append(part[0])
                rgb_parts.append(part[1])
        if not xyz_parts:
            return
        xyz, rgb = voxel_downsample(
            np.vstack(xyz_parts), np.concatenate(rgb_parts), self._voxel_size)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._target_frame
        self._fused_pub.publish(make_xyzrgb_cloud(header, xyz, rgb))

    def _to_target_frame(self, cloud):
        """Return one cloud's (xyz, packed rgb) in the target frame, or None."""
        if cloud.width * cloud.height == 0:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, cloud.header.frame_id, Time())
        except TransformException as exc:
            self.get_logger().warning(
                f"skipping cloud in {cloud.header.frame_id!r}: {exc}",
                throttle_duration_sec=5.0)
            return None
        points = point_cloud2.read_points_numpy(
            cloud, field_names=("x", "y", "z", "rgb"))
        matrix = transform_to_matrix(transform.transform)
        xyz = points[:, :3].astype(np.float64) @ matrix[:3, :3].T + matrix[:3, 3]
        # rgb stays float32 end to end: the packed bit patterns are
        # denormals that must never round-trip through arithmetic.
        return xyz, np.ascontiguousarray(points[:, 3])


def main(args=None):
    """Entry point registered as the `cloud_fusion_node` console script."""
    rclpy.init(args=args)
    node = CloudFusionNode()
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
