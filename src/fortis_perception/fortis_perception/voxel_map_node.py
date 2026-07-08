"""
Voxel mapping node.

Accumulates /fortis/perception/points_fused into a persistent voxel
grid in the odom frame (grid math lives in voxel_grid.py), publishes
the occupied-voxel centers as a latched /fortis/perception/map/cloud
(sensor_msgs/PointCloud2, XYZRGB), and offers save / load / clear
services so one run's map can become a later run's diff reference.
"""

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.srv import LoadMap, SaveMap
from fortis_perception.cloud_fusion_node import FUSED_CLOUD_TOPIC, transform_to_matrix
from fortis_perception.depth_to_cloud_node import make_xyzrgb_cloud, pack_rgb, unpack_rgb
from fortis_perception.voxel_grid import VoxelGrid

#: Node name registered with ROS.
NODE_NAME = "voxel_map"

#: Latched map output topic (see the fortis_perception README topic registry).
MAP_CLOUD_TOPIC = "/fortis/perception/map/cloud"


class VoxelMapNode(Node):
    """Integrate fused clouds into a voxel map and serve it to the stack."""

    def __init__(self, **kwargs):
        super().__init__(NODE_NAME, **kwargs)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("min_hits", 3)
        self.declare_parameter("integrate_rate_hz", 5.0)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("map_dir", "~/fortis_maps")
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._min_hits = int(self.get_parameter("min_hits").value)
        self._map_dir = os.path.expanduser(str(self.get_parameter("map_dir").value))
        self._grid = VoxelGrid(float(self.get_parameter("voxel_size").value))
        self._pending = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._map_pub = self.create_publisher(
            PointCloud2, MAP_CLOUD_TOPIC, latched_qos_profile())
        self.create_subscription(
            PointCloud2, FUSED_CLOUD_TOPIC, self._on_fused_cloud, 10)

        integrate_rate_hz = max(
            0.1, float(self.get_parameter("integrate_rate_hz").value))
        publish_rate_hz = max(
            0.1, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / integrate_rate_hz, self._integrate_pending)
        self.create_timer(1.0 / publish_rate_hz, self._publish_map)

        self.create_service(SaveMap, "~/save_map", self._on_save_map)
        self.create_service(LoadMap, "~/load_map", self._on_load_map)
        self.create_service(Trigger, "~/clear", self._on_clear)

    def _on_fused_cloud(self, msg):
        """Buffer the newest fused cloud; the integrate timer consumes it."""
        self._pending = msg

    def _integrate_pending(self):
        """Integrate the buffered cloud into the grid in the target frame."""
        cloud = self._pending
        if cloud is None:
            return
        if cloud.width * cloud.height == 0:
            self._pending = None
            return
        transform = self._lookup_transform(cloud)
        if transform is None:
            return  # keep the cloud pending until its transform shows up
        self._pending = None
        points = point_cloud2.read_points_numpy(
            cloud, field_names=("x", "y", "z", "rgb"))
        matrix = transform_to_matrix(transform.transform)
        xyz = points[:, :3].astype(np.float64) @ matrix[:3, :3].T + matrix[:3, 3]
        self._grid.integrate(xyz, unpack_rgb(points[:, 3]))

    def _lookup_transform(self, cloud):
        """Look up target <- cloud at the cloud stamp, falling back to latest."""
        for stamp in (Time.from_msg(cloud.header.stamp), Time()):
            try:
                return self._tf_buffer.lookup_transform(
                    self._target_frame, cloud.header.frame_id, stamp)
            except TransformException:
                continue
        self.get_logger().warning(
            f"no transform {self._target_frame} <- {cloud.header.frame_id}",
            throttle_duration_sec=5.0)
        return None

    def _publish_map(self):
        """Publish the occupied-voxel centers as the latched map cloud."""
        centers, colors = self._grid.to_points(self._min_hits)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._target_frame
        self._map_pub.publish(make_xyzrgb_cloud(header, centers, pack_rgb(colors)))

    def _on_save_map(self, request, response):
        """Persist the grid; an empty path means a timestamped file in map_dir."""
        path = request.path.strip()
        if not path:
            path = os.path.join(self._map_dir, f"map_{int(time.time())}.npz")
        path = os.path.expanduser(path)
        try:
            directory = os.path.dirname(path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            resolved = self._grid.save(path)
        except Exception as exc:  # a bad path must never take the node down
            response.success = False
            response.message = f"save failed: {exc}"
            return response
        response.success = True
        response.message = f"saved {len(self._grid)} voxels to {resolved}"
        return response

    def _on_load_map(self, request, response):
        """Replace the current grid with one loaded from disk."""
        path = os.path.expanduser(request.path.strip())
        try:
            grid = VoxelGrid.load(path)
        except Exception as exc:  # a bad file must never take the node down
            response.success = False
            response.message = f"load failed: {exc}"
            return response
        self._grid = grid
        self._pending = None
        response.success = True
        response.message = (
            f"loaded {len(grid)} voxels from {path} "
            f"(voxel_size {grid.voxel_size})")
        return response

    def _on_clear(self, request, response):
        """Reset the map to an empty grid at the current voxel size."""
        self._grid = VoxelGrid(self._grid.voxel_size)
        self._pending = None
        response.success = True
        response.message = "map cleared"
        return response


def main(args=None):
    """Entry point registered as the `voxel_map_node` console script."""
    rclpy.init(args=args)
    node = VoxelMapNode()
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
