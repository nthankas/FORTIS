"""
Cross-run map diff node.

Rebuilds a voxel grid from the live /fortis/perception/map/cloud,
diffs it against a reference map saved by a previous run, and
publishes the result two ways: CUBE_LIST markers (green added, red
removed) on /fortis/perception/map_diff/markers for the 3D panel, and
a latched machine-readable fortis_msgs/MapDiffSummary on
/fortis/perception/map_diff/summary. With no reference configured the
node idles until ~/load_reference supplies one.
"""

import os

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.msg import MapDiffSummary
from fortis_msgs.srv import LoadMap
from fortis_perception.voxel_grid import VoxelGrid
from fortis_perception.voxel_map_node import MAP_CLOUD_TOPIC

#: Node name registered with ROS.
NODE_NAME = "map_diff"

MARKERS_TOPIC = "/fortis/perception/map_diff/markers"
SUMMARY_TOPIC = "/fortis/perception/map_diff/summary"

#: rgba for marker id 0: voxels occupied live but absent in the reference.
ADDED_COLOR = (0.1, 0.9, 0.2, 0.8)
#: rgba for marker id 1: voxels occupied in the reference but absent live.
REMOVED_COLOR = (0.9, 0.15, 0.1, 0.5)


class MapDiffNode(Node):
    """Diff the live voxel map against a saved reference run."""

    def __init__(self, **kwargs):
        super().__init__(NODE_NAME, **kwargs)
        self.declare_parameter("reference_map", "")
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("min_hits", 3)
        self.declare_parameter("compute_rate_hz", 0.5)
        self._voxel_size = float(self.get_parameter("voxel_size").value)
        self._min_hits = int(self.get_parameter("min_hits").value)
        self._reference = None
        self._reference_path = ""
        self._live_cloud = None

        self._marker_pub = self.create_publisher(MarkerArray, MARKERS_TOPIC, 10)
        self._summary_pub = self.create_publisher(
            MapDiffSummary, SUMMARY_TOPIC, latched_qos_profile())
        self.create_subscription(
            PointCloud2, MAP_CLOUD_TOPIC, self._on_map_cloud,
            latched_qos_profile())
        self.create_service(LoadMap, "~/load_reference", self._on_load_reference)
        rate_hz = max(0.05, float(self.get_parameter("compute_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._compute_diff)

        initial_reference = str(self.get_parameter("reference_map").value).strip()
        if initial_reference:
            success, message = self._load_reference(initial_reference)
            log = self.get_logger().info if success else self.get_logger().error
            log(message)

    def _on_map_cloud(self, msg):
        """Buffer the newest live map cloud; the diff timer consumes it."""
        self._live_cloud = msg

    def _load_reference(self, path):
        """Load a reference grid, enforcing the voxel_size contract."""
        path = os.path.expanduser(path.strip())
        try:
            grid = VoxelGrid.load(path)
        except Exception as exc:  # a bad file must never take the node down
            return False, f"load failed: {exc}"
        if abs(grid.voxel_size - self._voxel_size) > 1e-9:
            return False, (
                f"voxel_size mismatch: reference {grid.voxel_size} "
                f"vs node {self._voxel_size}")
        self._reference = grid
        self._reference_path = path
        return True, f"reference {path} loaded ({len(grid)} voxels)"

    def _on_load_reference(self, request, response):
        """Serve ~/load_reference: swap in a new reference map."""
        response.success, response.message = self._load_reference(request.path)
        return response

    def _compute_diff(self):
        """Diff the live map against the reference and publish both outputs."""
        cloud = self._live_cloud
        if cloud is None or self._reference is None:
            return
        if cloud.width * cloud.height == 0:
            centers = np.empty((0, 3), dtype=np.float64)
        else:
            centers = point_cloud2.read_points_numpy(
                cloud, field_names=("x", "y", "z")).astype(np.float64)
        live = VoxelGrid(self._voxel_size)
        colors = np.zeros((centers.shape[0], 3), dtype=np.uint8)
        # The map cloud only carries voxels already past the map's own
        # min_hits threshold; re-integrate min_hits times so the diff
        # threshold does not filter them a second time.
        for _ in range(max(1, self._min_hits)):
            live.integrate(centers, colors)
        added, removed = live.diff(self._reference, min_hits=self._min_hits)
        stamp = self.get_clock().now().to_msg()
        frame = cloud.header.frame_id or "odom"
        self._marker_pub.publish(self._make_markers(added, removed, stamp, frame))
        self._summary_pub.publish(self._make_summary(added, removed, stamp))

    def _make_markers(self, added, removed, stamp, frame):
        """Build the two CUBE_LIST markers (id 0 added, id 1 removed)."""
        msg = MarkerArray()
        pairs = ((0, added, ADDED_COLOR), (1, removed, REMOVED_COLOR))
        for marker_id, indices, color in pairs:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame
            marker.ns = "map_diff"
            marker.id = marker_id
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = self._voxel_size
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            centers = (indices.astype(np.float64) + 0.5) * self._voxel_size
            marker.points = [
                Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))
                for c in centers]
            msg.markers.append(marker)
        return msg

    def _make_summary(self, added, removed, stamp):
        """Build the latched MapDiffSummary roll-up."""
        voxel_volume = self._voxel_size ** 3
        summary = MapDiffSummary()
        summary.stamp = stamp
        summary.reference_map = self._reference_path
        summary.voxel_size = float(self._voxel_size)
        summary.added_voxels = int(added.shape[0])
        summary.removed_voxels = int(removed.shape[0])
        summary.added_volume_m3 = float(added.shape[0] * voxel_volume)
        summary.removed_volume_m3 = float(removed.shape[0] * voxel_volume)
        return summary


def main(args=None):
    """Entry point registered as the `map_diff_node` console script."""
    rclpy.init(args=args)
    node = MapDiffNode()
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
