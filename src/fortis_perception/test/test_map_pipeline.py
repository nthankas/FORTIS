"""
End-to-end tests for the voxel map pipeline.

Feeds synthetic fused clouds (base_link) into a real VoxelMapNode under
an identity odom -> base_link static TF, saves the map through the
~/save_map service, then starts a MapDiffNode with that file as the
reference and asserts the diff outputs when an extra cluster appears.
House style follows fortis_drive's drive_node tests: real in-process
nodes, DDS round trips, bounded polling. All point sets sample voxel
interiors, so expected voxel counts are exact and seed-free.
"""

from __future__ import annotations

import os
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.msg import MapDiffSummary
from fortis_msgs.srv import SaveMap
from fortis_perception.depth_to_cloud_node import make_xyzrgb_cloud, pack_rgb
from fortis_perception.map_diff_node import MapDiffNode
from fortis_perception.voxel_grid import VoxelGrid
from fortis_perception.voxel_map_node import VoxelMapNode

VOXEL_SIZE = 0.05
SPIN_ONCE_TIMEOUT_S = 0.02
TIMEOUT_S = 15.0


def _plane_points():
    """Return a deterministic 0.4 x 0.4 m patch of points at z in voxel 2."""
    a = np.arange(0.0025, 0.4, 0.02)
    xx, yy = np.meshgrid(a, a, indexing="ij")
    return np.column_stack((xx.ravel(), yy.ravel(), np.full(xx.size, 0.1025)))


def _cluster_points():
    """Return a compact extra cluster well away from the plane."""
    a = np.arange(1.0025, 1.2, 0.02)
    xx, yy, zz = np.meshgrid(a, a, a, indexing="ij")
    return np.column_stack((xx.ravel(), yy.ravel(), zz.ravel()))


def _voxel_count(points):
    """Count the distinct voxels a point set touches (the assertion target)."""
    idx = np.floor(np.asarray(points) / VOXEL_SIZE).astype(np.int32)
    return np.unique(idx, axis=0).shape[0]


@pytest.fixture
def rclpy_session():
    """Function-scoped rclpy.init / shutdown (house style, see fortis_drive)."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness(rclpy_session, tmp_path):
    """Per-test harness: a VoxelMapNode plus an input/output helper."""
    h = _Harness(map_dir=str(tmp_path / "maps"))
    h.spin(0.3)  # let DDS discovery finish before the first publish
    try:
        yield h
    finally:
        h.cleanup()


class _Harness:
    """VoxelMapNode (+ optional MapDiffNode) plus an input/output helper."""

    def __init__(self, map_dir):
        self.map_dir = map_dir
        self.map_node = VoxelMapNode(parameter_overrides=[
            Parameter("voxel_size", Parameter.Type.DOUBLE, VOXEL_SIZE),
            # min_hits 1: the test feeds each cloud a bounded number of
            # times, so occupancy must not depend on repeat counts.
            Parameter("min_hits", Parameter.Type.INTEGER, 1),
            Parameter("integrate_rate_hz", Parameter.Type.DOUBLE, 20.0),
            Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 10.0),
            Parameter("map_dir", Parameter.Type.STRING, map_dir),
        ])
        self.diff_node = None
        self.helper = rclpy.create_node("map_pipeline_test_helper")
        self._broadcast_identity_tf()
        self.fused_pub = self.helper.create_publisher(
            PointCloud2, "/fortis/perception/points_fused", 10)
        self.map_msgs: list[PointCloud2] = []
        self.helper.create_subscription(
            PointCloud2, "/fortis/perception/map/cloud",
            self.map_msgs.append, latched_qos_profile())
        self.summary_msgs: list[MapDiffSummary] = []
        self.marker_msgs: list[MarkerArray] = []
        self.save_client = self.helper.create_client(
            SaveMap, "/voxel_map/save_map")

    def _broadcast_identity_tf(self):
        """Latch an identity odom -> base_link static transform."""
        self._static_broadcaster = StaticTransformBroadcaster(self.helper)
        tf = TransformStamped()
        tf.header.stamp = self.helper.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.rotation.w = 1.0
        self._static_broadcaster.sendTransform(tf)

    def start_diff_node(self, reference_path):
        """Start a MapDiffNode against a saved reference map."""
        self.diff_node = MapDiffNode(parameter_overrides=[
            Parameter("reference_map", Parameter.Type.STRING, reference_path),
            Parameter("voxel_size", Parameter.Type.DOUBLE, VOXEL_SIZE),
            Parameter("min_hits", Parameter.Type.INTEGER, 1),
            Parameter("compute_rate_hz", Parameter.Type.DOUBLE, 5.0),
        ])
        self.helper.create_subscription(
            MapDiffSummary, "/fortis/perception/map_diff/summary",
            self.summary_msgs.append, latched_qos_profile())
        self.helper.create_subscription(
            MarkerArray, "/fortis/perception/map_diff/markers",
            self.marker_msgs.append, 10)

    def spin(self, duration_s=0.3):
        """Drain the event loop on every live node for the given window."""
        nodes = [self.map_node, self.helper]
        if self.diff_node is not None:
            nodes.append(self.diff_node)
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            for node in nodes:
                rclpy.spin_once(node, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def publish_fused(self, xyz):
        """Publish one synthetic fused XYZRGB cloud in base_link."""
        header = Header()
        header.stamp = self.helper.get_clock().now().to_msg()
        header.frame_id = "base_link"
        colors = np.full((xyz.shape[0], 3), 120, dtype=np.uint8)
        self.fused_pub.publish(
            make_xyzrgb_cloud(header, xyz.astype(np.float32), pack_rgb(colors)))

    def wait_for_map_points(self, min_points, feed=None):
        """Poll (re-feeding a cloud if given) until the map has enough points."""
        end = time.monotonic() + TIMEOUT_S
        while time.monotonic() < end:
            if self.map_msgs and self.map_msgs[-1].width >= min_points:
                return self.map_msgs[-1]
            if feed is not None:
                self.publish_fused(feed)
            self.spin(0.2)
        raise AssertionError(f"map never reached {min_points} points")

    def call_save(self, path=""):
        """Invoke /voxel_map/save_map and return the response."""
        end = time.monotonic() + TIMEOUT_S
        while not self.save_client.service_is_ready() and time.monotonic() < end:
            self.spin(0.1)
        assert self.save_client.service_is_ready(), "save_map never appeared"
        request = SaveMap.Request()
        request.path = path
        future = self.save_client.call_async(request)
        end = time.monotonic() + TIMEOUT_S
        while not future.done() and time.monotonic() < end:
            self.spin(0.1)
        assert future.done(), "save_map call timed out"
        return future.result()

    def cleanup(self):
        """Tear down every node. Safe to call once after the test finishes."""
        self.helper.destroy_node()
        if self.diff_node is not None:
            self.diff_node.destroy_node()
        self.map_node.destroy_node()
        time.sleep(0.05)


def test_map_integrates_saves_and_diff_reports_added_cluster(harness, tmp_path):
    """Full pipeline: integrate -> save -> reference diff on a new cluster."""
    plane = _plane_points()
    expected_plane_voxels = _voxel_count(plane)

    # Feed two synthetic fused clouds, then keep feeding until the map
    # cloud reflects the full plane footprint.
    harness.publish_fused(plane)
    harness.spin(0.2)
    harness.publish_fused(plane)
    harness.wait_for_map_points(expected_plane_voxels, feed=plane)

    reference_path = str(tmp_path / "reference.npz")
    response = harness.call_save(reference_path)
    assert response.success, response.message
    assert reference_path in response.message
    assert os.path.exists(reference_path)
    loaded = VoxelGrid.load(reference_path)
    assert loaded.occupied(min_hits=1).shape[0] == expected_plane_voxels

    harness.start_diff_node(reference_path)
    scene = np.vstack((plane, _cluster_points()))
    expected_total = _voxel_count(scene)
    expected_added = expected_total - expected_plane_voxels
    harness.wait_for_map_points(expected_total, feed=scene)

    end = time.monotonic() + TIMEOUT_S
    while time.monotonic() < end:
        if harness.summary_msgs and harness.summary_msgs[-1].added_voxels > 0:
            break
        harness.spin(0.2)
    assert harness.summary_msgs, "no MapDiffSummary received"

    # The scene is static now: every further diff tick publishes the
    # same counts, so one more window makes summary + markers coherent.
    harness.spin(0.6)
    summary = harness.summary_msgs[-1]
    assert summary.added_voxels == expected_added
    assert summary.removed_voxels == 0
    assert summary.added_volume_m3 == pytest.approx(
        expected_added * VOXEL_SIZE ** 3, rel=1e-4)
    assert summary.removed_volume_m3 == 0.0
    assert summary.reference_map == reference_path
    assert summary.voxel_size == pytest.approx(VOXEL_SIZE)

    assert harness.marker_msgs, "no MarkerArray received"
    markers = {m.id: m for m in harness.marker_msgs[-1].markers}
    assert set(markers) == {0, 1}
    assert len(markers[0].points) == summary.added_voxels
    assert len(markers[1].points) == summary.removed_voxels
    assert markers[0].color.g > markers[0].color.r, "id 0 (added) is green"
    assert markers[1].color.r > markers[1].color.g, "id 1 (removed) is red"
    assert markers[0].scale.x == pytest.approx(VOXEL_SIZE)


def test_save_map_empty_path_uses_map_dir(harness):
    """Empty SaveMap.path lands a timestamped .npz under map_dir."""
    plane = _plane_points()
    harness.publish_fused(plane)
    harness.wait_for_map_points(1, feed=plane)
    response = harness.call_save("")
    assert response.success, response.message
    files = os.listdir(harness.map_dir)
    assert len(files) == 1
    assert files[0].startswith("map_") and files[0].endswith(".npz")
    saved = os.path.join(harness.map_dir, files[0])
    assert saved in response.message
    assert len(VoxelGrid.load(saved)) > 0
