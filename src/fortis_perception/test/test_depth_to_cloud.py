"""
Tests for fortis_perception.depth_to_cloud_node.

Stands up a real DepthToCloudNode plus a helper node that publishes a
synthetic camera triplet (CameraInfo, JPEG-compressed RGB, constant
16UC1 depth) and subscribes to the output cloud, mirroring the DDS
round-trip style of fortis_drive's drive_node tests. Geometry is
asserted against the pinhole model with a known K.
"""

from __future__ import annotations

import time

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from sensor_msgs_py import point_cloud2

from fortis_perception.depth_to_cloud_node import DepthToCloudNode, unpack_rgb

CAMERA_NAME = "testcam"
WIDTH, HEIGHT = 64, 48
STRIDE = 4
FX = FY = 50.0
CX, CY = 32.0, 24.0
DEPTH_MM = 1000
DEPTH_FRAME_ID = "testcam_rgb_camera_optical_frame"

SPIN_ONCE_TIMEOUT_S = 0.02
CLOUD_TIMEOUT_S = 10.0


@pytest.fixture
def rclpy_session():
    """Function-scoped rclpy.init / shutdown (house style, see fortis_drive)."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness(rclpy_session):
    """Per-test harness: a DepthToCloudNode plus an input/output helper."""
    h = _Harness()
    h.spin(0.3)  # let DDS discovery finish before the first publish
    try:
        yield h
    finally:
        h.cleanup()


class _Harness:
    """A DepthToCloudNode under test plus a helper node for inputs/outputs."""

    def __init__(self):
        self.node = DepthToCloudNode(parameter_overrides=[
            Parameter("camera_name", Parameter.Type.STRING, CAMERA_NAME),
            Parameter("stride", Parameter.Type.INTEGER, STRIDE),
        ])
        self.helper = rclpy.create_node("depth_to_cloud_test_helper")
        self.info_pub = self.helper.create_publisher(
            CameraInfo, f"/{CAMERA_NAME}/stereo/camera_info", 10)
        self.rgb_pub = self.helper.create_publisher(
            CompressedImage, f"/{CAMERA_NAME}/rgb/image_raw/compressed", 10)
        self.depth_pub = self.helper.create_publisher(
            Image, f"/{CAMERA_NAME}/stereo/image_raw", 10)
        self.clouds: list[PointCloud2] = []
        self.helper.create_subscription(
            PointCloud2, f"/fortis/perception/{CAMERA_NAME}/points",
            self.clouds.append, 10)
        self._jpeg = self._encode_jpeg()

    @staticmethod
    def _encode_jpeg():
        """Encode a solid-red test frame; red survives JPEG compression."""
        bgr = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        bgr[:, :, 2] = 200  # OpenCV is BGR: channel 2 is red
        ok, buf = cv2.imencode(".jpg", bgr)
        assert ok
        return buf.tobytes()

    def spin(self, duration_s):
        """Drain the event loop on both nodes for the given wall-clock window."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def publish_frame_set(self):
        """Publish CameraInfo + rgb + depth sharing one stamp."""
        stamp = self.helper.get_clock().now().to_msg()
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = DEPTH_FRAME_ID
        info.width = WIDTH
        info.height = HEIGHT
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        rgb = CompressedImage()
        rgb.header.stamp = stamp
        rgb.header.frame_id = DEPTH_FRAME_ID
        rgb.format = "jpeg"
        rgb.data = self._jpeg
        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = DEPTH_FRAME_ID
        depth.height = HEIGHT
        depth.width = WIDTH
        depth.encoding = "16UC1"
        depth.is_bigendian = False
        depth.step = WIDTH * 2
        depth.data = np.full((HEIGHT, WIDTH), DEPTH_MM, dtype=np.uint16).tobytes()
        self.info_pub.publish(info)
        self.rgb_pub.publish(rgb)
        self.depth_pub.publish(depth)

    def wait_for_cloud(self):
        """Re-publish frame sets until the node emits a cloud."""
        end = time.monotonic() + CLOUD_TIMEOUT_S
        while not self.clouds and time.monotonic() < end:
            self.publish_frame_set()
            self.spin(0.2)
        assert self.clouds, f"no cloud within {CLOUD_TIMEOUT_S}s"
        return self.clouds[-1]

    def cleanup(self):
        """Tear down both nodes. Safe to call once after the test finishes."""
        self.helper.destroy_node()
        self.node.destroy_node()
        time.sleep(0.05)


def test_cloud_geometry_and_frame(harness):
    """Constant 1 m depth reprojects to z == 1.0 at the strided pixel count."""
    cloud = harness.wait_for_cloud()
    assert cloud.header.frame_id == DEPTH_FRAME_ID
    pts = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"))
    expected_points = (HEIGHT // STRIDE) * (WIDTH // STRIDE)
    assert pts.shape[0] == expected_points
    assert np.allclose(pts[:, 2], 1.0, atol=1e-3)
    # Spot-check x for the pinhole model: x = (u - cx) * z / fx, one
    # distinct value per strided pixel column.
    xs = np.unique(pts[:, 0])
    expected_xs = np.sort((np.arange(0, WIDTH, STRIDE) - CX) * 1.0 / FX)
    assert xs.shape[0] == expected_xs.shape[0]
    assert np.allclose(xs, expected_xs, atol=1e-4)


def test_cloud_carries_packed_rgb(harness):
    """The rgb field decodes back to the (red) test frame's colour."""
    cloud = harness.wait_for_cloud()
    assert [f.name for f in cloud.fields] == ["x", "y", "z", "rgb"]
    packed = point_cloud2.read_points_numpy(
        cloud, field_names=("rgb",)).reshape(-1)
    colors = unpack_rgb(packed)
    assert colors.shape[0] == (HEIGHT // STRIDE) * (WIDTH // STRIDE)
    assert colors[:, 0].mean() > 100, "red channel must dominate"
    assert colors[:, 2].mean() < 80, "blue channel must stay low"
