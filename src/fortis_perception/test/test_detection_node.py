"""
Tests for fortis_perception.detection_node (blob backend).

A helper node streams a synthetic front-camera triple (jpeg RGB + 16UC1
depth + CameraInfo) over real DDS and captures the node's outputs -- same
in-process round-trip style as fortis_drive's drive_node tests, because
the QoS/topic contract is the thing under test, not just the math.
"""

from __future__ import annotations

import math
import time

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray, Detection3DArray

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.msg import GraspCandidate
from fortis_perception.detection_node import DetectionNode

CAM = "oak_chassis_front"
WIDTH, HEIGHT = 320, 200
FX = FY = 200.0
CX, CY = 160.0, 100.0
DEPTH_MM = 1500
OPTICAL_FRAME = "front_camera_optical_frame"

SPIN_ONCE_TIMEOUT_S = 0.02

# Independent recomputation of the node's hardcoded front-mount extrinsic
# (fortis_chassis.urdf.xacro): a point on the optical axis at depth z lands
# at base_link (cam_x - z*cos(0.524), 0, cam_z + z*sin(0.524)).
URDF_CAM_X = -(0.332 / 2 + 0.01933 + 0.017 / 2)
URDF_CAM_Z = 0.21514
EXPECTED_BASE_X = URDF_CAM_X - 1.5 * math.cos(0.524)
EXPECTED_BASE_Z = URDF_CAM_Z + 1.5 * math.sin(0.524)


def _jpeg_red_circle() -> bytes:
    """Encode a grey frame with a centered red disc as jpeg bytes."""
    img = np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8)
    cv2.circle(img, (int(CX), int(CY)), 40, (0, 0, 255), -1)
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return buf.tobytes()


@pytest.fixture
def rclpy_session():
    """Give each test a fresh rclpy context (mirrors fortis_drive's tests)."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness(rclpy_session):
    """Stand up the node under test plus the camera-stream helper."""
    h = _Harness()
    h.spin(0.3)  # DDS discovery before the first publish
    try:
        yield h
    finally:
        h.cleanup()


class _Harness:
    """A DetectionNode under test plus a helper node for inputs/outputs."""

    def __init__(self) -> None:
        self.node = DetectionNode(parameter_overrides=[
            Parameter("target_rate_hz", value=5.0),
            Parameter("candidate_timeout_s", value=1.0),
        ])
        self.helper = rclpy.create_node("detection_test_helper")

        self.rgb_pub = self.helper.create_publisher(
            CompressedImage, f"/{CAM}/rgb/image_raw/compressed", 10)
        self.depth_pub = self.helper.create_publisher(
            Image, f"/{CAM}/stereo/image_raw", 10)
        self.info_pub = self.helper.create_publisher(
            CameraInfo, f"/{CAM}/stereo/camera_info", 10)

        self.det2d: list[Detection2DArray] = []
        self.det3d: list[Detection3DArray] = []
        self.grasps: list[GraspCandidate] = []
        self.grasp_ok: list[Bool] = []
        self.helper.create_subscription(
            Detection2DArray, "/fortis/perception/detections",
            self.det2d.append, 10)
        self.helper.create_subscription(
            Detection3DArray, "/fortis/perception/detections3d",
            self.det3d.append, 10)
        self.helper.create_subscription(
            GraspCandidate, "/fortis/perception/grasp_candidate",
            self.grasps.append, 10)
        self.helper.create_subscription(
            Bool, "/fortis/context/grasp_candidate_ok",
            self.grasp_ok.append, latched_qos_profile())

        self._jpeg = _jpeg_red_circle()
        self._depth_bytes = np.full(
            (HEIGHT, WIDTH), DEPTH_MM, dtype=np.uint16).tobytes()

    def publish_frame(self) -> None:
        """Publish one synchronized rgb + depth + camera_info triple."""
        stamp = self.helper.get_clock().now().to_msg()

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = OPTICAL_FRAME
        info.width, info.height = WIDTH, HEIGHT
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        self.info_pub.publish(info)

        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = OPTICAL_FRAME
        depth.height, depth.width = HEIGHT, WIDTH
        depth.encoding = "16UC1"
        depth.step = WIDTH * 2
        depth.data = self._depth_bytes
        self.depth_pub.publish(depth)

        rgb = CompressedImage()
        rgb.header.stamp = stamp
        rgb.header.frame_id = OPTICAL_FRAME
        rgb.format = "jpeg"
        rgb.data = self._jpeg
        self.rgb_pub.publish(rgb)

    def spin(self, duration_s: float) -> None:
        """Drain the event loop on both nodes for a wall-clock window."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def stream_until(self, predicate, timeout_s: float = 10.0) -> bool:
        """Stream frames at ~10 Hz until predicate() holds or timeout."""
        end = time.monotonic() + timeout_s
        next_pub = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_pub:
                self.publish_frame()
                next_pub = time.monotonic() + 0.1
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            if predicate():
                return True
        return predicate()

    def wait_until(self, predicate, timeout_s: float = 5.0) -> bool:
        """Spin (no new frames) until predicate() holds or timeout."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            if predicate():
                return True
        return predicate()

    def cleanup(self) -> None:
        """Tear down both nodes (helper/publisher side first)."""
        self.helper.destroy_node()
        self.node.destroy_node()
        time.sleep(0.05)


def _has_red_sphere(arrays) -> bool:
    """Check whether any captured detection array contains a red_sphere."""
    return any(
        det.results and det.results[0].hypothesis.class_id == "red_sphere"
        for arr in arrays for det in arr.detections
    )


def test_blob_pipeline_and_candidate_timeout(harness):
    """Full pass: 2D + 3D + grasp candidate arrive, then the flag times out."""
    ok = harness.stream_until(
        lambda: _has_red_sphere(harness.det2d)
        and any(a.detections for a in harness.det3d)
        and harness.grasps
        and any(m.data for m in harness.grasp_ok)
    )
    assert ok, "detection pipeline outputs did not all arrive within timeout"

    # 2D: red_sphere with a centered bbox.
    assert _has_red_sphere(harness.det2d)

    # 3D: optical-frame center on the optical axis at the constant depth.
    det3d = next(a for a in reversed(harness.det3d) if a.detections)
    assert det3d.header.frame_id == OPTICAL_FRAME
    center = det3d.detections[-1].bbox.center.position
    assert center.z == pytest.approx(DEPTH_MM / 1000.0, abs=0.02)
    assert center.x == pytest.approx(0.0, abs=0.1)
    assert center.y == pytest.approx(0.0, abs=0.1)

    # Grasp candidate: plausible base_link pose per the URDF front mount.
    grasp = harness.grasps[-1]
    assert grasp.pose.position.x == pytest.approx(EXPECTED_BASE_X, abs=0.05)
    assert grasp.pose.position.y == pytest.approx(0.0, abs=0.05)
    assert grasp.pose.position.z == pytest.approx(EXPECTED_BASE_Z, abs=0.05)
    assert 0.0 < grasp.confidence <= 1.0
    norm = math.hypot(grasp.approach.x, grasp.approach.y)
    assert norm == pytest.approx(1.0, abs=1e-6)
    assert grasp.approach.x < -0.9  # object is toward base_link -X
    assert grasp.approach.z == 0.0

    # Latched flag went True while candidates were fresh.
    assert any(m.data for m in harness.grasp_ok)

    # Stop streaming: with candidate_timeout_s=1.0 the flag must flip False.
    flipped = harness.wait_until(
        lambda: harness.grasp_ok and harness.grasp_ok[-1].data is False,
        timeout_s=5.0,
    )
    assert flipped, "grasp_candidate_ok did not flip False after the timeout"
