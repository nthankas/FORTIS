"""
In-process topic-contract test for oak_replayer_node.

House style of fortis_drive/test/test_drive_node.py: a real
OakReplayerNode plus a helper subscriber node, spun together in-process
(no launch_testing), asserting the depthai v3 topic contract that every
downstream perception node keys on. Low-res (320x200 @ 5 fps), hold
trajectory, TF off -- the contract, not the imagery, is under test.
"""

from __future__ import annotations

import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu

from fortis_sim_support.oak_replayer_node import GROUND_TRUTH_TOPIC, OakReplayerNode

# --- Constants used by the harness -------------------------------------------

CAMERA_NAME = "oak_sim_test"
WIDTH, HEIGHT = 320, 200
FPS = 5.0
RGB_FRAME = f"{CAMERA_NAME}_rgb_camera_optical_frame"

#: Wall-clock budget for every topic to deliver at least one message once
#: DDS discovery has been drained. First frame lands at 1/fps = 0.2 s.
RECEIVE_TIMEOUT_S: float = 3.0

#: Per-spin_once timeout, small so the loop drains promptly per callback.
SPIN_ONCE_TIMEOUT_S: float = 0.02

#: Initial drain so in-process discovery completes before the deadline runs.
DISCOVERY_SPIN_S: float = 0.5


# --- Fixtures -----------------------------------------------------------------


@pytest.fixture
def rclpy_session():
    """Function-scoped rclpy.init / shutdown: a fresh DDS participant per test."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness(rclpy_session):
    """Per-test harness: a replayer under test plus a subscriber helper."""
    h = _Harness()
    h.wait_for_all()
    try:
        yield h
    finally:
        h.cleanup()


# --- Harness ------------------------------------------------------------------


class _Harness:
    """An OakReplayerNode under test plus a helper node capturing its output."""

    def __init__(self) -> None:
        overrides = [
            Parameter("camera_name", value=CAMERA_NAME),
            Parameter("width", value=WIDTH),
            Parameter("height", value=HEIGHT),
            Parameter("fps", value=FPS),
            Parameter("imu_rate_hz", value=50.0),
            Parameter("trajectory", value="hold"),
            Parameter("publish_tf", value=False),
        ]
        self.node = OakReplayerNode(parameter_overrides=overrides)
        self.helper = rclpy.create_node("replayer_test_helper")

        self.rgb: list[CompressedImage] = []
        self.rgb_info: list[CameraInfo] = []
        self.depth: list[Image] = []
        self.depth_info: list[CameraInfo] = []
        self.imu: list[Imu] = []
        self.ground_truth: list[Odometry] = []

        base = f"/{CAMERA_NAME}"
        self.helper.create_subscription(
            CompressedImage, f"{base}/rgb/image_raw/compressed", self.rgb.append, 10)
        self.helper.create_subscription(
            CameraInfo, f"{base}/rgb/camera_info", self.rgb_info.append, 10)
        self.helper.create_subscription(
            Image, f"{base}/stereo/image_raw", self.depth.append, 10)
        self.helper.create_subscription(
            CameraInfo, f"{base}/stereo/camera_info", self.depth_info.append, 10)
        self.helper.create_subscription(
            Imu, f"{base}/imu/data", self.imu.append, 10)
        self.helper.create_subscription(
            Odometry, GROUND_TRUTH_TOPIC, self.ground_truth.append, 10)

    def _stores(self) -> tuple[list, ...]:
        return (self.rgb, self.rgb_info, self.depth, self.depth_info,
                self.imu, self.ground_truth)

    def spin(self, duration_s: float) -> None:
        """Drain the event loop on both nodes for a wall-clock window."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def wait_for_all(self) -> None:
        """Spin until every subscribed topic has delivered at least one message."""
        self.spin(DISCOVERY_SPIN_S)
        end = time.monotonic() + RECEIVE_TIMEOUT_S
        while time.monotonic() < end and not all(self._stores()):
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def cleanup(self) -> None:
        """Tear down both nodes; safe to call once after the test finishes."""
        self.helper.destroy_node()
        self.node.destroy_node()
        time.sleep(0.05)


# --- Tests ---------------------------------------------------------------------


def test_all_topics_publish_within_budget(harness):
    """All five camera topics plus ground truth deliver within the budget."""
    missing = [name for name, store in zip(
        ("rgb", "rgb_info", "depth", "depth_info", "imu", "ground_truth"),
        harness._stores()) if not store]
    assert not missing, f"no message within {RECEIVE_TIMEOUT_S}s on: {missing}"


def test_depth_stream_is_16uc1(harness):
    """Depth publishes 16UC1 at the configured size in the rgb optical frame."""
    msg = harness.depth[-1]
    assert msg.encoding == "16UC1"
    assert (msg.width, msg.height) == (WIDTH, HEIGHT)
    assert msg.header.frame_id == RGB_FRAME


def test_rgb_stream_is_jpeg(harness):
    """The compressed RGB payload is a real JPEG in the rgb optical frame."""
    msg = harness.rgb[-1]
    assert bytes(msg.data[:2]) == b"\xff\xd8", "payload must start with the JPEG SOI"
    assert "jpeg" in msg.format
    assert msg.header.frame_id == RGB_FRAME


def test_camera_infos_are_consistent(harness):
    """rgb and stereo camera_info carry the same pinhole K (aligned depth)."""
    rgb_info = harness.rgb_info[-1]
    depth_info = harness.depth_info[-1]
    assert list(rgb_info.k) == list(depth_info.k)
    assert rgb_info.k[0] > 0.0
    assert (rgb_info.width, rgb_info.height) == (WIDTH, HEIGHT)
    assert (depth_info.width, depth_info.height) == (WIDTH, HEIGHT)
    assert rgb_info.header.frame_id == RGB_FRAME
    assert depth_info.header.frame_id == RGB_FRAME


def test_imu_frame_and_stationary_values(harness):
    """IMU uses the camera imu frame; a held pose reads +g on z, zero rates."""
    msg = harness.imu[-1]
    assert msg.header.frame_id == f"{CAMERA_NAME}_imu_frame"
    assert msg.linear_acceleration.z == pytest.approx(9.80665, abs=1e-6)
    assert msg.angular_velocity.z == pytest.approx(0.0, abs=1e-9)


def test_ground_truth_matches_hold_pose(harness):
    """Ground truth is odom->base_link with the hold pose and a zero twist."""
    msg = harness.ground_truth[-1]
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "base_link"
    # hold = the default orbit's t = 0 pose: (orbit_radius, 0, 0), yaw 0.
    assert msg.pose.pose.position.x == pytest.approx(1.0, abs=1e-12)
    assert msg.pose.pose.position.y == pytest.approx(0.0, abs=1e-12)
    assert msg.twist.twist.linear.x == pytest.approx(0.0, abs=1e-12)
    assert msg.twist.twist.linear.y == pytest.approx(0.0, abs=1e-12)
    assert msg.twist.twist.angular.z == pytest.approx(0.0, abs=1e-12)
