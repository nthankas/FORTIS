"""
In-process two-camera rig test for oak_replayer_node.

House style of test_replayer_contract.py: real OakReplayerNodes plus a
helper subscriber node, spun together in-process. Two replayers (front
+ rear mounts) exercise the multi-instance contract the 4-camera
launches rely on: each instance publishes its own derived topic
namespace and its own static base_link -> camera mount TF, while
exactly ONE instance (publish_base_tf) owns /fortis/sim/ground_truth.
Low-res, hold trajectory -- the rig wiring, not the imagery, is under
test.
"""

from __future__ import annotations

import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from tf2_ros import Buffer, TransformListener

from fortis_sim_support.oak_replayer_node import GROUND_TRUTH_TOPIC, OakReplayerNode

# --- Constants used by the harness -------------------------------------------

#: Derived namespaces: mount front/rear with camera_name left at its
#: empty default must yield the real rig's names.
FRONT_CAMERA = "oak_chassis_front"
REAR_CAMERA = "oak_chassis_rear"

WIDTH, HEIGHT = 160, 100
FPS = 5.0

#: Expected base_link -> camera translations, recomputed here from
#: fortis_constants.xacro (NOT imported from the node, so a bad MOUNTS
#: edit fails this test):
#:   cam_front_x = chassis_length/2 + cam_edge_to_housing + oak_lite_z/2
#:               = 0.332/2 + 0.01933 + 0.017/2 = 0.19383
#:   cam_height_z = 0.21514
#: User front is base_link -X, so front sits at -x and rear at +x.
FRONT_XYZ = (-0.19383, 0.0, 0.21514)
REAR_XYZ = (0.19383, 0.0, 0.21514)

#: Wall-clock budget for both streams + both static TFs after discovery.
RECEIVE_TIMEOUT_S: float = 10.0

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
    """Per-test harness: two replayers plus a subscriber/TF helper."""
    h = _Harness()
    h.wait_until_ready()
    try:
        yield h
    finally:
        h.cleanup()


# --- Harness ------------------------------------------------------------------


class _Harness:
    """Two OakReplayerNodes plus a helper node capturing topics and TF."""

    def __init__(self) -> None:
        common = [
            Parameter("width", value=WIDTH),
            Parameter("height", value=HEIGHT),
            Parameter("fps", value=FPS),
            Parameter("imu_rate_hz", value=50.0),
            Parameter("trajectory", value="hold"),
            Parameter("publish_tf", value=True),
        ]
        self.front = OakReplayerNode(parameter_overrides=common + [
            Parameter("mount", value="front"),
            Parameter("publish_base_tf", value=True),
        ])
        self.rear = OakReplayerNode(parameter_overrides=common + [
            Parameter("mount", value="rear"),
            Parameter("publish_base_tf", value=False),
        ])
        self.helper = rclpy.create_node("multicam_rig_test_helper")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.helper)

        self.rgb: dict[str, list[CompressedImage]] = {}
        self.depth: dict[str, list[Image]] = {}
        for camera in (FRONT_CAMERA, REAR_CAMERA):
            self.rgb[camera] = []
            self.depth[camera] = []
            self.helper.create_subscription(
                CompressedImage, f"/{camera}/rgb/image_raw/compressed",
                self.rgb[camera].append, 10)
            self.helper.create_subscription(
                Image, f"/{camera}/stereo/image_raw",
                self.depth[camera].append, 10)

    def _ready(self) -> bool:
        """Report whether both streams flow and both mount TFs resolve."""
        streams = all(self.rgb[camera] and self.depth[camera]
                      for camera in (FRONT_CAMERA, REAR_CAMERA))
        tfs = all(self.tf_buffer.can_transform("base_link", camera, Time())
                  for camera in (FRONT_CAMERA, REAR_CAMERA))
        return streams and tfs

    def spin(self, duration_s: float) -> None:
        """Drain the event loop on all three nodes for a wall-clock window."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self.front, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.rear, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def wait_until_ready(self) -> None:
        """Spin until both namespaces stream and both mount TFs resolve."""
        self.spin(DISCOVERY_SPIN_S)
        end = time.monotonic() + RECEIVE_TIMEOUT_S
        while time.monotonic() < end and not self._ready():
            rclpy.spin_once(self.front, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.rear, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def cleanup(self) -> None:
        """Tear down all three nodes; safe to call once after the test."""
        self.helper.destroy_node()
        self.rear.destroy_node()
        self.front.destroy_node()
        time.sleep(0.05)


# --- Tests ---------------------------------------------------------------------


def test_both_namespaces_stream_rgb_and_depth(harness):
    """Verify both derived camera namespaces publish rgb and depth."""
    for camera in (FRONT_CAMERA, REAR_CAMERA):
        assert harness.rgb[camera], (
            f"no rgb on /{camera} within {RECEIVE_TIMEOUT_S}s")
        assert harness.depth[camera], (
            f"no depth on /{camera} within {RECEIVE_TIMEOUT_S}s")


def test_exactly_one_ground_truth_publisher(harness):
    """Verify only the publish_base_tf instance offers ground truth."""
    count = harness.helper.count_publishers(GROUND_TRUTH_TOPIC)
    assert count == 1, (
        f"expected exactly 1 publisher on {GROUND_TRUTH_TOPIC}, found "
        f"{count}; publish_base_tf must gate the ground-truth owner")


def test_static_mount_tfs_match_urdf(harness):
    """Verify each instance broadcasts its own URDF mount translation."""
    for camera, expected in ((FRONT_CAMERA, FRONT_XYZ), (REAR_CAMERA, REAR_XYZ)):
        tf = harness.tf_buffer.lookup_transform("base_link", camera, Time())
        t = tf.transform.translation
        assert (t.x, t.y, t.z) == pytest.approx(expected, abs=1e-9), (
            f"base_link->{camera} translation ({t.x}, {t.y}, {t.z}) "
            f"!= xacro pose {expected}")
