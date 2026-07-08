"""
Tests for fortis_perception.target_selector_node.

A helper node broadcasts a static identity odom->base_link transform,
publishes /clicked_point, and captures the target pose, latched context
flag, and the FSM click event over real DDS.
"""

from __future__ import annotations

import math
import time

import pytest
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from rclpy.time import Time
from std_msgs.msg import Bool, Empty
from tf2_ros import StaticTransformBroadcaster

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_perception.target_selector_node import TargetSelectorNode

SPIN_ONCE_TIMEOUT_S = 0.02

#: Exact FSM event topic (Event.CHASSIS_CAM_CLICK lowercased by
#: fortis_safety.mission_state_node).
CLICK_EVENT_TOPIC = "/fortis/events/chassis_cam_click"


@pytest.fixture
def rclpy_session():
    """Give each test a fresh rclpy context (mirrors fortis_drive's tests)."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness(rclpy_session):
    """Stand up the node under test plus the TF/click helper."""
    h = _Harness()
    h.spin(0.3)  # DDS discovery before the first publish
    try:
        yield h
    finally:
        h.cleanup()


class _Harness:
    """A TargetSelectorNode under test plus a helper node for inputs/outputs."""

    def __init__(self) -> None:
        self.node = TargetSelectorNode()
        self.helper = rclpy.create_node("target_selector_test_helper")

        # Static identity odom->base_link so the node can anchor in odom.
        self._static_tf = StaticTransformBroadcaster(self.helper)
        tf = TransformStamped()
        tf.header.stamp = self.helper.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.rotation.w = 1.0
        self._static_tf.sendTransform(tf)

        self.click_pub = self.helper.create_publisher(
            PointStamped, "/clicked_point", 10)

        self.poses: list[PoseStamped] = []
        self.valids: list[Bool] = []
        self.events: list[Empty] = []
        self.helper.create_subscription(
            PoseStamped, "/fortis/target_pose",
            self.poses.append, latched_qos_profile())
        self.helper.create_subscription(
            Bool, "/fortis/context/target_pose_valid",
            self.valids.append, latched_qos_profile())
        self.helper.create_subscription(
            Empty, CLICK_EVENT_TOPIC, self.events.append, 10)

    def click(self, x: float, y: float, frame: str = "base_link") -> None:
        """Publish one /clicked_point."""
        msg = PointStamped()
        msg.header.stamp = self.helper.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.point.x = float(x)
        msg.point.y = float(y)
        self.click_pub.publish(msg)

    def spin(self, duration_s: float) -> None:
        """Drain the event loop on both nodes for a wall-clock window."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def spin_until(self, predicate, timeout_s: float = 5.0) -> bool:
        """Spin both nodes until predicate() holds or timeout."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            if predicate():
                return True
        return predicate()

    def wait_for_tf(self, timeout_s: float = 5.0) -> None:
        """Spin until the node's TF buffer holds odom->base_link."""
        ok = self.spin_until(
            lambda: self.node._tf_buffer.can_transform(
                "odom", "base_link", Time()),
            timeout_s,
        )
        assert ok, "odom->base_link static TF never reached the node"

    def cleanup(self) -> None:
        """Tear down both nodes (helper/publisher side first)."""
        self.helper.destroy_node()
        self.node.destroy_node()
        time.sleep(0.05)


def test_valid_click_publishes_pose_flags_and_event(harness):
    """An in-annulus click yields an odom pose, a True flag, and one FSM event."""
    harness.wait_for_tf()
    harness.click(1.5, 0.5)
    ok = harness.spin_until(
        lambda: harness.poses
        and harness.events
        and any(v.data for v in harness.valids)
    )
    assert ok, "target pose / flag / event did not all arrive within timeout"

    pose = harness.poses[-1]
    # Identity odom->base_link was available, so the pose is odom-anchored.
    assert pose.header.frame_id == "odom"
    assert pose.pose.position.x == pytest.approx(1.5, abs=1e-6)
    assert pose.pose.position.y == pytest.approx(0.5, abs=1e-6)

    # Orientation = yaw from the robot toward the target.
    yaw = math.atan2(0.5, 1.5)
    assert pose.pose.orientation.z == pytest.approx(math.sin(yaw / 2), abs=1e-6)
    assert pose.pose.orientation.w == pytest.approx(math.cos(yaw / 2), abs=1e-6)

    assert harness.valids[-1].data is True
    harness.spin(0.3)  # settle: no duplicate events
    assert len(harness.events) == 1


def test_out_of_range_click_rejected(harness):
    """A click outside the annulus publishes a False flag and no pose/event."""
    harness.wait_for_tf()
    harness.click(10.0, 0.0)  # far beyond max_target_range_m (3.5)
    # Initial latched False + the rejection's False = at least 2 samples.
    ok = harness.spin_until(lambda: len(harness.valids) >= 2)
    assert ok, "rejection flag did not arrive within timeout"

    assert not any(v.data for v in harness.valids)
    assert harness.poses == []
    assert harness.events == []
