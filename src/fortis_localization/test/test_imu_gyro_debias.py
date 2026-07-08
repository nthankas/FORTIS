"""
Tests for fortis_localization.imu_gyro_debias_node.

Two layers, mirroring the wheel_odometry test style:

1. Pure logic: drive ImuGyroDebiasNode's callbacks directly (no DDS) and
   assert the bias EMA, the armed-freeze, the per-message subtraction, the
   header/covariance pass-through, and the post-disarm settle skip.

2. ROS round trip: a real ImuGyroDebiasNode plus a helper node that publishes
   the raw IMU + the latched /fortis/drive/armed and subscribes to
   /imu/debiased, verifying the wiring and message contract over DDS.

Run with:
    cd /workspace
    colcon build --packages-select fortis_comms fortis_localization
    source install/setup.bash
    python3 -m pytest src/fortis_localization/test/test_imu_gyro_debias.py -v
"""

from __future__ import annotations

import time

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_localization.imu_gyro_debias_node import ImuGyroDebiasNode


# --- Harness constants ------------------------------------------------------

SPIN_DURATION_S: float = 0.3
SPIN_ONCE_TIMEOUT_S: float = 0.02
OUTPUT_TIMEOUT_S: float = 2.0


def _make_imu(wz: float, *, wx: float = 0.0, wy: float = 0.0) -> Imu:
    """Build an Imu with a known angular velocity and distinctive metadata.

    The orientation, frame_id, and covariances are set to non-default values
    so the pass-through assertions actually prove they survive untouched.
    """
    msg = Imu()
    msg.header.frame_id = "oak_chassis_front_imu_frame"
    msg.header.stamp.sec = 7
    msg.header.stamp.nanosec = 123
    msg.angular_velocity.x = wx
    msg.angular_velocity.y = wy
    msg.angular_velocity.z = wz
    # Non-identity orientation + sentinel covariances to verify pass-through.
    msg.orientation.x = 0.1
    msg.orientation.y = 0.2
    msg.orientation.z = 0.3
    msg.orientation.w = 0.9
    msg.orientation_covariance = [float(i) for i in range(9)]
    msg.angular_velocity_covariance = [float(i + 10) for i in range(9)]
    msg.linear_acceleration_covariance = [float(i + 20) for i in range(9)]
    return msg


# --- Layer 1: pure logic ----------------------------------------------------


class _StandaloneNode:
    """Drives ImuGyroDebiasNode callbacks directly, no DDS."""

    def __init__(self) -> None:
        rclpy.init()
        self.node = ImuGyroDebiasNode()
        self.out: list[Imu] = []
        # Intercept publishes so the math tests need no subscriber.
        self.node._debiased_pub.publish = self.out.append

    def set_armed(self, armed: bool) -> None:
        msg = Bool()
        msg.data = armed
        self.node._on_armed(msg)

    def feed(self, msg: Imu) -> Imu:
        self.node._on_imu(msg)
        return self.out[-1]

    @property
    def bias_z(self) -> float:
        return self.node._bias["z"]

    def destroy(self) -> None:
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_ema_converges_to_constant_offset_while_disarmed():
    """A constant bias fed while disarmed drives the EMA toward that value."""
    node = _StandaloneNode()
    try:
        node.set_armed(False)  # stationary -> estimate; no edge -> no settle wait
        bias = 0.03
        for _ in range(2000):
            node.feed(_make_imu(bias))
        # alpha=0.02 over 2000 samples converges to the constant input.
        assert node.bias_z == pytest.approx(bias, abs=1e-4)
    finally:
        node.destroy()


def test_bias_frozen_while_armed():
    """While armed the EMA must not move, even if the gyro reads a new value."""
    node = _StandaloneNode()
    try:
        node.set_armed(False)
        for _ in range(2000):
            node.feed(_make_imu(0.03))
        frozen = node.bias_z

        node.set_armed(True)
        # Feed a very different rate (real rotation): bias must stay put.
        for _ in range(500):
            node.feed(_make_imu(1.5))
        assert node.bias_z == frozen
    finally:
        node.destroy()


def test_output_subtracts_current_bias():
    """Output angular_velocity.z == input - bias on every message."""
    node = _StandaloneNode()
    try:
        node.set_armed(False)
        for _ in range(2000):
            node.feed(_make_imu(0.03))
        bias = node.bias_z

        # Freeze, then feed a known input and check the subtraction exactly.
        node.set_armed(True)
        out = node.feed(_make_imu(0.5))
        assert out.angular_velocity.z == pytest.approx(0.5 - bias, abs=1e-9)
    finally:
        node.destroy()


def test_header_frame_and_covariances_preserved():
    """Header, frame_id, orientation, and all covariances pass through unchanged."""
    node = _StandaloneNode()
    try:
        node.set_armed(True)  # bias 0 -> output equals input on every field
        src = _make_imu(0.4)
        out = node.feed(src)

        assert out.header.frame_id == src.header.frame_id
        assert out.header.stamp.sec == src.header.stamp.sec
        assert out.header.stamp.nanosec == src.header.stamp.nanosec
        assert out.orientation.x == src.orientation.x
        assert out.orientation.w == src.orientation.w
        assert list(out.orientation_covariance) == list(src.orientation_covariance)
        assert list(out.angular_velocity_covariance) == \
            list(src.angular_velocity_covariance)
        assert list(out.linear_acceleration_covariance) == \
            list(src.linear_acceleration_covariance)
    finally:
        node.destroy()


def test_settle_skip_ignores_samples_after_disarm_edge():
    """Samples within settle_skip_s of an arm->disarm edge do not move the bias."""
    node = _StandaloneNode()
    try:
        node.set_armed(True)
        node.set_armed(False)  # arm->disarm edge: starts the settle window

        # Inside the window (default 0.5 s of wall clock has not elapsed),
        # stationary samples are ignored: the bias must stay at its init 0.
        for _ in range(50):
            node.feed(_make_imu(0.03))
        assert node.bias_z == 0.0
    finally:
        node.destroy()


def test_disarmed_without_edge_trusts_gyro_immediately():
    """Disarmed with no prior edge (launched disarmed) skips the settle wait."""
    node = _StandaloneNode()
    try:
        # _armed starts as None (unknown, EMA frozen); force disarmed WITHOUT
        # an arm->disarm edge -- mirrors a node that booted already disarmed.
        node.node._armed = False
        node.feed(_make_imu(0.03))
        assert node.bias_z != 0.0
    finally:
        node.destroy()


# --- Layer 2: ROS round trip ------------------------------------------------


@pytest.fixture
def rclpy_session():
    """Function-scoped rclpy.init / shutdown for the round-trip tests."""
    rclpy.init()
    yield
    rclpy.shutdown()


class _Harness:
    """An ImuGyroDebiasNode under test plus a raw-IMU/armed pub and debiased sub."""

    RAW_TOPIC = "/oak_chassis_front/imu/data"
    DEBIASED_TOPIC = "/imu/debiased"

    def __init__(self) -> None:
        self.node = ImuGyroDebiasNode()
        self.helper: Node = rclpy.create_node("imu_debias_test_helper")
        self.imu_pub = self.helper.create_publisher(Imu, self.RAW_TOPIC, 10)
        self.armed_pub = self.helper.create_publisher(
            Bool, "/fortis/drive/armed", latched_qos_profile()
        )
        self.out_msgs: list[Imu] = []
        self.helper.create_subscription(
            Imu, self.DEBIASED_TOPIC, self.out_msgs.append, 10
        )

    def spin(self, duration_s: float = SPIN_DURATION_S) -> None:
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def set_armed(self, armed: bool) -> None:
        msg = Bool()
        msg.data = armed
        self.armed_pub.publish(msg)
        self.spin(0.2)

    def publish_imu(self, wz: float) -> None:
        initial = len(self.out_msgs)
        self.imu_pub.publish(_make_imu(wz))
        end = time.monotonic() + OUTPUT_TIMEOUT_S
        while len(self.out_msgs) == initial and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def cleanup(self) -> None:
        self.helper.destroy_node()
        self.node.destroy_node()
        time.sleep(0.05)


@pytest.fixture
def harness(rclpy_session):
    h = _Harness()
    h.spin(SPIN_DURATION_S)  # discovery
    try:
        yield h
    finally:
        h.cleanup()


def test_round_trip_republishes_debiased(harness):
    """A raw IMU in yields a debiased copy out with metadata preserved."""
    harness.set_armed(True)  # bias frozen at 0 -> output equals input
    harness.publish_imu(0.4)
    harness.spin()

    assert len(harness.out_msgs) >= 1, "expected a /imu/debiased message"
    out = harness.out_msgs[-1]
    assert out.header.frame_id == "oak_chassis_front_imu_frame"
    assert out.angular_velocity.z == pytest.approx(0.4, abs=1e-9)
    assert list(out.angular_velocity_covariance) == [float(i + 10) for i in range(9)]
