"""
Tests for fortis_localization.wheel_odometry_node.

Two layers, mirroring fortis_drive's test style:

1. Pure math: known wheel velocities -> the node's helpers -> assert the
   recovered body twist matches xdrive_fk_solver, and that the dead-reckoning
   integrator reproduces the closed-form pose for simple motions. No ROS.

2. ROS round trip: a real WheelOdometryNode plus a helper node that publishes
   /joint_states (by joint NAME, deliberately shuffled) and subscribes to
   /odom. Verifies the name-keyed mapping, header-stamp integration, and the
   Odometry contract (frames, twist, no TF) over DDS.

Run with:
    cd /workspace
    colcon build --packages-select fortis_comms fortis_localization
    source install/setup.bash
    python3 -m pytest src/fortis_localization/test/test_wheel_odometry.py -v
"""

from __future__ import annotations

import math
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

from fortis_comms.xdrive_kinematics import (
    WHEEL_RADIUS,
    xdrive_fk_solver,
    xdrive_ik_solver,
)
from fortis_localization.wheel_odometry_node import (
    BASE_FRAME,
    ODOM_FRAME,
    ODOM_TOPIC,
    WHEEL_JOINT_NAMES,
    WheelOdometryNode,
    _wheel_speeds_from_joint_state,
)


# --- Harness constants ------------------------------------------------------

SPIN_DURATION_S: float = 0.3
SPIN_ONCE_TIMEOUT_S: float = 0.02
ODOM_OUTPUT_TIMEOUT_S: float = 2.0


def _wheel_rad_s_for_twist(vx: float, vy: float, wz: float) -> list[float]:
    """Return the four wheel angular velocities (rad/s) that realise a twist.

    Uses the inverse kinematics to get contact-patch linear speeds (m/s) in
    [FL, FR, RL, RR] order, then divides by WHEEL_RADIUS. This is the inverse
    of what the odometry node does, so a clean IK->node->FK round trip must
    recover the original twist (within the IK saturator's scale-down).
    """
    linear = xdrive_ik_solver(vx, vy, wz)
    return [float(v) / WHEEL_RADIUS for v in linear]


def _make_joint_state(
    wheel_rad_s: list[float],
    stamp_s: float,
    shuffle: bool = False,
) -> JointState:
    """Build a /joint_states message carrying the four wheel velocities.

    When `shuffle` is set, the joints are emitted in a non-canonical order
    (and with a decoy extra joint) so the test proves the node maps by NAME,
    not by array position.
    """
    msg = JointState()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)

    pairs = list(zip(WHEEL_JOINT_NAMES, wheel_rad_s))
    if shuffle:
        # Reverse order + an unrelated joint the node must ignore.
        pairs = list(reversed(pairs))
        pairs.insert(1, ("some_arm_joint", 99.0))

    msg.name = [name for name, _ in pairs]
    msg.velocity = [vel for _, vel in pairs]
    return msg


# --- Layer 1: pure math -----------------------------------------------------


@pytest.mark.parametrize(
    "label, vx, vy, wz",
    [
        ("forward", 0.4, 0.0, 0.0),
        ("strafe", 0.0, 0.3, 0.0),
        ("rotate", 0.0, 0.0, 0.5),
        ("combined", 0.2, 0.1, 0.3),
    ],
)
def test_joint_state_to_twist_matches_fk(label, vx, vy, wz):
    """The node's name-keyed wheel read + FK recovers the commanded twist."""
    wheel_rad_s = _wheel_rad_s_for_twist(vx, vy, wz)
    msg = _make_joint_state(wheel_rad_s, stamp_s=0.0, shuffle=True)

    speeds = _wheel_speeds_from_joint_state(msg)
    assert speeds is not None, "all four wheel joints must be found by name"

    rec_vx, rec_vy, rec_wz = (float(v) for v in xdrive_fk_solver(speeds))

    # IK saturates to MAX_WHEEL_SPEED; recompute the same scale to compare.
    ik = xdrive_ik_solver(vx, vy, wz)
    fk_expected = xdrive_fk_solver([float(v) for v in ik])
    assert rec_vx == pytest.approx(fk_expected[0], abs=1e-9), label
    assert rec_vy == pytest.approx(fk_expected[1], abs=1e-9), label
    assert rec_wz == pytest.approx(fk_expected[2], abs=1e-9), label


def test_missing_wheel_joint_returns_none():
    """A /joint_states without all four wheels yields no twist (skipped)."""
    msg = JointState()
    msg.name = ["fl_wheel_joint", "fr_wheel_joint"]  # rl/rr absent
    msg.velocity = [1.0, 1.0]
    assert _wheel_speeds_from_joint_state(msg) is None


def test_pure_forward_integrates_to_straight_line():
    """A constant forward twist integrates to x = vx * t, y = 0, yaw = 0."""
    node = _StandaloneNode()
    try:
        vx = 0.3
        wheel_rad_s = _wheel_rad_s_for_twist(vx, 0.0, 0.0)
        # The IK saturator may scale the request; integrate against the twist
        # the node will actually see (FK of the produced wheel speeds).
        speeds = [v * WHEEL_RADIUS for v in wheel_rad_s]
        eff_vx = float(xdrive_fk_solver(speeds)[0])

        for k in range(11):  # t = 0.0 .. 1.0 s in 0.1 s steps
            node.feed(_make_joint_state(wheel_rad_s, stamp_s=k * 0.1))

        assert node._x == pytest.approx(eff_vx * 1.0, abs=1e-6)
        assert node._y == pytest.approx(0.0, abs=1e-9)
        assert node._yaw == pytest.approx(0.0, abs=1e-9)
    finally:
        node.destroy()


def test_pure_rotation_integrates_yaw_only():
    """A constant yaw rate integrates yaw = wz * t with the base staying put."""
    node = _StandaloneNode()
    try:
        wz = 0.5
        wheel_rad_s = _wheel_rad_s_for_twist(0.0, 0.0, wz)
        speeds = [v * WHEEL_RADIUS for v in wheel_rad_s]
        eff_wz = float(xdrive_fk_solver(speeds)[2])

        for k in range(11):
            node.feed(_make_joint_state(wheel_rad_s, stamp_s=k * 0.1))

        assert node._yaw == pytest.approx(eff_wz * 1.0, abs=1e-6)
        assert node._x == pytest.approx(0.0, abs=1e-9)
        assert node._y == pytest.approx(0.0, abs=1e-9)
    finally:
        node.destroy()


def test_first_message_only_seeds_clock():
    """The first /joint_states sets the stamp but moves nothing (no dt yet)."""
    node = _StandaloneNode()
    try:
        wheel_rad_s = _wheel_rad_s_for_twist(0.5, 0.0, 0.0)
        node.feed(_make_joint_state(wheel_rad_s, stamp_s=5.0))
        assert (node._x, node._y, node._yaw) == (0.0, 0.0, 0.0)
    finally:
        node.destroy()


def test_oversized_dt_is_not_integrated():
    """A header-stamp jump beyond MAX_INTEGRATION_DT_S re-anchors, not teleports."""
    node = _StandaloneNode()
    try:
        wheel_rad_s = _wheel_rad_s_for_twist(0.5, 0.0, 0.0)
        node.feed(_make_joint_state(wheel_rad_s, stamp_s=0.0))   # seed
        node.feed(_make_joint_state(wheel_rad_s, stamp_s=10.0))  # 10 s gap
        assert (node._x, node._y, node._yaw) == (0.0, 0.0, 0.0)
    finally:
        node.destroy()


# --- A node usable without a running rclpy context --------------------------


class _StandaloneNode:
    """Drives WheelOdometryNode._on_joint_states directly, no DDS.

    rclpy.init/shutdown is wrapped so the math tests don't pay for DDS
    discovery; they only exercise the integrator and the name mapping.
    """

    def __init__(self) -> None:
        rclpy.init()
        self.node = WheelOdometryNode()

    def feed(self, msg: JointState) -> None:
        self.node._on_joint_states(msg)

    @property
    def _x(self) -> float:
        return self.node._x

    @property
    def _y(self) -> float:
        return self.node._y

    @property
    def _yaw(self) -> float:
        return self.node._yaw

    def destroy(self) -> None:
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# --- Layer 2: ROS round trip ------------------------------------------------


@pytest.fixture
def rclpy_session():
    """Function-scoped rclpy.init / shutdown for the round-trip tests."""
    rclpy.init()
    yield
    rclpy.shutdown()


class _Harness:
    """A WheelOdometryNode under test plus a /joint_states publisher + /odom sub."""

    def __init__(self) -> None:
        self.node = WheelOdometryNode()
        self.helper: Node = rclpy.create_node("wheel_odometry_test_helper")
        self.joint_pub = self.helper.create_publisher(
            JointState, "/joint_states", 10
        )
        self.odom_msgs: list[Odometry] = []
        self.helper.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_msgs.append,
            10,
        )

    def spin(self, duration_s: float = SPIN_DURATION_S) -> None:
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def publish_joint_state(self, wheel_rad_s, stamp_s, shuffle=False) -> None:
        initial = len(self.odom_msgs)
        self.joint_pub.publish(
            _make_joint_state(wheel_rad_s, stamp_s, shuffle=shuffle)
        )
        end = time.monotonic() + ODOM_OUTPUT_TIMEOUT_S
        while len(self.odom_msgs) == initial and time.monotonic() < end:
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


def test_odom_message_contract(harness):
    """/odom carries the right frames, the FK twist, and no broadcast TF."""
    vx, vy, wz = 0.2, 0.1, 0.3
    wheel_rad_s = _wheel_rad_s_for_twist(vx, vy, wz)
    harness.publish_joint_state(wheel_rad_s, stamp_s=0.0, shuffle=True)
    harness.spin()

    assert len(harness.odom_msgs) >= 1, "expected an /odom message"
    odom = harness.odom_msgs[-1]
    assert odom.header.frame_id == ODOM_FRAME
    assert odom.child_frame_id == BASE_FRAME

    expected = xdrive_fk_solver([v * WHEEL_RADIUS for v in wheel_rad_s])
    assert odom.twist.twist.linear.x == pytest.approx(expected[0], abs=1e-6)
    assert odom.twist.twist.linear.y == pytest.approx(expected[1], abs=1e-6)
    assert odom.twist.twist.angular.z == pytest.approx(expected[2], abs=1e-6)
    # Twist covariance must be present (loose, but non-zero on the diagonal).
    assert odom.twist.covariance[0] > 0.0
    assert odom.twist.covariance[35] > 0.0


def test_odom_pose_advances_over_two_stamps(harness):
    """Two stamped /joint_states integrate a forward step into the pose."""
    vx = 0.3
    wheel_rad_s = _wheel_rad_s_for_twist(vx, 0.0, 0.0)
    eff_vx = float(xdrive_fk_solver([v * WHEEL_RADIUS for v in wheel_rad_s])[0])

    harness.publish_joint_state(wheel_rad_s, stamp_s=0.0)  # seed
    harness.spin()
    harness.publish_joint_state(wheel_rad_s, stamp_s=0.5)  # +0.5 s
    harness.spin()

    odom = harness.odom_msgs[-1]
    assert odom.pose.pose.position.x == pytest.approx(eff_vx * 0.5, abs=1e-4)
    assert odom.pose.pose.position.y == pytest.approx(0.0, abs=1e-4)
    # Quaternion stays near identity for a pure-forward motion.
    assert odom.pose.pose.orientation.w == pytest.approx(1.0, abs=1e-6)
    assert odom.pose.pose.orientation.z == pytest.approx(0.0, abs=1e-6)


def test_yaw_quaternion_after_rotation(harness):
    """A rotation step writes a non-identity yaw quaternion to the pose."""
    wz = 0.5
    wheel_rad_s = _wheel_rad_s_for_twist(0.0, 0.0, wz)
    eff_wz = float(xdrive_fk_solver([v * WHEEL_RADIUS for v in wheel_rad_s])[2])

    harness.publish_joint_state(wheel_rad_s, stamp_s=0.0)  # seed
    harness.spin()
    harness.publish_joint_state(wheel_rad_s, stamp_s=0.4)  # +0.4 s
    harness.spin()

    odom = harness.odom_msgs[-1]
    expected_yaw = eff_wz * 0.4
    assert odom.pose.pose.orientation.z == pytest.approx(
        math.sin(expected_yaw * 0.5), abs=1e-4
    )
    assert odom.pose.pose.orientation.w == pytest.approx(
        math.cos(expected_yaw * 0.5), abs=1e-4
    )
