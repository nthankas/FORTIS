"""
Tests for fortis_drive.heading_hold_node.

Two layers, mirroring how the node is structured:

  * The control LAW is exercised through HeadingController directly -- a pure
    class with no rclpy dependency, so the PID behaviour (hold, error
    direction, wrap-around, anti-windup, output clamp, turn pass-through) is
    tested deterministically without DDS timing. This is the bulk of the
    coverage and the part most likely to regress on a retune.

  * The ROS PLUMBING (Vx/Vy pass-through, omega replaced on the output Twist,
    output emitted on /odometry/filtered) is exercised through a real
    HeadingHoldNode + helper node round trip, the same harness style as
    test_drive_node.py.

ROS_DOMAIN_ID is pinned to 91 by test/conftest.py (the fortis_drive test
domain); we reuse it, we do not set our own.

Run with:
    cd /workspace
    colcon build --packages-select fortis_drive
    source install/setup.bash
    python3 -m pytest src/fortis_drive/test/test_heading_hold_node.py -v
"""

from __future__ import annotations

import math
import time

import pytest
import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from fortis_drive.heading_hold_node import (
    CMD_VEL_HEADING_TOPIC,
    CMD_VEL_TOPIC,
    ODOM_FILTERED_TOPIC,
    HeadingController,
    HeadingHoldGains,
    HeadingHoldNode,
    wrap_to_pi,
    yaw_from_quaternion,
)


# --- Pure helpers -----------------------------------------------------------


def test_wrap_to_pi_wraps_into_range():
    """wrap_to_pi maps any angle into (-pi, pi]."""
    assert wrap_to_pi(0.0) == pytest.approx(0.0)
    assert wrap_to_pi(math.pi) == pytest.approx(math.pi)
    # Just past +pi wraps to just past -pi.
    assert wrap_to_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1, abs=1e-9)
    # 3*pi/2 -> -pi/2.
    assert wrap_to_pi(1.5 * math.pi) == pytest.approx(-0.5 * math.pi)
    assert wrap_to_pi(-1.5 * math.pi) == pytest.approx(0.5 * math.pi)


def test_yaw_from_quaternion_recovers_planar_heading():
    """A pure +Z rotation quaternion round-trips back to its yaw."""
    for yaw in (-2.0, -0.3, 0.0, 0.7, 1.5):
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        assert yaw_from_quaternion(q) == pytest.approx(yaw, abs=1e-9)


# --- Control law: HeadingController -----------------------------------------


def _p_only_gains(**kw) -> HeadingHoldGains:
    """P-only gains (ki=kd=0) so the proportional behaviour is isolated."""
    base = dict(kp=2.0, ki=0.0, kd=0.0, max_omega=1.5,
                turn_deadband=0.05, i_clamp=0.5, yaw_sign=1.0)
    base.update(kw)
    return HeadingHoldGains(**base)


def test_holds_fixed_heading_zero_error_zero_omega():
    """On target with no operator turn, output omega is ~0 (it holds)."""
    ctrl = HeadingController(_p_only_gains())
    # First sample seeds the target to the measured yaw.
    out = ctrl.update(measured_yaw=0.5, omega_cmd=0.0, dt=0.0)
    assert out == pytest.approx(0.0)
    assert ctrl.target_yaw == pytest.approx(0.5)
    # Still on target on the next step -> still zero.
    out = ctrl.update(measured_yaw=0.5, omega_cmd=0.0, dt=0.1)
    assert out == pytest.approx(0.0)


def test_error_drives_omega_correct_direction():
    """A heading BEHIND target (measured < target) needs positive (CCW) omega.

    With yaw_sign=+1 (REP-103, CCW-positive), error = target - measured. If
    the robot has fallen short of the target heading, the correction must be
    a positive yaw rate to rotate it CCW up to target -- and the reverse for
    an overshoot.
    """
    ctrl = HeadingController(_p_only_gains())
    ctrl.update(measured_yaw=1.0, omega_cmd=0.0, dt=0.0)  # target := 1.0
    # Robot drifted to 0.8 (short of target): correct CCW (+).
    out = ctrl.update(measured_yaw=0.8, omega_cmd=0.0, dt=0.1)
    assert out > 0.0
    assert out == pytest.approx(2.0 * 0.2)  # kp * error
    # Robot drifted to 1.2 (past target): correct CW (-).
    out = ctrl.update(measured_yaw=1.2, omega_cmd=0.0, dt=0.1)
    assert out < 0.0


def test_yaw_sign_inverts_correction_direction():
    """yaw_sign=-1 flips the correction sense (gyro reads CW-positive)."""
    ctrl = HeadingController(_p_only_gains(yaw_sign=-1.0))
    ctrl.update(measured_yaw=1.0, omega_cmd=0.0, dt=0.0)  # target := 1.0
    out = ctrl.update(measured_yaw=0.8, omega_cmd=0.0, dt=0.1)
    # Same geometry as the +1 case above, but the sign is inverted.
    assert out < 0.0


def test_wrap_around_at_pi_takes_short_way():
    """Target near +pi, measured near -pi: correct the SHORT way, not 2*pi.

    Target +3.0 rad, measured -3.0 rad. The naive difference is 6.0 rad
    (~full turn); the wrapped error is (6.0 - 2*pi) ~= -0.283 rad, so the
    correction is small and NEGATIVE (CW the short way across the +/-pi seam),
    not a near-full-speed spin the long way. The magnitude, not the sign, is
    the point: |out| stays small instead of saturating.
    """
    ctrl = HeadingController(_p_only_gains())
    ctrl.update(measured_yaw=3.0, omega_cmd=0.0, dt=0.0)  # target := 3.0
    out = ctrl.update(measured_yaw=-3.0, omega_cmd=0.0, dt=0.1)
    expected_error = wrap_to_pi(3.0 - (-3.0))  # ~ -0.2832 (the short way)
    assert expected_error == pytest.approx(6.0 - 2 * math.pi)
    assert out == pytest.approx(2.0 * expected_error)
    assert -1.0 < out < 0.0  # small CW correction, not the max_omega spin


def test_output_clamps_to_max_omega():
    """A large error saturates the output at +/- max_omega, both signs."""
    ctrl = HeadingController(_p_only_gains(kp=10.0, max_omega=1.5))
    ctrl.update(measured_yaw=0.0, omega_cmd=0.0, dt=0.0)  # target := 0.0
    # Big positive error (target ahead): kp*error would be huge -> clamp +.
    out = ctrl.update(measured_yaw=-1.0, omega_cmd=0.0, dt=0.1)
    assert out == pytest.approx(1.5)
    # Big negative error -> clamp -.
    out = ctrl.update(measured_yaw=1.0, omega_cmd=0.0, dt=0.1)
    assert out == pytest.approx(-1.5)


def test_integral_anti_windup_clamps_the_i_term():
    """A standing error does not let the integral term grow past i_clamp.

    With ki>0 and a constant non-zero error, the raw integral keeps
    accumulating; the controller must clamp the integral TERM (ki*integral)
    to +/- i_clamp so it cannot wind up unbounded. We drive many steps and
    assert the i_term contribution never exceeds the clamp.
    """
    gains = HeadingHoldGains(kp=0.0, ki=1.0, kd=0.0, max_omega=100.0,
                             turn_deadband=0.05, i_clamp=0.3, yaw_sign=1.0)
    ctrl = HeadingController(gains)
    ctrl.update(measured_yaw=0.0, omega_cmd=0.0, dt=0.0)  # target := 0.0
    last = 0.0
    for _ in range(100):
        # Constant +0.5 rad error each step (measured behind target).
        last = ctrl.update(measured_yaw=-0.5, omega_cmd=0.0, dt=0.1)
    # kp=kd=0, so the entire output is the integral term, which must be
    # clamped to i_clamp and NOT keep growing.
    assert last == pytest.approx(0.3)
    assert ctrl._integral == pytest.approx(0.3 / 1.0)  # back-calculated


def test_turning_passes_omega_through_and_rearms_target():
    """While |omega_cmd| > deadband, pass it through and re-arm the target.

    The operator's omega is emitted verbatim, and target_yaw tracks the
    current measured yaw so that when they release, the hold begins at the
    new heading -- not a stale one.
    """
    ctrl = HeadingController(_p_only_gains(turn_deadband=0.05))
    ctrl.update(measured_yaw=0.0, omega_cmd=0.0, dt=0.0)  # target := 0.0
    # Operator turns hard; measured yaw is now 1.2.
    out = ctrl.update(measured_yaw=1.2, omega_cmd=0.8, dt=0.1)
    assert out == pytest.approx(0.8)            # passed through verbatim
    assert ctrl.target_yaw == pytest.approx(1.2)  # re-armed to current yaw
    # Operator releases (omega below deadband): now HOLD the new heading.
    out = ctrl.update(measured_yaw=1.2, omega_cmd=0.0, dt=0.1)
    assert out == pytest.approx(0.0)            # already at the re-armed target


def test_below_deadband_omega_does_not_pass_through():
    """A tiny omega_cmd (< deadband) is treated as 'not turning' -> HOLD."""
    ctrl = HeadingController(_p_only_gains(turn_deadband=0.05))
    ctrl.update(measured_yaw=0.0, omega_cmd=0.0, dt=0.0)  # target := 0.0
    # omega_cmd 0.01 < 0.05 deadband: ignored, controller holds (zero error).
    out = ctrl.update(measured_yaw=0.0, omega_cmd=0.01, dt=0.1)
    assert out == pytest.approx(0.0)
    assert out != pytest.approx(0.01)


# --- ROS plumbing round trip ------------------------------------------------


SPIN_DURATION_S: float = 0.3
SPIN_ONCE_TIMEOUT_S: float = 0.02
OUTPUT_TIMEOUT_S: float = 2.0


def _odom_msg(node: Node, yaw: float, stamp_s: float) -> Odometry:
    """Build an /odometry/filtered sample carrying a planar yaw at a stamp."""
    msg = Odometry()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
    msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
    msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
    return msg


class _Harness:
    """A HeadingHoldNode under test plus a helper that feeds it and listens."""

    def __init__(self) -> None:
        self.node = HeadingHoldNode()
        self.helper: Node = rclpy.create_node("heading_hold_test_helper")
        self.cmd_pub = self.helper.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.odom_pub = self.helper.create_publisher(
            Odometry, ODOM_FILTERED_TOPIC, 10
        )
        self.out_msgs: list[Twist] = []
        self.helper.create_subscription(
            Twist, CMD_VEL_HEADING_TOPIC, self.out_msgs.append, 10
        )

    def spin(self, duration_s: float = SPIN_DURATION_S) -> None:
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def publish_cmd(self, vx=0.0, vy=0.0, wz=0.0) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.cmd_pub.publish(msg)

    def publish_odom_and_wait(self, yaw: float, stamp_s: float) -> None:
        """Publish an odometry sample and wait for one output Twist."""
        initial = len(self.out_msgs)
        self.odom_pub.publish(_odom_msg(self.helper, yaw, stamp_s))
        end = time.monotonic() + OUTPUT_TIMEOUT_S
        while len(self.out_msgs) == initial and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def cleanup(self) -> None:
        self.helper.destroy_node()
        self.node.destroy_node()
        time.sleep(0.05)


@pytest.fixture
def rclpy_session():
    """Function-scoped rclpy.init / shutdown (one DDS participant per test)."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness(rclpy_session):
    """A HeadingHoldNode under test plus an input/output helper node."""
    h = _Harness()
    h.spin(SPIN_DURATION_S)  # let DDS discovery settle before publishing
    try:
        yield h
    finally:
        h.cleanup()


def test_node_passes_vx_vy_through_unchanged(harness):
    """Vx and Vy on the output Twist equal the operator's, always."""
    harness.publish_cmd(vx=0.4, vy=-0.2, wz=0.0)
    harness.spin()
    # Two odom samples (the first only seeds the target/stamp).
    harness.publish_odom_and_wait(yaw=0.0, stamp_s=1.0)
    harness.publish_odom_and_wait(yaw=0.0, stamp_s=1.1)

    assert len(harness.out_msgs) >= 1
    out = harness.out_msgs[-1]
    assert out.linear.x == pytest.approx(0.4)
    assert out.linear.y == pytest.approx(-0.2)


def test_node_passes_omega_through_while_turning(harness):
    """With omega_cmd above the deadband, the output yaw rate is passed through."""
    harness.publish_cmd(vx=0.1, vy=0.0, wz=0.6)  # actively turning
    harness.spin()
    harness.publish_odom_and_wait(yaw=0.0, stamp_s=1.0)
    harness.publish_odom_and_wait(yaw=0.3, stamp_s=1.1)

    assert len(harness.out_msgs) >= 1
    out = harness.out_msgs[-1]
    assert out.angular.z == pytest.approx(0.6)
    assert out.linear.x == pytest.approx(0.1)  # Vx still passes through


def test_node_holds_heading_when_not_turning(harness):
    """With no operator turn and a heading drift, output omega corrects it.

    Seed the target at yaw 0, then report a drifted yaw with zero omega_cmd:
    the node must emit a non-zero corrective yaw rate (default kp>0) in the
    direction that returns to target.
    """
    harness.publish_cmd(vx=0.0, vy=0.0, wz=0.0)
    harness.spin()
    harness.publish_odom_and_wait(yaw=0.0, stamp_s=1.0)   # target := 0.0
    harness.publish_odom_and_wait(yaw=-0.2, stamp_s=1.1)  # drifted behind

    assert len(harness.out_msgs) >= 1
    out = harness.out_msgs[-1]
    # measured behind target (yaw_sign +1) -> positive (CCW) correction.
    assert out.angular.z > 0.0
