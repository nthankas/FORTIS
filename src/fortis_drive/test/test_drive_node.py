"""
Tests for fortis_drive.drive_node.

Each test stands up a real DriveNode plus a helper node that publishes the
inputs (/cmd_vel, /fortis/mission_state) and subscribes to the outputs
(/fortis/drive/wheel_velocities, /fortis/drive/zero_velocities). We then
spin both nodes for a bounded duration and assert on what was published.

Why a real ROS round trip
-------------------------
The whole point of this node is the QoS contract with /fortis/mission_state
(latched TRANSIENT_LOCAL+RELIABLE) and the gating semantics that depend on
which messages reach which callbacks in which order. A unit test that
mocks rclpy and calls _on_cmd_vel directly would test the formula but not
the ROS plumbing. We do both: the formula is verified by comparing
against xdrive_ik_solver, and the plumbing is verified by going through
DDS.

Run with:
    cd /workspace
    colcon build --packages-select fortis_msgs fortis_drive
    source install/setup.bash
    python3 -m pytest src/fortis_drive/test/test_drive_node.py -v
"""

from __future__ import annotations

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from fortis_comms.xdrive_kinematics import WHEEL_RADIUS, xdrive_ik_solver
from fortis_drive.drive_node import (
    CMD_VEL_TOPIC,
    MISSION_STATE_TOPIC,
    WHEEL_VELOCITIES_TOPIC,
    ZERO_VELOCITIES_TOPIC,
    DriveNode,
)
from fortis_msgs.msg import WheelVelocities


# --- Constants used by the harness ------------------------------------------

#: Wall-clock time spent draining the ROS event loop between actions.
#: 300 ms is enough for DDS discovery between two in-process nodes on
#: every machine we have tried, and for a single message to round-trip
#: through both subscriptions. Increase if tests flake on a slower box.
SPIN_DURATION_S: float = 0.3

#: Per-spin_once timeout. Small enough that the harness drains promptly
#: after each callback rather than blocking for a full slice.
SPIN_ONCE_TIMEOUT_S: float = 0.02

#: Hard ceiling on how long publish_state will poll for the node to
#: observe a freshly-published state. Anything > a couple seconds
#: indicates DDS discovery has actually failed, not just that we are
#: under load.
STATE_PROPAGATION_TIMEOUT_S: float = 2.0

#: Hard ceiling on how long publish_twist will poll for the node's
#: wheel-output publisher to deliver a message back to the helper. Same
#: rationale as STATE_PROPAGATION_TIMEOUT_S: cmd_vel and wheel_velocities
#: discovery between two in-process nodes is fast on the happy path, but
#: parallel colcon test execution on a loaded box can stall it past the
#: 300 ms SPIN_DURATION_S used everywhere else.
WHEEL_OUTPUT_TIMEOUT_S: float = 2.0


# --- Fixtures ---------------------------------------------------------------


@pytest.fixture
def rclpy_session():
    """
    Function-scoped rclpy.init / shutdown.

    A fresh DDS participant per test prevents the previous test's
    TRANSIENT_LOCAL mission_state writer from delivering a stale sample
    to the next test's drive_node subscriber after the publisher has
    already been destroyed. fortis_arm's gating tests hit this same
    flake under module scope; we use the same fix here proactively
    before the parametrized surface grows (planned ODrive ROS 2 driver
    wrap, motor-controller stubs, additional QoS profiles).

    Cost is ~30-50 ms per test for context init + shutdown. Multi-init
    in the same process is supported by current rclpy (the older
    "historically flaky" warning predates fixes that have since landed
    upstream).
    """
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness(rclpy_session):
    """Per-test harness: a DriveNode under test plus an input/output helper."""
    h = _Harness()
    # Drain once after construction so DDS discovery between the two nodes
    # completes before the test starts publishing. Without this, the very
    # first published Twist can be dropped because the subscription has
    # not yet been matched.
    h.spin(SPIN_DURATION_S)
    try:
        yield h
    finally:
        h.cleanup()


# --- Harness ----------------------------------------------------------------


class _Harness:
    """
    A DriveNode under test plus a helper node for inputs and outputs.

    The helper is a separate Node (not a part of DriveNode) so the tests
    exercise the same DDS path that real publishers and subscribers will
    take in production. Captured messages are stored as plain Python
    lists; tests assert on length and field values.
    """

    def __init__(self) -> None:
        self.node = DriveNode()
        self.helper: Node = rclpy.create_node("drive_node_test_helper")

        # Match the latched QoS of mission_state_node so the state
        # message is delivered to DriveNode the same way it would be in
        # production. Mismatched QoS would cause DDS to refuse the match
        # and the test would silently send nothing.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.state_pub = self.helper.create_publisher(
            String, MISSION_STATE_TOPIC, latched_qos
        )
        self.cmd_pub = self.helper.create_publisher(
            Twist, CMD_VEL_TOPIC, 10
        )

        self.wheel_msgs: list[WheelVelocities] = []
        self.zero_msgs: list[WheelVelocities] = []

        self.helper.create_subscription(
            WheelVelocities,
            WHEEL_VELOCITIES_TOPIC,
            self.wheel_msgs.append,
            10,
        )
        self.helper.create_subscription(
            WheelVelocities,
            ZERO_VELOCITIES_TOPIC,
            self.zero_msgs.append,
            10,
        )

    def spin(self, duration_s: float = SPIN_DURATION_S) -> None:
        """Drain the event loop on both nodes for the given wall-clock window."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def publish_state(self, state: str) -> None:
        """
        Publish a String on /fortis/mission_state and wait for the node to see it.

        We poll the node's cached _current_state instead of just sleeping
        a fixed window. With colcon running multiple package test
        processes in parallel, DDS discovery for the latched state topic
        can take long enough that a fixed-duration spin races the
        subsequent /cmd_vel publish and the test sees an empty
        wheel_msgs list. Polling the actual state observation is the
        only timing-stable assertion.
        """
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        end = time.monotonic() + STATE_PROPAGATION_TIMEOUT_S
        while self.node._current_state != state and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        assert self.node._current_state == state, \
            f"state {state!r} did not propagate to node within " \
            f"{STATE_PROPAGATION_TIMEOUT_S}s"

    def publish_twist(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        wz: float = 0.0,
    ) -> None:
        """
        Publish a Twist on /cmd_vel and wait for the node to emit something.

        Returns once either wheel_msgs or zero_msgs has grown by at
        least one entry, or after WHEEL_OUTPUT_TIMEOUT_S. The drive
        node always emits exactly one of the two on every accepted or
        rejected /cmd_vel, so polling for the union is the right
        signal. As with publish_state, fixed-duration spins race DDS
        discovery under parallel test load.
        """
        initial = len(self.wheel_msgs) + len(self.zero_msgs)
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.cmd_pub.publish(msg)
        end = time.monotonic() + WHEEL_OUTPUT_TIMEOUT_S
        while (len(self.wheel_msgs) + len(self.zero_msgs)) == initial \
                and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def cleanup(self) -> None:
        """Tear down both nodes. Safe to call once after the test finishes."""
        # Destroy helper (publisher side) first so its writer-goodbye is
        # in flight before the subscriber goes away. Drain briefly so DDS
        # processes the goodbye inside this participant before
        # rclpy.shutdown collapses it -- without this, function-scoped
        # shutdown can leave a half-destroyed writer visible to the next
        # test's participant on some DDS implementations.
        self.helper.destroy_node()
        self.node.destroy_node()
        time.sleep(0.05)


# --- Reference computation --------------------------------------------------


def _expected_wheel_speeds(vx: float, vy: float, wz: float) -> list[float]:
    """
    Compute reference wheel speeds for a given Twist (the assertion target).

    Computed via the same xdrive_ik_solver that the node calls so the test
    verifies "the kinematics are being applied correctly" rather than
    re-deriving the H matrix here (which would just duplicate the
    library and test the duplicate). Asserting against this is the
    contract: same inputs in, same outputs out, with the m/s -> rad/s
    conversion applied.
    """
    linear = xdrive_ik_solver(vx, vy, wz)
    return [float(linear[i]) / WHEEL_RADIUS for i in range(4)]


# --- Tests ------------------------------------------------------------------


def test_orbit_accepts_cmd_vel_and_publishes_correct_wheel_velocities(harness):
    """ORBIT is a permitted state: cmd_vel is honoured and IK output published."""
    harness.publish_state("ORBIT")
    harness.spin()
    harness.publish_twist(vx=0.5, vy=0.0, wz=0.0)
    harness.spin()

    assert len(harness.wheel_msgs) >= 1, \
        "expected at least one WheelVelocities message in ORBIT"
    assert len(harness.zero_msgs) == 0, \
        "no zero_velocities should be published when motion is permitted"

    expected = _expected_wheel_speeds(0.5, 0.0, 0.0)
    msg = harness.wheel_msgs[-1]
    assert msg.fl == pytest.approx(expected[0], abs=1e-6)
    assert msg.fr == pytest.approx(expected[1], abs=1e-6)
    assert msg.bl == pytest.approx(expected[2], abs=1e-6)
    assert msg.br == pytest.approx(expected[3], abs=1e-6)


def test_idle_rejects_cmd_vel_and_publishes_zeros(harness):
    """
    IDLE is not permitted: cmd_vel is rejected and zeros are published.

    The throttled warning is emitted on the same code path that publishes
    the zeros; we verify the observable output (the message) and rely on
    code review to verify the warning text. Capturing rcutils log output
    in pytest is non-trivial and not worth the test machinery cost.
    """
    harness.publish_state("IDLE")
    harness.spin()
    harness.publish_twist(vx=0.5, vy=0.0, wz=0.0)
    harness.spin()

    assert len(harness.wheel_msgs) == 0, \
        "no wheel_velocities should be published in IDLE"
    assert len(harness.zero_msgs) >= 1, \
        "expected at least one zero_velocities message in IDLE"

    msg = harness.zero_msgs[-1]
    assert msg.fl == 0.0
    assert msg.fr == 0.0
    assert msg.bl == 0.0
    assert msg.br == 0.0


def test_return_home_accepts_cmd_vel(harness):
    """RETURN_HOME is also a permitted state for drive motion."""
    harness.publish_state("RETURN_HOME")
    harness.spin()
    harness.publish_twist(vx=0.3, vy=0.2, wz=0.0)
    harness.spin()

    assert len(harness.wheel_msgs) >= 1, \
        "expected at least one WheelVelocities message in RETURN_HOME"
    assert len(harness.zero_msgs) == 0, \
        "no zero_velocities should be published when motion is permitted"

    expected = _expected_wheel_speeds(0.3, 0.2, 0.0)
    msg = harness.wheel_msgs[-1]
    assert msg.fl == pytest.approx(expected[0], abs=1e-6)
    assert msg.fr == pytest.approx(expected[1], abs=1e-6)
    assert msg.bl == pytest.approx(expected[2], abs=1e-6)
    assert msg.br == pytest.approx(expected[3], abs=1e-6)


def test_state_transitions_gate_motion_correctly(harness):
    """A round trip IDLE -> ORBIT -> IDLE flips the gate each time."""
    # IDLE: rejects.
    harness.publish_state("IDLE")
    harness.spin()
    harness.publish_twist(vx=0.4)
    harness.spin()
    wheels_after_idle = len(harness.wheel_msgs)
    zeros_after_idle = len(harness.zero_msgs)
    assert wheels_after_idle == 0
    assert zeros_after_idle >= 1

    # IDLE -> ORBIT: accepts.
    harness.publish_state("ORBIT")
    harness.spin()
    harness.publish_twist(vx=0.4)
    harness.spin()
    wheels_after_orbit = len(harness.wheel_msgs)
    zeros_after_orbit = len(harness.zero_msgs)
    assert wheels_after_orbit > wheels_after_idle, \
        "ORBIT should have produced new wheel_velocities"
    assert zeros_after_orbit == zeros_after_idle, \
        "ORBIT should not produce additional zero_velocities"

    # ORBIT -> IDLE: rejects again.
    harness.publish_state("IDLE")
    harness.spin()
    harness.publish_twist(vx=0.4)
    harness.spin()
    assert len(harness.wheel_msgs) == wheels_after_orbit, \
        "returning to IDLE should stop producing wheel_velocities"
    assert len(harness.zero_msgs) > zeros_after_orbit, \
        "returning to IDLE should produce a new zero_velocities"


def test_no_state_received_rejects_cmd_vel(harness):
    """
    If /fortis/mission_state has not arrived yet, motion is rejected.

    This is the bring-up race: cmd_vel can be flowing before the state
    publisher has announced anything. The drive must not silently accept
    in that window.
    """
    # No publish_state(...) call. Just publish a twist and spin.
    harness.publish_twist(vx=0.5)
    harness.spin()

    assert len(harness.wheel_msgs) == 0, \
        "must not publish wheel_velocities before any state has been received"
    assert len(harness.zero_msgs) >= 1, \
        "must publish zero_velocities to make the rejection explicit"
