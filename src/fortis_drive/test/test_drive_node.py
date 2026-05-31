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
against xdrive_ik_solver (with WHEEL_DIRECTION applied), and the plumbing
is verified by going through DDS.

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
from std_msgs.msg import Float64MultiArray, String

from fortis_comms.xdrive_kinematics import WHEEL_RADIUS, xdrive_ik_solver
from fortis_drive.drive_node import (
    CMD_VEL_FRAME_SIGN,
    CMD_VEL_TOPIC,
    MISSION_STATE_TOPIC,
    WHEEL_CONTROLLER_COMMAND_TOPIC,
    WHEEL_DIRECTION,
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
#: Kept < CMD_VEL_TIMEOUT_S so a single drive+spin doesn't trip the
#: dead-man watchdog mid-assertion.
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

    def __init__(self, cmd_vel_timeout_s: float = 100.0) -> None:
        # Large watchdog timeout by default so the gating tests are immune to
        # the dead-man stop firing mid-assertion on a slow runner. The
        # watchdog test overrides this with a small value.
        self.node = DriveNode(cmd_vel_timeout_s=cmd_vel_timeout_s)
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
        # Float64MultiArray traffic on the ros2_control-bound topic.
        # drive_node publishes here on BOTH accepted and rejected
        # /cmd_vel; tests inspect the most recent entry.
        self.controller_msgs: list[Float64MultiArray] = []

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
        self.helper.create_subscription(
            Float64MultiArray,
            WHEEL_CONTROLLER_COMMAND_TOPIC,
            self.controller_msgs.append,
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

    Mirrors drive_node's full pipeline: CMD_VEL_FRAME_SIGN (chassis front
    convention) applied to the Twist, then xdrive_ik_solver, then / WHEEL_RADIUS
    (m/s -> rad/s), then * WHEEL_DIRECTION (motor mounting). Asserting against
    this is the contract: same inputs in, same signed outputs out.
    """
    linear = xdrive_ik_solver(
        vx * CMD_VEL_FRAME_SIGN[0],
        vy * CMD_VEL_FRAME_SIGN[1],
        wz * CMD_VEL_FRAME_SIGN[2],
    )
    return [
        float(linear[i]) / WHEEL_RADIUS * WHEEL_DIRECTION[i] for i in range(4)
    ]


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
    assert len(harness.controller_msgs) >= 1, \
        "expected at least one /wheel_velocity_controller/commands message"

    expected = _expected_wheel_speeds(0.5, 0.0, 0.0)
    msg = harness.wheel_msgs[-1]
    assert msg.fl == pytest.approx(expected[0], abs=1e-6)
    assert msg.fr == pytest.approx(expected[1], abs=1e-6)
    assert msg.rl == pytest.approx(expected[2], abs=1e-6)
    assert msg.rr == pytest.approx(expected[3], abs=1e-6)

    # Controller-bound Float64MultiArray must carry the same four signed
    # values in [fl, fr, rl, rr] order (WHEEL_DIRECTION applied).
    arr = harness.controller_msgs[-1]
    assert list(arr.data) == pytest.approx(expected, abs=1e-6), \
        "controller array must carry [fl, fr, rl, rr] in that order"


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
    assert msg.rl == 0.0
    assert msg.rr == 0.0

    # Controller path must also receive an explicit zero, not silence.
    # The motor controller (downstream) must never coast at the last
    # accepted setpoint on rejection.
    assert len(harness.controller_msgs) >= 1, \
        "rejected /cmd_vel must publish zeros into the controller topic"
    arr = harness.controller_msgs[-1]
    assert list(arr.data) == [0.0, 0.0, 0.0, 0.0]


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
    assert msg.rl == pytest.approx(expected[2], abs=1e-6)
    assert msg.rr == pytest.approx(expected[3], abs=1e-6)


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
    # NOTE: we no longer assert zeros stayed constant across IDLE->ORBIT.
    # The /cmd_vel dead-man watchdog may legitimately emit a zero if the
    # gap between commands (e.g. the publish_state poll) exceeds
    # CMD_VEL_TIMEOUT_S. The wheel-count growth is the gate signal.

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
    assert len(harness.controller_msgs) >= 1, \
        "rejection in the no-state window must also publish zeros to controller"
    assert list(harness.controller_msgs[-1].data) == [0.0, 0.0, 0.0, 0.0]


def test_controller_array_ordering_matches_kinematics(harness):
    """
    The controller_array order is the contract with fortis_control's YAML.

    The yaml's `joints: [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint,
    rr_wheel_joint]` line is the source of truth for which index of
    Float64MultiArray.data drives which physical wheel. If the order in
    drive_node._wheel_command_to_controller_array changes, this test
    fires before a bench session swaps two motors.

    Asymmetric Twist (vy != 0) so the four wheel speeds are all
    different and order-sensitive. Expected values include WHEEL_DIRECTION.
    """
    harness.publish_state("ORBIT")
    harness.spin()
    harness.publish_twist(vx=0.1, vy=0.2, wz=0.3)
    harness.spin()

    expected = _expected_wheel_speeds(0.1, 0.2, 0.3)
    arr = harness.controller_msgs[-1]
    assert len(arr.data) == 4
    # Pairwise: index 0 = FL, 1 = FR, 2 = RL, 3 = RR — same ordering
    # used by xdrive_kinematics, the WheelVelocities message, and the
    # JointGroupVelocityController joints list.
    assert arr.data[0] == pytest.approx(expected[0], abs=1e-6)
    assert arr.data[1] == pytest.approx(expected[1], abs=1e-6)
    assert arr.data[2] == pytest.approx(expected[2], abs=1e-6)
    assert arr.data[3] == pytest.approx(expected[3], abs=1e-6)


def test_wheel_direction_reverses_right_side(harness):
    """
    Right-side wheels (FR, RR) carry the inverse command of the left side.

    WHEEL_DIRECTION inverts the mirror-mounted right side, so for a pure
    forward/back command each right wheel carries the opposite sign of its left
    counterpart at equal magnitude. We assert the pairing (and a match to the
    canonical pipeline) rather than a fixed sign, so the test is independent of
    the CMD_VEL_FRAME_SIGN forward convention. Regression guard for the
    wrong-direction bug found on the first real drive.
    """
    harness.publish_state("ORBIT")
    harness.spin()
    harness.publish_twist(vx=0.5)  # pure forward/back command
    harness.spin()

    assert len(harness.wheel_msgs) >= 1
    msg = harness.wheel_msgs[-1]
    # Matches the full pipeline (frame sign + IK + WHEEL_DIRECTION).
    expected = _expected_wheel_speeds(0.5, 0.0, 0.0)
    assert msg.fl == pytest.approx(expected[0], abs=1e-6)
    assert msg.fr == pytest.approx(expected[1], abs=1e-6)
    assert msg.rl == pytest.approx(expected[2], abs=1e-6)
    assert msg.rr == pytest.approx(expected[3], abs=1e-6)
    # Right side is the per-wheel inverse of the left (equal mag, opposite sign).
    assert msg.fl == pytest.approx(-msg.fr, abs=1e-6)
    assert msg.rl == pytest.approx(-msg.rr, abs=1e-6)
    assert abs(msg.fl) > 0.0
    assert WHEEL_DIRECTION == (1.0, -1.0, 1.0, -1.0)


def test_cmd_vel_watchdog_stops_when_commands_go_stale(rclpy_session):
    """
    Dead-man: if /cmd_vel stops arriving, the watchdog commands zeros.

    Reproduces the "wheels keep spinning after the operator releases the
    teleop pad" bug: the teleop client stops publishing without sending a
    zero, so without a watchdog the controller latches the last command.

    Uses a short, explicit watchdog timeout; the default harness uses a
    large one so the gating tests aren't coupled to machine speed.
    """
    h = _Harness(cmd_vel_timeout_s=0.2)
    h.spin(SPIN_DURATION_S)  # discovery
    try:
        h.publish_state("ORBIT")
        h.spin()
        h.publish_twist(vx=0.5)
        h.spin()
        assert len(h.wheel_msgs) >= 1, "expected motion in ORBIT"

        # Stop publishing /cmd_vel and let the (short) watchdog time out.
        # After the stale window the LAST command to the controller must be
        # a zero, regardless of exactly when the watchdog fired.
        h.spin(0.5)

        assert list(h.controller_msgs[-1].data) == [0.0, 0.0, 0.0, 0.0], \
            "stale /cmd_vel must leave the controller commanded to zero"
        assert any(
            (m.fl, m.fr, m.rl, m.rr) == (0.0, 0.0, 0.0, 0.0)
            for m in h.zero_msgs
        ), "watchdog must publish an explicit zero"
    finally:
        h.cleanup()


def test_stop_transition_zeros_immediately(harness):
    """
    Leaving a driving state commands zeros at once, without a new /cmd_vel.

    Reproduces the "STOP doesn't stop the wheels" bug: on ORBIT->IDLE no
    /cmd_vel is in flight (operator released), so the rejection-zero never
    fired. drive_node must zero on the state transition itself.
    """
    harness.publish_state("ORBIT")
    harness.spin()
    harness.publish_twist(vx=0.5)
    harness.spin()
    zeros_before = len(harness.zero_msgs)

    # Transition ORBIT -> IDLE with NO /cmd_vel in flight.
    harness.publish_state("IDLE")
    harness.spin()

    assert len(harness.zero_msgs) > zeros_before, \
        "gate-close must publish zeros immediately, not wait for next /cmd_vel"
    assert list(harness.controller_msgs[-1].data) == [0.0, 0.0, 0.0, 0.0]
