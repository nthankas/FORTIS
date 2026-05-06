"""
State-gating tests for fortis_arm.arm_controller_node.

Drives the mission-state topic through the full FSM and asserts that
the open_gripper / close_gripper services are honoured only inside
ALLOWED_ARM_STATES. Mirrors the gating structure of
fortis_drive/test/test_drive_node.py.

Note: the move_to_pose action-server scaffold was retired and moved to
``legacy/deprecated_arm_action/``. Replaced by ros2_control + standard
ROS 2 packages. Action-gating tests have been removed.
"""

from __future__ import annotations

import time

import pytest
import rclpy
from fortis_arm.arm_controller_node import (
    ALLOWED_ARM_STATES,
    CLOSE_GRIPPER_SERVICE,
    MISSION_STATE_TOPIC,
    OPEN_GRIPPER_SERVICE,
    ArmControllerNode,
)
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


# --- Constants ---------------------------------------------------------------

#: Wall-clock window used for spin-draining between actions. 300 ms is
#: enough for DDS discovery between two in-process nodes on every box
#: we have tried, plus one round-trip of the message under test.
SPIN_DURATION_S: float = 0.3

#: Per-spin_once timeout. Small enough to drain promptly after each
#: callback rather than blocking for a full slice.
SPIN_ONCE_TIMEOUT_S: float = 0.02

#: Per-future timeout. The service stub resolves immediately, so anything
#: > a few hundred ms means something is wrong (DDS not discovered,
#: callback not registered, etc.).
FUTURE_TIMEOUT_S: float = 2.0

#: Every state defined in fortis_safety.mission_state_machine.State.
#: Listed explicitly here rather than imported so this test does not
#: depend on fortis_safety -- the contract is the topic and its String
#: payload, not the Python enum.
ALL_STATES: list[str] = [
    "IDLE",
    "ORBIT",
    "TARGETING",
    "ARM_AT_VIEW",
    "INSPECT",
    "PICK",
    "HOLDING",
    "RETURN_HOME",
    "FAULT",
]

DISALLOWED_STATES: list[str] = [s for s in ALL_STATES if s not in ALLOWED_ARM_STATES]


# --- Fixtures ----------------------------------------------------------------


@pytest.fixture
def rclpy_session():
    """
    Function-scoped rclpy.init / shutdown.

    A fresh DDS participant per test is the only way to guarantee that a
    prior test's TRANSIENT_LOCAL writers cannot deliver stale samples to
    the next test's subscribers.
    """
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness(rclpy_session):
    """Per-test harness: an ArmControllerNode plus an input/output helper."""
    h = _Harness()
    # Drain once so DDS discovery (state pub/sub, service server/client)
    # is complete before the test acts.
    h.spin(SPIN_DURATION_S)
    try:
        yield h
    finally:
        h.cleanup()


# --- Harness ----------------------------------------------------------------


class _Harness:
    """
    An ArmControllerNode under test plus a helper node for inputs and outputs.

    The helper is a separate Node (not a member of ArmControllerNode) so
    the tests exercise the same DDS paths that real publishers and
    clients would take in production. Mismatched QoS or a typo'd topic
    name shows up as a missing message, not a green test.
    """

    def __init__(self) -> None:
        self.node = ArmControllerNode()
        self.helper: Node = rclpy.create_node("arm_controller_test_helper")

        # Match the latched QoS of mission_state_node so the state
        # message reaches ArmControllerNode the same way it would in
        # production.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.state_pub = self.helper.create_publisher(
            String, MISSION_STATE_TOPIC, latched_qos
        )

        self.open_client = self.helper.create_client(
            Trigger, OPEN_GRIPPER_SERVICE
        )
        self.close_client = self.helper.create_client(
            Trigger, CLOSE_GRIPPER_SERVICE
        )

        # Wait for the service servers to come up before any test sends
        # a request. Otherwise the very first call_async races the
        # discovery handshake and times out.
        assert self.open_client.wait_for_service(timeout_sec=FUTURE_TIMEOUT_S), \
            "open_gripper service did not advertise within timeout"
        assert self.close_client.wait_for_service(timeout_sec=FUTURE_TIMEOUT_S), \
            "close_gripper service did not advertise within timeout"

    def spin(self, duration_s: float = SPIN_DURATION_S) -> None:
        """Drain the event loop on both nodes for the given wall-clock window."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    def publish_state(self, state: str) -> None:
        """
        Publish a state and wait for the node to observe it.

        We poll the node's cached _current_state rather than sleeping a
        fixed interval. The first call in a fresh test process otherwise
        races DDS matching of the state topic; a fixed sleep sometimes
        works and sometimes doesn't, which is the worst kind of test
        flake.
        """
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        end = time.monotonic() + FUTURE_TIMEOUT_S
        while self.node._current_state != state and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        assert self.node._current_state == state, \
            f"state {state!r} did not propagate to node within {FUTURE_TIMEOUT_S}s"

    def call_open_gripper(self) -> Trigger.Response:
        future = self.open_client.call_async(Trigger.Request())
        self._spin_until_complete(future)
        return future.result()

    def call_close_gripper(self) -> Trigger.Response:
        future = self.close_client.call_async(Trigger.Request())
        self._spin_until_complete(future)
        return future.result()

    def _spin_until_complete(self, future) -> None:
        """Drain both nodes until the future resolves or FUTURE_TIMEOUT_S elapses."""
        end = time.monotonic() + FUTURE_TIMEOUT_S
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            rclpy.spin_once(self.helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        assert future.done(), "future did not complete within timeout"

    def cleanup(self) -> None:
        # Order matters: destroy the helper (the publisher side) first so
        # its writer-goodbye is in flight before we tear down the
        # subscriber. Then drain briefly so DDS can process the goodbye
        # inside this participant before rclpy.shutdown collapses it.
        self.helper.destroy_node()
        self.node.destroy_node()
        time.sleep(0.05)


# --- Gripper-service gating -------------------------------------------------


@pytest.mark.parametrize("state", sorted(ALLOWED_ARM_STATES))
def test_open_gripper_in_allowed_states_returns_stub(harness, state):
    harness.publish_state(state)
    harness.spin()

    response = harness.call_open_gripper()
    assert response.success is False, \
        f"scaffold open_gripper must return success=False (stub) in state {state}"
    assert "gripper actuation not implemented" in response.message, \
        f"unexpected stub message in state {state}: {response.message!r}"


@pytest.mark.parametrize("state", sorted(DISALLOWED_STATES))
def test_open_gripper_rejected_in_disallowed_states(harness, state):
    harness.publish_state(state)
    harness.spin()

    response = harness.call_open_gripper()
    assert response.success is False, \
        f"open_gripper must return success=False in disallowed state {state}"
    assert "rejected" in response.message, \
        f"disallowed-state response must mention rejection; got {response.message!r}"


@pytest.mark.parametrize("state", sorted(ALLOWED_ARM_STATES))
def test_close_gripper_in_allowed_states_returns_stub(harness, state):
    harness.publish_state(state)
    harness.spin()

    response = harness.call_close_gripper()
    assert response.success is False
    assert "gripper actuation not implemented" in response.message


@pytest.mark.parametrize("state", sorted(DISALLOWED_STATES))
def test_close_gripper_rejected_in_disallowed_states(harness, state):
    harness.publish_state(state)
    harness.spin()

    response = harness.call_close_gripper()
    assert response.success is False
    assert "rejected" in response.message
