"""
State-gating tests for fortis_arm.arm_controller_node.

Drives the mission-state topic through the full FSM and asserts that
the move_to_pose action and the open_gripper / close_gripper services
are honoured only inside ALLOWED_ARM_STATES. Mirrors the gating
structure of fortis_drive/test/test_drive_node.py.
"""

from __future__ import annotations

import time

import pytest
import rclpy
from fortis_arm.arm_controller_node import (
    ALLOWED_ARM_STATES,
    CLOSE_GRIPPER_SERVICE,
    MISSION_STATE_TOPIC,
    MOVE_TO_POSE_ACTION,
    OPEN_GRIPPER_SERVICE,
    ArmControllerNode,
)
from fortis_msgs.action import MoveToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
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

#: Per-future timeout. The action server resolves stub goals immediately,
#: so anything > a few hundred ms means something is wrong (DDS not
#: discovered, callback not registered, etc.).
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


@pytest.fixture(scope="module")
def rclpy_session():
    """
    Module-scoped rclpy.init / shutdown.

    rclpy.init can only be called once per process; doing it module-wide
    keeps individual tests cheap (just node create/destroy).
    """
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness(rclpy_session):
    """Per-test harness: an ArmControllerNode plus an input/output helper."""
    h = _Harness()
    # Drain once so DDS discovery (state pub/sub, action server/client,
    # service server/client) is complete before the test acts.
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

        self.move_client = ActionClient(
            self.helper, MoveToPose, MOVE_TO_POSE_ACTION
        )
        self.open_client = self.helper.create_client(
            Trigger, OPEN_GRIPPER_SERVICE
        )
        self.close_client = self.helper.create_client(
            Trigger, CLOSE_GRIPPER_SERVICE
        )

        # Wait for the action and service servers to come up before any
        # test sends a request. Otherwise the very first send_goal_async
        # races the discovery handshake and times out.
        assert self.move_client.wait_for_server(timeout_sec=FUTURE_TIMEOUT_S), \
            "move_to_pose action server did not advertise within timeout"
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
        fixed interval. The first action call in a fresh test process
        otherwise races DDS matching of the state topic; a fixed sleep
        sometimes works and sometimes doesn't, which is the worst kind
        of test flake.
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

    def send_move_to_pose(self) -> bool:
        """
        Send a move_to_pose goal and return whether it was accepted.

        Returns True if the goal was accepted by the server (regardless
        of whether the eventual result is succeeded=True/False), False
        if it was rejected.
        """
        goal = MoveToPose.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "base_link"

        future = self.move_client.send_goal_async(goal)
        self._spin_until_complete(future)
        goal_handle = future.result()
        return goal_handle.accepted

    def send_move_to_pose_and_get_result(self) -> MoveToPose.Result:
        """Send a goal that is expected to be accepted; return the final result."""
        goal = MoveToPose.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "base_link"

        send_future = self.move_client.send_goal_async(goal)
        self._spin_until_complete(send_future)
        goal_handle = send_future.result()
        assert goal_handle.accepted, "expected accepted goal"

        result_future = goal_handle.get_result_async()
        self._spin_until_complete(result_future)
        return result_future.result().result

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
        self.helper.destroy_node()
        self.node.destroy_node()


# --- Action gating ----------------------------------------------------------


@pytest.mark.parametrize("state", sorted(ALLOWED_ARM_STATES))
def test_move_to_pose_accepted_in_allowed_states(harness, state):
    """In every allowed state the action server accepts and returns the stub result."""
    harness.publish_state(state)
    harness.spin()

    result = harness.send_move_to_pose_and_get_result()
    assert result.succeeded is False, \
        f"scaffold result must be succeeded=False in state {state}"
    assert "kinematics not implemented" in result.message, \
        f"unexpected stub message in state {state}: {result.message!r}"


@pytest.mark.parametrize("state", sorted(DISALLOWED_STATES))
def test_move_to_pose_rejected_in_disallowed_states(harness, state):
    """In every disallowed state the action server rejects the goal at the gate."""
    harness.publish_state(state)
    harness.spin()

    accepted = harness.send_move_to_pose()
    assert accepted is False, \
        f"goal must be rejected in state {state}"


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


# --- State transitions flip the gate ----------------------------------------


def test_state_transition_flips_action_gate(harness):
    """A round trip IDLE -> ARM_AT_VIEW -> IDLE flips the gate each time."""
    # IDLE: rejects.
    harness.publish_state("IDLE")
    harness.spin()
    assert harness.send_move_to_pose() is False

    # ARM_AT_VIEW: accepts.
    harness.publish_state("ARM_AT_VIEW")
    harness.spin()
    assert harness.send_move_to_pose() is True

    # Back to IDLE: rejects again.
    harness.publish_state("IDLE")
    harness.spin()
    assert harness.send_move_to_pose() is False
