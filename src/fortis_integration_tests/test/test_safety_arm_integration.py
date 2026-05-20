"""
launch_testing integration test for fortis_safety + fortis_arm.

Brings up the real mission_state_node and arm_controller in the same
test process and exercises the gripper-service gating contract end to
end. Mirrors test_safety_drive_integration.py: unit tests for each
side already cover behaviour in isolation; this test catches QoS,
timing, topic-name, and service-name mismatches the unit tests do not
see.

Replaces an earlier scaffold that referenced the now-retired MoveToPose
action server (moved to legacy/deprecated_arm_action). The arm exposes
gripper services today; the action will return when MoveIt 2 lands.

Test cases:

1. IDLE, ORBIT, TARGETING, and FAULT reject both gripper services with
   the "rejected: state X not in {ALLOWED_ARM_STATES}" message.
2. ARM_AT_VIEW, INSPECT, PICK, HOLDING, and RETURN_HOME accept the
   gripper services with the "gripper actuation not implemented" stub
   message, since the Teensy serial layer has not landed.

Allowlist comes from fortis_arm.arm_controller_node.ALLOWED_ARM_STATES.
Topic names, service names, and FSM event/state enums are imported
from the production modules so a typo in either side fails one of
the unit tests, not just this one. This file's job is the cross-
package seam: state -> service gate.

Lives in fortis_integration_tests for the same reason the safety_drive
test does: launch_testing brings up a process that publishes
/fortis/mission_state with TRANSIENT_LOCAL durability, which would
cross-talk with fortis_arm's unit tests if they shared a pytest
process. conftest.py pins ROS_DOMAIN_ID per-PID for the whole package.
"""

from __future__ import annotations

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from fortis_arm.arm_controller_node import (
    ALLOWED_ARM_STATES,
    CLOSE_GRIPPER_SERVICE,
    MISSION_STATE_TOPIC,
    OPEN_GRIPPER_SERVICE,
)
from fortis_safety.mission_state_machine import Event, State
from fortis_safety.mission_state_node import CONTEXT_FIELDS
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


SPIN_ONCE_TIMEOUT_S: float = 0.02
DEFAULT_WAIT_TIMEOUT_S: float = 5.0
SERVICE_RESPONSE_TIMEOUT_S: float = 3.0

STUB_GRIPPER_MESSAGE: str = "gripper actuation not implemented"
REJECT_MESSAGE_PREFIX: str = "rejected: state "


@pytest.mark.launch_test
def generate_test_description():
    """Real mission_state_node + arm_controller in one process."""
    safety_node = launch_ros.actions.Node(
        package='fortis_safety',
        executable='mission_state_node',
        name='mission_state_node',
        output='screen',
    )
    arm_node = launch_ros.actions.Node(
        package='fortis_arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
    )
    return (
        launch.LaunchDescription([
            safety_node,
            arm_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'safety_node': safety_node,
            'arm_node': arm_node,
        },
    )


def _event_topic(event: Event) -> str:
    return f"/fortis/events/{event.name.lower()}"


def _context_topic(field: str) -> str:
    return f"/fortis/context/{field}"


class TestSafetyArmIntegration(unittest.TestCase):
    """End-to-end tests exercising fortis_safety + fortis_arm together."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('safety_arm_integration_test')

        cls.latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        cls.event_pubs: dict[str, object] = {
            event.name: cls.node.create_publisher(
                Empty, _event_topic(event), 10
            )
            for event in Event
        }
        cls.context_pubs: dict[str, object] = {
            field: cls.node.create_publisher(
                Bool, _context_topic(field), 10
            )
            for field in CONTEXT_FIELDS
        }

        cls.state_msgs: list[String] = []
        cls.node.create_subscription(
            String,
            MISSION_STATE_TOPIC,
            cls.state_msgs.append,
            cls.latched_qos,
        )

        cls.open_client = cls.node.create_client(
            Trigger, OPEN_GRIPPER_SERVICE
        )
        cls.close_client = cls.node.create_client(
            Trigger, CLOSE_GRIPPER_SERVICE
        )

        cls._wait_for_first_state(timeout_s=DEFAULT_WAIT_TIMEOUT_S)
        cls._wait_for_services(timeout_s=DEFAULT_WAIT_TIMEOUT_S)
        # Also wait for the arm_controller to have observed the
        # initial mission_state. _wait_for_first_state only proves the
        # test node received it; the arm_controller is a separate
        # process with its own subscription that may not have caught
        # up. Without this, test_01 races the arm and sees the
        # "<no_state_received>" rejection branch instead of an "IDLE"
        # rejection.
        cls._wait_for_arm_to_propagate_state(timeout_s=DEFAULT_WAIT_TIMEOUT_S)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        # Drop back to IDLE before every test so test order is
        # irrelevant. Wildcard FAULT works from any state; RESET
        # requires operator_ack=True.
        self._reset_fsm_to_idle()

    @classmethod
    def _spin_for(cls, duration_s: float) -> None:
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    @classmethod
    def _wait_for_first_state(cls, timeout_s: float) -> None:
        end = time.monotonic() + timeout_s
        while not cls.state_msgs and time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        assert cls.state_msgs, (
            f"mission_state never published within {timeout_s}s; "
            "is mission_state_node up?"
        )

    @classmethod
    def _wait_for_arm_to_propagate_state(cls, timeout_s: float) -> None:
        """
        Probe open_gripper until the rejection message names a real
        state. Until the arm_controller subscription has received the
        first mission_state message, the rejection path uses the
        "<no_state_received>" sentinel; once propagated, the message
        names IDLE. The probe is harmless: IDLE rejects gripper calls
        anyway, so we are just polling on a known response shape.
        """
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            future = cls.open_client.call_async(Trigger.Request())
            inner_end = time.monotonic() + SERVICE_RESPONSE_TIMEOUT_S
            while not future.done() and time.monotonic() < inner_end:
                rclpy.spin_once(cls.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
            if future.done():
                response = future.result()
                if (
                    response is not None
                    and "<no_state_received>" not in response.message
                ):
                    return
            cls._spin_for(0.05)
        raise AssertionError(
            "arm_controller never observed mission_state within "
            f"{timeout_s}s; subscription QoS mismatch or startup race?"
        )

    @classmethod
    def _wait_for_services(cls, timeout_s: float) -> None:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if (
                cls.open_client.service_is_ready()
                and cls.close_client.service_is_ready()
            ):
                return
            rclpy.spin_once(cls.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        assert cls.open_client.service_is_ready(), (
            "open_gripper service never advertised; is arm_controller up?"
        )
        assert cls.close_client.service_is_ready(), (
            "close_gripper service never advertised; is arm_controller up?"
        )

    def _current_state(self) -> str | None:
        return self.state_msgs[-1].data if self.state_msgs else None

    def _wait_for_state(
        self, target: str, timeout_s: float = DEFAULT_WAIT_TIMEOUT_S
    ) -> None:
        end = time.monotonic() + timeout_s
        while self._current_state() != target and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        assert self._current_state() == target, (
            f"FSM did not reach {target!r} within {timeout_s}s "
            f"(last seen: {self._current_state()!r})"
        )

    def _publish_event(self, event: Event) -> None:
        self.event_pubs[event.name].publish(Empty())

    def _set_context(self, field: str, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.context_pubs[field].publish(msg)
        # Brief spin so the context callback runs before the next
        # event. Context and events race otherwise (different topics,
        # no ordering across topics in DDS).
        self._spin_for(0.1)

    def _call_gripper(self, client) -> Trigger.Response:
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + SERVICE_RESPONSE_TIMEOUT_S
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        assert future.done(), (
            f"gripper service did not respond within "
            f"{SERVICE_RESPONSE_TIMEOUT_S}s"
        )
        return future.result()

    def _assert_rejected(self, response: Trigger.Response, state_name: str):
        self.assertFalse(
            response.success,
            f"gripper call must be rejected in {state_name}, got success=True"
        )
        self.assertTrue(
            response.message.startswith(REJECT_MESSAGE_PREFIX),
            f"expected rejection prefix {REJECT_MESSAGE_PREFIX!r} in "
            f"{state_name}, got: {response.message!r}"
        )
        self.assertIn(
            state_name,
            response.message,
            f"rejection message in {state_name} should name the state; "
            f"got: {response.message!r}"
        )

    def _assert_stub_accepted(self, response: Trigger.Response, state_name: str):
        # The arm controller returns success=False even for accepted
        # gripper calls because the Teensy serial layer has not landed.
        # Contract: explicit stub message, not silence, so the operator
        # sees the difference between "rejected by gate" and "accepted
        # but actuation missing".
        self.assertFalse(
            response.success,
            f"stub gripper call should return success=False in {state_name} "
            "until the Teensy layer lands"
        )
        self.assertEqual(
            response.message,
            STUB_GRIPPER_MESSAGE,
            f"in {state_name} the gripper should accept the gate and return "
            f"the stub message {STUB_GRIPPER_MESSAGE!r}, got: "
            f"{response.message!r}"
        )

    def _reset_fsm_to_idle(self) -> None:
        if self._current_state() == State.IDLE.name:
            return
        self._set_context("operator_ack", True)
        self._publish_event(Event.FAULT)
        self._wait_for_state(State.FAULT.name)
        self._publish_event(Event.RESET)
        self._wait_for_state(State.IDLE.name)

    def _drive_to_orbit(self) -> None:
        self._publish_event(Event.START_ORBIT)
        self._wait_for_state(State.ORBIT.name)

    def _drive_to_targeting(self) -> None:
        self._set_context("target_pose_valid", True)
        self._publish_event(Event.START_ORBIT)
        self._wait_for_state(State.ORBIT.name)
        self._publish_event(Event.CHASSIS_CAM_CLICK)
        self._wait_for_state(State.TARGETING.name)

    def _drive_to_arm_at_view(self) -> None:
        self._set_context("target_pose_valid", True)
        self._set_context("ik_ok", True)
        self._publish_event(Event.START_ORBIT)
        self._wait_for_state(State.ORBIT.name)
        self._publish_event(Event.CHASSIS_CAM_CLICK)
        self._wait_for_state(State.TARGETING.name)
        self._publish_event(Event.ARM_AT_VIEW_POSE)
        self._wait_for_state(State.ARM_AT_VIEW.name)

    def _drive_to_inspect(self) -> None:
        self._drive_to_arm_at_view()
        self._publish_event(Event.SELECT_OBSERVE)
        self._wait_for_state(State.INSPECT.name)

    def _drive_to_pick(self) -> None:
        self._set_context("grasp_candidate_ok", True)
        self._drive_to_arm_at_view()
        self._publish_event(Event.SELECT_PICK)
        self._wait_for_state(State.PICK.name)

    def _drive_to_holding(self) -> None:
        self._drive_to_pick()
        self._set_context("gripper_closed", True)
        self._publish_event(Event.GRASP_SUCCESS)
        self._wait_for_state(State.HOLDING.name)

    def _drive_to_return_home(self) -> None:
        # Shortest path that does not depend on grasp guards.
        self._drive_to_inspect()
        self._publish_event(Event.DONE)
        self._wait_for_state(State.RETURN_HOME.name)

    def _drive_to_fault(self) -> None:
        self._publish_event(Event.FAULT)
        self._wait_for_state(State.FAULT.name)

    # --- Premise --------------------------------------------------------

    def test_00_allowlist_contract_unchanged(self):
        """
        ALLOWED_ARM_STATES is the source of truth for every per-state
        test below. If this set changes, the per-state expectations
        below need to change with it; pinning membership here surfaces
        any drift as a single named failure.
        """
        self.assertEqual(
            ALLOWED_ARM_STATES,
            frozenset({
                "ARM_AT_VIEW",
                "INSPECT",
                "PICK",
                "HOLDING",
                "RETURN_HOME",
            }),
        )

    # --- Rejecting states -----------------------------------------------

    def test_01_idle_rejects_both_gripper_services(self):
        self.assertEqual(self._current_state(), State.IDLE.name)
        self._assert_rejected(self._call_gripper(self.open_client), "IDLE")
        self._assert_rejected(self._call_gripper(self.close_client), "IDLE")

    def test_02_orbit_rejects_both_gripper_services(self):
        self._drive_to_orbit()
        self._assert_rejected(self._call_gripper(self.open_client), "ORBIT")
        self._assert_rejected(self._call_gripper(self.close_client), "ORBIT")

    def test_03_targeting_rejects_both_gripper_services(self):
        self._drive_to_targeting()
        self._assert_rejected(
            self._call_gripper(self.open_client), "TARGETING"
        )
        self._assert_rejected(
            self._call_gripper(self.close_client), "TARGETING"
        )

    def test_04_fault_rejects_both_gripper_services(self):
        self._drive_to_fault()
        self._assert_rejected(self._call_gripper(self.open_client), "FAULT")
        self._assert_rejected(self._call_gripper(self.close_client), "FAULT")

    # --- Allowed states (stub success) ---------------------------------

    def test_05_arm_at_view_accepts_with_stub_message(self):
        self._drive_to_arm_at_view()
        self._assert_stub_accepted(
            self._call_gripper(self.open_client), "ARM_AT_VIEW"
        )
        self._assert_stub_accepted(
            self._call_gripper(self.close_client), "ARM_AT_VIEW"
        )

    def test_06_inspect_accepts_with_stub_message(self):
        self._drive_to_inspect()
        self._assert_stub_accepted(
            self._call_gripper(self.open_client), "INSPECT"
        )
        self._assert_stub_accepted(
            self._call_gripper(self.close_client), "INSPECT"
        )

    def test_07_pick_accepts_with_stub_message(self):
        self._drive_to_pick()
        self._assert_stub_accepted(
            self._call_gripper(self.open_client), "PICK"
        )
        self._assert_stub_accepted(
            self._call_gripper(self.close_client), "PICK"
        )

    def test_08_holding_accepts_with_stub_message(self):
        self._drive_to_holding()
        self._assert_stub_accepted(
            self._call_gripper(self.open_client), "HOLDING"
        )
        self._assert_stub_accepted(
            self._call_gripper(self.close_client), "HOLDING"
        )

    def test_09_return_home_accepts_with_stub_message(self):
        self._drive_to_return_home()
        self._assert_stub_accepted(
            self._call_gripper(self.open_client), "RETURN_HOME"
        )
        self._assert_stub_accepted(
            self._call_gripper(self.close_client), "RETURN_HOME"
        )
