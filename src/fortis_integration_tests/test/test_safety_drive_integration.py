"""
launch_testing integration test for fortis_safety + fortis_drive.

Brings up the real mission_state_node and drive_node in the same test
process and exercises the contract end to end. Unit tests for each
side already cover their behaviour in isolation; this test catches QoS,
timing, and topic-name mismatches that the unit tests do not see.

Test cases cover the four scenarios called out in the integration-test
spec for this branch:

1. Default startup state rejects /cmd_vel. mission_state_node publishes
   IDLE on startup with TRANSIENT_LOCAL durability, so the drive node
   sees IDLE the moment the subscription matches. A non-zero Twist must
   produce explicit zero_velocities (not silence) for >= 1 s.

2. After a START_ORBIT event drives the FSM to ORBIT, a non-zero Twist
   produces wheel_velocities matching the X-drive IK output within a
   tight tolerance.

3. Transitioning out of ORBIT to a non-allowed state (IDLE via STOP)
   makes the next Twist produce zeros within 200 ms of being published.

4. Following the FSM all the way to RETURN_HOME (an allowed state for
   driving) restores motion: a non-zero Twist produces wheel_velocities
   matching the IK output.

Topic names, QoS profiles, and event names are imported from the
production modules rather than hardcoded; a typo in either side would
break unit tests separately, but only this integration test catches a
QoS-profile drift between publisher and subscriber that DDS silently
suppresses.

Lives in fortis_integration_tests rather than fortis_drive/test/
because pytest collects a package's test directory as one process; if
this launch_testing case shared a process with fortis_drive's unit
tests, the launched mission_state_node would still hold a publisher on
/fortis/mission_state when the next unit test starts, cross-talking
with the unit test's own helper publisher and producing flakes. A
separate package gets a separate pytest invocation and a separate
ROS_DOMAIN_ID via test/conftest.py.
"""

from __future__ import annotations

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from fortis_comms.xdrive_kinematics import WHEEL_RADIUS, xdrive_ik_solver
from fortis_drive.drive_node import (
    ALLOWED_DRIVE_STATES,
    CMD_VEL_TOPIC,
    MISSION_STATE_TOPIC,
    WHEEL_VELOCITIES_TOPIC,
    ZERO_VELOCITIES_TOPIC,
)
from fortis_msgs.msg import WheelVelocities
from fortis_safety.mission_state_machine import Event, State
from fortis_safety.mission_state_node import CONTEXT_FIELDS
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Empty, String


# --- Constants ---------------------------------------------------------------

#: Per-spin_once timeout. Small enough to drain promptly; large enough
#: to avoid burning CPU when nothing is queued.
SPIN_ONCE_TIMEOUT_S: float = 0.02

#: Hard ceiling on how long any wait_for_* helper will poll before
#: failing the test. Anything past a few seconds means a launched node
#: did not come up, not that we are slow.
DEFAULT_WAIT_TIMEOUT_S: float = 5.0

#: Window for "no wheel_velocities should arrive" / "zeros should keep
#: arriving" assertions in the default-IDLE test. Spec requires >= 1 s.
DEFAULT_IDLE_OBSERVATION_WINDOW_S: float = 1.2

#: Bound on the response time after publishing /cmd_vel into a
#: rejecting state. Drive-side latency is one callback hop; the 200 ms
#: spec covers DDS round-trip + state-callback ordering on a loaded
#: box. Anything tighter races the integration harness, anything looser
#: stops being a meaningful guarantee.
TRANSITION_RESPONSE_DEADLINE_S: float = 0.2

#: Tolerance for floating-point comparisons of wheel speeds. Matches
#: the spec for this branch (1e-3 m/s); the drive node divides linear
#: speeds by WHEEL_RADIUS internally so this tolerance applies to the
#: rad/s comparison as well.
WHEEL_SPEED_TOLERANCE: float = 1e-3


@pytest.mark.launch_test
def generate_test_description():
    """
    Launch description: real mission_state_node + drive_node in one process.

    Uses the canonical entry-point names registered by each package's
    setup.py, so this test exercises the same launch path a deployment
    would. ReadyToTest signals launch_testing that fixtures are up and
    test methods can begin.
    """
    safety_node = launch_ros.actions.Node(
        package='fortis_safety',
        executable='mission_state_node',
        name='mission_state_node',
        output='screen',
    )
    drive_node = launch_ros.actions.Node(
        package='fortis_drive',
        executable='drive_node',
        name='drive_node',
        output='screen',
    )
    return (
        launch.LaunchDescription([
            safety_node,
            drive_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'safety_node': safety_node,
            'drive_node': drive_node,
        },
    )


# --- Helpers ----------------------------------------------------------------


def _event_topic(event: Event) -> str:
    """Translate an Event enum value into its mission_state_node topic."""
    return f"/fortis/events/{event.name.lower()}"


def _context_topic(field: str) -> str:
    """Translate a context-field name into its mission_state_node topic."""
    return f"/fortis/context/{field}"


def _expected_wheel_speeds(vx: float, vy: float, wz: float) -> list[float]:
    """Compute the IK output that the drive node should produce on (vx, vy, wz)."""
    linear = xdrive_ik_solver(vx, vy, wz)
    return [float(linear[i]) / WHEEL_RADIUS for i in range(4)]


# --- Test class -------------------------------------------------------------


class TestSafetyDriveIntegration(unittest.TestCase):
    """End-to-end tests exercising fortis_safety + fortis_drive together."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('safety_drive_integration_test')

        # Latched QoS for the mission_state subscription; matches both
        # the safety node's publisher and the drive node's subscriber.
        # Mismatched profiles cause DDS to silently refuse the match,
        # which is exactly the failure mode this integration test is
        # here to catch.
        cls.latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        cls.cmd_pub = cls.node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
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

        cls.wheel_msgs: list[WheelVelocities] = []
        cls.zero_msgs: list[WheelVelocities] = []
        cls.state_msgs: list[String] = []

        cls.node.create_subscription(
            WheelVelocities,
            WHEEL_VELOCITIES_TOPIC,
            cls.wheel_msgs.append,
            10,
        )
        cls.node.create_subscription(
            WheelVelocities,
            ZERO_VELOCITIES_TOPIC,
            cls.zero_msgs.append,
            10,
        )
        cls.node.create_subscription(
            String,
            MISSION_STATE_TOPIC,
            cls.state_msgs.append,
            cls.latched_qos,
        )

        # Block until both downstream nodes are visible. Without this
        # the first test races discovery and observes nothing.
        cls._wait_for_first_state(timeout_s=DEFAULT_WAIT_TIMEOUT_S)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        # Reset to IDLE before every test so test order does not matter.
        # Path: any state -> FAULT (always allowed) -> IDLE (RESET +
        # operator_ack=True). Some tests already finish in IDLE, but
        # idempotently FAULT-then-RESET is cheap and removes the need
        # to special-case the no-op path.
        self._reset_fsm_to_idle()

        # Discard captured messages from the prior test so per-test
        # asserts read off a clean buffer.
        self.wheel_msgs.clear()
        self.zero_msgs.clear()

    # --- Spin / wait helpers --------------------------------------------

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
        assert cls.state_msgs, \
            f"mission_state never published within {timeout_s}s; " \
            "is mission_state_node up?"

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

    # --- Publish helpers ------------------------------------------------

    def _publish_event(self, event: Event) -> None:
        self.event_pubs[event.name].publish(Empty())

    def _set_context(self, field: str, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.context_pubs[field].publish(msg)
        # Spin briefly so the context callback in mission_state_node
        # runs before the next event we send. Context updates and event
        # publications race otherwise -- they arrive on different
        # topics and DDS does not order across topics.
        self._spin_for(0.1)

    def _publish_twist(
        self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0
    ) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.cmd_pub.publish(msg)

    # --- FSM driving ----------------------------------------------------

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

    def _drive_to_return_home(self) -> None:
        # IDLE -> ORBIT -> TARGETING -> ARM_AT_VIEW -> INSPECT ->
        # RETURN_HOME (DONE from INSPECT). The observe path is the
        # shortest sequence to RETURN_HOME without involving the
        # gripper or grasp guards.
        self._set_context("target_pose_valid", True)
        self._set_context("ik_ok", True)

        self._publish_event(Event.START_ORBIT)
        self._wait_for_state(State.ORBIT.name)
        self._publish_event(Event.CHASSIS_CAM_CLICK)
        self._wait_for_state(State.TARGETING.name)
        self._publish_event(Event.ARM_AT_VIEW_POSE)
        self._wait_for_state(State.ARM_AT_VIEW.name)
        self._publish_event(Event.SELECT_OBSERVE)
        self._wait_for_state(State.INSPECT.name)
        self._publish_event(Event.DONE)
        self._wait_for_state(State.RETURN_HOME.name)

    # --- Tests ----------------------------------------------------------

    def test_01_default_idle_rejects_cmd_vel_for_one_second(self):
        """
        Sustained Twist in IDLE: only zero_velocities, no wheel_velocities.

        IDLE is not in ALLOWED_DRIVE_STATES, so every /cmd_vel must be
        explicitly stopped (not silently ignored) -- the safety
        contract is "downstream brakes never have to infer suppression
        from message absence." We sample for 1.2 s of sustained input.
        """
        assert "IDLE" not in ALLOWED_DRIVE_STATES, \
            "test premise violated: IDLE should be a rejecting state"
        self.assertEqual(self._current_state(), State.IDLE.name)

        end = time.monotonic() + DEFAULT_IDLE_OBSERVATION_WINDOW_S
        while time.monotonic() < end:
            self._publish_twist(vx=0.5, vy=0.0, wz=0.0)
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)

        # Drain any in-flight messages so the assertions read the
        # complete picture, not a partial one.
        self._spin_for(0.3)

        self.assertEqual(
            len(self.wheel_msgs), 0,
            f"wheel_velocities must not be published in IDLE; "
            f"got {len(self.wheel_msgs)} messages"
        )
        self.assertGreater(
            len(self.zero_msgs), 0,
            "zero_velocities must be published in IDLE so downstream "
            "brakes see an explicit stop, not silence"
        )
        for msg in self.zero_msgs:
            self.assertEqual(msg.fl, 0.0)
            self.assertEqual(msg.fr, 0.0)
            self.assertEqual(msg.bl, 0.0)
            self.assertEqual(msg.br, 0.0)

    def test_02_orbit_accepts_cmd_vel_and_publishes_correct_ik(self):
        """ORBIT path: Twist -> wheel_velocities matching xdrive_ik_solver."""
        self._drive_to_orbit()
        # Discard any zero_velocities that arrived while we were in
        # IDLE; we only care about post-transition output.
        self.zero_msgs.clear()
        self.wheel_msgs.clear()

        vx, vy, wz = 0.5, 0.0, 0.0
        self._publish_twist(vx=vx, vy=vy, wz=wz)

        end = time.monotonic() + DEFAULT_WAIT_TIMEOUT_S
        while not self.wheel_msgs and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)

        self.assertGreater(
            len(self.wheel_msgs), 0,
            "wheel_velocities should be published in ORBIT"
        )
        self.assertEqual(
            len(self.zero_msgs), 0,
            "zero_velocities should not be published in ORBIT"
        )

        expected = _expected_wheel_speeds(vx, vy, wz)
        msg = self.wheel_msgs[-1]
        self.assertTrue(math.isclose(msg.fl, expected[0],
                                     abs_tol=WHEEL_SPEED_TOLERANCE))
        self.assertTrue(math.isclose(msg.fr, expected[1],
                                     abs_tol=WHEEL_SPEED_TOLERANCE))
        self.assertTrue(math.isclose(msg.bl, expected[2],
                                     abs_tol=WHEEL_SPEED_TOLERANCE))
        self.assertTrue(math.isclose(msg.br, expected[3],
                                     abs_tol=WHEEL_SPEED_TOLERANCE))

    def test_03_transition_out_of_orbit_returns_zeros_within_200ms(self):
        """
        ORBIT -> IDLE via STOP: next Twist must produce zeros in <= 200 ms.

        Measured from the moment the Twist is published; the drive node
        must observe the IDLE state (already propagated by _wait_for_state)
        and reject within one DDS round-trip.
        """
        self._drive_to_orbit()
        # Sanity: motion is honoured before transition.
        self._publish_twist(vx=0.5)
        end = time.monotonic() + DEFAULT_WAIT_TIMEOUT_S
        while not self.wheel_msgs and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        self.assertGreater(len(self.wheel_msgs), 0,
                           "preconditions: ORBIT must accept Twist")

        # Transition out.
        self._publish_event(Event.STOP)
        self._wait_for_state(State.IDLE.name)
        self.zero_msgs.clear()
        self.wheel_msgs.clear()

        # Time the post-transition reject.
        deadline = self.node.get_clock().now() + Duration(
            nanoseconds=int(TRANSITION_RESPONSE_DEADLINE_S * 1e9)
        )
        self._publish_twist(vx=0.5)

        while not self.zero_msgs and self.node.get_clock().now() < deadline:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)

        self.assertGreater(
            len(self.zero_msgs), 0,
            f"zero_velocities did not arrive within "
            f"{TRANSITION_RESPONSE_DEADLINE_S * 1000:.0f} ms of /cmd_vel "
            "in the rejecting state"
        )
        self.assertEqual(
            len(self.wheel_msgs), 0,
            "wheel_velocities should not be published after transition to IDLE"
        )
        msg = self.zero_msgs[-1]
        self.assertEqual(msg.fl, 0.0)
        self.assertEqual(msg.fr, 0.0)
        self.assertEqual(msg.bl, 0.0)
        self.assertEqual(msg.br, 0.0)

    def test_04_return_home_accepts_cmd_vel_and_publishes_correct_ik(self):
        """RETURN_HOME path: full FSM walk, then verify IK output as in ORBIT."""
        self._drive_to_return_home()
        self.zero_msgs.clear()
        self.wheel_msgs.clear()

        vx, vy, wz = 0.3, 0.2, 0.0
        self._publish_twist(vx=vx, vy=vy, wz=wz)

        end = time.monotonic() + DEFAULT_WAIT_TIMEOUT_S
        while not self.wheel_msgs and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)

        self.assertGreater(
            len(self.wheel_msgs), 0,
            "wheel_velocities should be published in RETURN_HOME"
        )
        self.assertEqual(
            len(self.zero_msgs), 0,
            "zero_velocities should not be published in RETURN_HOME"
        )

        expected = _expected_wheel_speeds(vx, vy, wz)
        msg = self.wheel_msgs[-1]
        self.assertTrue(math.isclose(msg.fl, expected[0],
                                     abs_tol=WHEEL_SPEED_TOLERANCE))
        self.assertTrue(math.isclose(msg.fr, expected[1],
                                     abs_tol=WHEEL_SPEED_TOLERANCE))
        self.assertTrue(math.isclose(msg.bl, expected[2],
                                     abs_tol=WHEEL_SPEED_TOLERANCE))
        self.assertTrue(math.isclose(msg.br, expected[3],
                                     abs_tol=WHEEL_SPEED_TOLERANCE))
