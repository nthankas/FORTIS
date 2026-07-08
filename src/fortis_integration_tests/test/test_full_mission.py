"""
launch_testing acceptance test for the FULL FORTIS mission loop.

Extends test_perception_chain.py's composition with the arm stack --
teensy_bridge against the pty firmware mock, arm_motion (IK + MoveToPose)
and arm_controller (gripper) -- and drives the mission FSM through its
whole happy path with real context guards:

    IDLE --START_ORBIT--> ORBIT
         --CHASSIS_CAM_CLICK [target_pose_valid]--> TARGETING
         --ARM_AT_VIEW_POSE  [ik_ok]--> ARM_AT_VIEW
         --SELECT_PICK       [grasp_candidate_ok]--> PICK
         --GRASP_SUCCESS     [gripper_closed]--> HOLDING
         --DONE--> RETURN_HOME
         --HOME_REACHED      [arm_at_home & chassis_at_home]--> IDLE

target_pose_valid comes from target_selector, ik_ok from arm_motion,
grasp_candidate_ok from detection, and the ARM_AT_VIEW_POSE event is
emitted by arm_motion when its MoveToPose goal completes. gripper_closed,
arm_at_home and chassis_at_home have no production publisher yet (gripper
force feedback and home-pose monitors are future work), so this test
publishes those context flags exactly as the operator console would.

The click point (-0.55, 0, 0.30) in base_link satisfies BOTH gates it must
pass: horizontal range 0.55 m sits inside target_selector's [0.3, 3.5] m
annulus, and via the fixed base_link->arm_mount transform (x 0.0898,
z 0.203, yaw pi) it maps to (0.6398, 0, 0.097) in the arm frame, whose
level-approach wrist point lies 0.53 m from the shoulder -- comfortably
inside the [0.095, 0.693] m annulus from arm_ik.reachable_workspace().
The MoveToPose goal is sent in base_link (robot-relative) so the orbiting
replayer cannot drift the target out of reach mid-move.

Ordered methods, one warm pipeline, every wait bounded; a green run
completes well inside the 150 s budget. Domain-isolated via
test/conftest.py (ROS_DOMAIN_ID 94).
"""

from __future__ import annotations

import atexit
import os
import subprocess
import sys
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from fortis_arm.arm_controller_node import CLOSE_GRIPPER_SERVICE
from fortis_arm.arm_motion_node import (
    ARM_AT_VIEW_EVENT_TOPIC,
    IK_OK_TOPIC,
    MOVE_TO_POSE_ACTION,
)
from fortis_arm.teensy_bridge_node import ARM_STATUS_TOPIC
from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.action import MoveToPose
from fortis_msgs.msg import ArmStatus
from fortis_perception.detection_node import GRASP_OK_TOPIC
from fortis_perception.target_selector_node import CLICKED_POINT_TOPIC
from fortis_safety.mission_state_machine import Event, State
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


# --- Constants ---------------------------------------------------------------

CAMERA = "oak_chassis_front"
MISSION_STATE_TOPIC = "/fortis/mission_state"

#: Chosen click: in-annulus for target_selector AND arm-reachable (see the
#: module docstring for the numbers).
CLICK_POINT_XYZ = (-0.55, 0.0, 0.30)

#: The teensy bridge's default gripper_closed_us; the mock boots at 1500,
#: so a successful close_gripper must land the servo here.
GRIPPER_CLOSED_US = 1100

SPIN_ONCE_TIMEOUT_S: float = 0.02
#: Mission state latch + bridge<->mock serial connect.
STARTUP_TIMEOUT_S: float = 20.0
ORBIT_TIMEOUT_S: float = 10.0
#: Click -> TARGETING and arm_motion's ik_ok latch.
TARGETING_TIMEOUT_S: float = 30.0
SERVER_READY_TIMEOUT_S: float = 10.0
#: The ~1.4 rad elbow move at 0.3 rad/s plus settling, with margin.
MOVE_RESULT_TIMEOUT_S: float = 45.0
ARM_AT_VIEW_TIMEOUT_S: float = 10.0
#: The sphere is visible over most of the ~21 s orbit; one period + margin.
DETECTION_TIMEOUT_S: float = 30.0
PICK_TIMEOUT_S: float = 30.0
GRIPPER_TIMEOUT_S: float = 10.0
#: Test-published context + event hops (no physics involved).
HOP_TIMEOUT_S: float = 10.0
PTY_WAIT_S: float = 5.0


def _event_topic(event: Event) -> str:
    """Return the /fortis/events topic mission_state_node subscribes for event."""
    return f"/fortis/events/{event.name.lower()}"


def _context_topic(field: str) -> str:
    """Return the /fortis/context Bool topic for a guard field."""
    return f"/fortis/context/{field}"


# --- Firmware mock plumbing ----------------------------------------------------

def _repo_root() -> Path:
    override = os.environ.get("FORTIS_REPO_ROOT")
    if override:
        return Path(override)
    return Path(__file__).resolve().parents[3]


_MOCK_PATH = _repo_root() / "tools" / "mock_teensy.py"
if not _MOCK_PATH.exists():
    pytest.skip(
        f"mock_teensy.py not found at {_MOCK_PATH}", allow_module_level=True)

_mock_proc = None


def _read_pty(proc, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        line = proc.stdout.readline()
        if not line:
            if proc.poll() is not None:
                break
            continue
        if "pty slave:" in line:
            return line.split("pty slave:")[1].strip()
    proc.terminate()
    raise AssertionError("mock_teensy did not report a pty path in time")


def _stop_mock():
    global _mock_proc
    if _mock_proc is None:
        return
    _mock_proc.terminate()
    try:
        _mock_proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        _mock_proc.kill()
        _mock_proc.wait(timeout=3.0)
    _mock_proc = None


@pytest.mark.launch_test
def generate_test_description():
    """Launch perception + FSM + the full arm stack against the mock pty."""
    global _mock_proc
    _mock_proc = subprocess.Popen(
        [sys.executable, str(_MOCK_PATH)],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    atexit.register(_stop_mock)
    pty_path = _read_pty(_mock_proc, PTY_WAIT_S)

    replayer = launch_ros.actions.Node(
        package='fortis_sim_support',
        executable='oak_replayer_node',
        name='oak_replayer',
        output='screen',
        parameters=[{
            'camera_name': CAMERA,
            'width': 320,
            'height': 200,
            'fps': 5.0,
            'trajectory': 'orbit',
            'scene': 'baseline',
            'publish_tf': True,
            'seed': 0,
        }],
    )
    depth_to_cloud = launch_ros.actions.Node(
        package='fortis_perception',
        executable='depth_to_cloud_node',
        name='depth_to_cloud',
        output='screen',
        parameters=[{'camera_name': CAMERA}],
    )
    cloud_fusion = launch_ros.actions.Node(
        package='fortis_perception',
        executable='cloud_fusion_node',
        name='cloud_fusion',
        output='screen',
        parameters=[{
            'input_topics': [f'/fortis/perception/{CAMERA}/points'],
        }],
    )
    voxel_map = launch_ros.actions.Node(
        package='fortis_perception',
        executable='voxel_map_node',
        name='voxel_map',
        output='screen',
    )
    detection = launch_ros.actions.Node(
        package='fortis_perception',
        executable='detection_node',
        name='detection',
        output='screen',
        parameters=[{'camera_name': CAMERA, 'detector': 'blob'}],
    )
    target_selector = launch_ros.actions.Node(
        package='fortis_perception',
        executable='target_selector_node',
        name='target_selector',
        output='screen',
    )
    mission_state = launch_ros.actions.Node(
        package='fortis_safety',
        executable='mission_state_node',
        name='mission_state_node',
        output='screen',
    )
    teensy_bridge = launch_ros.actions.Node(
        package='fortis_arm',
        executable='teensy_bridge',
        name='teensy_bridge',
        output='screen',
        parameters=[{
            'serial_port': pty_path,
            'status_rate_hz': 20.0,
            'heartbeat_rate_hz': 20.0,
        }],
    )
    arm_motion = launch_ros.actions.Node(
        package='fortis_arm',
        executable='arm_motion',
        name='arm_motion',
        output='screen',
    )
    arm_controller = launch_ros.actions.Node(
        package='fortis_arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
    )
    return (
        launch.LaunchDescription([
            replayer,
            depth_to_cloud,
            cloud_fusion,
            voxel_map,
            detection,
            target_selector,
            mission_state,
            teensy_bridge,
            arm_motion,
            arm_controller,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestFullMission(unittest.TestCase):
    """Ordered end-to-end mission assertions; each test builds on the last.

    No per-test FSM reset on purpose: the numbered sequence IS the mission
    (orbit -> click -> arm at view -> pick -> hold -> home) and unittest's
    name-ordered execution encodes it.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('full_mission_test')
        latched = latched_qos_profile()

        cls.state_msgs: list[String] = []
        cls.ik_ok_msgs: list[Bool] = []
        cls.grasp_ok_msgs: list[Bool] = []
        cls.status_msgs: list[ArmStatus] = []
        cls.view_events: list[Empty] = []
        cls.progress: list[float] = []

        cls.node.create_subscription(
            String, MISSION_STATE_TOPIC, cls.state_msgs.append, latched)
        cls.node.create_subscription(
            Bool, IK_OK_TOPIC, cls.ik_ok_msgs.append, latched)
        cls.node.create_subscription(
            Bool, GRASP_OK_TOPIC, cls.grasp_ok_msgs.append, latched)
        cls.node.create_subscription(
            ArmStatus, ARM_STATUS_TOPIC, cls.status_msgs.append, latched)
        cls.node.create_subscription(
            Empty, ARM_AT_VIEW_EVENT_TOPIC, cls.view_events.append, 10)

        cls.click_pub = cls.node.create_publisher(
            PointStamped, CLICKED_POINT_TOPIC, 10)
        cls.start_orbit_pub = cls.node.create_publisher(
            Empty, _event_topic(Event.START_ORBIT), 10)
        cls.select_pick_pub = cls.node.create_publisher(
            Empty, _event_topic(Event.SELECT_PICK), 10)
        cls.grasp_success_pub = cls.node.create_publisher(
            Empty, _event_topic(Event.GRASP_SUCCESS), 10)
        cls.done_pub = cls.node.create_publisher(
            Empty, _event_topic(Event.DONE), 10)
        cls.home_reached_pub = cls.node.create_publisher(
            Empty, _event_topic(Event.HOME_REACHED), 10)
        cls.gripper_closed_pub = cls.node.create_publisher(
            Bool, _context_topic('gripper_closed'), 10)
        cls.arm_at_home_pub = cls.node.create_publisher(
            Bool, _context_topic('arm_at_home'), 10)
        cls.chassis_at_home_pub = cls.node.create_publisher(
            Bool, _context_topic('chassis_at_home'), 10)

        cls.move_client = ActionClient(cls.node, MoveToPose, MOVE_TO_POSE_ACTION)
        cls.close_gripper_client = cls.node.create_client(
            Trigger, CLOSE_GRIPPER_SERVICE)

        cls._wait_until(
            lambda: cls.state_msgs, STARTUP_TIMEOUT_S,
            "mission_state never published; is mission_state_node up?")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # --- Spin / wait helpers ----------------------------------------------

    @classmethod
    def _spin_for(cls, duration_s: float) -> None:
        """Spin the helper node for a fixed wall-clock duration."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    @classmethod
    def _wait_until(cls, predicate, timeout_s: float, message: str) -> None:
        """Spin until predicate() is truthy or fail with message."""
        end = time.monotonic() + timeout_s
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        assert predicate(), message

    def _current_state(self) -> "str | None":
        """Return the latest observed mission state name."""
        return self.state_msgs[-1].data if self.state_msgs else None

    def _ik_ok(self) -> bool:
        """Return the latest latched ik_ok value."""
        return bool(self.ik_ok_msgs and self.ik_ok_msgs[-1].data)

    def _click_msg(self) -> PointStamped:
        """Build the chosen in-annulus, arm-reachable click in base_link."""
        msg = PointStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = CLICK_POINT_XYZ
        return msg

    def _drive_event(self, event_pub, target_state: str, timeout_s: float,
                     context: "tuple[tuple, ...]" = ()) -> None:
        """Re-publish context flags + an event until the FSM reaches a state.

        Retrying is deliberate: context flags and events travel on separate
        topics, so the FSM can see an event before its guard flag; a
        duplicate event in the target state is rejected noise (try_step).
        """
        deadline = time.monotonic() + timeout_s
        while (self._current_state() != target_state
               and time.monotonic() < deadline):
            for pub, value in context:
                pub.publish(Bool(data=value))
            event_pub.publish(Empty())
            self._spin_for(0.5)
        self.assertEqual(
            self._current_state(), target_state,
            f"FSM did not reach {target_state} within {timeout_s}s "
            f"(last seen: {self._current_state()!r})",
        )

    # --- Tests --------------------------------------------------------------

    def test_01_stack_up_and_arm_connected(self):
        """Verify the FSM latched a state and the bridge reached the mock."""
        self.assertEqual(self._current_state(), State.IDLE.name)
        self._wait_until(
            lambda: any(m.connected for m in list(self.status_msgs)),
            STARTUP_TIMEOUT_S,
            "teensy_bridge never reported connected=True against the mock",
        )

    def test_02_start_orbit(self):
        """Publish START_ORBIT and verify IDLE -> ORBIT."""
        self._drive_event(
            self.start_orbit_pub, State.ORBIT.name, ORBIT_TIMEOUT_S)

    def test_03_click_yields_targeting_and_ik_ok(self):
        """Verify a click drives ORBIT -> TARGETING and arm_motion latches ik_ok."""
        self.assertEqual(self._current_state(), State.ORBIT.name)
        deadline = time.monotonic() + TARGETING_TIMEOUT_S
        while ((self._current_state() != State.TARGETING.name
                or not self._ik_ok())
               and time.monotonic() < deadline):
            self.click_pub.publish(self._click_msg())
            self._spin_for(1.0)
        self.assertEqual(
            self._current_state(), State.TARGETING.name,
            f"FSM did not reach TARGETING within {TARGETING_TIMEOUT_S}s "
            f"(last seen: {self._current_state()!r})",
        )
        self.assertTrue(
            self._ik_ok(),
            f"{IK_OK_TOPIC} never latched True for the reachable click "
            f"{CLICK_POINT_XYZ}; is arm_motion solving the target?",
        )

    def test_04_move_to_pose_reaches_arm_at_view(self):
        """Drive MoveToPose to success and verify TARGETING -> ARM_AT_VIEW."""
        self.assertEqual(self._current_state(), State.TARGETING.name)
        self._wait_until(
            self.move_client.server_is_ready, SERVER_READY_TIMEOUT_S,
            "move_to_pose action server never became ready")

        goal = MoveToPose.Goal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        pos = goal.target_pose.pose.position
        pos.x, pos.y, pos.z = (float(c) for c in CLICK_POINT_XYZ)
        goal.target_pose.pose.orientation.w = 1.0

        send_future = self.move_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self.progress.append(
                fb.feedback.progress))
        self._wait_until(
            send_future.done, SERVER_READY_TIMEOUT_S,
            "move_to_pose goal send never resolved")
        handle = send_future.result()
        self.assertTrue(handle.accepted, "goal rejected in TARGETING")

        result_future = handle.get_result_async()
        self._wait_until(
            result_future.done, MOVE_RESULT_TIMEOUT_S,
            f"move_to_pose result did not arrive in {MOVE_RESULT_TIMEOUT_S}s")
        result = result_future.result().result
        self.assertTrue(result.succeeded, f"move failed: {result.message}")
        self.assertTrue(
            self.progress and max(self.progress) >= 1.0,
            f"feedback progress never reached 1.0: {self.progress[-5:]}")
        self._wait_until(
            lambda: self.view_events, ARM_AT_VIEW_TIMEOUT_S,
            f"{ARM_AT_VIEW_EVENT_TOPIC} never fired after the move")
        self._wait_until(
            lambda: self._current_state() == State.ARM_AT_VIEW.name,
            ARM_AT_VIEW_TIMEOUT_S,
            "FSM did not take ARM_AT_VIEW_POSE under the ik_ok guard "
            f"(last seen: {self._current_state()!r})")

    def test_05_select_pick_enters_pick(self):
        """Verify grasp_candidate_ok from detection gates SELECT_PICK -> PICK."""
        self.assertEqual(self._current_state(), State.ARM_AT_VIEW.name)
        # The blob detector must have latched the red sphere at least once
        # by now; the flag can flap as the orbit looks away, so the event
        # retry loop below rides through the False windows.
        self._wait_until(
            lambda: any(m.data for m in list(self.grasp_ok_msgs)),
            DETECTION_TIMEOUT_S,
            f"{GRASP_OK_TOPIC} never went True; did detection see the "
            "baseline scene's red sphere?",
        )
        self._drive_event(
            self.select_pick_pub, State.PICK.name, PICK_TIMEOUT_S)

    def test_06_close_gripper_moves_servo(self):
        """Call close_gripper in PICK and verify the mock's servo moved."""
        self.assertEqual(self._current_state(), State.PICK.name)
        self._wait_until(
            self.close_gripper_client.service_is_ready,
            SERVER_READY_TIMEOUT_S,
            f"{CLOSE_GRIPPER_SERVICE} never became available")
        before_us = self.status_msgs[-1].gripper_us
        future = self.close_gripper_client.call_async(Trigger.Request())
        self._wait_until(
            future.done, GRIPPER_TIMEOUT_S, "close_gripper never responded")
        response = future.result()
        self.assertTrue(
            response.success, f"close_gripper rejected: {response.message}")
        self._wait_until(
            lambda: (self.status_msgs
                     and self.status_msgs[-1].gripper_us == GRIPPER_CLOSED_US),
            GRIPPER_TIMEOUT_S,
            f"gripper_us never reached {GRIPPER_CLOSED_US} "
            f"(was {before_us}); did the command reach the mock?",
        )
        self.assertNotEqual(self.status_msgs[-1].gripper_us, before_us)

    def test_07_grasp_success_enters_holding(self):
        """Publish gripper_closed context + GRASP_SUCCESS and verify HOLDING."""
        self._drive_event(
            self.grasp_success_pub, State.HOLDING.name, HOP_TIMEOUT_S,
            context=((self.gripper_closed_pub, True),))

    def test_08_done_enters_return_home(self):
        """Publish DONE and verify HOLDING -> RETURN_HOME."""
        self._drive_event(
            self.done_pub, State.RETURN_HOME.name, HOP_TIMEOUT_S)

    def test_09_home_reached_closes_loop_to_idle(self):
        """Publish home context + HOME_REACHED and verify the loop closes."""
        self._drive_event(
            self.home_reached_pub, State.IDLE.name, HOP_TIMEOUT_S,
            context=(
                (self.arm_at_home_pub, True),
                (self.chassis_at_home_pub, True),
            ))
