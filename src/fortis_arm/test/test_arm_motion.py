"""
In-process tests for fortis_arm.arm_motion_node.

Full-fidelity harness: the pty-backed firmware mock plus a real
TeensyBridgeNode provide /joint_states that physically integrate toward
/fortis/arm/command, so MoveToPose runs against the same feedback loop the
robot has. Mock spawning mirrors the helpers of test_teensy_bridge.py; the
whole file stays well under the 45 s budget (the single move takes ~5 s).

Targets are published in base_link, which arm_motion consumes without TF,
so no transform broadcaster is needed here.
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.action import MoveToPose
from fortis_msgs.msg import ArmStatus

from fortis_arm.arm_motion_node import (
    ARM_AT_VIEW_EVENT_TOPIC,
    IK_OK_TOPIC,
    MISSION_STATE_TOPIC,
    MOVE_TO_POSE_ACTION,
    TARGET_POSE_TOPIC,
    ArmMotionNode,
)
from fortis_arm.teensy_bridge_node import (
    ARM_STATUS_TOPIC,
    JOINT_STATES_TOPIC,
    TeensyBridgeNode,
)

PTY_WAIT_S = 5.0

#: base_link (-0.55, 0, 0.30) -> arm frame (0.6398, 0, 0.097): inside the
#: arm workspace (see test_arm_ik.test_elbow_up_and_down_both_returned).
REACHABLE_XYZ = (-0.55, 0.0, 0.30)
#: base_link (-3.0, 0, 0.30) -> arm frame x ~3.09 m: far beyond reach.
UNREACHABLE_XYZ = (-3.0, 0.0, 0.30)


def _repo_root() -> Path:
    override = os.environ.get("FORTIS_REPO_ROOT")
    if override:
        return Path(override)
    return Path(__file__).resolve().parents[3]


def _mock_path() -> Path:
    return _repo_root() / "tools" / "mock_teensy.py"


def _start_mock(extra_args=()):
    mock = _mock_path()
    if not mock.exists():
        pytest.skip(f"mock_teensy.py not found at {mock}")
    proc = subprocess.Popen(
        [sys.executable, str(mock), *extra_args],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return proc, _read_pty(proc, PTY_WAIT_S)


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


def _stop_mock(proc):
    proc.terminate()
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=3.0)


class _Harness:
    """Bridge + arm_motion under test plus a helper node for I/O."""

    def __init__(self, pty_path):
        overrides = [
            Parameter("serial_port", value=pty_path),
            Parameter("status_rate_hz", value=20.0),
            Parameter("heartbeat_rate_hz", value=20.0),
        ]
        self.bridge = TeensyBridgeNode(parameter_overrides=overrides)
        self.motion = ArmMotionNode()
        self.helper = rclpy.create_node("arm_motion_test_helper")

        self.status_msgs = []
        self.joint_msgs = []
        self.ik_ok_msgs = []
        self.view_events = []
        self.progress = []

        latched = latched_qos_profile()
        self.helper.create_subscription(
            ArmStatus, ARM_STATUS_TOPIC, self.status_msgs.append, latched)
        self.helper.create_subscription(
            JointState, JOINT_STATES_TOPIC, self.joint_msgs.append, 10)
        self.helper.create_subscription(
            Bool, IK_OK_TOPIC, self.ik_ok_msgs.append, latched)
        self.helper.create_subscription(
            Empty, ARM_AT_VIEW_EVENT_TOPIC, self.view_events.append, 10)

        self.state_pub = self.helper.create_publisher(
            String, MISSION_STATE_TOPIC, latched)
        self.target_pub = self.helper.create_publisher(
            PoseStamped, TARGET_POSE_TOPIC, latched)
        self.action_client = ActionClient(
            self.helper, MoveToPose, MOVE_TO_POSE_ACTION)

        self.executor = MultiThreadedExecutor()
        for node in (self.bridge, self.motion, self.helper):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def set_state(self, name: str) -> None:
        """Latch a mission state exactly like mission_state_node does."""
        self.state_pub.publish(String(data=name))

    def publish_target(self, xyz) -> None:
        """Publish a latched base_link target pose."""
        self.target_pub.publish(self.make_pose(xyz))

    def make_pose(self, xyz) -> PoseStamped:
        """Build a base_link PoseStamped at xyz."""
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.helper.get_clock().now().to_msg()
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.w = 1.0
        return msg

    def wait_for(self, predicate, timeout=10.0):
        """Poll predicate until true or timeout; return its final value."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.02)
        return predicate()

    def shutdown(self):
        """Stop the executor and destroy all three nodes."""
        self.executor.shutdown()
        self.bridge.destroy_node()
        self.motion.destroy_node()
        self.helper.destroy_node()


@pytest.fixture
def rclpy_session():
    """Give each test a fresh rclpy context."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_ik_ok_tracks_target_reachability(rclpy_session):
    """Latch ik_ok True for a reachable target, False for an unreachable one."""
    proc, pty = _start_mock()
    h = _Harness(pty)
    try:
        assert h.wait_for(lambda: len(h.ik_ok_msgs) > 0), \
            "arm_motion never latched its initial ik_ok"
        assert h.ik_ok_msgs[-1].data is False, \
            "ik_ok must start False before any target"
        h.publish_target(REACHABLE_XYZ)
        assert h.wait_for(lambda: h.ik_ok_msgs and h.ik_ok_msgs[-1].data), \
            "ik_ok never went True for a reachable target"
        h.publish_target(UNREACHABLE_XYZ)
        assert h.wait_for(
            lambda: h.ik_ok_msgs and not h.ik_ok_msgs[-1].data), \
            "ik_ok never dropped for an unreachable target"
    finally:
        h.shutdown()
        _stop_mock(proc)


def test_move_to_pose_reaches_target(rclpy_session):
    """Drive a MoveToPose goal end to end against the mock firmware."""
    proc, pty = _start_mock()
    h = _Harness(pty)
    try:
        h.set_state("TARGETING")
        assert h.wait_for(
            lambda: any(m.connected for m in list(h.status_msgs)), 15.0), \
            "bridge never connected to the mock"
        assert h.wait_for(lambda: len(h.joint_msgs) > 0), \
            "bridge never published /joint_states"
        assert h.action_client.wait_for_server(timeout_sec=10.0), \
            "move_to_pose action server never advertised"

        goal = MoveToPose.Goal()
        goal.target_pose = h.make_pose(REACHABLE_XYZ)
        send_future = h.action_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: h.progress.append(
                fb.feedback.progress))
        assert h.wait_for(send_future.done), "goal send never resolved"
        handle = send_future.result()
        assert handle.accepted, "goal rejected in TARGETING"

        result_future = handle.get_result_async()
        assert h.wait_for(result_future.done, timeout=40.0), \
            "move_to_pose result never arrived"
        result = result_future.result().result
        assert result.succeeded, f"move failed: {result.message}"
        assert h.progress and max(h.progress) == pytest.approx(1.0), \
            f"progress never reached 1.0 (max {max(h.progress or [0.0])})"
        assert h.wait_for(lambda: len(h.view_events) > 0, 5.0), \
            "arm_at_view_pose event never published after success"
    finally:
        h.shutdown()
        _stop_mock(proc)


def test_goal_rejected_outside_allowed_states(rclpy_session):
    """Reject MoveToPose goals while the mission state is IDLE."""
    proc, pty = _start_mock()
    h = _Harness(pty)
    try:
        h.set_state("IDLE")
        assert h.wait_for(lambda: h.motion._mission_state == "IDLE"), \
            "arm_motion never received the latched IDLE state"
        assert h.action_client.wait_for_server(timeout_sec=10.0), \
            "move_to_pose action server never advertised"
        goal = MoveToPose.Goal()
        goal.target_pose = h.make_pose(REACHABLE_XYZ)
        send_future = h.action_client.send_goal_async(goal)
        assert h.wait_for(send_future.done), "goal send never resolved"
        assert send_future.result().accepted is False, \
            "goal must be rejected outside ALLOWED_MOTION_STATES"
    finally:
        h.shutdown()
        _stop_mock(proc)
