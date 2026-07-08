"""
Integration test: teensy_bridge_node against tools/mock_teensy.py.

Spawns the pty-backed firmware mock as a subprocess, points a real
TeensyBridgeNode at the reported pty, and drives both nodes through a
MultiThreadedExecutor (mirroring the in-process style of
fortis_drive/test/test_drive_node.py). Asserts the connect, enable,
command-tracking and fault-forwarding paths end to end.

The mock lives in the repo tools/ dir, located from this test file
(parents[3] == workspace root in both the source tree and the container
checkout), with a FORTIS_REPO_ROOT override for other layouts. The test
is skipped if the mock cannot be found.
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
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.msg import ArmStatus

from fortis_arm.teensy_bridge_node import (
    ARM_COMMAND_TOPIC,
    ARM_JOINT_NAMES,
    ARM_STATUS_TOPIC,
    DEFAULT_STEPS_PER_RAD_J1,
    FAULT_EVENT_TOPIC,
    JOINT_STATES_TOPIC,
    TeensyBridgeNode,
)


ENABLED_BIT = 1 << 0
DRIVER_ALARM_J1_BIT = 1 << 1
PTY_WAIT_S = 5.0


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
    def __init__(self, pty_path):
        overrides = [
            Parameter("serial_port", value=pty_path),
            Parameter("status_rate_hz", value=20.0),
            Parameter("heartbeat_rate_hz", value=20.0),
        ]
        self.bridge = TeensyBridgeNode(parameter_overrides=overrides)
        self.helper = rclpy.create_node("teensy_bridge_test_helper")

        self.status_msgs = []
        self.joint_msgs = []
        self.fault_msgs = []
        self.helper.create_subscription(
            ArmStatus, ARM_STATUS_TOPIC,
            self.status_msgs.append, latched_qos_profile())
        self.helper.create_subscription(
            JointState, JOINT_STATES_TOPIC, self.joint_msgs.append, 10)
        self.helper.create_subscription(
            Empty, FAULT_EVENT_TOPIC, self.fault_msgs.append, 10)
        self.cmd_pub = self.helper.create_publisher(
            JointState, ARM_COMMAND_TOPIC, 10)
        self.enable_cli = self.helper.create_client(
            Trigger, "/teensy_bridge/enable")
        self.disable_cli = self.helper.create_client(
            Trigger, "/teensy_bridge/disable")

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.helper)
        self.thread = threading.Thread(
            target=self.executor.spin, daemon=True)
        self.thread.start()

    def latest_status(self):
        msgs = self.status_msgs
        return msgs[-1] if msgs else None

    def wait_for(self, predicate, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.02)
        return predicate()

    def call(self, client, timeout=5.0):
        assert client.wait_for_service(timeout_sec=timeout), \
            "service did not advertise"
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        assert future.done(), "service call did not complete"
        return future.result()

    def shutdown(self):
        self.executor.shutdown()
        self.bridge.destroy_node()
        self.helper.destroy_node()


@pytest.fixture
def rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_connected_status_and_joint_states(rclpy_session):
    proc, pty = _start_mock()
    h = _Harness(pty)
    try:
        assert h.wait_for(
            lambda: any(m.connected for m in list(h.status_msgs))), \
            "no connected ArmStatus received from bridge"
        assert h.wait_for(lambda: len(h.joint_msgs) > 0), \
            "bridge never published /joint_states"
        names = set(h.joint_msgs[-1].name)
        assert set(ARM_JOINT_NAMES).issubset(names), \
            f"joint_states missing arm joints: {names}"
    finally:
        h.shutdown()
        _stop_mock(proc)


def test_enable_disable_reflected_in_state_bits(rclpy_session):
    proc, pty = _start_mock()
    h = _Harness(pty)
    try:
        assert h.wait_for(
            lambda: any(m.connected for m in list(h.status_msgs))), \
            "bridge never connected"
        disable = h.call(h.disable_cli)
        assert disable.success, f"disable failed: {disable.message}"
        assert h.wait_for(
            lambda: h.latest_status() is not None
            and not (h.latest_status().state_bits & ENABLED_BIT)), \
            "enabled bit did not clear after disable"
        enable = h.call(h.enable_cli)
        assert enable.success, f"enable failed: {enable.message}"
        assert h.wait_for(
            lambda: h.latest_status() is not None
            and (h.latest_status().state_bits & ENABLED_BIT)), \
            "enabled bit did not set after enable"
    finally:
        h.shutdown()
        _stop_mock(proc)


def test_joint_command_is_tracked(rclpy_session):
    proc, pty = _start_mock()
    h = _Harness(pty)
    try:
        assert h.wait_for(
            lambda: any(m.connected for m in list(h.status_msgs))), \
            "bridge never connected"
        target_rad = 0.05
        expected = round(target_rad * DEFAULT_STEPS_PER_RAD_J1)
        cmd = JointState()
        cmd.name = ["joint_j1"]
        cmd.position = [target_rad]

        def commanded_reached():
            h.cmd_pub.publish(cmd)
            st = h.latest_status()
            return st is not None and abs(st.joint_steps[0] - expected) <= 3

        assert h.wait_for(commanded_reached, timeout=15.0), \
            "mock did not track commanded joint_j1 target"
    finally:
        h.shutdown()
        _stop_mock(proc)


def test_injected_driver_alarm_sets_fault_and_event(rclpy_session):
    proc, pty = _start_mock(["--inject-fault", "driver_alm"])
    h = _Harness(pty)
    try:
        assert h.wait_for(
            lambda: any(
                m.connected and (m.fault_flags & DRIVER_ALARM_J1_BIT)
                for m in list(h.status_msgs)),
            timeout=15.0), \
            "driver alarm fault never appeared in ArmStatus"
        assert h.wait_for(lambda: len(h.fault_msgs) > 0, timeout=15.0), \
            "no /fortis/events/fault published for injected fault"
    finally:
        h.shutdown()
        _stop_mock(proc)
