"""
Bring-up race test for fortis_arm.arm_controller_node.

Mirrors test_drive_node.test_no_state_received_rejects_cmd_vel: if a
client sends a request before /fortis/mission_state has announced
anything, the arm controller must reject (action goal rejected,
gripper services return success=False with an explanatory message).
"""

from __future__ import annotations

import time

import pytest
import rclpy
from fortis_arm.arm_controller_node import (
    CLOSE_GRIPPER_SERVICE,
    MOVE_TO_POSE_ACTION,
    OPEN_GRIPPER_SERVICE,
    ArmControllerNode,
)
from fortis_msgs.action import MoveToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


SPIN_ONCE_TIMEOUT_S: float = 0.02
DISCOVERY_DURATION_S: float = 0.3
FUTURE_TIMEOUT_S: float = 2.0


@pytest.fixture(scope="module")
def rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def bringup_harness(rclpy_session):
    """
    Set up node + helper but never publish a mission state.

    Reproduces the bring-up window where requests can arrive before the
    state publisher is up. Discovery is completed for the action and the
    services so the test sees gate-stage rejection, not discovery-stage
    failure.
    """
    node = ArmControllerNode()
    helper: Node = rclpy.create_node("arm_bringup_test_helper")

    move_client = ActionClient(helper, MoveToPose, MOVE_TO_POSE_ACTION)
    open_client = helper.create_client(Trigger, OPEN_GRIPPER_SERVICE)
    close_client = helper.create_client(Trigger, CLOSE_GRIPPER_SERVICE)

    end = time.monotonic() + DISCOVERY_DURATION_S
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        rclpy.spin_once(helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)

    assert move_client.wait_for_server(timeout_sec=FUTURE_TIMEOUT_S)
    assert open_client.wait_for_service(timeout_sec=FUTURE_TIMEOUT_S)
    assert close_client.wait_for_service(timeout_sec=FUTURE_TIMEOUT_S)

    yield node, helper, move_client, open_client, close_client

    helper.destroy_node()
    node.destroy_node()


def _spin_until_complete(node, helper, future) -> None:
    end = time.monotonic() + FUTURE_TIMEOUT_S
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=SPIN_ONCE_TIMEOUT_S)
        rclpy.spin_once(helper, timeout_sec=SPIN_ONCE_TIMEOUT_S)
    assert future.done(), "future did not complete within timeout"


def test_move_to_pose_rejected_before_any_state_received(bringup_harness):
    """No state has been published; goal must be rejected at the gate."""
    node, helper, move_client, _, _ = bringup_harness

    goal = MoveToPose.Goal()
    goal.target_pose = PoseStamped()
    goal.target_pose.header.frame_id = "base_link"

    future = move_client.send_goal_async(goal)
    _spin_until_complete(node, helper, future)
    goal_handle = future.result()

    assert goal_handle.accepted is False, \
        "must reject move_to_pose before any /fortis/mission_state has been seen"


def test_open_gripper_rejected_before_any_state_received(bringup_harness):
    """Same race for the gripper service."""
    node, helper, _, open_client, _ = bringup_harness

    future = open_client.call_async(Trigger.Request())
    _spin_until_complete(node, helper, future)
    response = future.result()

    assert response.success is False, \
        "open_gripper must return success=False before any state has arrived"
    assert "rejected" in response.message, \
        f"expected rejection message; got {response.message!r}"


def test_close_gripper_rejected_before_any_state_received(bringup_harness):
    node, helper, _, _, close_client = bringup_harness

    future = close_client.call_async(Trigger.Request())
    _spin_until_complete(node, helper, future)
    response = future.result()

    assert response.success is False
    assert "rejected" in response.message
