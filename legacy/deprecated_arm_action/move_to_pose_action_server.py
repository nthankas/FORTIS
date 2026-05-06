"""
DEPRECATED: MoveToPose action-server scaffold extracted from
``fortis_arm/arm_controller_node.py``.

This file is preserved for history and is not imported by the current
build. Replaced by ros2_control + standard ROS 2 packages.

Originally lived inside ``ArmControllerNode`` in
``src/fortis_arm/fortis_arm/arm_controller_node.py``. The node exposed
a ``move_to_pose`` action whose goal callback rejected when the cached
mission state was not in ``ALLOWED_ARM_STATES``; accepted goals always
returned ``succeeded=False, message="kinematics not implemented"``.

The block below reproduces the action-server wiring as a standalone
node so the original contract can be inspected without having to dig
through git history. Gripper services and mission-state subscription
behaviour are kept here only to make the file runnable in isolation;
the live versions of those pieces remain in ``arm_controller_node.py``.
"""

from __future__ import annotations

import rclpy
from fortis_msgs.action import MoveToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String


ALLOWED_ARM_STATES: frozenset[str] = frozenset({
    "ARM_AT_VIEW",
    "INSPECT",
    "PICK",
    "HOLDING",
    "RETURN_HOME",
})

REJECT_LOG_THROTTLE_S: float = 1.0
MISSION_STATE_TOPIC: str = "/fortis/mission_state"
MOVE_TO_POSE_ACTION: str = "move_to_pose"
_UNKNOWN_STATE_KEY: str = "<no_state_received>"
_STUB_ACTION_MESSAGE: str = "kinematics not implemented"


class MoveToPoseActionServer(Node):
    """Standalone copy of the original MoveToPose action server scaffold."""

    def __init__(self) -> None:
        super().__init__("move_to_pose_action_server")

        self._current_state: str | None = None
        self._last_reject_log: dict[str, Time] = {}

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            String,
            MISSION_STATE_TOPIC,
            self._on_mission_state,
            latched_qos,
        )

        self._action_server = ActionServer(
            self,
            MoveToPose,
            MOVE_TO_POSE_ACTION,
            execute_callback=self._execute_move_to_pose,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

    def _on_mission_state(self, msg: String) -> None:
        new_state = msg.data
        if new_state != self._current_state:
            self.get_logger().info(
                f"mission_state: {self._current_state} -> {new_state}"
            )
        self._current_state = new_state

    def _state_allows_motion(self) -> bool:
        return (
            self._current_state is not None
            and self._current_state in ALLOWED_ARM_STATES
        )

    def _log_rejection(self, kind: str) -> None:
        state_key = self._current_state or _UNKNOWN_STATE_KEY
        throttle_key = f"{state_key}:{kind}"
        now = self.get_clock().now()
        last = self._last_reject_log.get(throttle_key)
        if last is None or (now - last) > Duration(seconds=REJECT_LOG_THROTTLE_S):
            self._last_reject_log[throttle_key] = now
            self.get_logger().warn(
                f"Rejected {kind} in state {state_key}: "
                f"motion only allowed in {sorted(ALLOWED_ARM_STATES)}"
            )

    def _goal_callback(self, _goal_request) -> GoalResponse:
        if self._state_allows_motion():
            return GoalResponse.ACCEPT
        self._log_rejection("move_to_pose goal")
        return GoalResponse.REJECT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_move_to_pose(
        self, goal_handle: ServerGoalHandle
    ) -> MoveToPose.Result:
        self.get_logger().info(
            "move_to_pose accepted in state "
            f"{self._current_state}; returning stub result."
        )
        goal_handle.succeed()
        result = MoveToPose.Result()
        result.succeeded = False
        result.message = _STUB_ACTION_MESSAGE
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
