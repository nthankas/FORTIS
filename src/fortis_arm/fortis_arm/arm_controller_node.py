"""
ROS 2 arm controller seam for FORTIS, gated by mission state.

Subscribes
----------
    /fortis/mission_state   std_msgs/String   current mission state name,
                                              latched (TRANSIENT_LOCAL).
                                              Same topic and QoS as
                                              fortis_drive consumes; copy
                                              the pattern, do not invent
                                              a new one.

Action server
-------------
    move_to_pose   fortis_msgs/action/MoveToPose
        Goal     geometry_msgs/PoseStamped target_pose
        Result   bool succeeded, string message
        Feedback float32 progress

        Goal callback rejects when the cached mission state is not in
        ALLOWED_ARM_STATES (ARM_AT_VIEW, INSPECT, PICK, HOLDING,
        RETURN_HOME). Accepted goals always return
        succeeded=False, message="kinematics not implemented" -- the
        contract is the deliverable, the motion is deferred to a later
        pass that wires up Teensy serial + IK + trajectory generation.

Services
--------
    open_gripper    std_srvs/srv/Trigger
    close_gripper   std_srvs/srv/Trigger
        Both are state-gated identically to the action. Calls outside
        ALLOWED_ARM_STATES return success=False with an explanatory
        message; calls inside return success=False with
        message="gripper actuation not implemented".

Why std_srvs/Trigger
--------------------
The gripper is a binary "open" / "close" command from the operator.
Trigger has no payload, just a success/message response, which matches
the contract exactly. A custom service in fortis_msgs would add a build
dependency for no behavioural gain.

Gating rules
------------
Arm motion (action and gripper services) is honoured only while the
mission state is one of ALLOWED_ARM_STATES. Any other state -- including
"no state has been received yet" -- causes a rejection. Rejection log
messages are throttled per state name so a flood of incoming requests
cannot flood the log.

Threading
---------
SingleThreadedExecutor is sufficient: the state subscription, the
action callbacks, and the service callbacks all do trivial work
(state comparison + log + immediate response). MultiThreaded would buy
nothing and would invite a data race on the cached state string. When
real motion lands and the action callback starts running a trajectory,
revisit -- a ReentrantCallbackGroup on the action server is the
standard upgrade path.
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
from std_srvs.srv import Trigger


# --- Constants ---------------------------------------------------------------

#: Mission states in which arm motion (action + gripper services) is
#: honoured. Anything else => reject. Frozenset so accidental mutation
#: at runtime raises rather than silently widening the gate.
ALLOWED_ARM_STATES: frozenset[str] = frozenset({
    "ARM_AT_VIEW",
    "INSPECT",
    "PICK",
    "HOLDING",
    "RETURN_HOME",
})

#: Throttle interval for the per-state "rejected request" warning.
REJECT_LOG_THROTTLE_S: float = 1.0

MISSION_STATE_TOPIC: str = "/fortis/mission_state"
MOVE_TO_POSE_ACTION: str = "move_to_pose"
OPEN_GRIPPER_SERVICE: str = "open_gripper"
CLOSE_GRIPPER_SERVICE: str = "close_gripper"

#: Throttle-history key used in place of a real state name when logging
#: a rejection that happened before any /fortis/mission_state arrived.
_UNKNOWN_STATE_KEY: str = "<no_state_received>"

#: Stub messages returned on accepted goals / service calls. Replace
#: when the actual kinematics + Teensy serial protocol land.
_STUB_ACTION_MESSAGE: str = "kinematics not implemented"
_STUB_GRIPPER_MESSAGE: str = "gripper actuation not implemented"


# --- Node --------------------------------------------------------------------


class ArmControllerNode(Node):
    """
    ROS node exposing the arm action + gripper services, gated by mission state.

    Holds the most recent mission state (or None until the first message
    arrives) and a per-state throttle map for rejection log messages.
    """

    def __init__(self) -> None:
        super().__init__("arm_controller")

        # Most recent mission state seen. None until first message;
        # guarantees the node rejects requests that race ahead of the
        # state publisher on bring-up.
        self._current_state: str | None = None

        # Per-state timestamp of the last reject warning. Manual throttle
        # because rclpy's throttle is per call site, not per message
        # content; we want one warning per state per second.
        self._last_reject_log: dict[str, Time] = {}

        # Latched QoS for the mission_state subscription. Matches the
        # publisher in fortis_safety/mission_state_node and the
        # subscription in fortis_drive/drive_node: TRANSIENT_LOCAL +
        # RELIABLE so a subscriber that connects after the publisher has
        # already announced a state still gets the most recent value on
        # connect, depth=1 because only the latest matters. Diverging
        # from the publisher's profile causes DDS to silently drop the
        # connection -- keep them in sync.
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

        self.create_service(
            Trigger, OPEN_GRIPPER_SERVICE, self._handle_open_gripper
        )
        self.create_service(
            Trigger, CLOSE_GRIPPER_SERVICE, self._handle_close_gripper
        )

        self.get_logger().info(
            "arm_controller up. Allowed states for arm motion: "
            f"{sorted(ALLOWED_ARM_STATES)}. Awaiting first mission_state."
        )

    # --- Subscription -----------------------------------------------------

    def _on_mission_state(self, msg: String) -> None:
        new_state = msg.data
        if new_state != self._current_state:
            self.get_logger().info(
                f"mission_state: {self._current_state} -> {new_state}"
            )
        self._current_state = new_state

    # --- Gating -----------------------------------------------------------

    def _state_allows_motion(self) -> bool:
        return (
            self._current_state is not None
            and self._current_state in ALLOWED_ARM_STATES
        )

    def _log_rejection(self, kind: str) -> None:
        """Log a rejection, throttled per state name + request kind."""
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

    # --- Action server ----------------------------------------------------

    def _goal_callback(self, _goal_request) -> GoalResponse:
        """Accept or reject a new goal based on current mission state."""
        if self._state_allows_motion():
            return GoalResponse.ACCEPT
        self._log_rejection("move_to_pose goal")
        return GoalResponse.REJECT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        # Cancellation is always accepted -- a scaffold goal does no
        # work, so cancelling is free. When real motion lands, this
        # callback is the place to wire up trajectory abort.
        return CancelResponse.ACCEPT

    def _execute_move_to_pose(
        self, goal_handle: ServerGoalHandle
    ) -> MoveToPose.Result:
        """Stub execution: immediately return succeeded=False, "not implemented"."""
        self.get_logger().info(
            "move_to_pose accepted in state "
            f"{self._current_state}; returning stub result."
        )
        goal_handle.succeed()
        result = MoveToPose.Result()
        result.succeeded = False
        result.message = _STUB_ACTION_MESSAGE
        return result

    # --- Gripper services -------------------------------------------------

    def _handle_open_gripper(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._handle_gripper(response, kind="open_gripper")

    def _handle_close_gripper(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._handle_gripper(response, kind="close_gripper")

    def _handle_gripper(
        self, response: Trigger.Response, kind: str
    ) -> Trigger.Response:
        if not self._state_allows_motion():
            self._log_rejection(kind)
            response.success = False
            response.message = (
                f"rejected: state {self._current_state or _UNKNOWN_STATE_KEY} "
                f"not in {sorted(ALLOWED_ARM_STATES)}"
            )
            return response
        # Allowed-state path: stub. False + explanation.
        response.success = False
        response.message = _STUB_GRIPPER_MESSAGE
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
