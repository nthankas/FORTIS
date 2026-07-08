"""
Arm motion node: IK-backed context flag plus MoveToPose execution.

Subscribes
----------
    /fortis/target_pose     geometry_msgs/PoseStamped (latched)
        From fortis_perception/target_selector, anchored in `odom` when TF
        allows, else `base_link`. Every target is solved with
        fortis_arm.arm_ik and drives the latched /fortis/context/ik_ok
        flag -- the TARGETING -> ARM_AT_VIEW guard in
        fortis_safety.mission_state_machine.
    /joint_states           sensor_msgs/JointState
        Arm feedback from teensy_bridge; start point for interpolation and
        the convergence check.
    /fortis/mission_state   std_msgs/String (latched)
        Gate for MoveToPose goals, mirroring arm_controller's gripper gate.

Publishes
---------
    /fortis/context/ik_ok            std_msgs/Bool (latched)
    /fortis/arm/command              sensor_msgs/JointState waypoints
    /fortis/events/arm_at_view_pose  std_msgs/Empty
        Fired after a successful move when publish_view_event is true;
        Event.ARM_AT_VIEW_POSE moves the FSM TARGETING -> ARM_AT_VIEW under
        the ik_ok guard.

Action
------
    move_to_pose    fortis_msgs/action/MoveToPose
        IK the goal pose; abort with a message when unreachable or TF
        fails; otherwise stream joint waypoints at joint_speed_rad_s with
        feedback progress 0..1, succeeding once /joint_states settles
        within joint_tolerance_rad (abort after move_timeout_s).

State gating: goals are accepted only in TARGETING, ARM_AT_VIEW, INSPECT
and PICK. TARGETING is where the view-pose move happens (its success emits
the event that leaves TARGETING); the other three are reposition states.
HOLDING and RETURN_HOME are excluded on purpose: large repositions while
carrying a payload are a mission-level decision, not an operator shortcut.

Frames: targets are reduced to base_link via TF (skipped when already in
base_link), then into the arm base (arm_mount) frame with the fixed URDF
transform hardcoded below.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.action import MoveToPose

from fortis_arm.arm_ik import inverse_kinematics

NODE_NAME = "arm_motion"

BASE_FRAME = "base_link"
TARGET_POSE_TOPIC = "/fortis/target_pose"
IK_OK_TOPIC = "/fortis/context/ik_ok"
ARM_COMMAND_TOPIC = "/fortis/arm/command"
JOINT_STATES_TOPIC = "/joint_states"
MISSION_STATE_TOPIC = "/fortis/mission_state"
#: Exact FSM event topic: mission_state_node subscribes
#: /fortis/events/<Event name lowercased> for Event.ARM_AT_VIEW_POSE.
ARM_AT_VIEW_EVENT_TOPIC = "/fortis/events/arm_at_view_pose"
MOVE_TO_POSE_ACTION = "move_to_pose"

#: base_link -> arm_mount, from fortis_chassis.urdf.xacro: xyz =
#: (j1_mount_x, 0, belly_clearance + chassis_height) = (0.0898, 0,
#: 0.051 + 0.152) with yaw pi, so arm +X points along base_link -X (the
#: FORTIS chassis front).
ARM_MOUNT_X = 0.0898
ARM_MOUNT_Z = 0.203

#: Mission states in which MoveToPose goals are honoured (see module
#: docstring for the rationale).
ALLOWED_MOTION_STATES: "frozenset[str]" = frozenset({
    "TARGETING",
    "ARM_AT_VIEW",
    "INSPECT",
    "PICK",
})

#: Commanded arm joints, in fortis_arm.arm_ik solution order.
MOTION_JOINT_NAMES = ("joint_j1", "joint_j2", "joint_j3", "joint_j4")

TF_TIMEOUT_S = 0.5
#: How long the execute path waits for the first full /joint_states.
_JOINT_WAIT_S = 2.0
#: Poll period while waiting for the arm to settle on target.
_SETTLE_POLL_S = 0.05


def _base_to_arm(x: float, y: float, z: float) -> "tuple[float, float, float]":
    """Map a base_link point into the arm base (arm_mount) frame."""
    # Inverse of translate(ARM_MOUNT_X, 0, ARM_MOUNT_Z) then Rz(pi).
    return (ARM_MOUNT_X - x, -y, z - ARM_MOUNT_Z)


class ArmMotionNode(Node):
    """Own the arm's ik_ok context flag and the MoveToPose action server."""

    def __init__(self, **kwargs) -> None:
        super().__init__(NODE_NAME, **kwargs)

        self._joint_speed = self._declf("joint_speed_rad_s", 0.3)
        self._command_rate = self._declf("command_rate_hz", 20.0)
        self._tolerance = self._declf("joint_tolerance_rad", 0.05)
        self._move_timeout = self._declf("move_timeout_s", 30.0)
        self._publish_view_event = bool(
            self.declare_parameter("publish_view_event", True).value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._mission_state: "str | None" = None
        self._joints: "dict[str, float]" = {}
        self._joints_lock = threading.Lock()
        self._ik_ok: "bool | None" = None

        latched = latched_qos_profile()
        self._ik_ok_pub = self.create_publisher(Bool, IK_OK_TOPIC, latched)
        self._cmd_pub = self.create_publisher(
            JointState, ARM_COMMAND_TOPIC, 10)
        self._view_event_pub = self.create_publisher(
            Empty, ARM_AT_VIEW_EVENT_TOPIC, 10)

        self.create_subscription(
            String, MISSION_STATE_TOPIC, self._on_mission_state, latched)
        self.create_subscription(
            PoseStamped, TARGET_POSE_TOPIC, self._on_target, latched)
        self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_states, 10)

        # Reentrant group so a long-running execute cannot starve goal /
        # cancel handling under the MultiThreadedExecutor.
        self._action_server = ActionServer(
            self,
            MoveToPose,
            MOVE_TO_POSE_ACTION,
            execute_callback=self._execute_move,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

        self._publish_ik_ok(False)
        self.get_logger().info(
            "arm_motion up; move_to_pose goals allowed in "
            f"{sorted(ALLOWED_MOTION_STATES)}")

    # --- parameter helper ---------------------------------------------------

    def _declf(self, name: str, default: float) -> float:
        return float(self.declare_parameter(name, default).value)

    # --- subscriptions --------------------------------------------------------

    def _on_mission_state(self, msg: String) -> None:
        self._mission_state = msg.data

    def _on_joint_states(self, msg: JointState) -> None:
        with self._joints_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joints[name] = float(pos)

    def _on_target(self, msg: PoseStamped) -> None:
        """Refresh the latched ik_ok flag for every published target."""
        solutions = self._solve_pose(msg)
        self._publish_ik_ok(bool(solutions))

    # --- IK plumbing ----------------------------------------------------------

    def _publish_ik_ok(self, ok: bool) -> None:
        """Publish the latched context flag, only on change."""
        if ok == self._ik_ok:
            return
        self._ik_ok = ok
        self._ik_ok_pub.publish(Bool(data=ok))
        self.get_logger().info(f"ik_ok -> {ok}")

    def _solve_pose(self, pose: PoseStamped):
        """Return IK solutions for a stamped pose, or None when TF fails."""
        frame = pose.header.frame_id or BASE_FRAME
        p = pose.pose.position
        x, y, z = p.x, p.y, p.z
        if frame != BASE_FRAME:
            try:
                tf = self._tf_buffer.lookup_transform(
                    BASE_FRAME, frame, Time(),
                    timeout=Duration(seconds=TF_TIMEOUT_S))
            except TransformException as exc:
                self.get_logger().warning(
                    f"target in frame '{frame}' unresolved, no TF to "
                    f"{BASE_FRAME}: {exc}", throttle_duration_sec=5.0)
                return None
            ps = PointStamped()
            ps.header.frame_id = frame
            ps.point.x, ps.point.y, ps.point.z = x, y, z
            p_base = tf2_geometry_msgs.do_transform_point(ps, tf)
            x, y, z = p_base.point.x, p_base.point.y, p_base.point.z
        return inverse_kinematics(_base_to_arm(x, y, z))

    # --- action server --------------------------------------------------------

    def _on_goal(self, _goal_request) -> GoalResponse:
        """Accept goals only inside the allowed mission states."""
        state = self._mission_state
        if state in ALLOWED_MOTION_STATES:
            return GoalResponse.ACCEPT
        self.get_logger().warning(
            f"move_to_pose goal rejected in state {state}: allowed "
            f"{sorted(ALLOWED_MOTION_STATES)}", throttle_duration_sec=1.0)
        return GoalResponse.REJECT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        """Honour every cancel request."""
        return CancelResponse.ACCEPT

    def _execute_move(self, goal_handle) -> MoveToPose.Result:
        """Run one accepted MoveToPose goal to completion."""
        result = MoveToPose.Result()
        solutions = self._solve_pose(goal_handle.request.target_pose)
        if solutions is None:
            return self._abort(
                goal_handle, result, "no TF to base_link for target frame")
        if not solutions:
            return self._abort(
                goal_handle, result, "target unreachable for the 4-DOF arm")
        start = self._current_positions()
        if start is None:
            return self._abort(
                goal_handle, result, "no /joint_states from the arm bridge")

        goal_q = min(
            solutions,
            key=lambda s: sum(abs(a - b) for a, b in zip(s, start)))
        deltas = [g - s for g, s in zip(goal_q, start)]
        period = 1.0 / max(self._command_rate, 1.0)
        duration = max(
            max(abs(d) for d in deltas) / max(self._joint_speed, 1e-3),
            period)
        steps = max(1, math.ceil(duration / period))

        feedback = MoveToPose.Feedback()
        for i in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, result)
            alpha = i / steps
            self._publish_command(
                [s + alpha * d for s, d in zip(start, deltas)])
            # Reserve the last 10% of progress for physical convergence.
            feedback.progress = 0.9 * alpha
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        deadline = time.monotonic() + self._move_timeout
        while not self._within_tolerance(goal_q):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, result)
            if time.monotonic() > deadline:
                return self._abort(
                    goal_handle, result,
                    f"arm did not settle within {self._move_timeout:.0f}s")
            time.sleep(_SETTLE_POLL_S)

        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        if self._publish_view_event:
            self._view_event_pub.publish(Empty())
        goal_handle.succeed()
        result.succeeded = True
        result.message = "reached target within tolerance"
        return result

    # --- execute helpers --------------------------------------------------------

    def _abort(self, goal_handle, result, message: str) -> MoveToPose.Result:
        """Abort a goal with an explanatory result message."""
        self.get_logger().warning(f"move_to_pose aborted: {message}")
        goal_handle.abort()
        result.succeeded = False
        result.message = message
        return result

    def _cancel(self, goal_handle, result) -> MoveToPose.Result:
        """Finish a goal as canceled."""
        goal_handle.canceled()
        result.succeeded = False
        result.message = "canceled"
        return result

    def _current_positions(self) -> "list[float] | None":
        """Return the arm joint vector, waiting briefly for feedback."""
        deadline = time.monotonic() + _JOINT_WAIT_S
        while time.monotonic() < deadline:
            with self._joints_lock:
                if all(n in self._joints for n in MOTION_JOINT_NAMES):
                    return [self._joints[n] for n in MOTION_JOINT_NAMES]
            time.sleep(_SETTLE_POLL_S)
        return None

    def _publish_command(self, positions) -> None:
        """Publish one joint waypoint to the teensy bridge."""
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(MOTION_JOINT_NAMES)
        cmd.position = [float(p) for p in positions]
        self._cmd_pub.publish(cmd)

    def _within_tolerance(self, goal_q) -> bool:
        """Return True when every arm joint is within tolerance of goal_q."""
        with self._joints_lock:
            current = [self._joints.get(n) for n in MOTION_JOINT_NAMES]
        return all(
            c is not None and abs(c - g) <= self._tolerance
            for c, g in zip(current, goal_q))


def main(args=None):
    """Run the arm_motion node under a MultiThreadedExecutor."""
    rclpy.init(args=args)
    node = ArmMotionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
