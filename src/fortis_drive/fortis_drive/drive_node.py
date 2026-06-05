"""
ROS 2 wrapper around the FORTIS X-drive inverse kinematics, gated by mission state.

Subscribes
----------
    /cmd_vel                geometry_msgs/Twist   desired chassis velocity
    /fortis/mission_state   std_msgs/String       current mission state name,
                                                  latched (TRANSIENT_LOCAL)

Publishes
---------
    /fortis/drive/wheel_velocities   fortis_msgs/WheelVelocities
        Per-wheel angular velocity command (rad/s at the wheel shaft).
        Republished on every accepted /cmd_vel.
        Kept alongside the controller-bound topic during the
        odrive_ros2_control bring-up so existing consumers (tests,
        Foxglove dashboards) keep working. Slated for retirement once
        the new path is bench-verified.

    /fortis/drive/zero_velocities    fortis_msgs/WheelVelocities
        All-zero wheel velocities. Republished on every rejected /cmd_vel
        so a downstream stop is explicit, not silent.

    /wheel_velocity_controller/commands   std_msgs/Float64MultiArray
        Per-wheel angular velocity command (rad/s) for
        controller_manager's velocity_controllers/JointGroupVelocityController.
        Array order is locked by fortis_control/config/fortis_drive_controllers.yaml:
            [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint]
        Republished on every accepted /cmd_vel AND on every rejected
        /cmd_vel (as all zeros), so the controller never has to infer a
        stop from the absence of a message.

Why two output channels for the same data
-----------------------------------------
A safety brake or motor controller should never have to infer "motion is
suppressed" from the *absence* of a message. The zero_velocities topic is
the explicit "I would have moved you, but the current state forbids it"
channel; downstream nodes can react to it the same way they react to
wheel_velocities (latch onto it, transition state, light an LED, etc.).

The wheel_velocity_controller/commands topic carries the same intent but
in the shape ros2_control wants. We publish to both because:
  * fortis_msgs/WheelVelocities preserves the explicit "this is the
    rejection" semantics via the separate zero_velocities topic for
    code that reads it (tests, dashboards, future safety hooks).
  * Float64MultiArray is the only shape JointGroupVelocityController
    accepts; on rejected /cmd_vel we publish zeros into it so the
    motors are explicitly commanded to a halt rather than left at the
    last accepted setpoint.

Gating rules
------------
The drive accepts /cmd_vel only while the mission state is one of the
ALLOWED_DRIVE_STATES (ORBIT or RETURN_HOME). Any other state -- including
"no state has been received yet" -- causes the command to be rejected and
zeros to be published. Rejections log a warning at most once per second per
state name, throttled manually so a flood of /cmd_vel messages can't flood
the log.

Safety stops
------------
Two mechanisms guarantee the wheels stop independent of the teleop client:
a /cmd_vel dead-man watchdog (CMD_VEL_TIMEOUT_S) and a gate-close stop on
leaving a driving state. See the CMD_VEL_TIMEOUT_S constant and _on_state /
_on_watchdog for the details and rationale.

Wheel direction / command frame
--------------------------------
Two hardware-boundary corrections sit between the kinematics and the wheels,
each documented in full on its constant: WHEEL_DIRECTION (per-wheel sign for
the mirror-mounted right-side motors) and CMD_VEL_FRAME_SIGN (the base_link
-X "front" convention). The H matrix stays ideal; the corrections live here.

Why std_msgs/String for the state subscription
----------------------------------------------
fortis_safety/mission_state_node currently publishes the canonical state
as a latched std_msgs/String. We mirror that here rather than depending on
fortis_msgs/MissionState so this node keeps working through the planned
migration to the richer message type. When the canonical publisher
switches, this subscription updates; the gating logic does not change.

Threading
---------
Single-threaded executor on purpose. The callbacks (cmd_vel, mission_state,
watchdog timer) are short, allocate nothing significant, and have no async
work to wait on. A MultiThreadedExecutor would add no throughput and
introduce a data race on the cached state. If real-time guarantees are ever
needed, revisit -- but it should not be the default.
"""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from fortis_comms.qos_profiles import latched_qos_profile
from fortis_comms.xdrive_kinematics import WHEEL_RADIUS, xdrive_ik_solver
from fortis_msgs.msg import WheelVelocities
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray, String


# --- Constants ---------------------------------------------------------------

#: Mission states in which /cmd_vel is honoured. Anything else => publish
#: zeros and warn. Kept as a frozenset so accidental mutation at runtime
#: raises rather than silently widening the gate.
ALLOWED_DRIVE_STATES: frozenset[str] = frozenset({"ORBIT", "RETURN_HOME"})

#: Throttle interval for the per-state "rejected /cmd_vel" warning. /cmd_vel
#: typically arrives at 20-50 Hz; without throttling the log would be
#: useless during any non-driving state.
REJECT_LOG_THROTTLE_S: float = 1.0

#: Per-wheel command-direction sign, canonical [FL, FR, RL, RR] order. The
#: right-side motors (FR, RR) are mirror-mounted, so a positive command
#: spins them backward; these signs invert them so "+ = roll forward" holds
#: for every wheel. This is a physical *mounting* correction applied at the
#: hardware boundary -- it lives here and NOT in the kinematics (the H matrix
#: stays "ideal"). Verified by single-wheel bench test 2026-05-31. Signs the
#: COMMAND only; encoder feedback keeps each motor's native sign.
WHEEL_DIRECTION: tuple[float, float, float, float] = (1.0, -1.0, 1.0, -1.0)

#: Sign applied to the /cmd_vel (Vx, Vy, wz) before the kinematics, in canonical
#: order. FORTIS's chassis "front" is base_link -X (opposite REP-103; see the
#: chassis-front convention), but teleop sends +linear.x to mean "drive toward
#: the front". So Vx is negated: a forward command moves the robot toward its
#: front, not its back. This is a *command-frame* correction at the hardware
#: boundary -- the H matrix stays pure base_link kinematics, like WHEEL_DIRECTION
#: is the motor-mounting correction.
#: NOTE: only Vx is flipped (the confirmed forward/back inversion). If strafe
#: (Vy) is ALSO inverted on the robot, the front flip is a full 180deg yaw and
#: Vy should become -1.0 too; left at +1.0 until strafe is verified. wz (yaw) is
#: unchanged -- a 180deg yaw about Z does not flip the rotation sense.
CMD_VEL_FRAME_SIGN: tuple[float, float, float] = (-1.0, 1.0, 1.0)

#: /cmd_vel watchdog timeout. Teleop sources stop publishing on release
#: without sending a zero, and the velocity controller latches its last
#: setpoint -- so without this the robot keeps moving. If no /cmd_vel
#: arrives within this window, command zeros (dead-man stop).
CMD_VEL_TIMEOUT_S: float = 0.5

#: Watchdog tick period (10 Hz): a released teleop stops within
#: ~CMD_VEL_TIMEOUT_S + WATCHDOG_PERIOD_S, cheap enough to ignore.
WATCHDOG_PERIOD_S: float = 0.1

CMD_VEL_TOPIC: str = "/cmd_vel"
MISSION_STATE_TOPIC: str = "/fortis/mission_state"
WHEEL_VELOCITIES_TOPIC: str = "/fortis/drive/wheel_velocities"
ZERO_VELOCITIES_TOPIC: str = "/fortis/drive/zero_velocities"
#: Topic consumed by velocity_controllers/JointGroupVelocityController as
#: configured in fortis_control/config/fortis_drive_controllers.yaml.
#: Array order MUST match that config's `joints:` list.
WHEEL_CONTROLLER_COMMAND_TOPIC: str = "/wheel_velocity_controller/commands"

#: Throttle-history key used in place of a real state name when logging a
#: rejection that happened before any /fortis/mission_state arrived. Kept
#: separate so the unknown-state path doesn't share a slot with any real
#: state name.
_UNKNOWN_STATE_KEY: str = "<no_state_received>"


# --- Helpers -----------------------------------------------------------------


@dataclass(frozen=True)
class WheelCommand:
    """Per-wheel angular velocity command in rad/s, in canonical FL/FR/RL/RR order."""

    fl: float
    fr: float
    rl: float
    rr: float

    @classmethod
    def zero(cls) -> WheelCommand:
        """Return an all-zero command, used for rejections and explicit stops."""
        return cls(fl=0.0, fr=0.0, rl=0.0, rr=0.0)


def _twist_to_wheel_command(cmd: Twist) -> WheelCommand:
    """
    Map a Twist (Vx, Vy, omega) to per-wheel angular velocities (rad/s).

    Calls into xdrive_kinematics.xdrive_ik_solver (which returns wheel linear
    speeds in m/s, already saturated to MAX_WHEEL_SPEED), divides by the wheel
    radius to get wheel shaft angular velocity, then applies WHEEL_DIRECTION to
    correct for the mirror-mounted right-side motors. The H matrix encodes the
    FL/FR/RL/RR wheel order; we preserve it.
    """
    wheel_linear = xdrive_ik_solver(
        cmd.linear.x * CMD_VEL_FRAME_SIGN[0],
        cmd.linear.y * CMD_VEL_FRAME_SIGN[1],
        cmd.angular.z * CMD_VEL_FRAME_SIGN[2],
    )
    return WheelCommand(
        fl=float(wheel_linear[0]) / WHEEL_RADIUS * WHEEL_DIRECTION[0],
        fr=float(wheel_linear[1]) / WHEEL_RADIUS * WHEEL_DIRECTION[1],
        rl=float(wheel_linear[2]) / WHEEL_RADIUS * WHEEL_DIRECTION[2],
        rr=float(wheel_linear[3]) / WHEEL_RADIUS * WHEEL_DIRECTION[3],
    )


def _wheel_command_to_msg(cmd: WheelCommand, stamp: TimeMsg) -> WheelVelocities:
    """Serialise a WheelCommand into a fortis_msgs/WheelVelocities message."""
    msg = WheelVelocities()
    msg.fl = cmd.fl
    msg.fr = cmd.fr
    msg.rl = cmd.rl
    msg.rr = cmd.rr
    msg.stamp = stamp
    return msg


def _wheel_command_to_controller_array(cmd: WheelCommand) -> Float64MultiArray:
    """Serialise a WheelCommand into the controller's Float64MultiArray.

    Array order is [fl, fr, rl, rr] to match
    fortis_control/config/fortis_drive_controllers.yaml's `joints:` list,
    which the velocity_controllers/JointGroupVelocityController consumes.
    """
    msg = Float64MultiArray()
    msg.data = [cmd.fl, cmd.fr, cmd.rl, cmd.rr]
    return msg


# --- Node --------------------------------------------------------------------


class DriveNode(Node):
    """
    ROS node that converts /cmd_vel to wheel velocities, gated by mission state.

    Runtime state: the most recent mission state string (or None), a per-state
    timestamp of the last "rejected /cmd_vel" warning (for throttling), and the
    timestamp of the last /cmd_vel (for the dead-man watchdog).
    """

    def __init__(self, cmd_vel_timeout_s: float = CMD_VEL_TIMEOUT_S) -> None:
        super().__init__("drive_node")

        # Dead-man timeout for this instance. Injectable so tests can pass a
        # large value (watchdog effectively disabled, for timing-stable gating
        # tests) or a tiny one (fires quickly). Production uses the
        # CMD_VEL_TIMEOUT_S default.
        self._cmd_vel_timeout_s: float = cmd_vel_timeout_s

        # Most recent mission state seen on /fortis/mission_state. None
        # until the first message arrives; this guarantees we reject
        # /cmd_vel that races ahead of the state publisher on bring-up.
        self._current_state: str | None = None

        # Per-state timestamp of the last reject warning. Manual throttling
        # because rclpy's throttle is per call site, not per message
        # content; we want one warning per state per second, not one
        # warning per second across all states.
        self._last_reject_log: dict[str, Time] = {}

        # Dead-man watchdog state: time of the last /cmd_vel, and whether we
        # have already commanded a stop because it went stale (so we zero
        # once per stale episode, not every tick).
        self._last_cmd_vel_time: Time | None = None
        self._cmd_vel_stale: bool = False

        # Latched QoS for the mission_state subscription. Matches the
        # publisher in fortis_safety/mission_state_node: TRANSIENT_LOCAL
        # durability + RELIABLE so a subscriber that connects after the
        # publisher has already announced a state still gets the most
        # recent value on connect, depth=1 because only the latest
        # matters. If this profile diverges from the publisher's, DDS
        # silently drops the connection -- keep them in sync.
        latched_qos = latched_qos_profile()

        self._wheel_pub = self.create_publisher(
            WheelVelocities, WHEEL_VELOCITIES_TOPIC, 10
        )
        self._zero_pub = self.create_publisher(
            WheelVelocities, ZERO_VELOCITIES_TOPIC, 10
        )
        # Bound for ros2_control's JointGroupVelocityController. Single
        # publisher used for both the accepted and rejected paths -- a
        # rejected /cmd_vel publishes an all-zero array here so the
        # controller is explicitly commanded to halt, never left at the
        # last accepted setpoint waiting for a message that may not come.
        self._wheel_controller_pub = self.create_publisher(
            Float64MultiArray, WHEEL_CONTROLLER_COMMAND_TOPIC, 10
        )

        self._state_sub = self.create_subscription(
            String,
            MISSION_STATE_TOPIC,
            self._on_state,
            latched_qos,
        )
        self._cmd_sub = self.create_subscription(
            Twist,
            CMD_VEL_TOPIC,
            self._on_cmd_vel,
            10,
        )

        # Dead-man watchdog: zeros the wheels if /cmd_vel goes stale.
        self._watchdog_timer = self.create_timer(
            WATCHDOG_PERIOD_S, self._on_watchdog
        )

        self.get_logger().info(
            f"drive_node up. Gated by states: "
            f"{sorted(ALLOWED_DRIVE_STATES)}. cmd_vel watchdog "
            f"{self._cmd_vel_timeout_s:.2f}s. Awaiting first mission_state."
        )

    # --- Callbacks ----------------------------------------------------------

    def _on_state(self, msg: String) -> None:
        """Cache the latest mission state; stop the wheels on gate-close."""
        new_state = msg.data
        if new_state == self._current_state:
            return  # idempotent re-publish; nothing to do
        previous = self._current_state
        self._current_state = new_state
        # Drop the per-state throttle history when leaving a rejecting
        # state so the *next* rejection in that state is logged
        # immediately rather than being suppressed by a stale entry.
        if previous is None:
            self._last_reject_log.pop(_UNKNOWN_STATE_KEY, None)
        else:
            self._last_reject_log.pop(previous, None)
        # Entering a non-driving state: command an explicit stop NOW rather
        # than waiting for the next /cmd_vel (which may never come if the
        # operator released teleop). Makes STOP / any gate-close halt the
        # wheels immediately.
        if new_state not in ALLOWED_DRIVE_STATES:
            self._publish_zero(self.get_clock().now().to_msg())
        previous_label = previous if previous is not None else "<none>"
        self.get_logger().info(
            f"mission_state: {previous_label} -> {new_state}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Translate /cmd_vel into wheel velocities, gated by mission state."""
        now = self.get_clock().now()
        # Freshen the watchdog: a command just arrived.
        self._last_cmd_vel_time = now
        self._cmd_vel_stale = False
        stamp = now.to_msg()
        state = self._current_state

        if state is None or state not in ALLOWED_DRIVE_STATES:
            self._publish_zero(stamp)
            self._log_reject(state)
            return

        cmd = _twist_to_wheel_command(msg)
        self._wheel_pub.publish(_wheel_command_to_msg(cmd, stamp))
        self._wheel_controller_pub.publish(_wheel_command_to_controller_array(cmd))
        self.get_logger().debug(
            f"cmd_vel accepted in {self._current_state}: "
            f"Vx={msg.linear.x:.3f} Vy={msg.linear.y:.3f} "
            f"wz={msg.angular.z:.3f} -> "
            f"FL={cmd.fl:.2f} FR={cmd.fr:.2f} "
            f"RL={cmd.rl:.2f} RR={cmd.rr:.2f} rad/s"
        )

    def _on_watchdog(self) -> None:
        """Dead-man stop: if /cmd_vel has gone stale, command zeros once.

        Teleop panels stop publishing on release without sending a zero, and
        the velocity controller latches its last setpoint -- so without this
        the wheels keep spinning after the operator lets go. We zero once per
        stale episode (the controller then holds zero); a fresh /cmd_vel
        clears the stale flag in _on_cmd_vel.
        """
        if self._last_cmd_vel_time is None:
            return  # never commanded; nothing to stop
        if self._cmd_vel_stale:
            return  # already stopped; controller is holding zero
        elapsed = self.get_clock().now() - self._last_cmd_vel_time
        if elapsed <= Duration(seconds=self._cmd_vel_timeout_s):
            return
        self._cmd_vel_stale = True
        self._publish_zero(self.get_clock().now().to_msg())
        self.get_logger().warning(
            f"/cmd_vel stale > {self._cmd_vel_timeout_s:.2f}s; "
            f"commanding zero (dead-man stop)."
        )

    # --- Helpers ------------------------------------------------------------

    def _publish_zero(self, stamp: TimeMsg) -> None:
        """Publish all-zero wheel velocities on the explicit-stop topics.

        Emits on both the legacy WheelVelocities zero topic AND the
        ros2_control controller-bound topic. The controller path must
        receive zeros explicitly so it never coasts at the last accepted
        setpoint -- used by the rejection path, the gate-close stop, and the
        dead-man watchdog.
        """
        zero = WheelCommand.zero()
        self._zero_pub.publish(_wheel_command_to_msg(zero, stamp))
        self._wheel_controller_pub.publish(_wheel_command_to_controller_array(zero))

    def _log_reject(self, state: str | None) -> None:
        """Log a per-state-throttled warning that /cmd_vel was rejected."""
        key = state if state is not None else _UNKNOWN_STATE_KEY
        now = self.get_clock().now()
        last = self._last_reject_log.get(key)
        if last is not None:
            elapsed = now - last
            if elapsed < Duration(seconds=REJECT_LOG_THROTTLE_S):
                return
        self._last_reject_log[key] = now
        if state is None:
            self.get_logger().warning(
                "Rejecting /cmd_vel: no /fortis/mission_state received yet"
            )
        else:
            self.get_logger().warning(
                f"Rejecting /cmd_vel in state {state}: "
                f"motion only allowed in {sorted(ALLOWED_DRIVE_STATES)}"
            )


def main(args: list[str] | None = None) -> None:
    """Entry point registered as the `drive_node` console script."""
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
