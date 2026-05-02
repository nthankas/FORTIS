"""
ROS 2 wrapper around the FORTIS X-drive inverse kinematics, gated by mission
state.

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

    /fortis/drive/zero_velocities    fortis_msgs/WheelVelocities
        All-zero wheel velocities. Republished on every rejected /cmd_vel
        so a downstream stop is explicit, not silent.

Why two output topics
---------------------
A safety brake or motor controller should never have to infer "motion is
suppressed" from the *absence* of a message. The zero_velocities topic is
the explicit "I would have moved you, but the current state forbids it"
channel; downstream nodes can react to it the same way they react to
wheel_velocities (latch onto it, transition state, light an LED, etc.).

Gating rules
------------
The drive accepts /cmd_vel only while the mission state is one of the
ALLOWED_DRIVE_STATES (ORBIT or RETURN_HOME). Any other state -- including
"no state has been received yet" -- causes the command to be rejected and
zeros to be published. Rejections log a warning at most once per second per
state name, throttled manually so a flood of /cmd_vel messages can't flood
the log.

Why std_msgs/String for the state subscription
----------------------------------------------
fortis_safety/mission_state_node currently publishes the canonical state
as a latched std_msgs/String. We mirror that here rather than depending on
fortis_msgs/MissionState so this node keeps working through the planned
migration to the richer message type. When the canonical publisher
switches, this subscription updates; the gating logic does not change.

Threading
---------
Single-threaded executor on purpose. The two callbacks (cmd_vel and
mission_state) are short, allocate nothing significant, and have no async
work to wait on. A MultiThreadedExecutor would add no throughput and
introduce a data race on the cached state string. If real-time guarantees
are ever needed, revisit -- but it should not be the default.

Importing fortis_comms
----------------------
fortis_comms is a pre-ROS Python library at <repo>/control/fortis_comms.
It is not yet packaged (no setup.py), so colcon doesn't put it on
PYTHONPATH. We resolve <repo>/control at startup by walking up from this
file looking for the kinematics module, and prepend it to sys.path. See
_ensure_fortis_comms_importable() below for the rationale and limits of
this approach. The follow-up is to add a proper setup.py to fortis_comms;
when that lands, the sys.path shim here can be deleted.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String


# --- fortis_comms import shim ------------------------------------------------
#
# Done before importing xdrive_kinematics so the import resolves when this
# module is loaded from either src/ (during pytest) or
# install/<pkg>/lib/python3.10/site-packages/ (after colcon build).

def _ensure_fortis_comms_importable() -> None:
    """Add <repo>/control to sys.path if fortis_comms is not already importable.

    Walks up from this file looking for control/fortis_comms/xdrive_kinematics.py
    -- this finds the workspace root whether we are running from a colcon
    install tree or directly from src/. Falls back to /workspace (the dev
    container mount point) if the walk fails. Idempotent and silent on the
    happy path so it costs essentially nothing on subsequent imports.
    """
    try:
        import fortis_comms.xdrive_kinematics  # noqa: F401
        return
    except ImportError:
        pass

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "control" / "fortis_comms" / "xdrive_kinematics.py"
        if candidate.is_file():
            sys.path.insert(0, str(parent / "control"))
            return

    fallback = Path("/workspace/control")
    if (fallback / "fortis_comms" / "xdrive_kinematics.py").is_file():
        sys.path.insert(0, str(fallback))


_ensure_fortis_comms_importable()

from fortis_comms.xdrive_kinematics import (  # noqa: E402
    WHEEL_RADIUS,
    xdrive_ik_solver,
)
from fortis_msgs.msg import WheelVelocities  # noqa: E402


# --- Constants ---------------------------------------------------------------

#: Mission states in which /cmd_vel is honoured. Anything else => publish
#: zeros and warn. Kept as a frozenset so accidental mutation at runtime
#: raises rather than silently widening the gate.
ALLOWED_DRIVE_STATES: frozenset[str] = frozenset({"ORBIT", "RETURN_HOME"})

#: Throttle interval for the per-state "rejected /cmd_vel" warning. /cmd_vel
#: typically arrives at 20-50 Hz; without throttling the log would be
#: useless during any non-driving state.
REJECT_LOG_THROTTLE_S: float = 1.0

CMD_VEL_TOPIC: str = "/cmd_vel"
MISSION_STATE_TOPIC: str = "/fortis/mission_state"
WHEEL_VELOCITIES_TOPIC: str = "/fortis/drive/wheel_velocities"
ZERO_VELOCITIES_TOPIC: str = "/fortis/drive/zero_velocities"

#: Throttle-history key used in place of a real state name when logging a
#: rejection that happened before any /fortis/mission_state arrived. Kept
#: separate so the unknown-state path doesn't share a slot with any real
#: state name.
_UNKNOWN_STATE_KEY: str = "<no_state_received>"


# --- Helpers -----------------------------------------------------------------


@dataclass(frozen=True)
class WheelCommand:
    """Per-wheel angular velocity command in rad/s, in canonical FL/FR/BL/BR order."""

    fl: float
    fr: float
    bl: float
    br: float

    @classmethod
    def zero(cls) -> WheelCommand:
        """Return an all-zero command, used for rejections and explicit stops."""
        return cls(fl=0.0, fr=0.0, bl=0.0, br=0.0)


def _twist_to_wheel_command(cmd: Twist) -> WheelCommand:
    """Map a Twist (Vx, Vy, omega) to per-wheel angular velocities (rad/s).

    Calls into xdrive_kinematics.xdrive_ik_solver (which returns wheel linear
    speeds in m/s, already saturated to MAX_WHEEL_SPEED) and divides by the
    wheel radius to get wheel shaft angular velocity. The H matrix in
    fortis_comms encodes the FL/FR/BL/BR wheel order; we preserve it.
    """
    wheel_linear = xdrive_ik_solver(
        cmd.linear.x,
        cmd.linear.y,
        cmd.angular.z,
    )
    return WheelCommand(
        fl=float(wheel_linear[0]) / WHEEL_RADIUS,
        fr=float(wheel_linear[1]) / WHEEL_RADIUS,
        bl=float(wheel_linear[2]) / WHEEL_RADIUS,
        br=float(wheel_linear[3]) / WHEEL_RADIUS,
    )


def _wheel_command_to_msg(cmd: WheelCommand, stamp: TimeMsg) -> WheelVelocities:
    """Serialise a WheelCommand into a fortis_msgs/WheelVelocities message."""
    msg = WheelVelocities()
    msg.fl = cmd.fl
    msg.fr = cmd.fr
    msg.bl = cmd.bl
    msg.br = cmd.br
    msg.stamp = stamp
    return msg


# --- Node --------------------------------------------------------------------


class DriveNode(Node):
    """ROS node that converts /cmd_vel to wheel velocities, gated by mission state.

    The node holds two pieces of runtime state: the most recent mission state
    string (or None if none has been received) and a per-state timestamp of
    the last "rejected /cmd_vel" warning, used for throttling.
    """

    def __init__(self) -> None:
        super().__init__("drive_node")

        # Most recent mission state seen on /fortis/mission_state. None
        # until the first message arrives; this guarantees we reject
        # /cmd_vel that races ahead of the state publisher on bring-up.
        self._current_state: str | None = None

        # Per-state timestamp of the last reject warning. Manual throttling
        # because rclpy's throttle is per call site, not per message
        # content; we want one warning per state per second, not one
        # warning per second across all states.
        self._last_reject_log: dict[str, Time] = {}

        # Latched QoS for the mission_state subscription. Matches the
        # publisher in fortis_safety/mission_state_node: TRANSIENT_LOCAL
        # durability + RELIABLE so a subscriber that connects after the
        # publisher has already announced a state still gets the most
        # recent value on connect, depth=1 because only the latest
        # matters. If this profile diverges from the publisher's, DDS
        # silently drops the connection -- keep them in sync.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._wheel_pub = self.create_publisher(
            WheelVelocities, WHEEL_VELOCITIES_TOPIC, 10
        )
        self._zero_pub = self.create_publisher(
            WheelVelocities, ZERO_VELOCITIES_TOPIC, 10
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

        self.get_logger().info(
            f"drive_node up. Gated by states: "
            f"{sorted(ALLOWED_DRIVE_STATES)}. Awaiting first mission_state."
        )

    # --- Callbacks ----------------------------------------------------------

    def _on_state(self, msg: String) -> None:
        """Cache the latest mission state and log transitions at info level."""
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
        previous_label = previous if previous is not None else "<none>"
        self.get_logger().info(
            f"mission_state: {previous_label} -> {new_state}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Translate /cmd_vel into wheel velocities, gated by mission state."""
        stamp = self.get_clock().now().to_msg()
        state = self._current_state

        if state is None or state not in ALLOWED_DRIVE_STATES:
            self._publish_zero(stamp)
            self._log_reject(state)
            return

        cmd = _twist_to_wheel_command(msg)
        self._wheel_pub.publish(_wheel_command_to_msg(cmd, stamp))
        self.get_logger().debug(
            f"cmd_vel accepted in {self._current_state}: "
            f"Vx={msg.linear.x:.3f} Vy={msg.linear.y:.3f} "
            f"wz={msg.angular.z:.3f} -> "
            f"FL={cmd.fl:.2f} FR={cmd.fr:.2f} "
            f"BL={cmd.bl:.2f} BR={cmd.br:.2f} rad/s"
        )

    # --- Helpers ------------------------------------------------------------

    def _publish_zero(self, stamp: TimeMsg) -> None:
        """Publish all-zero wheel velocities on the explicit-stop topic."""
        self._zero_pub.publish(_wheel_command_to_msg(WheelCommand.zero(), stamp))

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
