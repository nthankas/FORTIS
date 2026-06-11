"""
orbit_node -- hold-to-run, face-the-center orbit generator for the FORTIS X-drive.

What it does
------------
Turns a HELD direction command (one Foxglove Teleop button) into a continuous
face-the-center orbit: the chassis circles a point at a fixed radius while its
front cameras stay pointed at that point. Because the X-drive is holonomic, a
face-center orbit is a CONSTANT body-frame twist -- a tangential strafe plus a
coupled yaw rate -- so no path planner and no localization are needed
(open-loop). Speed defaults to 0.1 m/s because open-loop omni motion drifts;
the radius is a parameter ("arbitrary for now").

Subscribes
----------
    /fortis/commands/orbit_dir   geometry_msgs/Twist
        Held orbit command from the Foxglove Teleop panel. Only the SIGN of
        angular.z is read: > 0 => one way round, < 0 => the other, ~0 => stop.
        The panel republishes this at its publishRate while a button is held
        and stops the instant it is released -- which is what makes "hold to
        orbit, let go to stop" work with no latched state in this node.

Publishes
---------
    /cmd_vel   geometry_msgs/Twist
        The orbit twist (linear.y strafe + angular.z yaw), streamed at
        publish_rate_hz while the command is fresh. drive_node consumes this
        exactly like teleop /cmd_vel: it applies the mission-state gate, its
        own dead-man watchdog, and the wheel-direction / front-convention sign
        corrections. orbit_node stays in pure base_link body frame and never
        duplicates those hardware-boundary fixes.

Hold dead-man
-------------
A held button streams orbit_dir; releasing it stops the stream. If no orbit_dir
arrives within cmd_timeout_s the command is "stale": orbit_node publishes a
single zero /cmd_vel (explicit stop) and then goes quiet. Same zero-on-stale
discipline drive_node's watchdog uses, one layer up, so release-to-stop is
crisp instead of waiting out the drive watchdog's longer 0.5 s timeout.

Sign convention (bench-flippable)
---------------------------------
orbit_twist() encodes the IDEAL face-center kinematics in one convention. Which
physical direction is which, and whether the yaw must flip relative to the
strafe to keep the cameras (FORTIS front = base_link -X) pointed at the center,
depends on the mounting and is verified on the bench. That correction is the
omega_sign parameter applied here at the boundary -- flip it (or just swap the
two buttons) if the robot orbits facing outward. The kinematics in orbit_twist
stay ideal, mirroring how drive_node keeps the H matrix ideal and corrects
signs at the edge.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time


# --- Constants ---------------------------------------------------------------

ORBIT_DIR_TOPIC: str = "/fortis/commands/orbit_dir"
CMD_VEL_TOPIC: str = "/cmd_vel"

#: Default orbit speed (m/s at the contact patch). 0.1 is deliberately slow:
#: open-loop omni motion drifts, so the first bring-up keeps the tangential
#: speed low. Overridable via the orbit_speed parameter.
DEFAULT_ORBIT_SPEED: float = 0.1

#: Default orbit radius (m). "Arbitrary for now" -- set orbit_radius to taste.
#: Smaller R => the robot must yaw faster (omega = v / R) to keep facing center.
DEFAULT_ORBIT_RADIUS: float = 1.0

#: Boundary sign correction for the yaw direction (see module docstring).
#: Bench-verified 2026-06-11: -1.0 makes the FRONT (cameras) face the orbit
#: center; +1.0 orbited facing outward (back toward the center post).
DEFAULT_OMEGA_SIGN: float = -1.0

#: Held-command dead-man timeout (s). No orbit_dir within this window => the
#: button was released => stop. Shorter than drive_node's 0.5 s watchdog so
#: release-to-stop is crisp.
DEFAULT_CMD_TIMEOUT_S: float = 0.3

#: /cmd_vel publish rate (Hz) while orbiting. Stays well above 1/cmd_timeout_s
#: and drive_node's watchdog rate so the stream never looks stale mid-orbit.
DEFAULT_PUBLISH_RATE_HZ: float = 20.0

#: Below this |angular.z| an orbit_dir command counts as "stop", not a turn, so
#: a stray ~0 message cannot creep the robot.
DIR_DEADBAND: float = 1e-3


# --- Pure kinematics ---------------------------------------------------------


def orbit_twist(direction: float, speed: float, radius: float) -> tuple[float, float]:
    """
    Map an orbit direction to the body-frame (Vy, omega) of a face-center orbit.

    A holonomic robot circling a point at ``radius`` while keeping one face
    pointed AT that point moves purely tangentially (a strafe, no forward/back)
    and yaws at the orbit's own angular rate so its heading tracks the center.
    The behaviour is therefore a single constant twist with no Vx component.

    Parameters
    ----------
    direction : float
        Orbit direction. Sign picks the way round (+ one way, - the other);
        0 = stop. Callers pass +1.0, -1.0, or 0.0.
    speed : float
        Tangential orbit speed v (m/s at the contact patch).
    radius : float
        Orbit radius R (m), > 0 (the node validates this before calling).

    Returns
    -------
    (vy, omega) : tuple[float, float]
        vy    -- body-frame strafe (m/s) along base_link Y.
        omega -- body-frame yaw rate (rad/s) about base_link Z.
    """
    # Face-center orbit: all motion is tangential (a pure strafe at the orbit
    # speed -- no Vx), and the robot yaws at the orbit's own angular rate
    # omega = v / R so its nose tracks the center as it goes around. direction
    # (+1/-1/0) scales both so reversing it mirrors the orbit and 0 stops.
    vy = direction * speed
    omega = direction * speed / radius
    return (vy, omega)


# --- Node --------------------------------------------------------------------


class OrbitNode(Node):
    """ROS wrapper: a held orbit_dir command -> a streamed face-center /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("orbit_node")

        self.declare_parameter("orbit_speed", DEFAULT_ORBIT_SPEED)
        self.declare_parameter("orbit_radius", DEFAULT_ORBIT_RADIUS)
        self.declare_parameter("omega_sign", DEFAULT_OMEGA_SIGN)
        self.declare_parameter("cmd_timeout_s", DEFAULT_CMD_TIMEOUT_S)
        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)

        self._speed = float(self.get_parameter("orbit_speed").value)
        self._radius = float(self.get_parameter("orbit_radius").value)
        self._omega_sign = float(self.get_parameter("omega_sign").value)
        self._cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        if self._radius <= 0.0:
            raise ValueError(f"orbit_radius must be > 0, got {self._radius}")

        # Latest commanded direction (+1/-1/0) and when it last arrived. The
        # dead-man compares against _last_cmd_time; _stopped tracks whether the
        # single stop-zero has already gone out so we emit it once per release,
        # not every tick.
        self._direction: float = 0.0
        self._last_cmd_time: Time | None = None
        self._stopped: bool = True

        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self._dir_sub = self.create_subscription(
            Twist, ORBIT_DIR_TOPIC, self._on_orbit_dir, 10
        )
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f"orbit_node up. {ORBIT_DIR_TOPIC} (held) -> {CMD_VEL_TOPIC}. "
            f"speed={self._speed:.3f} m/s radius={self._radius:.3f} m "
            f"omega_sign={self._omega_sign:+.0f} "
            f"cmd_timeout={self._cmd_timeout_s:.2f}s @ {publish_rate:.0f} Hz. "
            f"Hold an ORBIT button to run; release to stop."
        )

    def _on_orbit_dir(self, msg: Twist) -> None:
        """Read the held command's sign and freshen the dead-man timer."""
        z = msg.angular.z
        if z > DIR_DEADBAND:
            self._direction = 1.0
        elif z < -DIR_DEADBAND:
            self._direction = -1.0
        else:
            self._direction = 0.0
        self._last_cmd_time = self.get_clock().now()

    def _on_timer(self) -> None:
        """Stream the orbit twist while held; emit one stop-zero on release."""
        now = self.get_clock().now()
        fresh = (
            self._last_cmd_time is not None
            and (now - self._last_cmd_time)
            <= Duration(seconds=self._cmd_timeout_s)
        )
        if fresh and self._direction != 0.0:
            vy, omega = orbit_twist(self._direction, self._speed, self._radius)
            out = Twist()
            out.linear.y = vy
            out.angular.z = omega * self._omega_sign
            self._cmd_pub.publish(out)
            self._stopped = False
        elif not self._stopped:
            # Released or stale: one explicit stop, then go quiet so we don't
            # fight teleop /cmd_vel when the operator is not orbiting.
            self._cmd_pub.publish(Twist())
            self._stopped = True


def main(args: list[str] | None = None) -> None:
    """Entry point registered as the `orbit_node` console script."""
    rclpy.init(args=args)
    node = OrbitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
