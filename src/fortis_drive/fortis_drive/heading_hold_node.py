"""
heading_hold_node -- closed-loop heading hold for the FORTIS X-drive.

Subscribes
----------
    /cmd_vel                geometry_msgs/Twist
        Operator velocity command. Vx, Vy pass through untouched; angular.z
        (omega_cmd) is the turn intent the controller arbitrates against the
        heading hold.
    /odometry/filtered      nav_msgs/Odometry
        The robot_localization EKF estimate. We read ONLY its yaw (from the
        pose quaternion) -- the EKF yaw is IMU-gyro-dominated (see
        fortis_localization/config/ekf.yaml: wheel vyaw is not fused), which
        is the whole reason heading hold can work despite the RR/RL wheels
        slipping. Closing the loop on wheel-odometry yaw would chase slip.

Publishes
---------
    /cmd_vel_heading        geometry_msgs/Twist
        The corrected command: Vx/Vy from the operator verbatim, angular.z
        replaced by either the operator's omega (while actively turning) or
        the PID heading-hold output (while holding). drive_node consumes this
        instead of /cmd_vel when heading hold is enabled (bringup arg
        heading_hold:=true remaps drive_node's /cmd_vel -> /cmd_vel_heading).

Control law
-----------
    If |omega_cmd| > turn_deadband the operator is actively turning:
        pass omega_cmd straight through AND continuously re-arm the hold
        target to the CURRENT measured yaw. So the instant the operator
        releases the turn, the controller holds the heading they stopped at,
        with no jump-back to a stale target.
    Otherwise (operator not turning) HOLD:
        omega_out = PID(wrap_to_pi(yaw_sign * (target_yaw - measured_yaw)))
        clamped to +/- max_omega, with integral anti-windup.

Sign dependency (MUST bench-verify before trusting on hardware)
---------------------------------------------------------------
The law assumes EKF yaw increases CCW (REP-103): a positive heading error
needs a positive (CCW) omega to correct it. The BMI270 gyro Z sign as it
reaches the EKF is NOT verified on the real robot. Rather than bake in a
flip, the sign is exposed as the `yaw_sign` parameter (default +1.0). The
operator bench-verifies the gyro direction (rotate the robot CCW by hand,
confirm /odometry/filtered yaw INCREASES) and sets yaw_sign to -1.0 only if
it decreases. A wrong sign turns heading hold into positive feedback: it
drives the heading AWAY from target, accelerating to the max_omega clamp.

dt source
---------
The PID integral and derivative use the interval between successive
/odometry/filtered header STAMPS, not wall clock, so the controller behaves
correctly under delayed, bursty, or bag-replayed odometry -- the same
stamp-driven discipline wheel_odometry_node uses for its integration.

Parameters (declare_parameter; defaults overridable from
fortis_bringup/config/bringup_params.yaml, same loading path drive_node uses)
----------------------------------------------------------------------------
    kp, ki, kd        PID gains on the wrapped heading error (rad).
    max_omega         output clamp, rad/s (also clamps pass-through? no --
                      pass-through omega is the operator's own command).
    turn_deadband     |omega_cmd| above this = "actively turning", rad/s.
    i_clamp           symmetric clamp on the integral TERM (ki * integral),
                      rad/s, for anti-windup.
    yaw_sign          +1.0 or -1.0; gyro-direction correction (see above).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time


# --- Constants ---------------------------------------------------------------

CMD_VEL_TOPIC: str = "/cmd_vel"
ODOM_FILTERED_TOPIC: str = "/odometry/filtered"
CMD_VEL_HEADING_TOPIC: str = "/cmd_vel_heading"

#: Default PID/limit parameters. Mirrored in
#: fortis_bringup/config/bringup_params.yaml under heading_hold_node; these
#: in-code values are the authoritative defaults if no YAML is loaded.
DEFAULT_KP: float = 2.0
DEFAULT_KI: float = 0.0
DEFAULT_KD: float = 0.1
DEFAULT_MAX_OMEGA: float = 1.5          # rad/s
DEFAULT_TURN_DEADBAND: float = 0.05     # rad/s
DEFAULT_I_CLAMP: float = 0.5            # rad/s (clamp on the integral TERM)
DEFAULT_YAW_SIGN: float = 1.0

#: Largest odometry-stamp gap we integrate the PID over in one step. A gap
#: larger than this means /odometry/filtered stalled (node restart, bag
#: seek); running the integral/derivative across it would windup or spike, so
#: we re-anchor the clock and skip the I/D update for that step. Matches the
#: spirit of wheel_odometry_node's MAX_INTEGRATION_DT_S.
MAX_CONTROL_DT_S: float = 0.5


# --- Helpers -----------------------------------------------------------------


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle (rad) into (-pi, pi].

    The heading error must wrap so that, e.g., a target just past +pi and a
    measurement just past -pi produce a small error, not a ~2*pi one that
    would command a full-speed spin the long way round.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract the planar (Z) yaw from a quaternion.

    Standard ZYX yaw extraction; for the planar EKF output (two_d_mode zeroes
    roll/pitch) this reduces to the heading about +Z.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class HeadingHoldGains:
    """Tunable parameters for the heading-hold controller."""

    kp: float = DEFAULT_KP
    ki: float = DEFAULT_KI
    kd: float = DEFAULT_KD
    max_omega: float = DEFAULT_MAX_OMEGA
    turn_deadband: float = DEFAULT_TURN_DEADBAND
    i_clamp: float = DEFAULT_I_CLAMP
    yaw_sign: float = DEFAULT_YAW_SIGN


class HeadingController:
    """
    Pure heading-hold PID, independent of rclpy.

    Extracted as a plain class (no Node base) so the control law can be
    unit-tested without DDS or a spinning executor -- the same split
    fortis_safety uses for compute_aggregate_health. The node below owns the
    ROS plumbing and delegates every decision here.

    State carried between updates: the current hold target (target_yaw), the
    integral accumulator, and the previous wrapped error (for the derivative).
    target_yaw is None until the first measurement seeds it.
    """

    def __init__(self, gains: HeadingHoldGains) -> None:
        self.gains = gains
        self.target_yaw: float | None = None
        self._integral: float = 0.0
        self._prev_error: float | None = None

    def reset(self) -> None:
        """Clear integral/derivative history (e.g. after a stale-odom gap)."""
        self._integral = 0.0
        self._prev_error = None

    def update(self, measured_yaw: float, omega_cmd: float, dt: float) -> float:
        """
        Compute the corrected angular velocity for one control step.

        Parameters
        ----------
        measured_yaw : float
            EKF yaw (rad), already extracted from the odometry quaternion.
        omega_cmd : float
            Operator's commanded angular.z (rad/s).
        dt : float
            Interval since the previous accepted step (s), from message
            stamps. A non-positive or oversized dt suppresses the I/D update
            (the caller passes 0.0 to mean "no valid interval"); proportional
            action still applies so the hold never goes fully open-loop.

        Returns
        -------
        float
            angular.z to publish. While actively turning, this is omega_cmd
            verbatim; while holding, the clamped PID output.
        """
        g = self.gains

        # Seed the hold target on the first sample so we never PID against a
        # None target.
        if self.target_yaw is None:
            self.target_yaw = measured_yaw

        if abs(omega_cmd) > g.turn_deadband:
            # Operator is actively turning: pass their command through and
            # continuously re-arm the target to where we are NOW, so releasing
            # the turn holds the new heading with no snap-back to a stale one.
            self.target_yaw = measured_yaw
            # Reset I/D so the hold that begins on release starts clean rather
            # than acting on error accumulated before the turn.
            self.reset()
            return omega_cmd

        # HOLD: drive the (sign-corrected, wrapped) heading error to zero.
        error = wrap_to_pi(g.yaw_sign * (self.target_yaw - measured_yaw))

        # Integral with anti-windup: accumulate, then clamp the integral TERM
        # (ki * integral) to +/- i_clamp. Clamping the term (not the raw
        # accumulator) keeps the clamp meaningful when ki is retuned.
        derivative = 0.0
        if dt > 0.0:
            self._integral += error * dt
            if self._prev_error is not None:
                # Derivative on the wrapped error. wrap the delta too so a
                # measurement crossing +/-pi doesn't spike the derivative.
                derivative = wrap_to_pi(error - self._prev_error) / dt
            self._prev_error = error

        i_term = g.ki * self._integral
        if i_term > g.i_clamp:
            i_term = g.i_clamp
            # Back-calculate the accumulator to the clamp so it doesn't keep
            # growing past it (true anti-windup, not just output saturation).
            if g.ki != 0.0:
                self._integral = i_term / g.ki
        elif i_term < -g.i_clamp:
            i_term = -g.i_clamp
            if g.ki != 0.0:
                self._integral = i_term / g.ki

        omega = g.kp * error + i_term + g.kd * derivative

        # Output clamp.
        if omega > g.max_omega:
            omega = g.max_omega
        elif omega < -g.max_omega:
            omega = -g.max_omega
        return omega


# --- Node --------------------------------------------------------------------


class HeadingHoldNode(Node):
    """
    ROS wrapper: arbitrates /cmd_vel against EKF yaw, publishes /cmd_vel_heading.

    Holds the latest operator Twist and the latest EKF yaw; on every
    /odometry/filtered sample it runs the controller and republishes the
    corrected Twist. Driving the output off the ODOMETRY rate (not the cmd_vel
    rate) means the hold keeps correcting even when the operator stops sending
    commands -- which is exactly when drift would otherwise accumulate.
    """

    def __init__(self) -> None:
        super().__init__("heading_hold_node")

        # declare_parameter for each knob -- defaults overridable from
        # bringup_params.yaml via the launch Node(parameters=[...]) path.
        self.declare_parameter("kp", DEFAULT_KP)
        self.declare_parameter("ki", DEFAULT_KI)
        self.declare_parameter("kd", DEFAULT_KD)
        self.declare_parameter("max_omega", DEFAULT_MAX_OMEGA)
        self.declare_parameter("turn_deadband", DEFAULT_TURN_DEADBAND)
        self.declare_parameter("i_clamp", DEFAULT_I_CLAMP)
        self.declare_parameter("yaw_sign", DEFAULT_YAW_SIGN)

        gains = HeadingHoldGains(
            kp=float(self.get_parameter("kp").value),
            ki=float(self.get_parameter("ki").value),
            kd=float(self.get_parameter("kd").value),
            max_omega=float(self.get_parameter("max_omega").value),
            turn_deadband=float(self.get_parameter("turn_deadband").value),
            i_clamp=float(self.get_parameter("i_clamp").value),
            yaw_sign=float(self.get_parameter("yaw_sign").value),
        )
        self._controller = HeadingController(gains)

        # Latest operator Twist. Default zeros so that, before any /cmd_vel,
        # an incoming odometry sample still holds heading (omega from the PID)
        # with zero translation rather than doing nothing.
        self._last_cmd: Twist = Twist()
        #: Stamp of the last processed odometry sample, for the control dt.
        self._last_stamp: Time | None = None

        self._cmd_pub = self.create_publisher(
            Twist, CMD_VEL_HEADING_TOPIC, 10
        )
        self._cmd_sub = self.create_subscription(
            Twist, CMD_VEL_TOPIC, self._on_cmd_vel, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, ODOM_FILTERED_TOPIC, self._on_odometry, 10
        )

        self.get_logger().info(
            f"heading_hold_node up. {ODOM_FILTERED_TOPIC} + {CMD_VEL_TOPIC} "
            f"-> {CMD_VEL_HEADING_TOPIC}. kp={gains.kp} ki={gains.ki} "
            f"kd={gains.kd} max_omega={gains.max_omega} "
            f"turn_deadband={gains.turn_deadband} yaw_sign={gains.yaw_sign:+.0f}. "
            f"VERIFY yaw_sign against the live gyro before trusting hold."
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Cache the latest operator command; output is emitted on odometry."""
        self._last_cmd = msg

    def _on_odometry(self, msg: Odometry) -> None:
        """Run the controller on a fresh EKF sample and publish the result."""
        measured_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        stamp = Time.from_msg(msg.header.stamp)

        # Stamp-driven dt; 0.0 means "no valid interval" (first sample, or a
        # stalled / out-of-order odometry stream) and suppresses I/D this step.
        dt = 0.0
        if self._last_stamp is not None:
            measured_dt = (stamp - self._last_stamp).nanoseconds * 1e-9
            if 0.0 < measured_dt <= MAX_CONTROL_DT_S:
                dt = measured_dt
            else:
                # Stall or non-monotonic stamp: drop integral/derivative
                # history so the hold restarts clean on the next good step.
                self._controller.reset()
        self._last_stamp = stamp

        cmd = self._last_cmd
        omega_out = self._controller.update(
            measured_yaw=measured_yaw,
            omega_cmd=cmd.angular.z,
            dt=dt,
        )

        out = Twist()
        # Vx, Vy pass through unchanged; only the yaw rate is arbitrated.
        out.linear.x = cmd.linear.x
        out.linear.y = cmd.linear.y
        out.linear.z = cmd.linear.z
        out.angular.x = cmd.angular.x
        out.angular.y = cmd.angular.y
        out.angular.z = omega_out
        self._cmd_pub.publish(out)


def main(args: list[str] | None = None) -> None:
    """Entry point registered as the `heading_hold_node` console script."""
    rclpy.init(args=args)
    node = HeadingHoldNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
