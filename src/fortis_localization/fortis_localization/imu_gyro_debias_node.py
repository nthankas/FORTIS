"""
Automatic gyro-bias estimator/remover, upstream of the localization EKF.

Why this exists
---------------
The FORTIS EKF (config/ekf.yaml) fuses the front OAK-D Lite gyro yaw rate as
the SOLE yaw source -- wheel vyaw is off because the rear wheels slip. The
BMI270 gyro carries a small zero-rate bias, so the integrated yaw drifts and
the heading-hold controller slowly creeps the robot in one direction. This
node subtracts the gyro-Z bias from every IMU sample BEFORE the EKF sees it,
so at rest the EKF observes a zero-mean yaw rate and the heading stays put.

Stationarity = DISARMED
-----------------------
The bias is only observable when the robot is physically still: a stationary
gyro reads pure bias. We key "stationary" on the drive being DISARMED
(/fortis/drive/armed == false) rather than on a measured wheel/cmd velocity.
That avoids a chicken-and-egg trap: the bias-driven creep keeps the wheels
turning, so a velocity-based "stationary" test would never fire while the very
drift we are trying to cancel is active. When disarmed the motors are idle and
the chassis cannot move, so the gyro reading is bias by definition; we fold it
into a slow EMA. When armed we FREEZE the estimate and keep subtracting the
last value -- the robot is moving, so the gyro reading is real rotation, not
bias.

Settle skip
-----------
A disarm edge is followed by a brief mechanical settle (the chassis rocks as
the motors release). We ignore the first `settle_skip_s` of samples after each
arm->disarm transition so that transient does not poison the bias EMA.

Subscribes
----------
    <raw_imu_topic>   sensor_msgs/Imu   (default /oak_chassis_front/imu/data)
    /fortis/drive/armed   std_msgs/Bool   (latched, TRANSIENT_LOCAL)

Publishes
---------
    <debiased_imu_topic>   sensor_msgs/Imu   (default /imu/debiased)
        A copy of every input message with angular_velocity reduced by the
        current bias estimate on the axes in `debias_axes`. Header, frame_id,
        orientation, and ALL covariances are passed through unchanged.
"""

from __future__ import annotations

import copy

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from fortis_comms.qos_profiles import latched_qos_profile

#: How often (seconds) to log the live bias estimate. Periodic, not per-sample
#: -- the IMU streams at ~100-200 Hz and per-message logging would flood.
_BIAS_LOG_PERIOD_S: float = 5.0


class ImuGyroDebiasNode(Node):
    """Estimate the gyro zero-rate bias while disarmed and subtract it always.

    Runtime state: the per-axis bias estimate (rad/s), the latest armed flag,
    and the time of the last arm->disarm edge (to enforce the settle skip).
    The EMA only advances while disarmed and past the settle window; the
    subtraction runs on every message regardless of armed state.
    """

    def __init__(self) -> None:
        super().__init__("imu_gyro_debias_node")

        self._raw_imu_topic = (
            self.declare_parameter("raw_imu_topic", "/oak_chassis_front/imu/data")
            .get_parameter_value()
            .string_value
        )
        self._debiased_imu_topic = (
            self.declare_parameter("debiased_imu_topic", "/imu/debiased")
            .get_parameter_value()
            .string_value
        )
        # Slow by design: the bias is quasi-static, so a small alpha averages
        # out per-sample gyro noise and only tracks the slow thermal drift.
        self._ema_alpha = (
            self.declare_parameter("ema_alpha", 0.02)
            .get_parameter_value()
            .double_value
        )
        # Which angular_velocity axes to debias. Default z only: yaw rate is
        # the sole EKF yaw input, and two_d_mode discards x/y rates anyway.
        self._debias_axes = list(
            self.declare_parameter("debias_axes", ["z"])
            .get_parameter_value()
            .string_array_value
        )
        self._settle_skip_s = (
            self.declare_parameter("settle_skip_s", 0.5)
            .get_parameter_value()
            .double_value
        )

        #: Current bias estimate per axis (rad/s). Starts at zero so a freshly
        #: launched, already-armed robot passes the gyro through untouched
        #: until the first disarm lets us measure the real bias.
        self._bias = {"x": 0.0, "y": 0.0, "z": 0.0}
        #: True once at least one disarmed, post-settle sample has updated the
        #: EMA -- gates the "converged" wording in the periodic log only.
        self._has_estimate = False
        #: Latest /fortis/drive/armed value. None = not yet known; the EMA
        #: stays frozen until the first armed msg so we never fold MOTION
        #: into the bias, but a first msg of "disarmed" is NOT an edge (the
        #: motors were never released, so there is no settle transient).
        self._armed: bool | None = None
        #: Monotonic-ish ROS time of the most recent arm->disarm edge; the EMA
        #: stays frozen until settle_skip_s past this. None = no edge seen yet.
        self._disarm_time: Time | None = None

        self._debiased_pub = self.create_publisher(
            Imu, self._debiased_imu_topic, 10
        )
        self.create_subscription(
            Imu, self._raw_imu_topic, self._on_imu, 10
        )
        # Match the latched publisher in drive_enable_node so a late-joining
        # subscriber (this node) still receives the current armed state.
        self.create_subscription(
            Bool, "/fortis/drive/armed", self._on_armed, latched_qos_profile()
        )

        self._log_timer = self.create_timer(_BIAS_LOG_PERIOD_S, self._log_bias)

        self.get_logger().info(
            f"imu_gyro_debias_node up. Debiasing {self._raw_imu_topic} "
            f"-> {self._debiased_imu_topic} on axes {self._debias_axes} "
            f"(EMA alpha {self._ema_alpha}, settle {self._settle_skip_s}s). "
            f"Estimating while DISARMED, frozen while ARMED."
        )

    def _on_armed(self, msg: Bool) -> None:
        """Track the armed flag and stamp each arm->disarm edge for the settle skip."""
        armed = bool(msg.data)
        if self._armed is True and not armed:
            # Just disarmed: start the settle window before trusting the gyro.
            self._disarm_time = self.get_clock().now()
        self._armed = armed

    def _on_imu(self, msg: Imu) -> None:
        """Update the bias if stationary, then republish the debiased copy."""
        if self._stationary_and_settled():
            self._update_bias(msg)
        self._publish_debiased(msg)

    def _stationary_and_settled(self) -> bool:
        """Report whether we are disarmed AND past the post-disarm settle window."""
        if self._armed is not False:
            # Armed, or armed-state unknown: never fold motion into the bias.
            return False
        if self._disarm_time is None:
            # Disarmed since before any edge was observed (e.g. launched
            # disarmed): no settle transient to wait out, so trust the gyro.
            return True
        elapsed = (self.get_clock().now() - self._disarm_time).nanoseconds * 1e-9
        return elapsed >= self._settle_skip_s

    def _update_bias(self, msg: Imu) -> None:
        """Fold this stationary sample's angular velocity into the bias EMA."""
        sample = {
            "x": msg.angular_velocity.x,
            "y": msg.angular_velocity.y,
            "z": msg.angular_velocity.z,
        }
        a = self._ema_alpha
        for axis in self._debias_axes:
            self._bias[axis] = (1.0 - a) * self._bias[axis] + a * sample[axis]
        self._has_estimate = True

    def _publish_debiased(self, msg: Imu) -> None:
        """Republish a copy with the bias removed on the debiased axes.

        Deep-copies the message so header, frame_id, orientation, and every
        covariance pass through byte-for-byte; only the selected
        angular_velocity components are reduced by the current bias.
        """
        out = copy.deepcopy(msg)
        if "x" in self._debias_axes:
            out.angular_velocity.x = msg.angular_velocity.x - self._bias["x"]
        if "y" in self._debias_axes:
            out.angular_velocity.y = msg.angular_velocity.y - self._bias["y"]
        if "z" in self._debias_axes:
            out.angular_velocity.z = msg.angular_velocity.z - self._bias["z"]
        self._debiased_pub.publish(out)

    def _log_bias(self) -> None:
        """Periodically surface the live bias estimate and the gating state."""
        state = "estimating" if self._armed is False else "frozen (armed/unknown)"
        seen = "converging" if self._has_estimate else "no estimate yet"
        self.get_logger().info(
            f"gyro bias [{state}, {seen}] (rad/s): "
            f"x={self._bias['x']:+.5f} y={self._bias['y']:+.5f} "
            f"z={self._bias['z']:+.5f}"
        )


def main(args: list[str] | None = None) -> None:
    """Entry point registered as the `imu_gyro_debias_node` console script."""
    rclpy.init(args=args)
    node = ImuGyroDebiasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
