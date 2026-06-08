"""
Wheel odometry for the FORTIS X-drive base.

Subscribes
----------
    /joint_states   sensor_msgs/JointState
        From joint_state_broadcaster. Carries the four wheel joints
        (fl/fr/rl/rr_wheel_joint) with `velocity` in rad/s at the wheel
        shaft. Read by JOINT NAME, never by array index -- the broadcaster
        does not guarantee a stable order across the joints it publishes.

Publishes
---------
    /odom   nav_msgs/Odometry
        frame_id=odom, child_frame_id=base_link. Carries the integrated
        pose (x, y, yaw) AND the body twist (vx, vy, vyaw). This node does
        NOT broadcast the odom->base_link TF -- the robot_localization EKF
        owns that transform (see config/ekf.yaml, publish_tf: true). Two
        publishers of the same TF would fight.

Method
------
The four wheel angular velocities (rad/s) are mapped by name into canonical
[FL, FR, RL, RR] order, converted to contact-patch linear speed (m/s) via
WHEEL_RADIUS, and passed to xdrive_fk_solver to recover the body twist
(Vx, Vy, yaw_rate) in base_link. That twist is integrated to a pose in the
odom frame using the interval between consecutive message header stamps
(NOT wall clock), so the estimate stays correct if /joint_states is delayed,
bursty, or replayed from a bag. The pose is exact-integrated through the yaw
turned over the interval (the standard 2D dead-reckoning update) rather than
a zeroth-order hold, which removes the systematic inward drift a pure
Euler step accumulates while turning.

Covariance
----------
The twist covariance is intentionally LOOSE. Wheel velocity reaches us
stale: it is sampled on the ODrive, framed over CAN, read by the ros2_control
loop, and republished on /joint_states, so by the time the EKF fuses it the
robot has already moved on -- and X-drive omniwheels slip laterally far more
than they roll. Inflating the twist covariance (especially the yaw-rate term)
tells the EKF to trust the IMU's gyro for fast yaw and lean on the wheels
mainly for translation. See the *_COV constants for the per-axis rationale.
Pose covariance grows without bound (dead reckoning has no absolute
reference); we report a large fixed pose covariance rather than integrating
it, since the EKF consumes the TWIST from this message, not the pose.
"""

from __future__ import annotations

import math

import rclpy
from fortis_comms.xdrive_kinematics import WHEEL_RADIUS, xdrive_fk_solver
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState


# --- Constants ---------------------------------------------------------------

#: Wheel joint names in canonical [FL, FR, RL, RR] order -- the order the H
#: matrix in fortis_comms.xdrive_kinematics expects. /joint_states is read by
#: these names (not by array position), so this list is the single source of
#: truth for the wheel->index mapping. Must match the joint names published by
#: joint_state_broadcaster (fortis_control) and the URDF wheel joints.
WHEEL_JOINT_NAMES: tuple[str, str, str, str] = (
    "fl_wheel_joint",
    "fr_wheel_joint",
    "rl_wheel_joint",
    "rr_wheel_joint",
)

ODOM_FRAME: str = "odom"
BASE_FRAME: str = "base_link"
JOINT_STATES_TOPIC: str = "/joint_states"
ODOM_TOPIC: str = "/odom"

#: Largest header-stamp gap we will integrate over in one step. A gap larger
#: than this means /joint_states stalled (bag seek, node restart, dropped
#: frames); integrating across it would teleport the pose. We skip the step
#: and re-anchor on the new stamp instead.
MAX_INTEGRATION_DT_S: float = 0.5

#: Per-axis TWIST covariance (variance on the diagonal) reported on /odom, in
#: canonical (vx, vy, vyaw) terms. Deliberately loose -- see the module
#: "Covariance" section. vy (lateral) is looser than vx because omniwheels
#: slip sideways, and vyaw is the loosest so the EKF defers to the IMU gyro
#: for yaw rate. Units: (m/s)^2 for the linear terms, (rad/s)^2 for yaw.
VX_TWIST_COV: float = 0.04     # ~0.2 m/s 1-sigma
VY_TWIST_COV: float = 0.09     # ~0.3 m/s 1-sigma; lateral omniwheel slip
VYAW_TWIST_COV: float = 0.25   # ~0.5 rad/s 1-sigma; defer to IMU gyro

#: Pose covariance reported on /odom. Large and fixed: this is open-loop dead
#: reckoning with no absolute reference, so the pose is never trustworthy on
#: its own. The EKF fuses the TWIST from this message, not the pose, so these
#: values are mostly informational for any raw consumer of /odom.
POSE_XY_COV: float = 1.0
POSE_YAW_COV: float = 1.0

#: Index of the (vx, vy, vyaw) diagonal entries in a 6x6 row-major ROS
#: covariance (order: x, y, z, roll, pitch, yaw).
_VX_IDX: int = 0      # row/col 0
_VY_IDX: int = 7      # row/col 1
_VYAW_IDX: int = 35   # row/col 5
_POSE_X_IDX: int = 0
_POSE_Y_IDX: int = 7
_POSE_YAW_IDX: int = 35


# --- Helpers -----------------------------------------------------------------


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Return the quaternion for a planar rotation of `yaw` about +Z."""
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _wheel_speeds_from_joint_state(msg: JointState) -> list[float] | None:
    """
    Pull the four wheel linear speeds (m/s) from a JointState, by name.

    Returns them in canonical [FL, FR, RL, RR] order, converting the
    per-joint angular velocity (rad/s at the shaft) to contact-patch linear
    speed via WHEEL_RADIUS. Returns None if any wheel joint is missing from
    the message or the message carries no velocity field -- the caller skips
    that update rather than integrating a partial twist.
    """
    if not msg.velocity:
        return None
    index_by_name = {name: i for i, name in enumerate(msg.name)}
    speeds: list[float] = []
    for joint in WHEEL_JOINT_NAMES:
        i = index_by_name.get(joint)
        if i is None or i >= len(msg.velocity):
            return None
        speeds.append(msg.velocity[i] * WHEEL_RADIUS)
    return speeds


# --- Node --------------------------------------------------------------------


class WheelOdometryNode(Node):
    """
    Integrates X-drive wheel velocities into a /odom Odometry message.

    Runtime state: the integrated planar pose (x, y, yaw) in the odom frame
    and the header stamp of the last processed /joint_states (for the dt of
    the next integration step). TF is intentionally NOT broadcast here; the
    EKF owns odom->base_link.
    """

    def __init__(self) -> None:
        super().__init__("wheel_odometry_node")

        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0
        # Stamp of the last integrated message. None until the first message
        # arrives; the first message only seeds the stamp (no dt to integrate
        # yet) so we never integrate against an undefined interval.
        self._last_stamp: Time | None = None

        self._odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self._joint_sub = self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._on_joint_states,
            10,
        )

        self.get_logger().info(
            f"wheel_odometry_node up. Integrating {JOINT_STATES_TOPIC} "
            f"-> {ODOM_TOPIC} (frame {ODOM_FRAME}->{BASE_FRAME}). "
            f"TF owned by the EKF, not this node."
        )

    def _on_joint_states(self, msg: JointState) -> None:
        """Integrate one /joint_states sample into the pose and publish /odom."""
        speeds = _wheel_speeds_from_joint_state(msg)
        if speeds is None:
            return  # missing a wheel joint or no velocity field; skip

        vx, vy, yaw_rate = (float(v) for v in xdrive_fk_solver(speeds))
        stamp = Time.from_msg(msg.header.stamp)

        if self._last_stamp is not None:
            dt = (stamp - self._last_stamp).nanoseconds * 1e-9
            # Guard both ends: a non-positive dt (duplicate/out-of-order
            # stamp) and an oversized dt (a stall we must not integrate
            # across) only re-anchor the clock, leaving the pose untouched.
            if 0.0 < dt <= MAX_INTEGRATION_DT_S:
                self._integrate(vx, vy, yaw_rate, dt)
        self._last_stamp = stamp

        self._publish_odom(msg.header.stamp, vx, vy, yaw_rate)

    def _integrate(self, vx: float, vy: float, yaw_rate: float, dt: float) -> None:
        """Advance the planar pose by a body twist held over `dt` seconds.

        Rotates the body-frame velocity into the odom frame at the
        mid-interval heading (yaw + 0.5*yaw_rate*dt) before integrating, the
        standard 2D dead-reckoning update. Using the mid-interval heading
        rather than the start-of-interval one cancels the first-order arc
        error a plain Euler step leaves while the robot is turning.
        """
        mid_yaw = self._yaw + 0.5 * yaw_rate * dt
        cos_y = math.cos(mid_yaw)
        sin_y = math.sin(mid_yaw)
        self._x += (vx * cos_y - vy * sin_y) * dt
        self._y += (vx * sin_y + vy * cos_y) * dt
        self._yaw += yaw_rate * dt

    def _publish_odom(self, stamp, vx: float, vy: float, yaw_rate: float) -> None:
        """Build and publish the /odom Odometry message for this step."""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = _yaw_to_quaternion(self._yaw)
        odom.pose.covariance[_POSE_X_IDX] = POSE_XY_COV
        odom.pose.covariance[_POSE_Y_IDX] = POSE_XY_COV
        odom.pose.covariance[_POSE_YAW_IDX] = POSE_YAW_COV

        # The twist is the EKF's actual input from this message: a body-frame
        # velocity in child_frame_id (base_link), per the Odometry contract.
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = yaw_rate
        odom.twist.covariance[_VX_IDX] = VX_TWIST_COV
        odom.twist.covariance[_VY_IDX] = VY_TWIST_COV
        odom.twist.covariance[_VYAW_IDX] = VYAW_TWIST_COV

        self._odom_pub.publish(odom)


def main(args: list[str] | None = None) -> None:
    """Entry point registered as the `wheel_odometry_node` console script."""
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
