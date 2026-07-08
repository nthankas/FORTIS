"""
Click-to-target node.

Turns a Foxglove 3D-panel click (/clicked_point, geometry_msgs/PointStamped)
into the mission target: validates the click against a horizontal range
annulus around the robot, publishes /fortis/target_pose plus the latched
/fortis/context/target_pose_valid flag, then fires the FSM's
/fortis/events/chassis_cam_click event (Event.CHASSIS_CAM_CLICK: ORBIT ->
TARGETING when the target_pose_valid guard holds, see
fortis_safety.mission_state_machine). The /fortis/context/ik_ok flag is
owned by fortis_arm's arm_motion node, which solves the arm IK for every
published target.

Frame policy: the pose is anchored in `odom` when the odom->base_link TF is
available at click time, so the target does not drift while the robot keeps
moving; without odom it falls back to `base_link`. Image-pixel clicks
(fortis_msgs/ChassisCamClick) stay unwired -- Foxglove image panels cannot
publish clicks.
"""

from __future__ import annotations

import math

import rclpy
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Empty
from tf2_ros import Buffer, TransformException, TransformListener

from fortis_comms.qos_profiles import latched_qos_profile

NODE_NAME = "target_selector"

CLICKED_POINT_TOPIC = "/clicked_point"
TARGET_POSE_TOPIC = "/fortis/target_pose"
TARGET_VALID_TOPIC = "/fortis/context/target_pose_valid"
#: Exact FSM event topic: mission_state_node subscribes
#: /fortis/events/<Event name lowercased> for Event.CHASSIS_CAM_CLICK.
CLICK_EVENT_TOPIC = "/fortis/events/chassis_cam_click"

BASE_FRAME = "base_link"
ODOM_FRAME = "odom"
TF_TIMEOUT_S = 0.5


def _yaw_orientation(yaw: float) -> tuple[float, float]:
    """Return (z, w) quaternion components for a pure-yaw orientation."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TargetSelectorNode(Node):
    """Validate operator clicks and publish the mission target pose."""

    def __init__(self, **node_kwargs):
        super().__init__(NODE_NAME, **node_kwargs)

        self.declare_parameter("min_target_range_m", 0.3)
        self.declare_parameter("max_target_range_m", 3.5)
        self._min_range = float(self.get_parameter("min_target_range_m").value)
        self._max_range = float(self.get_parameter("max_target_range_m").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = latched_qos_profile()
        # Latched so the UI / planner see the current target on connect.
        self._pose_pub = self.create_publisher(
            PoseStamped, TARGET_POSE_TOPIC, latched)
        self._valid_pub = self.create_publisher(Bool, TARGET_VALID_TOPIC, latched)
        self._event_pub = self.create_publisher(Empty, CLICK_EVENT_TOPIC, 10)

        self.create_subscription(
            PointStamped, CLICKED_POINT_TOPIC, self._on_click, 10)

        self._set_validity(False)

    # --- Click handling ------------------------------------------------------

    def _on_click(self, msg: PointStamped) -> None:
        """Validate one click and publish pose, context flag, and FSM event."""
        point = msg
        if msg.header.frame_id and msg.header.frame_id != BASE_FRAME:
            try:
                tf = self._tf_buffer.lookup_transform(
                    BASE_FRAME, msg.header.frame_id, Time(),
                    timeout=Duration(seconds=TF_TIMEOUT_S))
            except TransformException as exc:
                self.get_logger().warning(
                    f"click in frame '{msg.header.frame_id}' dropped, no TF"
                    f" to {BASE_FRAME}: {exc}", throttle_duration_sec=5.0)
                self._set_validity(False)
                return
            point = tf2_geometry_msgs.do_transform_point(msg, tf)

        x, y = point.point.x, point.point.y
        rng = math.hypot(x, y)
        if not (self._min_range <= rng <= self._max_range):
            self.get_logger().warning(
                f"click at {rng:.2f} m outside [{self._min_range:.2f},"
                f" {self._max_range:.2f}] m annulus; ignored",
                throttle_duration_sec=2.0)
            self._set_validity(False)
            return

        pose = self._anchor_pose(point)
        self._pose_pub.publish(pose)
        # Context flag before the event: mission_state_node evaluates the
        # target_pose_valid guard from its cached context when the event
        # lands.
        self._set_validity(True)
        self._event_pub.publish(Empty())
        self.get_logger().info(
            f"target accepted at {rng:.2f} m -> frame {pose.header.frame_id}")

    def _anchor_pose(self, point: PointStamped) -> PoseStamped:
        """Build the outgoing PoseStamped, anchored in odom when TF allows."""
        x, y, z = point.point.x, point.point.y, point.point.z
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        try:
            tf = self._tf_buffer.lookup_transform(
                ODOM_FRAME, BASE_FRAME, Time(),
                timeout=Duration(seconds=TF_TIMEOUT_S))
        except TransformException:
            pose.header.frame_id = BASE_FRAME
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            qz, qw = _yaw_orientation(math.atan2(y, x))
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            return pose
        ps = PointStamped()
        ps.header.frame_id = BASE_FRAME
        ps.point = point.point
        p_odom = tf2_geometry_msgs.do_transform_point(ps, tf)
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        yaw = math.atan2(p_odom.point.y - ry, p_odom.point.x - rx)
        pose.header.frame_id = ODOM_FRAME
        pose.pose.position = p_odom.point
        qz, qw = _yaw_orientation(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    # --- Context flags ---------------------------------------------------------

    def _set_validity(self, valid: bool) -> None:
        """Publish the latched target_pose_valid context flag for the FSM."""
        self._valid_pub.publish(Bool(data=valid))


def main(args=None):
    """Entry point registered as the `target_selector_node` console script."""
    rclpy.init(args=args)
    node = TargetSelectorNode()
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
