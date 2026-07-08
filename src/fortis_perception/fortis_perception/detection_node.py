"""
Object detection node.

Consumes one chassis OAK camera stream (device-encoded jpeg RGB + aligned
16UC1 depth + CameraInfo), runs a fortis_perception.detectors backend on a
rate-limited timer, and publishes the topic-registry set: 2D detections,
depth-backprojected 3D detections (optical frame), sphere markers in
base_link, Foxglove image annotations, and the nearest graspable object as
a fortis_msgs/GraspCandidate with its latched
/fortis/context/grasp_candidate_ok flag.

Degradation contract: with detector:=yolo and no weights on disk the node
logs one warning, publishes empty Detection2DArray at rate plus a WARN on
/diagnostics, and stays alive so launch files and CI remain green.
"""

from __future__ import annotations

import math
import os
import zlib

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from foxglove_msgs.msg import (
    Color,
    ImageAnnotations,
    Point2,
    PointsAnnotation,
    TextAnnotation,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.msg import GraspCandidate
from fortis_perception.detectors import (
    HsvBlobDetector,
    ModelUnavailable,
    YoloV8OnnxDetector,
)

NODE_NAME = "detection"

DETECTIONS_TOPIC = "/fortis/perception/detections"
DETECTIONS3D_TOPIC = "/fortis/perception/detections3d"
MARKERS_TOPIC = "/fortis/perception/detection_markers"
ANNOTATIONS_TOPIC_FMT = "/fortis/perception/annotations/{cam}"
GRASP_CANDIDATE_TOPIC = "/fortis/perception/grasp_candidate"
GRASP_OK_TOPIC = "/fortis/context/grasp_candidate_ok"
DIAGNOSTICS_TOPIC = "/diagnostics"

BASE_FRAME = "base_link"

#: A graspable detection farther than this (horizontal range from base_link
#: origin) is out of the arm's useful envelope and never becomes a candidate.
MAX_GRASP_RANGE_M = 2.5

#: Marker sphere diameter clamp (bbox-derived metric size can explode on
#: bad depth).
MARKER_DIAMETER_MIN_M = 0.05
MARKER_DIAMETER_MAX_M = 0.5


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the 3x3 rotation for URDF fixed-axis rpy (R = Rz @ Ry @ Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


# Front chassis camera extrinsic, base_link <- optical, hardcoded from the
# URDF (fortis_description/urdf/fortis_chassis.urdf.xacro: front mount
# xyz=(-cam_front_x, 0, cam_height_z), rpy=(0, -0.524, pi); optical joint
# rpy=(-pi/2, 0, -pi/2); fortis_constants.xacro: cam_front_x =
# chassis_length/2 + cam_edge_to_housing + oak_lite_z/2 =
# 0.332/2 + 0.01933 + 0.017/2, cam_height_z = 0.21514). The mount is bolted,
# so a constant beats a TF dependency here; only the FRONT camera's
# detections are lifted into base_link.
_FRONT_CAM_XYZ = np.array([-(0.332 / 2 + 0.01933 + 0.017 / 2), 0.0, 0.21514])
_FRONT_CAM_R = (_rpy_matrix(0.0, -0.524, math.pi)
                @ _rpy_matrix(-math.pi / 2, 0.0, -math.pi / 2))


def optical_to_base(p_optical) -> np.ndarray:
    """Map a point from the front camera's optical frame into base_link."""
    return _FRONT_CAM_R @ np.asarray(p_optical, dtype=float) + _FRONT_CAM_XYZ


def _class_color(name: str) -> tuple[float, float, float]:
    """Derive a stable RGB in [0, 1] from a class name.

    Uses crc32, not builtin hash(), because the latter is salted per process
    and would recolor every class on restart.
    """
    h = zlib.crc32(name.encode())
    return ((h & 0xFF) / 255.0,
            ((h >> 8) & 0xFF) / 255.0,
            ((h >> 16) & 0xFF) / 255.0)


class DetectionNode(Node):
    """Detect objects on one camera and publish 2D/3D/marker/grasp outputs."""

    def __init__(self, **node_kwargs):
        super().__init__(NODE_NAME, **node_kwargs)

        self.declare_parameter("camera_name", "oak_chassis_front")
        self.declare_parameter("detector", "blob")
        self.declare_parameter("model_path", "~/.cache/fortis/models/yolov8n.onnx")
        self.declare_parameter("target_rate_hz", 2.0)
        self.declare_parameter("graspable_classes",
                               ["bottle", "cup", "sports ball", "red_sphere"])
        self.declare_parameter("candidate_timeout_s", 3.0)
        self.declare_parameter("min_score", 0.3)

        cam = str(self.get_parameter("camera_name").value)
        self._graspable = set(self.get_parameter("graspable_classes").value)
        self._min_score = float(self.get_parameter("min_score").value)
        self._candidate_timeout_s = float(
            self.get_parameter("candidate_timeout_s").value)

        self._detector = self._make_detector()

        # Cache-latest inputs; all work happens on the rate timer so a fast
        # camera can never outrun the detector. RGB is consumed (cleared)
        # per pass so a stalled camera stops producing candidates.
        self._rgb: CompressedImage | None = None
        self._depth: Image | None = None
        self._info: CameraInfo | None = None

        self.create_subscription(
            CompressedImage, f"/{cam}/rgb/image_raw/compressed",
            self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(
            Image, f"/{cam}/stereo/image_raw",
            self._on_depth, qos_profile_sensor_data)
        # Depth is device-aligned to color, so the stereo CameraInfo K is
        # the shared intrinsic for both streams.
        self.create_subscription(
            CameraInfo, f"/{cam}/stereo/camera_info",
            self._on_info, qos_profile_sensor_data)

        self._det2d_pub = self.create_publisher(
            Detection2DArray, DETECTIONS_TOPIC, 10)
        self._det3d_pub = self.create_publisher(
            Detection3DArray, DETECTIONS3D_TOPIC, 10)
        self._marker_pub = self.create_publisher(MarkerArray, MARKERS_TOPIC, 10)
        self._annotation_pub = self.create_publisher(
            ImageAnnotations, ANNOTATIONS_TOPIC_FMT.format(cam=cam), 10)
        self._grasp_pub = self.create_publisher(
            GraspCandidate, GRASP_CANDIDATE_TOPIC, 10)
        self._grasp_ok_pub = self.create_publisher(
            Bool, GRASP_OK_TOPIC, latched_qos_profile())
        self._diag_pub = self.create_publisher(
            DiagnosticArray, DIAGNOSTICS_TOPIC, 10)

        self._last_candidate_time = None
        self._grasp_ok: bool | None = None
        self._publish_grasp_ok(False)

        rate = float(self.get_parameter("target_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._on_timer)

    # --- Detector construction ----------------------------------------------

    def _make_detector(self):
        """Build the configured backend, degrading to None if yolo weights are absent."""
        kind = str(self.get_parameter("detector").value)
        if kind == "yolo":
            model_path = os.path.expanduser(
                str(self.get_parameter("model_path").value))
            try:
                return YoloV8OnnxDetector(model_path)
            except ModelUnavailable as exc:
                self.get_logger().warning(
                    f"{exc}; run `ros2 run fortis_perception download_models`."
                    " Publishing empty detections until then.")
                return None
        return HsvBlobDetector()

    # --- Input caching --------------------------------------------------------

    def _on_rgb(self, msg: CompressedImage) -> None:
        """Cache the latest jpeg RGB frame."""
        self._rgb = msg

    def _on_depth(self, msg: Image) -> None:
        """Cache the latest aligned depth frame."""
        self._depth = msg

    def _on_info(self, msg: CameraInfo) -> None:
        """Cache the latest camera intrinsics."""
        self._info = msg

    # --- Timer pipeline --------------------------------------------------------

    def _on_timer(self) -> None:
        """Run one detection pass on the latest cached frame."""
        self._refresh_grasp_ok()
        if self._detector is None:
            self._publish_degraded()
            return
        rgb = self._rgb
        if rgb is None:
            return
        self._rgb = None  # each frame is processed at most once
        bgr = cv2.imdecode(np.frombuffer(rgb.data, dtype=np.uint8),
                           cv2.IMREAD_COLOR)
        if bgr is None:
            return
        detections = [d for d in self._detector.detect(bgr)
                      if d.score >= self._min_score]
        lifted = self._lift_detections(detections)
        self._publish_2d(rgb, detections)
        self._publish_annotations(rgb, detections)
        self._publish_3d(detections, lifted)
        self._publish_markers(detections, lifted)
        self._publish_grasp(detections, lifted)

    def _lift_detections(self, detections):
        """Backproject detections to (xyz_optical, (size_x_m, size_y_m)) or None."""
        if self._depth is None or self._info is None:
            return [None] * len(detections)
        depth_msg = self._depth
        if depth_msg.encoding not in ("16UC1", "mono16"):
            return [None] * len(detections)
        depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
            depth_msg.height, depth_msg.width)
        k = self._info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        lifted = []
        for det in detections:
            x1, y1, x2, y2 = det.xyxy
            bw, bh = x2 - x1, y2 - y1
            # Median over the central 30% of the box rejects background
            # pixels bleeding into the corners.
            ua = max(0, int(x1 + 0.35 * bw))
            ub = min(depth_msg.width, max(ua + 1, int(x2 - 0.35 * bw)))
            va = max(0, int(y1 + 0.35 * bh))
            vb = min(depth_msg.height, max(va + 1, int(y2 - 0.35 * bh)))
            patch = depth[va:vb, ua:ub]
            valid = patch[patch > 0]
            if valid.size == 0:
                lifted.append(None)
                continue
            z = float(np.median(valid)) / 1000.0  # mm -> m
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            xyz = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
            lifted.append((xyz, (bw * z / fx, bh * z / fy)))
        return lifted

    # --- Publishers --------------------------------------------------------------

    def _publish_2d(self, rgb: CompressedImage, detections) -> None:
        """Publish the Detection2DArray stamped from the source frame."""
        arr = Detection2DArray()
        arr.header.stamp = rgb.header.stamp
        arr.header.frame_id = rgb.header.frame_id
        for det in detections:
            d = Detection2D()
            d.header = arr.header
            x1, y1, x2, y2 = det.xyxy
            d.bbox.center.position.x = (x1 + x2) / 2.0
            d.bbox.center.position.y = (y1 + y2) / 2.0
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = det.class_name
            hyp.hypothesis.score = float(det.score)
            d.results.append(hyp)
            arr.detections.append(d)
        self._det2d_pub.publish(arr)

    def _publish_3d(self, detections, lifted) -> None:
        """Publish Detection3DArray in the depth frame's OPTICAL frame."""
        if self._depth is None:
            return
        arr = Detection3DArray()
        arr.header.stamp = self._depth.header.stamp
        arr.header.frame_id = self._depth.header.frame_id
        for det, lift in zip(detections, lifted):
            if lift is None:
                continue
            xyz, (sx, sy) = lift
            d3 = Detection3D()
            d3.header = arr.header
            d3.bbox.center.position.x = float(xyz[0])
            d3.bbox.center.position.y = float(xyz[1])
            d3.bbox.center.position.z = float(xyz[2])
            d3.bbox.center.orientation.w = 1.0
            d3.bbox.size.x = float(sx)
            d3.bbox.size.y = float(sy)
            d3.bbox.size.z = float((sx + sy) / 2.0)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = det.class_name
            hyp.hypothesis.score = float(det.score)
            d3.results.append(hyp)
            arr.detections.append(d3)
        self._det3d_pub.publish(arr)

    def _publish_markers(self, detections, lifted) -> None:
        """Publish sphere markers in base_link via the constant front-mount extrinsic."""
        markers = MarkerArray()
        stamp = (self._depth.header.stamp if self._depth is not None
                 else self.get_clock().now().to_msg())
        for i, (det, lift) in enumerate(zip(detections, lifted)):
            if lift is None:
                continue
            xyz, (sx, sy) = lift
            base = optical_to_base(xyz)
            m = Marker()
            m.header.frame_id = BASE_FRAME
            m.header.stamp = stamp
            m.ns = "fortis_detections"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(base[0])
            m.pose.position.y = float(base[1])
            m.pose.position.z = float(base[2])
            m.pose.orientation.w = 1.0
            dia = min(MARKER_DIAMETER_MAX_M,
                      max(MARKER_DIAMETER_MIN_M, float(max(sx, sy))))
            m.scale.x = m.scale.y = m.scale.z = dia
            r, g, b = _class_color(det.class_name)
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.8
            m.lifetime = DurationMsg(sec=2)
            markers.markers.append(m)
        self._marker_pub.publish(markers)

    def _publish_annotations(self, rgb: CompressedImage, detections) -> None:
        """Publish Foxglove image annotations (box outline + label) per detection."""
        ann = ImageAnnotations()
        stamp = rgb.header.stamp
        for det in detections:
            x1, y1, x2, y2 = det.xyxy
            r, g, b = _class_color(det.class_name)
            pts = PointsAnnotation()
            pts.timestamp = stamp
            pts.type = PointsAnnotation.LINE_LOOP
            for px, py in ((x1, y1), (x2, y1), (x2, y2), (x1, y2)):
                pts.points.append(Point2(x=float(px), y=float(py)))
            pts.outline_color = Color(r=r, g=g, b=b, a=1.0)
            pts.thickness = 2.0
            ann.points.append(pts)
            txt = TextAnnotation()
            txt.timestamp = stamp
            txt.position = Point2(x=float(x1), y=float(max(y1 - 4, 0)))
            txt.text = f"{det.class_name} {det.score:.2f}"
            txt.font_size = 14.0
            txt.text_color = Color(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.background_color = Color(r=r, g=g, b=b, a=0.6)
            ann.texts.append(txt)
        self._annotation_pub.publish(ann)

    def _publish_grasp(self, detections, lifted) -> None:
        """Publish the nearest graspable in-range detection as a GraspCandidate."""
        best = None
        for det, lift in zip(detections, lifted):
            if lift is None or det.class_name not in self._graspable:
                continue
            base = optical_to_base(lift[0])
            rng = math.hypot(float(base[0]), float(base[1]))
            if rng > MAX_GRASP_RANGE_M or rng < 1e-6:
                continue
            if best is None or rng < best[1]:
                best = (det, rng, base)
        if best is None:
            return
        det, rng, base = best
        msg = GraspCandidate()
        msg.pose.position.x = float(base[0])
        msg.pose.position.y = float(base[1])
        msg.pose.position.z = float(base[2])
        yaw = math.atan2(float(base[1]), float(base[0]))
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        msg.approach.x = float(base[0]) / rng
        msg.approach.y = float(base[1]) / rng
        msg.approach.z = 0.0
        msg.confidence = float(det.score)
        if self._depth is not None:
            msg.stamp = self._depth.header.stamp
        else:
            msg.stamp = self.get_clock().now().to_msg()
        self._grasp_pub.publish(msg)
        self._last_candidate_time = self.get_clock().now()
        self._publish_grasp_ok(True)

    # --- Context flag + degradation --------------------------------------------

    def _refresh_grasp_ok(self) -> None:
        """Drop grasp_candidate_ok once no fresh candidate has been seen."""
        if not self._grasp_ok or self._last_candidate_time is None:
            return
        age_s = (self.get_clock().now()
                 - self._last_candidate_time).nanoseconds * 1e-9
        if age_s > self._candidate_timeout_s:
            self._publish_grasp_ok(False)

    def _publish_grasp_ok(self, value: bool) -> None:
        """Publish the latched context flag, only on change."""
        if value == self._grasp_ok:
            return
        self._grasp_ok = value
        self._grasp_ok_pub.publish(Bool(data=value))

    def _publish_degraded(self) -> None:
        """Keep the topic contract alive while yolo weights are missing."""
        arr = Detection2DArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        self._det2d_pub.publish(arr)
        diag = DiagnosticArray()
        diag.header.stamp = arr.header.stamp
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.WARN
        status.name = "fortis/detection"
        status.hardware_id = "fortis"
        status.message = "yolo weights missing; publishing empty detections"
        diag.status.append(status)
        self._diag_pub.publish(diag)


def main(args=None):
    """Entry point registered as the `detection_node` console script."""
    rclpy.init(args=args)
    node = DetectionNode()
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
