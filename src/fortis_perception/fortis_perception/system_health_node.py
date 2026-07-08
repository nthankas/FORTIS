"""
System health aggregator.

Publishes a diagnostic_msgs/DiagnosticArray on /diagnostics at 1 Hz
summarizing the perception / VIO / mission / arm stack: per-camera RGB
rates, VO rate + inlier count, fused-cloud rate, voxel-map size, mission
state, and arm status. Every input is optional: a missing publisher shows
up as an ERROR/stale entry, never a crash, so this node can run on a
partially-launched stack.
"""

from __future__ import annotations

import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Float32, String

from fortis_comms.qos_profiles import latched_qos_profile

try:
    from fortis_msgs.msg import ArmStatus
except ImportError:  # pragma: no cover - workspace predates ArmStatus
    ArmStatus = None

NODE_NAME = "system_health"
HARDWARE_ID = "fortis"

DEFAULT_CAMERAS = [
    "oak_chassis_front",
    "oak_chassis_rear",
    "oak_chassis_left",
    "oak_chassis_right",
]


class RateTracker:
    """Sliding-window message-rate counter (wall clock, default 5 s window)."""

    def __init__(self, window_s: float = 5.0):
        self._window_s = float(window_s)
        self._stamps: deque[float] = deque()

    def tick(self) -> None:
        """Record one message arrival at the current wall-clock time."""
        now = time.monotonic()
        self._stamps.append(now)
        self._trim(now)

    def rate(self) -> float:
        """Return the mean arrival rate over the window, in Hz."""
        self._trim(time.monotonic())
        return len(self._stamps) / self._window_s

    def _trim(self, now: float) -> None:
        """Drop stamps that have aged out of the window."""
        while self._stamps and now - self._stamps[0] > self._window_s:
            self._stamps.popleft()


def _rate_status(name: str, rate: float, expected: float) -> DiagnosticStatus:
    """Grade a topic rate against its expected value."""
    status = DiagnosticStatus(name=f"fortis/{name}", hardware_id=HARDWARE_ID)
    status.values.append(KeyValue(key="rate_hz", value=f"{rate:.2f}"))
    status.values.append(KeyValue(key="expected_hz", value=f"{expected:.1f}"))
    if rate > 0.5 * expected:
        status.level = DiagnosticStatus.OK
        status.message = "ok"
    elif rate > 0.0:
        status.level = DiagnosticStatus.WARN
        status.message = "degraded rate"
    else:
        status.level = DiagnosticStatus.ERROR
        status.message = "stale (no messages)"
    return status


class SystemHealthNode(Node):
    """Aggregate stack health onto /diagnostics."""

    def __init__(self, **node_kwargs):
        super().__init__(NODE_NAME, **node_kwargs)

        self.declare_parameter("cameras", DEFAULT_CAMERAS)
        self.declare_parameter("expected_camera_rate_hz", 15.0)
        self.declare_parameter("expected_vo_rate_hz", 10.0)
        self.declare_parameter("expected_fused_rate_hz", 5.0)

        self._cameras = list(self.get_parameter("cameras").value)
        self._expected_cam = float(
            self.get_parameter("expected_camera_rate_hz").value)
        self._expected_vo = float(
            self.get_parameter("expected_vo_rate_hz").value)
        self._expected_fused = float(
            self.get_parameter("expected_fused_rate_hz").value)

        self._cam_rates = {cam: RateTracker() for cam in self._cameras}
        self._vo_rate = RateTracker()
        self._fused_rate = RateTracker()
        self._vo_inliers: float | None = None
        self._map_voxels: int | None = None
        self._mission_state: str | None = None
        self._arm_status = None

        for cam in self._cameras:
            self.create_subscription(
                CompressedImage, f"/{cam}/rgb/image_raw/compressed",
                self._make_cam_callback(cam), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/fortis/vo",
            lambda _msg: self._vo_rate.tick(), qos_profile_sensor_data)
        self.create_subscription(
            Float32, "/fortis/vo/inliers", self._on_inliers, 10)
        self.create_subscription(
            PointCloud2, "/fortis/perception/points_fused",
            lambda _msg: self._fused_rate.tick(), qos_profile_sensor_data)
        # Latched publishers (map cloud, mission state, arm status) need a
        # TRANSIENT_LOCAL subscriber to replay the current value on connect.
        self.create_subscription(
            PointCloud2, "/fortis/perception/map/cloud",
            self._on_map_cloud, latched_qos_profile())
        self.create_subscription(
            String, "/fortis/mission_state",
            self._on_mission_state, latched_qos_profile())
        if ArmStatus is not None:
            self.create_subscription(
                ArmStatus, "/fortis/arm/status",
                self._on_arm_status, latched_qos_profile())

        self._diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0, self._on_timer)

    # --- Input callbacks ------------------------------------------------------

    def _make_cam_callback(self, cam: str):
        """Bind a per-camera rate-tick callback (closure captures cam's tracker)."""
        tracker = self._cam_rates[cam]

        def callback(_msg: CompressedImage) -> None:
            tracker.tick()
        return callback

    def _on_inliers(self, msg: Float32) -> None:
        """Cache the latest VO inlier count."""
        self._vo_inliers = float(msg.data)

    def _on_map_cloud(self, msg: PointCloud2) -> None:
        """Cache the voxel count of the latest map cloud."""
        self._map_voxels = int(msg.width) * max(int(msg.height), 1)

    def _on_mission_state(self, msg: String) -> None:
        """Cache the latest mission state name."""
        self._mission_state = msg.data

    def _on_arm_status(self, msg) -> None:
        """Cache the latest arm bridge status."""
        self._arm_status = msg

    # --- Diagnostic assembly ----------------------------------------------------

    def _on_timer(self) -> None:
        """Publish one DiagnosticArray snapshot."""
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for cam in self._cameras:
            arr.status.append(_rate_status(
                f"camera/{cam}/rgb", self._cam_rates[cam].rate(),
                self._expected_cam))
        arr.status.append(self._vo_entry())
        arr.status.append(_rate_status(
            "points_fused", self._fused_rate.rate(), self._expected_fused))
        arr.status.append(self._map_entry())
        arr.status.append(self._mission_entry())
        arr.status.append(self._arm_entry())
        self._diag_pub.publish(arr)

    def _vo_entry(self) -> DiagnosticStatus:
        """Summarize VO rate plus the last inlier count."""
        status = _rate_status("vo", self._vo_rate.rate(), self._expected_vo)
        inliers = "n/a" if self._vo_inliers is None else f"{self._vo_inliers:.0f}"
        status.values.append(KeyValue(key="inliers", value=inliers))
        return status

    def _map_entry(self) -> DiagnosticStatus:
        """Summarize the voxel map size."""
        status = DiagnosticStatus(name="fortis/map", hardware_id=HARDWARE_ID)
        if self._map_voxels is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "stale (no map cloud)"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"
            status.values.append(
                KeyValue(key="voxels", value=str(self._map_voxels)))
        return status

    def _mission_entry(self) -> DiagnosticStatus:
        """Relay the current mission state."""
        status = DiagnosticStatus(
            name="fortis/mission_state", hardware_id=HARDWARE_ID)
        if self._mission_state is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "stale (no mission state)"
        else:
            status.level = DiagnosticStatus.OK
            status.message = self._mission_state
        return status

    def _arm_entry(self) -> DiagnosticStatus:
        """Relay the arm bridge connection and fault flags."""
        status = DiagnosticStatus(name="fortis/arm", hardware_id=HARDWARE_ID)
        if ArmStatus is None:
            status.level = DiagnosticStatus.WARN
            status.message = "fortis_msgs.ArmStatus not available"
            return status
        msg = self._arm_status
        if msg is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "stale (no /fortis/arm/status)"
            return status
        status.values.append(
            KeyValue(key="connected", value=str(msg.connected)))
        status.values.append(
            KeyValue(key="fault_flags", value=f"0x{msg.fault_flags:04x}"))
        if not msg.connected:
            status.level = DiagnosticStatus.WARN
            status.message = "serial link down"
        elif msg.fault_flags:
            status.level = DiagnosticStatus.ERROR
            status.message = "arm fault"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"
        return status


def main(args=None):
    """Entry point registered as the `system_health_node` console script."""
    rclpy.init(args=args)
    node = SystemHealthNode()
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
