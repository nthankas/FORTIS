"""
Bridge from the upstream odrive_can status topics to fortis_msgs/OdriveHealth.

Subscribes (per axis, node_id 0..3)
-----------------------------------
    /odrive_axis<N>/odrive_status       odrive_can/ODriveStatus
    /odrive_axis<N>/controller_status   odrive_can/ControllerStatus
        Published by the upstream odrive_can node (one per S1, namespaced
        by CAN node_id). The FORTIS chain order fixes the wheel mapping:
        FL=0, FR=1, RR=2, RL=3.

Publishes
---------
    /fortis/drive/odrive_health   fortis_msgs/OdriveHealth
        The four-axis snapshot consumed by odrive_health_monitor_node.
        Published on a 10 Hz timer, but ONLY while every axis has fresh
        data on BOTH status streams: a silent axis stops the snapshot
        stream entirely, so the monitor's own watchdog flips
        drive_healthy False instead of trusting a half-stale snapshot.

Translation choices
-------------------
    armed          axis_state == CLOSED_LOOP_CONTROL (8). Any other state
                   (IDLE, calibration, error) reports disarmed.
    active_errors  OR of both upstream bitfields PLUS disarm_reason. The
                   firmware clears active_errors once the condition
                   passes but latches disarm_reason; without the fold a
                   disarmed-by-fault axis would read clean the moment
                   the transient cleared. Recovery therefore requires a
                   real error-clear, matching the FSM's
                   operator-acknowledged RESET.

The vendored odrive_can msgs are imported lazily: FORTIS deliberately
never builds the pruned odrive_node package in the dev container, so
this module must import (for the pure helpers and tests) without them.
Constructing the node without the msgs installed raises with a pointer
at the vendor build.
"""

from __future__ import annotations

from typing import Optional, Sequence

import rclpy
from fortis_msgs.msg import OdriveAxisHealth, OdriveHealth
from rclpy.node import Node
from rclpy.time import Time

try:
    from odrive_can.msg import ControllerStatus, ODriveStatus
except ImportError:  # pruned vendor package absent; see module docstring
    ControllerStatus = None
    ODriveStatus = None


#: Output topic; matches odrive_health_monitor_node's DEFAULT_HEALTH_TOPIC.
HEALTH_TOPIC: str = "/fortis/drive/odrive_health"

#: Wheel -> CAN node_id, in chain order (FL=0 FR=1 RR=2 RL=3; RL is the
#: terminated far end). Field names match OdriveHealth.
WHEEL_NODE_IDS: tuple[tuple[str, int], ...] = (
    ("fl", 0), ("fr", 1), ("rr", 2), ("rl", 3),
)

#: odrive_enums.h AXIS_STATE_CLOSED_LOOP_CONTROL.
AXIS_STATE_CLOSED_LOOP_CONTROL: int = 8

#: Snapshot publish period. 10 Hz is comfortably inside the monitor's
#: 1.0 s watchdog and the upstream ~10 Hz heartbeat-driven status rate.
PUBLISH_PERIOD_S: float = 0.1

#: Max age (s) of any single upstream status message before the whole
#: snapshot is withheld. Half the monitor watchdog: the stream stops
#: early enough for staleness to be attributed to the silent axis.
STALE_AFTER_S: float = 0.5


def axis_health(
    node_id: int, odrive_status, controller_status
) -> OdriveAxisHealth:
    """Merge one axis's two upstream status messages into OdriveAxisHealth.

    Accepts any objects with the odrive_can field names so the mapping is
    testable without the vendored msgs installed.
    """
    health = OdriveAxisHealth()
    health.node_id = node_id
    health.armed = (
        controller_status.axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL
    )
    # Fold disarm_reason in: see "Translation choices" in the module
    # docstring for why a latched disarm must keep the axis unhealthy.
    health.active_errors = (
        int(odrive_status.active_errors)
        | int(controller_status.active_errors)
        | int(odrive_status.disarm_reason)
    )
    health.vbus_voltage = float(odrive_status.bus_voltage)
    health.motor_temperature = float(odrive_status.motor_temperature)
    health.fet_temperature = float(odrive_status.fet_temperature)
    return health


def snapshot_is_fresh(
    ages_s: Sequence[Optional[float]], stale_after_s: float
) -> bool:
    """True iff every per-stream age exists and is inside the window."""
    return all(age is not None and age <= stale_after_s for age in ages_s)


class OdriveStatusBridgeNode(Node):
    """Translate per-axis odrive_can status into the OdriveHealth contract."""

    def __init__(self) -> None:
        super().__init__("odrive_status_bridge")

        if ODriveStatus is None or ControllerStatus is None:
            raise RuntimeError(
                "odrive_can msgs are not installed. Build the vendored "
                "odrive_can package (tools/vendor_import.sh + colcon) on "
                "the hardware image before launching odrive_bridge:=true."
            )

        # Latest message + receive time per wheel per stream. Keyed
        # (wheel, kind) with kind in {"odrv", "ctrl"}.
        self._latest: dict[tuple[str, str], object] = {}
        self._stamps: dict[tuple[str, str], Time] = {}

        for wheel, node_id in WHEEL_NODE_IDS:
            self.create_subscription(
                ODriveStatus,
                f"/odrive_axis{node_id}/odrive_status",
                self._make_callback(wheel, "odrv"),
                10,
            )
            self.create_subscription(
                ControllerStatus,
                f"/odrive_axis{node_id}/controller_status",
                self._make_callback(wheel, "ctrl"),
                10,
            )

        self._health_pub = self.create_publisher(OdriveHealth, HEALTH_TOPIC, 10)
        self._timer = self.create_timer(PUBLISH_PERIOD_S, self._on_tick)

        self.get_logger().info(
            "odrive_status_bridge up: /odrive_axis{0..3}/* -> "
            f"{HEALTH_TOPIC} (stale_after_s={STALE_AFTER_S})"
        )

    # --- Callbacks ---------------------------------------------------------

    def _make_callback(self, wheel: str, kind: str):
        """Build a subscription callback bound to one (wheel, stream) slot."""
        key = (wheel, kind)

        def _on_status(msg) -> None:
            self._latest[key] = msg
            self._stamps[key] = self.get_clock().now()

        return _on_status

    def _on_tick(self) -> None:
        """Publish a snapshot when every axis is fresh on both streams."""
        now = self.get_clock().now()
        ages = [
            (
                (now - self._stamps[(wheel, kind)]).nanoseconds / 1e9
                if (wheel, kind) in self._stamps
                else None
            )
            for wheel, _ in WHEEL_NODE_IDS
            for kind in ("odrv", "ctrl")
        ]
        if not snapshot_is_fresh(ages, STALE_AFTER_S):
            return

        snapshot = OdriveHealth()
        for wheel, node_id in WHEEL_NODE_IDS:
            setattr(
                snapshot,
                wheel,
                axis_health(
                    node_id,
                    self._latest[(wheel, "odrv")],
                    self._latest[(wheel, "ctrl")],
                ),
            )
        snapshot.stamp = now.to_msg()
        self._health_pub.publish(snapshot)


def main(args: Optional[list[str]] = None) -> None:
    """Entry point registered as the `odrive_status_bridge` script."""
    rclpy.init(args=args)
    node = OdriveStatusBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
