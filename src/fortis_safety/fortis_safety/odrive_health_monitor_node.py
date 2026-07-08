"""
ODrive health aggregator for the FORTIS safety pipeline.

Subscribes
----------
    /fortis/drive/odrive_health   fortis_msgs/OdriveHealth
        Snapshot of all four ODrive S1 axes, published by a separate
        bridge node that translates the upstream odrive_ros2_control
        status into the FORTIS-internal contract. Defaults to that
        topic; override via the `health_topic` parameter.

Publishes
---------
    /fortis/context/drive_healthy   std_msgs/Bool
        Latched (TRANSIENT_LOCAL) Boolean consumed by mission_state_node
        as a context field. True iff a fresh health snapshot has arrived
        AND all four axes report `active_errors == 0`. Goes False on any
        non-zero active_errors bitfield, or if no health snapshot arrives
        within `watchdog_timeout_s`.

    /fortis/events/fault           std_msgs/Empty
        Emitted on the True->False edge of drive_healthy. This is the
        signal that triggers mission_state_machine.Event.FAULT and drives
        the FSM into State.FAULT (the wildcard FAULT transition in
        mission_state_machine.TRANSITIONS). Not emitted on the inverse
        edge — recovery requires explicit operator acknowledgement
        (Event.RESET + operator_ack=True).

Why this node exists
--------------------
The upstream odrive_ros2_control plugin owns closed-loop control of the
four S1s but does NOT surface per-axis disarm / undervoltage / overcurrent
events into ros2_control's hardware-interface state. The mission FSM in
fortis_safety needs that visibility to react to motor faults; this node
closes that gap on the FORTIS side.

Design choice: this node consumes a FORTIS-internal OdriveHealth message
rather than the upstream /odrive_status directly. A small translator
node (not yet built) will convert upstream -> FORTIS-internal. That
decoupling means future upstream-schema churn is contained to the
translator, and the safety boundary speaks a stable FORTIS-owned
contract.

Startup semantics
-----------------
On startup the node publishes `drive_healthy = False` immediately, with
the latched QoS so any subscriber that comes up later sees the
conservative default. It only flips to True after a real OdriveHealth
message arrives with all four axes clean. This means a mission FSM
guarding ORBIT entry on `drive_healthy` will refuse to enter ORBIT until
the bridge is up and reporting — failing safe.

The startup-False is intentional and does NOT emit Event.FAULT (the
event fires only on a True->False edge, and the initial value was never
True).
"""

from __future__ import annotations

from typing import Optional

import rclpy
from fortis_comms.qos_profiles import latched_qos_profile
from fortis_msgs.msg import OdriveHealth
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Empty


DEFAULT_HEALTH_TOPIC: str = "/fortis/drive/odrive_health"
DRIVE_HEALTHY_CONTEXT_TOPIC: str = "/fortis/context/drive_healthy"
FAULT_EVENT_TOPIC: str = "/fortis/events/fault"

# Watchdog timer tick. Has to be short relative to watchdog_timeout_s so
# stale-data detection is responsive. 100 ms is fast enough that a 1.0 s
# default timeout fires within ~one tick of becoming stale.
WATCHDOG_TICK_S: float = 0.1


def compute_aggregate_health(
    latest_health: Optional[object],
    age_s: Optional[float],
    watchdog_timeout_s: float,
) -> bool:
    """Decide drive_healthy from a (possibly stale) health snapshot.

    Pure-logic core, extracted as a module-level function so tests can
    exercise the aggregation logic without spinning up rclpy or
    constructing the node.

    Inputs:
      latest_health         the most recent OdriveHealth-shaped object,
                            with attributes fl / fr / rl / rr each
                            exposing `.active_errors` (int). Pass None if
                            no message has ever arrived.
      age_s                 seconds elapsed since latest_health arrived.
                            Pass None when no message has arrived.
      watchdog_timeout_s    threshold beyond which a snapshot is treated
                            as stale (regardless of its contents).

    Returns True iff there is a fresh snapshot AND every axis reports
    `active_errors == 0`. Per-axis `armed` is intentionally NOT part of
    this check — an axis being IDLE is normal operating state in the
    non-driving mission phases; only a latched error code constitutes a
    fault.
    """
    if latest_health is None or age_s is None:
        return False
    if age_s > watchdog_timeout_s:
        return False
    for side in ("fl", "fr", "rl", "rr"):
        if getattr(latest_health, side).active_errors != 0:
            return False
    return True


class OdriveHealthMonitorNode(Node):
    """Aggregate per-axis ODrive health into a single drive_healthy signal."""

    def __init__(self) -> None:
        super().__init__("odrive_health_monitor_node")

        self.declare_parameter("health_topic", DEFAULT_HEALTH_TOPIC)
        self.declare_parameter("watchdog_timeout_s", 1.0)

        health_topic: str = self.get_parameter("health_topic").value
        self._watchdog_timeout_s: float = float(
            self.get_parameter("watchdog_timeout_s").value
        )

        # Latched QoS on the context publication. Mirrors the pattern in
        # mission_state_node: TRANSIENT_LOCAL durability + RELIABLE so a
        # late subscriber sees the most recent drive_healthy value on
        # connect rather than the absence of one. depth=1 is sufficient
        # because only the latest matters.
        latched_qos = latched_qos_profile()

        self._context_pub = self.create_publisher(
            Bool, DRIVE_HEALTHY_CONTEXT_TOPIC, latched_qos
        )
        # Fault event uses default (RELIABLE, VOLATILE) QoS. The event is
        # an edge trigger; a missed message would mean the FSM doesn't
        # transition to FAULT even though one occurred, which is bad —
        # but TRANSIENT_LOCAL on an Event topic would replay old faults
        # to new subscribers, which is worse. Volatile + reliable is the
        # right tradeoff here; the mission_state_node's event subscriber
        # uses the same shape.
        self._fault_pub = self.create_publisher(Empty, FAULT_EVENT_TOPIC, 10)

        self._health_sub = self.create_subscription(
            OdriveHealth, health_topic, self._on_health, 10
        )

        # Watchdog timer. Fires _evaluate() so stale data is detected
        # without waiting for the next OdriveHealth message (which by
        # definition won't arrive if the upstream bridge is dead).
        self._timer = self.create_timer(WATCHDOG_TICK_S, self._on_watchdog_tick)

        self._latest_health: Optional[OdriveHealth] = None
        self._last_message_time: Optional[Time] = None
        # Tri-state: None means "never published yet"; this lets the
        # startup publish always fire even if the conservative default is
        # False.
        self._was_healthy: Optional[bool] = None

        # Conservative startup: publish False immediately so latched
        # subscribers see a value the moment they connect. This does NOT
        # emit Event.FAULT (no True->False edge has occurred yet).
        self._publish_context(False)
        self._was_healthy = False

        self.get_logger().info(
            f"odrive_health_monitor_node up. "
            f"Subscribed to {health_topic}, "
            f"watchdog_timeout_s={self._watchdog_timeout_s}. "
            f"Initial drive_healthy=False (awaiting first health snapshot)."
        )

    # --- Callbacks ---------------------------------------------------------

    def _on_health(self, msg: OdriveHealth) -> None:
        """Snapshot a fresh health message and re-evaluate the aggregate."""
        self._latest_health = msg
        self._last_message_time = self.get_clock().now()
        self._evaluate(reason="health snapshot received")

    def _on_watchdog_tick(self) -> None:
        """Re-evaluate periodically so stale data flips drive_healthy off."""
        self._evaluate(reason="watchdog tick")

    # --- Aggregation logic -------------------------------------------------

    def _compute_healthy(self) -> bool:
        """Thin wrapper around compute_aggregate_health using rclpy clock."""
        age_s: Optional[float] = None
        if self._last_message_time is not None:
            now = self.get_clock().now()
            age_s = (now - self._last_message_time).nanoseconds / 1e9
        return compute_aggregate_health(
            self._latest_health, age_s, self._watchdog_timeout_s
        )

    def _evaluate(self, reason: str) -> None:
        """Compute current health, publish on change, emit fault on edge."""
        healthy = self._compute_healthy()
        if healthy == self._was_healthy:
            return

        self._publish_context(healthy)

        # Emit Event.FAULT only on the True->False edge. The reverse edge
        # (False->True) is recovery, which must require explicit operator
        # acknowledgement per the FSM's Transition(FAULT, RESET, IDLE,
        # operator_ack) rule — never automatic.
        if self._was_healthy is True and healthy is False:
            self.get_logger().warn(
                f"drive_healthy True->False ({reason}); emitting Event.FAULT"
            )
            self._fault_pub.publish(Empty())
        else:
            self.get_logger().info(f"drive_healthy={healthy} ({reason})")

        self._was_healthy = healthy

    # --- Helpers ---------------------------------------------------------

    def _publish_context(self, healthy: bool) -> None:
        msg = Bool()
        msg.data = healthy
        self._context_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    """Entry point registered as the `odrive_health_monitor_node` script."""
    rclpy.init(args=args)
    node = OdriveHealthMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
