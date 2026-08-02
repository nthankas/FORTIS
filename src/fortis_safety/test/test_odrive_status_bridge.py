"""
Tests for fortis_safety.odrive_status_bridge_node.

Two layers, mirroring test_odrive_health_monitor_node:

1. The pure translation core (axis_health / snapshot_is_fresh) is tested
   with plain attribute objects -- no rclpy, no vendored messages -- so
   the per-axis field mapping and the freshness gate are covered on
   every run.
2. A ROS round-trip test publishes real odrive_can status messages on
   the eight upstream topics and asserts the assembled OdriveHealth
   snapshot. It SKIPS when the vendored odrive_can msgs are not
   installed (the FORTIS build deliberately never builds odrive_node,
   so this only runs where the vendor package has been built -- i.e.
   the Jetson hardware image).

Run with:
    cd /workspace
    source install/setup.bash
    python3 -m pytest src/fortis_safety/test/test_odrive_status_bridge.py -v
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from fortis_safety.odrive_status_bridge_node import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    WHEEL_NODE_IDS,
    axis_health,
    snapshot_is_fresh,
)

try:
    from odrive_can.msg import ControllerStatus, ODriveStatus  # noqa: F401
    HAVE_ODRIVE_CAN = True
except ImportError:
    HAVE_ODRIVE_CAN = False

requires_odrive_can = pytest.mark.skipif(
    not HAVE_ODRIVE_CAN,
    reason=(
        "vendored odrive_can msgs not installed (FORTIS never builds the "
        "pruned odrive_node package); ROS round-trip runs on the hardware "
        "image only"
    ),
)


# --- Helpers ----------------------------------------------------------------


def _odrive_status(
    bus_voltage: float = 48.0,
    fet_temperature: float = 30.0,
    motor_temperature: float = 25.0,
    active_errors: int = 0,
    disarm_reason: int = 0,
) -> SimpleNamespace:
    """Fake odrive_can/ODriveStatus-shaped object."""
    return SimpleNamespace(
        bus_voltage=bus_voltage,
        bus_current=1.0,
        fet_temperature=fet_temperature,
        motor_temperature=motor_temperature,
        active_errors=active_errors,
        disarm_reason=disarm_reason,
    )


def _controller_status(
    axis_state: int = AXIS_STATE_CLOSED_LOOP_CONTROL,
    active_errors: int = 0,
) -> SimpleNamespace:
    """Fake odrive_can/ControllerStatus-shaped object."""
    return SimpleNamespace(
        pos_estimate=0.0,
        vel_estimate=0.0,
        torque_target=0.0,
        torque_estimate=0.0,
        iq_setpoint=0.0,
        iq_measured=0.0,
        active_errors=active_errors,
        axis_state=axis_state,
        procedure_result=0,
        trajectory_done_flag=False,
    )


# --- Wheel/node_id contract -------------------------------------------------


def test_wheel_node_id_map_matches_chain_order():
    """FL=0 FR=1 RR=2 RL=3: chain position equals node_id, RL is the far end."""
    assert WHEEL_NODE_IDS == (("fl", 0), ("fr", 1), ("rr", 2), ("rl", 3))


# --- axis_health field mapping ----------------------------------------------


def test_axis_health_maps_fields():
    """Every OdriveAxisHealth field comes from the right upstream field."""
    health = axis_health(
        2,
        _odrive_status(
            bus_voltage=47.5, fet_temperature=41.0, motor_temperature=33.5),
        _controller_status(axis_state=AXIS_STATE_CLOSED_LOOP_CONTROL),
    )
    assert health.node_id == 2
    assert health.armed is True
    assert health.active_errors == 0
    assert health.vbus_voltage == pytest.approx(47.5)
    assert health.fet_temperature == pytest.approx(41.0)
    assert health.motor_temperature == pytest.approx(33.5)


@pytest.mark.parametrize("axis_state", [0, 1, 3, 6, 7])
def test_axis_health_not_armed_outside_closed_loop(axis_state: int):
    """armed is True iff axis_state == CLOSED_LOOP_CONTROL (8)."""
    health = axis_health(
        0, _odrive_status(), _controller_status(axis_state=axis_state))
    assert health.armed is False


def test_axis_health_ors_both_error_bitfields():
    """A fault on EITHER status stream must surface in active_errors."""
    only_odrv = axis_health(
        0, _odrive_status(active_errors=0x10), _controller_status())
    only_ctrl = axis_health(
        0, _odrive_status(), _controller_status(active_errors=0x02))
    both = axis_health(
        0,
        _odrive_status(active_errors=0x10),
        _controller_status(active_errors=0x02),
    )
    assert only_odrv.active_errors == 0x10
    assert only_ctrl.active_errors == 0x02
    assert both.active_errors == 0x12


def test_axis_health_disarm_reason_counts_as_error():
    """A latched disarm_reason with no active error is still a fault.

    The firmware clears active_errors once the condition passes but
    leaves disarm_reason latched; the safety layer must not report a
    disarmed-by-fault axis as clean.
    """
    health = axis_health(
        0, _odrive_status(active_errors=0, disarm_reason=0x40),
        _controller_status())
    assert health.active_errors != 0


# --- Freshness gate ----------------------------------------------------------


def test_snapshot_fresh_when_all_axes_recent():
    """All eight per-axis ages inside the window => publishable."""
    assert snapshot_is_fresh([0.0, 0.1, 0.2, 0.3], stale_after_s=0.5) is True


def test_snapshot_not_fresh_with_missing_axis():
    """An axis that has never reported (age None) blocks the snapshot."""
    assert snapshot_is_fresh([0.0, None, 0.1, 0.1], stale_after_s=0.5) is False


def test_snapshot_not_fresh_with_stale_axis():
    """One silent axis past the window blocks the snapshot, so the
    downstream health monitor's watchdog flips drive_healthy False."""
    assert snapshot_is_fresh([0.0, 0.1, 0.9, 0.1], stale_after_s=0.5) is False


# --- ROS round trip (needs the vendored odrive_can msgs) ---------------------


@requires_odrive_can
def test_bridge_round_trip_publishes_odrive_health():
    """Eight upstream status streams in => one mapped OdriveHealth out."""
    import time

    import rclpy
    from fortis_safety.odrive_status_bridge_node import (
        HEALTH_TOPIC,
        OdriveStatusBridgeNode,
    )
    from fortis_msgs.msg import OdriveHealth

    rclpy.init()
    try:
        bridge = OdriveStatusBridgeNode()
        helper = rclpy.create_node("odrive_bridge_test_helper")
        received: list[OdriveHealth] = []
        helper.create_subscription(
            OdriveHealth, HEALTH_TOPIC, received.append, 10)

        pubs = {}
        for wheel, node_id in WHEEL_NODE_IDS:
            pubs[wheel] = (
                helper.create_publisher(
                    ODriveStatus, f"/odrive_axis{node_id}/odrive_status", 10),
                helper.create_publisher(
                    ControllerStatus,
                    f"/odrive_axis{node_id}/controller_status", 10),
            )

        def spin(duration_s: float) -> None:
            end = time.monotonic() + duration_s
            while time.monotonic() < end:
                rclpy.spin_once(bridge, timeout_sec=0.02)
                rclpy.spin_once(helper, timeout_sec=0.02)

        spin(0.3)  # discovery

        # FR (node_id 1) carries a fault; everything else is clean+armed.
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            for wheel, node_id in WHEEL_NODE_IDS:
                odrv = ODriveStatus()
                odrv.bus_voltage = 48.0
                ctrl = ControllerStatus()
                ctrl.axis_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                if wheel == "fr":
                    odrv.active_errors = 0x20
                odrv_pub, ctrl_pub = pubs[wheel]
                odrv_pub.publish(odrv)
                ctrl_pub.publish(ctrl)
            spin(0.3)

        assert received, "bridge never published an OdriveHealth snapshot"
        snapshot = received[-1]
        assert snapshot.fl.node_id == 0
        assert snapshot.fr.node_id == 1
        assert snapshot.rr.node_id == 2
        assert snapshot.rl.node_id == 3
        assert snapshot.fr.active_errors == 0x20
        assert snapshot.fl.active_errors == 0
        assert snapshot.fl.armed is True
        assert snapshot.fl.vbus_voltage == pytest.approx(48.0)

        helper.destroy_node()
        bridge.destroy_node()
    finally:
        rclpy.shutdown()
