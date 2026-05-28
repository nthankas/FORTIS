"""
Tests for the pure-logic core of odrive_health_monitor_node.

We test compute_aggregate_health() directly rather than spinning up
rclpy. The aggregation logic is the interesting part; the rclpy plumbing
in the node class itself is exercised by integration tests
(fortis_integration_tests, when one is added for fault propagation).

Run with:
    cd /workspace
    source install/setup.bash
    python -m pytest src/fortis_safety/test/test_odrive_health_monitor_node.py -v
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from fortis_safety.odrive_health_monitor_node import compute_aggregate_health


# --- Helpers ---------------------------------------------------------------


def _make_axis(active_errors: int = 0) -> SimpleNamespace:
    """Fake OdriveAxisHealth-shaped object. Only active_errors matters here."""
    return SimpleNamespace(
        node_id=0,
        armed=True,
        active_errors=active_errors,
        vbus_voltage=48.0,
        motor_temperature=25.0,
        fet_temperature=25.0,
    )


def _make_health(
    fl_err: int = 0,
    fr_err: int = 0,
    rl_err: int = 0,
    rr_err: int = 0,
) -> SimpleNamespace:
    """Fake OdriveHealth-shaped object with per-side active_errors knobs."""
    return SimpleNamespace(
        fl=_make_axis(active_errors=fl_err),
        fr=_make_axis(active_errors=fr_err),
        rl=_make_axis(active_errors=rl_err),
        rr=_make_axis(active_errors=rr_err),
    )


WATCHDOG_S = 1.0


# --- "Insufficient information" -> False (fail safe on startup) ------------


def test_no_snapshot_yet_is_unhealthy():
    """Before any message arrives, drive_healthy must be False (fail safe)."""
    assert compute_aggregate_health(None, None, WATCHDOG_S) is False


def test_snapshot_without_age_is_unhealthy():
    """A snapshot with no recorded receive time is treated as unhealthy."""
    msg = _make_health()
    assert compute_aggregate_health(msg, None, WATCHDOG_S) is False


# --- Fresh + clean -> True --------------------------------------------------


def test_all_axes_clean_and_fresh_is_healthy():
    """Fresh snapshot, all four axes report zero active_errors."""
    msg = _make_health()
    assert compute_aggregate_health(msg, age_s=0.0, watchdog_timeout_s=WATCHDOG_S) is True


def test_snapshot_at_exact_timeout_boundary_is_healthy():
    """age_s == watchdog_timeout_s is NOT stale (the check is strict >)."""
    msg = _make_health()
    assert compute_aggregate_health(msg, age_s=WATCHDOG_S, watchdog_timeout_s=WATCHDOG_S) is True


# --- Staleness -> False ----------------------------------------------------


def test_stale_snapshot_is_unhealthy_even_if_clean():
    """A snapshot older than the watchdog is treated as if no data exists."""
    msg = _make_health()
    age = WATCHDOG_S + 0.01
    assert compute_aggregate_health(msg, age_s=age, watchdog_timeout_s=WATCHDOG_S) is False


# --- Any non-zero active_errors on any axis -> False -----------------------


@pytest.mark.parametrize(
    "fl,fr,rl,rr",
    [
        (1, 0, 0, 0),
        (0, 1, 0, 0),
        (0, 0, 1, 0),
        (0, 0, 0, 1),
        (0xDEAD_BEEF, 0, 0, 0),  # arbitrary bitfield value
        (1, 1, 1, 1),
    ],
)
def test_any_axis_with_errors_is_unhealthy(fl: int, fr: int, rl: int, rr: int):
    """Any non-zero active_errors on any wheel must flip drive_healthy False."""
    msg = _make_health(fl_err=fl, fr_err=fr, rl_err=rl, rr_err=rr)
    assert (
        compute_aggregate_health(msg, age_s=0.0, watchdog_timeout_s=WATCHDOG_S)
        is False
    )


def test_axis_label_to_active_errors_isolation():
    """Each axis name maps to its own active_errors slot, no cross-talk."""
    # If the function read the wrong attribute (e.g. always 'fl') this
    # test would silently pass; the parametrized test above guards
    # against that. This test guards against a different regression:
    # changing the order of side iteration (which shouldn't matter but
    # would surface here if any axis-specific side effect crept in).
    msg = _make_health(rl_err=42)
    assert (
        compute_aggregate_health(msg, age_s=0.0, watchdog_timeout_s=WATCHDOG_S)
        is False
    )
    msg = _make_health(rr_err=42)
    assert (
        compute_aggregate_health(msg, age_s=0.0, watchdog_timeout_s=WATCHDOG_S)
        is False
    )
