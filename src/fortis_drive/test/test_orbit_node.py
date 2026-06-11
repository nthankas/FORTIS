"""
Tests for fortis_drive.orbit_node.

The orbit kinematics live in the pure ``orbit_twist()`` function -- no rclpy --
so the face-center geometry (a tangential strafe plus a yaw rate coupled by the
radius) is tested deterministically, the same pure-core split that
drive_node/heading_hold_node use. The ROS plumbing (the held-command dead-man
and the /cmd_vel stream) is covered by a node harness once the kinematics land.

The asserts here are convention-INDEPENDENT on purpose: magnitudes (|Vy| = v,
|omega| = v/R), the 1/R coupling, and reversal symmetry. The absolute CW/CCW
sense -- and whether omega must flip relative to Vy to keep the cameras on the
center -- is mount-dependent and lives in the node's OMEGA_SIGN bench knob, not
in this pure function, so the tests do not pin it.

ROS_DOMAIN_ID is pinned to 91 by test/conftest.py.

Run with:
    cd /workspace
    colcon build --packages-select fortis_drive
    source install/setup.bash
    python3 -m pytest src/fortis_drive/test/test_orbit_node.py -v
"""

from __future__ import annotations

import pytest

from fortis_drive.orbit_node import orbit_twist


def test_stop_direction_is_zero_twist():
    """direction 0 (button released) yields no motion at all."""
    vy, omega = orbit_twist(0.0, speed=0.1, radius=1.0)
    assert vy == pytest.approx(0.0)
    assert omega == pytest.approx(0.0)


def test_strafe_speed_equals_orbit_speed():
    """The tangential strafe magnitude is exactly the orbit speed."""
    vy, _ = orbit_twist(1.0, speed=0.1, radius=1.0)
    assert abs(vy) == pytest.approx(0.1)


def test_yaw_rate_is_speed_over_radius():
    """Facing the center means yawing at omega = v / R as you go around."""
    _, omega = orbit_twist(1.0, speed=0.1, radius=1.0)
    assert abs(omega) == pytest.approx(0.1 / 1.0)


def test_larger_radius_gives_gentler_yaw():
    """omega scales as 1/R: doubling the radius halves the yaw rate."""
    _, omega_tight = orbit_twist(1.0, speed=0.1, radius=1.0)
    _, omega_wide = orbit_twist(1.0, speed=0.1, radius=2.0)
    assert abs(omega_wide) == pytest.approx(abs(omega_tight) / 2.0)


def test_radius_couples_strafe_and_yaw():
    """The defining orbit relationship: |omega| / |vy| == 1 / radius."""
    vy, omega = orbit_twist(1.0, speed=0.2, radius=0.5)
    assert abs(omega) / abs(vy) == pytest.approx(1.0 / 0.5)


def test_reversing_direction_negates_both_components():
    """CW is the mirror of CCW: both the strafe and the yaw flip sign.

    Holds for either coupling convention (same- or opposite-sign), because
    reversing ``direction`` negates whatever the +1 case produced.
    """
    vy_ccw, omega_ccw = orbit_twist(1.0, speed=0.1, radius=1.0)
    vy_cw, omega_cw = orbit_twist(-1.0, speed=0.1, radius=1.0)
    assert vy_cw == pytest.approx(-vy_ccw)
    assert omega_cw == pytest.approx(-omega_ccw)
