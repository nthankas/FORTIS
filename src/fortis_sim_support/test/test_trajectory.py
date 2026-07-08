"""Closed-form checks for the analytic trajectories (pure numpy, no rclpy)."""

from __future__ import annotations

import numpy as np
import pytest

from fortis_sim_support.trajectory import hold, line, orbit


def _yaw(quat_xyzw: np.ndarray) -> float:
    """Return the yaw of a planar (yaw-only) xyzw quaternion."""
    return 2.0 * np.arctan2(quat_xyzw[2], quat_xyzw[3])


def test_orbit_radius_speed_and_rate():
    """Orbit position stays on the circle; |v| = R * |omega|; wz = omega."""
    center = (0.5, -0.3)
    radius, omega = 1.2, 0.4
    traj = orbit(center=center, radius=radius, omega=omega, z=0.15)
    for t in np.linspace(0.0, 25.0, 41):
        pos, _, lin, ang = traj.sample(t)
        dist = np.hypot(pos[0] - center[0], pos[1] - center[1])
        assert dist == pytest.approx(radius, abs=1e-12)
        assert pos[2] == pytest.approx(0.15, abs=1e-12)
        assert np.linalg.norm(lin) == pytest.approx(radius * omega, abs=1e-12)
        assert list(ang) == pytest.approx([0.0, 0.0, omega], abs=1e-12)


def test_orbit_front_faces_center():
    """base_link -X (the FORTIS front) points at the center for all t, both ways."""
    center = np.array([1.0, 2.0])
    radius = 0.8
    for omega in (0.4, -0.25):
        traj = orbit(center=tuple(center), radius=radius, omega=omega)
        for t in np.linspace(0.0, 30.0, 31):
            pos, quat, _, _ = traj.sample(t)
            yaw = _yaw(quat)
            front = -np.array([np.cos(yaw), np.sin(yaw)])
            to_center = (center - pos[:2]) / radius
            assert np.allclose(front, to_center, atol=1e-9)


def test_orbit_velocity_matches_position_derivative():
    """The closed-form twist is the exact derivative of the closed-form pose."""
    traj = orbit(center=(0.0, 0.0), radius=1.0, omega=0.3)
    h = 1e-6
    for t in (0.0, 1.7, 9.3):
        p_plus, _, _, _ = traj.sample(t + h)
        p_minus, _, _, _ = traj.sample(t - h)
        _, _, lin, _ = traj.sample(t)
        numeric = (p_plus - p_minus) / (2.0 * h)
        assert np.allclose(numeric, lin, atol=1e-6)


def test_orbit_rejects_bad_radius():
    """A non-positive radius is a configuration error, not a silent NaN."""
    with pytest.raises(ValueError):
        orbit(radius=0.0)


def test_line_position_linear_in_t():
    """Line position is start + velocity * t exactly; heading is constant."""
    start = np.array([1.0, -2.0, 0.0])
    velocity = np.array([0.2, 0.1, 0.0])
    traj = line(start=tuple(start), velocity=tuple(velocity), yaw=0.3)
    for t in (0.0, 0.5, 3.25, 10.0):
        pos, quat, lin, ang = traj.sample(t)
        assert np.allclose(pos, start + velocity * t, atol=1e-12)
        assert np.allclose(lin, velocity, atol=1e-12)
        assert np.allclose(ang, 0.0, atol=1e-12)
        assert _yaw(quat) == pytest.approx(0.3, abs=1e-12)


def test_hold_is_constant_with_zero_twist():
    """Hold returns the same pose and a zero twist at every t."""
    traj = hold(xyz=(1.0, 0.0, 0.0), yaw=-0.7)
    p0, q0, _, _ = traj.sample(0.0)
    for t in (0.0, 2.0, 100.0):
        pos, quat, lin, ang = traj.sample(t)
        assert np.allclose(pos, p0)
        assert np.allclose(quat, q0)
        assert np.allclose(lin, 0.0)
        assert np.allclose(ang, 0.0)
