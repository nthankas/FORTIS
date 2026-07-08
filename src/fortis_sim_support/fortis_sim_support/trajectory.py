"""
Analytic base_link trajectories in the odom frame.

Closed-form pose AND twist at any t -- exact ground truth for VIO and
mapping error bounds (no integration error). All trajectories are
planar: fixed z, yaw-only orientation. Every trajectory object exposes
sample(t) -> (xyz, quat_xyzw, lin_vel_world, ang_vel_world), each a
numpy array.

Orbit heading convention (matches fortis_drive.orbit_node's
face-the-center semantics): the robot circles the center while its
FRONT -- base_link -X per the FORTIS chassis convention -- points AT
the center. Equivalently base_link +X points radially outward, so
yaw(t) equals the polar angle of the robot position about the center.
This holds for both signs of omega.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def quat_from_yaw(yaw: float) -> np.ndarray:
    """Return the (x, y, z, w) quaternion for a planar rotation about +Z."""
    return np.array([0.0, 0.0, np.sin(0.5 * yaw), np.cos(0.5 * yaw)])


@dataclass(frozen=True)
class Hold:
    """A constant pose with zero twist."""

    xyz: tuple[float, float, float]
    yaw: float

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return (xyz, quat_xyzw, lin_vel_world, ang_vel_world) at time t."""
        del t  # constant by definition
        return (
            np.asarray(self.xyz, dtype=np.float64),
            quat_from_yaw(self.yaw),
            np.zeros(3),
            np.zeros(3),
        )


@dataclass(frozen=True)
class Line:
    """A constant-velocity straight line at a fixed heading."""

    start: tuple[float, float, float]
    velocity: tuple[float, float, float]
    yaw: float

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return (xyz, quat_xyzw, lin_vel_world, ang_vel_world) at time t."""
        vel = np.asarray(self.velocity, dtype=np.float64)
        pos = np.asarray(self.start, dtype=np.float64) + vel * t
        return pos, quat_from_yaw(self.yaw), vel, np.zeros(3)


@dataclass(frozen=True)
class Orbit:
    """A constant-rate circle with the chassis front (-X) facing the center."""

    center: tuple[float, float]
    radius: float
    omega: float
    z: float
    theta0: float

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return (xyz, quat_xyzw, lin_vel_world, ang_vel_world) at time t."""
        theta = self.theta0 + self.omega * t
        c, s = np.cos(theta), np.sin(theta)
        pos = np.array([
            self.center[0] + self.radius * c,
            self.center[1] + self.radius * s,
            self.z,
        ])
        # yaw = theta puts base_link +X radially outward, so the front
        # (-X, the camera side) faces the center at every t.
        lin = self.radius * self.omega * np.array([-s, c, 0.0])
        ang = np.array([0.0, 0.0, self.omega])
        return pos, quat_from_yaw(theta), lin, ang


def hold(xyz: tuple[float, float, float] = (0.0, 0.0, 0.0), yaw: float = 0.0) -> Hold:
    """Build a constant-pose trajectory."""
    return Hold(xyz=tuple(xyz), yaw=float(yaw))


def line(
    start: tuple[float, float, float],
    velocity: tuple[float, float, float],
    yaw: float = 0.0,
) -> Line:
    """Build a constant-velocity straight-line trajectory."""
    return Line(start=tuple(start), velocity=tuple(velocity), yaw=float(yaw))


def orbit(
    center: tuple[float, float] = (0.0, 0.0),
    radius: float = 1.0,
    omega: float = 0.3,
    z: float = 0.0,
    theta0: float = 0.0,
) -> Orbit:
    """Build a face-the-center orbit; front = base_link -X points at center."""
    if radius <= 0.0:
        raise ValueError(f"orbit radius must be > 0, got {radius}")
    return Orbit(center=tuple(center), radius=float(radius), omega=float(omega),
                 z=float(z), theta0=float(theta0))
