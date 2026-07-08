"""Exact-geometry unit tests for the pure-numpy raycaster (no rclpy)."""

from __future__ import annotations

import numpy as np

from fortis_sim_support.raycaster import render
from fortis_sim_support.synthetic_scene import (
    Box,
    GroundPlane,
    Material,
    Scene,
    Sphere,
)

WIDTH, HEIGHT = 160, 120
FOCAL = 300.0
CX, CY = WIDTH // 2, HEIGHT // 2

#: Speckle-only material (checker disabled) so texture assertions isolate
#: the RNG speckle.
_MAT = Material(rgb=(0.6, 0.5, 0.4), checker_scale=0.0, seed=7)


def _k() -> np.ndarray:
    return np.array([
        [FOCAL, 0.0, WIDTH / 2.0],
        [0.0, FOCAL, HEIGHT / 2.0],
        [0.0, 0.0, 1.0],
    ])


def _scene(boxes=(), spheres=(), ground=None) -> Scene:
    return Scene(ground=ground, boxes=tuple(boxes), spheres=tuple(spheres))


def _level_camera_pose(height_m: float) -> np.ndarray:
    """Build a camera at (0, 0, height) looking along world +X, image-up = +Z."""
    t = np.eye(4)
    t[:3, 0] = (0.0, -1.0, 0.0)  # optical X (right) = world -Y
    t[:3, 1] = (0.0, 0.0, -1.0)  # optical Y (down)  = world -Z
    t[:3, 2] = (1.0, 0.0, 0.0)   # optical Z (fwd)   = world +X
    t[2, 3] = height_m
    return t


def test_box_face_depth_at_center_pixel():
    """A box face square-on at 2 m reads exactly 2000 mm at the center pixel."""
    box = Box(center=(0.0, 0.0, 2.25), size=(0.5, 0.5, 0.5), material=_MAT)
    _, depth = render(_scene(boxes=[box]), _k(), np.eye(4), WIDTH, HEIGHT)
    assert abs(int(depth[CY, CX]) - 2000) <= 2


def test_sphere_depth_at_center_pixel():
    """A sphere on the optical axis reads center depth minus radius."""
    sphere = Sphere(center=(0.0, 0.0, 3.0), radius=0.5, material=_MAT)
    _, depth = render(_scene(spheres=[sphere]), _k(), np.eye(4), WIDTH, HEIGHT)
    assert abs(int(depth[CY, CX]) - 2500) <= 2


def test_ground_plane_depth_profile():
    """Ground depth is exact and monotonic down the image; sky and far rows are 0.

    For a level camera above the plane, planar depth at row v is
    z = h * fy / (v - cy): steeper rays (larger v) hit the ground CLOSER,
    so depth strictly decreases toward the image bottom. Rows just below
    the horizon exceed max_range and must read 0 (no return), as must
    everything above the horizon.
    """
    scene = _scene(ground=GroundPlane(_MAT))
    _, depth = render(scene, _k(), _level_camera_pose(1.0), WIDTH, HEIGHT,
                      max_range=8.0)
    col = depth[:, CX].astype(int)

    rows = np.arange(CY + 40, HEIGHT)  # far enough below the horizon to be in range
    vals = col[rows]
    assert (vals > 0).all()
    assert (np.diff(vals) < 0).all(), "steeper rays must hit the ground closer"
    assert abs(vals[0] - 7500) <= 2  # exact: 1.0 m * 300 px / 40 rows

    assert col[CY - 30] == 0  # above the horizon: no return
    assert col[CY + 5] == 0   # below the horizon but beyond max_range: no return


def test_empty_region_depth_is_zero():
    """Pixels whose rays miss every primitive report depth 0."""
    box = Box(center=(0.0, 0.0, 2.25), size=(0.5, 0.5, 0.5), material=_MAT)
    _, depth = render(_scene(boxes=[box]), _k(), np.eye(4), WIDTH, HEIGHT)
    assert int(depth[0, 0]) == 0
    assert int(depth[HEIGHT - 1, WIDTH - 1]) == 0


def test_speckle_texture_is_nonuniform():
    """The RNG speckle produces real intensity variance inside one surface."""
    box = Box(center=(0.0, 0.0, 2.5), size=(2.0, 2.0, 1.0), material=_MAT)
    rgb, depth = render(_scene(boxes=[box]), _k(), np.eye(4), WIDTH, HEIGHT)
    patch_depth = depth[CY - 15:CY + 15, CX - 15:CX + 15]
    assert (patch_depth > 0).all(), "patch must lie fully on the box face"
    gray = rgb[CY - 15:CY + 15, CX - 15:CX + 15].astype(np.float64).mean(axis=2)
    assert gray.std() > 3.0, "speckle must leave ORB-trackable texture"
