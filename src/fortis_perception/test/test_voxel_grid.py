"""
Tests for fortis_perception.voxel_grid.

Pure numpy: synthetic box-face and cube-shell point sets with known
voxel footprints, integrated at controlled hit counts, so occupancy,
persistence, and diff behaviour are all asserted against exact or
tightly banded expected values. No rclpy anywhere.
"""

import numpy as np
import pytest

from fortis_perception.voxel_grid import VoxelGrid

VOXEL_SIZE = 0.05
#: In-cell sampling offset: keeps every sample strictly inside a voxel
#: layer so a face's voxel footprint is exact, not boundary-dependent.
INSET = 0.0025
STEP = 0.01

#: Voxels covered by one 0.4 m face at 0.05 m resolution (8 x 8 layer).
FACE_VOXELS = 64
#: Full voxel shell of a 0.4 m cube at 0.05 m resolution: 8^3 - 6^3.
SHELL_VOXELS = 8 ** 3 - 6 ** 3


def _face_points(axis, value, lo=0.0, hi=0.4):
    """Sample one axis-aligned square face at STEP spacing."""
    a = np.arange(lo + INSET, hi, STEP)
    g1, g2 = np.meshgrid(a, a, indexing="ij")
    pts = np.empty((g1.size, 3))
    others = [i for i in range(3) if i != axis]
    pts[:, axis] = value
    pts[:, others[0]] = g1.ravel()
    pts[:, others[1]] = g2.ravel()
    return pts


def _two_face_box():
    """Return points on the bottom and top faces of the origin test box."""
    return np.vstack([
        _face_points(2, INSET),          # z layer 0
        _face_points(2, 0.4 - INSET),    # z layer 7
    ])


def _cube_shell(lo=1.0, size=0.4):
    """Sample all six faces of an axis-aligned cube (a full voxel shell)."""
    faces = []
    for axis in range(3):
        for value in (lo + INSET, lo + size - INSET):
            faces.append(_face_points(axis, value, lo=lo, hi=lo + size))
    return np.vstack(faces)


def _colors(n, rgb=(200, 120, 40)):
    """Return n copies of one RGB colour."""
    return np.tile(np.array(rgb, dtype=np.uint8), (n, 1))


def _integrate(grid, points, times=1, rgb=(200, 120, 40)):
    """Integrate the same point set a fixed number of times."""
    for _ in range(times):
        grid.integrate(points, _colors(points.shape[0], rgb))


def test_box_faces_occupied_count_in_band():
    """Two 0.4 m box faces occupy two 8x8 voxel layers."""
    grid = VoxelGrid(VOXEL_SIZE)
    _integrate(grid, _two_face_box(), times=3)
    n = grid.occupied(min_hits=3).shape[0]
    assert 0.8 * 2 * FACE_VOXELS <= n <= 1.2 * 2 * FACE_VOXELS
    # sampling is strictly cell-interior, so the count is in fact exact
    assert n == 2 * FACE_VOXELS


def test_min_hits_filters_single_hit_noise():
    """Voxels hit once (noise) fall below min_hits=3 and disappear."""
    grid = VoxelGrid(VOXEL_SIZE)
    _integrate(grid, _two_face_box(), times=3)
    # 20 deterministic single-hit noise points on a coarse lattice at
    # x >= 10 m (voxel index >= 200), one voxel each.
    ks = np.arange(20, dtype=np.float64)
    noise = np.column_stack(
        (10.025 + 0.1 * ks, np.full(20, 10.025), np.full(20, 10.025)))
    grid.integrate(noise, _colors(20))
    occ3 = grid.occupied(min_hits=3)
    assert occ3.shape[0] == 2 * FACE_VOXELS, "noise must not pass min_hits=3"
    assert np.all(occ3 < 200), "no noise-band voxel may survive the filter"
    assert grid.occupied(min_hits=1).shape[0] == 2 * FACE_VOXELS + 20


def test_save_load_round_trip_exact(tmp_path):
    """A saved grid reloads with identical voxels, counts, and colours."""
    grid = VoxelGrid(VOXEL_SIZE)
    _integrate(grid, _two_face_box(), times=3)
    _integrate(grid, _cube_shell(), times=1, rgb=(10, 220, 90))
    path = grid.save(str(tmp_path / "map"))
    assert path.endswith(".npz")
    loaded = VoxelGrid.load(path)
    assert loaded.voxel_size == grid.voxel_size
    assert len(loaded) == len(grid)
    for min_hits in (1, 3):
        centers, colors = grid.to_points(min_hits)
        loaded_centers, loaded_colors = loaded.to_points(min_hits)
        assert np.array_equal(centers, loaded_centers)
        assert np.array_equal(colors, loaded_colors)


def test_diff_identical_maps_is_empty():
    """Diffing a map against itself reports zero added and zero removed."""
    grid = VoxelGrid(VOXEL_SIZE)
    _integrate(grid, _two_face_box(), times=3)
    added, removed = grid.diff(grid, min_hits=3)
    assert added.shape[0] == 0
    assert removed.shape[0] == 0


def test_diff_reports_added_cube():
    """A cube shell present only in the live map shows up as added voxels."""
    reference = VoxelGrid(VOXEL_SIZE)
    _integrate(reference, _two_face_box(), times=3)
    live = VoxelGrid(VOXEL_SIZE)
    _integrate(live, _two_face_box(), times=3)
    _integrate(live, _cube_shell(), times=3)
    added, removed = live.diff(reference, min_hits=3)
    assert removed.shape[0] == 0
    assert abs(added.shape[0] - SHELL_VOXELS) <= 0.2 * SHELL_VOXELS


def test_diff_reports_removed_cube():
    """A cube shell present only in the reference shows up as removed voxels."""
    reference = VoxelGrid(VOXEL_SIZE)
    _integrate(reference, _two_face_box(), times=3)
    _integrate(reference, _cube_shell(), times=3)
    live = VoxelGrid(VOXEL_SIZE)
    _integrate(live, _two_face_box(), times=3)
    added, removed = live.diff(reference, min_hits=3)
    assert added.shape[0] == 0
    assert abs(removed.shape[0] - SHELL_VOXELS) <= 0.2 * SHELL_VOXELS


def test_diff_voxel_size_mismatch_raises():
    """Grids at different resolutions must refuse to diff."""
    with pytest.raises(ValueError):
        VoxelGrid(0.05).diff(VoxelGrid(0.1))
