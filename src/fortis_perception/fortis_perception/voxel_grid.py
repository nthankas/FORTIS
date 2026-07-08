"""
Pure voxel-grid logic for the FORTIS perception map.

Holds the sparse occupancy voxel grid used by voxel_map_node and
map_diff_node: point integration with per-voxel hit counts and
first-seen colour, occupancy thresholding, PointCloud2-ready export,
.npz persistence, and cross-run set diffs. Named voxel_grid (not
voxel_map) so the module does not shadow the voxel_map node name. Kept
free of ROS imports so it is unit-testable in isolation.
"""

import numpy as np

#: Format tag written into every saved map so load() can reject foreign files.
_FORMAT_METADATA = "fortis_perception.voxel_grid v1"


def _index_array(keys):
    """Convert an iterable of (ix, iy, iz) keys into a sorted Nx3 int32 array."""
    keys = sorted(keys)
    if not keys:
        return np.empty((0, 3), dtype=np.int32)
    return np.array(keys, dtype=np.int32)


class VoxelGrid:
    """Sparse voxel occupancy grid with hit counts and first-seen colour."""

    def __init__(self, voxel_size=0.05):
        if voxel_size <= 0.0:
            raise ValueError(f"voxel_size must be positive, got {voxel_size}")
        self.voxel_size = float(voxel_size)
        #: (ix, iy, iz) -> [hit_count, (r, g, b)]
        self._cells = {}

    def __len__(self):
        """Return the number of voxels hit at least once."""
        return len(self._cells)

    def integrate(self, points_xyz, colors_rgb):
        """Accumulate hits (and first-seen colour) for each point's voxel."""
        points = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
        colors = np.asarray(colors_rgb, dtype=np.uint8).reshape(-1, 3)
        if points.shape[0] != colors.shape[0]:
            raise ValueError("points and colors must have matching lengths")
        if points.shape[0] == 0:
            return
        idx = np.ascontiguousarray(
            np.floor(points / self.voxel_size).astype(np.int32))
        keys = idx.view(np.dtype((np.void, idx.dtype.itemsize * 3))).ravel()
        # first-occurrence indices keep the colour choice deterministic
        _, first, counts = np.unique(keys, return_index=True, return_counts=True)
        cells = self._cells
        for i, n in zip(first.tolist(), counts.tolist()):
            key = (int(idx[i, 0]), int(idx[i, 1]), int(idx[i, 2]))
            cell = cells.get(key)
            if cell is None:
                cells[key] = [
                    n, (int(colors[i, 0]), int(colors[i, 1]), int(colors[i, 2]))]
            else:
                cell[0] += n

    def occupied(self, min_hits=3):
        """Return sorted Nx3 int32 indices of voxels with >= min_hits hits."""
        return _index_array(
            k for k, cell in self._cells.items() if cell[0] >= min_hits)

    def to_points(self, min_hits=3):
        """Return (centers Nx3 float64, colors Nx3 uint8) for occupied voxels."""
        items = sorted(
            (k, cell[1]) for k, cell in self._cells.items() if cell[0] >= min_hits)
        if not items:
            return (np.empty((0, 3), dtype=np.float64),
                    np.empty((0, 3), dtype=np.uint8))
        idx = np.array([k for k, _ in items], dtype=np.float64)
        colors = np.array([c for _, c in items], dtype=np.uint8)
        return (idx + 0.5) * self.voxel_size, colors

    def save(self, path):
        """Write the grid to a .npz file and return the resolved path."""
        if not path.endswith(".npz"):
            path += ".npz"
        items = sorted(self._cells.items())
        if items:
            indices = np.array([k for k, _ in items], dtype=np.int32)
            counts = np.array(
                [min(cell[0], 0xFFFF) for _, cell in items], dtype=np.uint16)
            colors = np.array([cell[1] for _, cell in items], dtype=np.uint8)
        else:
            indices = np.empty((0, 3), dtype=np.int32)
            counts = np.empty(0, dtype=np.uint16)
            colors = np.empty((0, 3), dtype=np.uint8)
        np.savez_compressed(
            path, indices=indices, counts=counts, colors=colors,
            voxel_size=np.float64(self.voxel_size), metadata=_FORMAT_METADATA)
        return path

    @classmethod
    def load(cls, path):
        """Read a grid previously written by save()."""
        with np.load(path, allow_pickle=False) as data:
            metadata = str(data["metadata"])
            if metadata != _FORMAT_METADATA:
                raise ValueError(f"{path}: unrecognized map format {metadata!r}")
            grid = cls(voxel_size=float(data["voxel_size"]))
            indices = data["indices"].astype(np.int32).reshape(-1, 3)
            counts = data["counts"]
            colors = data["colors"].astype(np.uint8).reshape(-1, 3)
        for i in range(indices.shape[0]):
            key = (int(indices[i, 0]), int(indices[i, 1]), int(indices[i, 2]))
            grid._cells[key] = [
                int(counts[i]),
                (int(colors[i, 0]), int(colors[i, 1]), int(colors[i, 2]))]
        return grid

    def diff(self, reference, min_hits=3):
        """Compare against a reference grid; return (added, removed) indices.

        added: voxels occupied here but not in the reference. removed:
        occupied in the reference but not here. Both are sorted Nx3 int32
        index arrays. Raises ValueError when the voxel sizes differ.
        """
        if abs(self.voxel_size - reference.voxel_size) > 1e-9:
            raise ValueError(
                f"voxel_size mismatch: {self.voxel_size} vs "
                f"{reference.voxel_size}")
        live = {k for k, cell in self._cells.items() if cell[0] >= min_hits}
        ref = {k for k, cell in reference._cells.items() if cell[0] >= min_hits}
        return _index_array(live - ref), _index_array(ref - live)
