"""
Procedural 3D test scene for the synthetic OAK replayer.

Pure numpy -- no ROS imports. A Scene is a small set of analytic
primitives (the z = 0 ground plane, axis-aligned boxes, spheres), each
carrying a Material evaluated analytically at world-space hit points:
a base RGB, a world-space checkerboard modulation, and a
high-frequency deterministic per-primitive RNG speckle. The speckle is
load-bearing: it gives every surface ORB-trackable features, which the
visual-odometry workstream depends on.

scene_baseline() and scene_modified() differ by exactly one box added
(ADDED_BOX) and one box removed (REMOVED_BOX), both exported as module
constants so reconstruction-diff tests can assert exact volumes.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import numpy as np

#: Lattice points per axis of the cached 3D speckle noise tile.
_SPECKLE_TILE_N: int = 64


@dataclass(frozen=True)
class Material:
    """Analytic surface appearance: base color, checker, RNG speckle.

    rgb is the base albedo in [0, 1]. checker_scale is the world-space
    checker cell size in meters (<= 0 disables the checker); dark cells
    are dimmed by checker_dim. speckle_amp modulates intensity by up to
    +/- amp with a deterministic noise tile seeded by `seed`, sampled
    at speckle_scale cells per meter.
    """

    rgb: tuple[float, float, float]
    checker_scale: float = 0.25
    checker_dim: float = 0.7
    speckle_amp: float = 0.35
    speckle_scale: float = 150.0
    seed: int = 0


@dataclass(frozen=True)
class GroundPlane:
    """The infinite z = 0 ground plane."""

    material: Material


@dataclass(frozen=True)
class Box:
    """Axis-aligned box given by center and full size, both in meters."""

    center: tuple[float, float, float]
    size: tuple[float, float, float]
    material: Material

    def volume(self) -> float:
        """Return the box volume in cubic meters."""
        sx, sy, sz = self.size
        return sx * sy * sz


@dataclass(frozen=True)
class Sphere:
    """Sphere given by center (m) and radius (m)."""

    center: tuple[float, float, float]
    radius: float
    material: Material


@dataclass(frozen=True)
class Scene:
    """A renderable set of primitives; ground may be None in unit tests."""

    ground: GroundPlane | None
    boxes: tuple[Box, ...]
    spheres: tuple[Sphere, ...]


@lru_cache(maxsize=32)
def _speckle_tile(seed: int) -> np.ndarray:
    """Return the cached [-0.5, 0.5) 3D noise tile for one material seed."""
    rng = np.random.default_rng(seed)
    return rng.random((_SPECKLE_TILE_N,) * 3, dtype=np.float32) - 0.5


def _speckle_octave(
    tile: np.ndarray,
    points: np.ndarray,
    scale: float,
    footprint_m: np.ndarray | None,
) -> np.ndarray:
    """Sample one zero-mean speckle octave, faded where a pixel spans cells.

    Point-sampled noise ALIASES once the per-pixel surface footprint
    exceeds the noise cell: the same surface renders differently from
    each viewpoint, which actively corrupts feature matching. Fading the
    octave out as footprint/cell grows (a poor man's mip-map) keeps the
    texture view-stable at every range, like a real slightly-soft camera.
    """
    idx = np.floor(points * scale).astype(np.int64) % _SPECKLE_TILE_N
    noise = tile[idx[:, 0], idx[:, 1], idx[:, 2]].astype(np.float64)
    if footprint_m is None:
        return noise
    cell = 1.0 / scale
    return noise / (1.0 + (footprint_m / cell) ** 2)


def shade(
    material: Material,
    points: np.ndarray,
    footprint_m: np.ndarray | None = None,
) -> np.ndarray:
    """Evaluate the material color at world-space points.

    points is (N, 3); returns (N, 3) RGB in [0, 1]. Both the checker
    and the speckle are functions of world position only, so the same
    surface point always renders the same color from any camera pose --
    exactly what feature tracking across frames needs. footprint_m (N,)
    is the surface size one pixel covers at each hit; when given, each
    speckle octave fades out as it would alias (see _speckle_octave).
    Two octaves (fine + 8x coarser) keep surfaces ORB-trackable both up
    close and across the room.
    """
    base = np.asarray(material.rgb, dtype=np.float64)
    factor = np.ones(points.shape[0], dtype=np.float64)
    if material.checker_scale > 0.0:
        cells = np.floor(points / material.checker_scale).astype(np.int64)
        dark = (cells.sum(axis=1) % 2) == 0
        factor[dark] *= material.checker_dim
    if material.speckle_amp > 0.0:
        tile = _speckle_tile(material.seed)
        noise = _speckle_octave(tile, points, material.speckle_scale, footprint_m)
        noise += 0.6 * _speckle_octave(
            tile, points, material.speckle_scale / 8.0, footprint_m
        )
        factor *= 1.0 + 2.0 * material.speckle_amp * noise
    return np.clip(factor[:, None] * base[None, :], 0.0, 1.0)


# --- Scene content ------------------------------------------------------------
#
# Layout is sized for the replayer's default orbit (radius 1 m around the
# world origin): the front camera sits 0.215 m up, pitched 30 deg UP, so at
# ~0.8 m horizontal from the center it sees the height band ~0.31-1.29 m.
# The pillar/spheres live in that band; the perimeter walls keep speckled
# texture in view over the whole 360 deg orbit (no featureless sky frames).

_GROUND = GroundPlane(Material(
    rgb=(0.52, 0.52, 0.52), checker_scale=0.25, checker_dim=0.72,
    speckle_amp=0.30, seed=11,
))

_CENTER_PILLAR = Box(
    center=(0.0, 0.0, 0.6), size=(0.3, 0.3, 1.2),
    material=Material(rgb=(0.60, 0.52, 0.34), checker_scale=0.15, seed=21),
)

#: Box present in the BASELINE scene but absent from the modified one.
REMOVED_BOX = Box(
    center=(-0.55, 0.40, 0.50), size=(0.35, 0.35, 0.35),
    material=Material(rgb=(0.20, 0.65, 0.25), checker_scale=0.12, seed=22),
)

#: Box present ONLY in the modified scene: a 0.4 m cube at a known free spot.
ADDED_BOX = Box(
    center=(0.55, -0.50, 0.55), size=(0.4, 0.4, 0.4),
    material=Material(rgb=(0.90, 0.55, 0.10), checker_scale=0.12, seed=23),
)

#: Exact volumes for reconstruction-diff assertions (m^3).
REMOVED_BOX_VOLUME_M3: float = REMOVED_BOX.volume()
ADDED_BOX_VOLUME_M3: float = ADDED_BOX.volume()

#: Red sphere: the object-recognition workstream's HSV blob target. Low
#: speckle keeps its hue clean for the color detector.
_RED_SPHERE = Sphere(
    center=(0.45, 0.35, 0.75), radius=0.15,
    material=Material(rgb=(0.85, 0.10, 0.10), checker_scale=0.0,
                      speckle_amp=0.18, seed=31),
)
_BLUE_SPHERE = Sphere(
    center=(-0.40, -0.45, 0.50), radius=0.12,
    material=Material(rgb=(0.15, 0.25, 0.80), checker_scale=0.0,
                      speckle_amp=0.25, seed=32),
)

_WALLS = (
    Box(center=(3.5, 0.0, 1.5), size=(0.2, 7.2, 3.0),
        material=Material(rgb=(0.66, 0.60, 0.50), checker_scale=0.5, seed=41)),
    Box(center=(-3.5, 0.0, 1.5), size=(0.2, 7.2, 3.0),
        material=Material(rgb=(0.62, 0.58, 0.52), checker_scale=0.5, seed=42)),
    Box(center=(0.0, 3.5, 1.5), size=(7.2, 0.2, 3.0),
        material=Material(rgb=(0.60, 0.62, 0.55), checker_scale=0.5, seed=43)),
    Box(center=(0.0, -3.5, 1.5), size=(7.2, 0.2, 3.0),
        material=Material(rgb=(0.64, 0.57, 0.48), checker_scale=0.5, seed=44)),
)

_BASELINE_BOXES = (_CENTER_PILLAR, REMOVED_BOX) + _WALLS


def scene_baseline() -> Scene:
    """Build the baseline scene: ground, pillar, REMOVED_BOX, walls, two spheres."""
    return Scene(ground=_GROUND, boxes=_BASELINE_BOXES,
                 spheres=(_RED_SPHERE, _BLUE_SPHERE))


def scene_modified() -> Scene:
    """Build the modified scene: baseline plus ADDED_BOX, minus REMOVED_BOX."""
    boxes = tuple(b for b in _BASELINE_BOXES if b is not REMOVED_BOX) + (ADDED_BOX,)
    return Scene(ground=_GROUND, boxes=boxes, spheres=(_RED_SPHERE, _BLUE_SPHERE))
