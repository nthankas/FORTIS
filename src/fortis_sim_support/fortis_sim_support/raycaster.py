"""
Vectorized pinhole raycaster over synthetic_scene primitives.

Pure numpy, no ROS imports. render() casts one ray per pixel from a
pinhole camera in OPTICAL convention (+Z forward, +X right, +Y down)
and returns an (rgb uint8, depth uint16 mm) pair matching the real
OAK's aligned-depth output: depth is the PLANAR Z distance in the
camera frame (not euclidean ray length), and 0 means "no return"
(miss, or hit beyond max_range).

The per-(K, width, height) ray-direction grid is precomputed and
lru_cached, so repeated renders at a fixed resolution pay only the
intersection cost -- a 640x400 frame stays inside a 15 fps budget.
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np

from fortis_sim_support.synthetic_scene import Box, Scene, Sphere, shade

#: Minimum accepted ray parameter; rejects self-hits at the camera origin.
_T_EPS: float = 1e-6

#: Direction components below this are clamped instead of special-casing
#: division by zero in the slab test; the induced error is negligible.
_DIR_MIN: float = 1e-12

#: RGB for pixels with no primitive hit (a featureless dark "sky").
_BACKGROUND_RGB = np.array([18, 22, 30], dtype=np.uint8)

#: Default planar-depth cutoff (m); beyond it depth renders 0, like the sensor.
DEFAULT_MAX_RANGE_M: float = 8.0


@lru_cache(maxsize=8)
def _pixel_rays(k_bytes: bytes, width: int, height: int) -> np.ndarray:
    """Build the (H*W, 3) camera-frame ray grid for one intrinsic matrix.

    Rays are left UNNORMALIZED with z = 1, so the hit parameter t along
    a ray IS the planar camera-Z depth -- no per-pixel rescaling later.
    """
    k = np.frombuffer(k_bytes, dtype=np.float64).reshape(3, 3)
    x = (np.arange(width, dtype=np.float64) - k[0, 2]) / k[0, 0]
    y = (np.arange(height, dtype=np.float64) - k[1, 2]) / k[1, 1]
    xx, yy = np.meshgrid(x, y)
    return np.stack([xx, yy, np.ones_like(xx)], axis=-1).reshape(-1, 3)


def _plane_t(origin: np.ndarray, dirs: np.ndarray) -> np.ndarray:
    """Intersect all rays with the z = 0 plane; inf where there is no hit."""
    dz = np.where(np.abs(dirs[:, 2]) < _DIR_MIN, _DIR_MIN, dirs[:, 2])
    t = -origin[2] / dz
    return np.where(t > _T_EPS, t, np.inf)


def _box_t(origin: np.ndarray, dirs: np.ndarray, box: Box) -> np.ndarray:
    """Intersect all rays with an AABB via the slab method; inf on miss."""
    half = np.asarray(box.size, dtype=np.float64) * 0.5
    center = np.asarray(box.center, dtype=np.float64)
    d = np.where(np.abs(dirs) < _DIR_MIN, _DIR_MIN, dirs)
    t1 = (center - half - origin) / d
    t2 = (center + half - origin) / d
    tnear = np.minimum(t1, t2).max(axis=1)
    tfar = np.maximum(t1, t2).min(axis=1)
    hit = (tnear <= tfar) & (tnear > _T_EPS)
    return np.where(hit, tnear, np.inf)


def _sphere_t(origin: np.ndarray, dirs: np.ndarray, sphere: Sphere) -> np.ndarray:
    """Intersect all rays with a sphere (nearest front hit); inf on miss."""
    oc = origin - np.asarray(sphere.center, dtype=np.float64)
    a = np.einsum("ij,ij->i", dirs, dirs)
    b = 2.0 * (dirs @ oc)
    c = oc @ oc - sphere.radius ** 2
    disc = b * b - 4.0 * a * c
    hit = disc >= 0.0
    t = (-b - np.sqrt(np.where(hit, disc, 0.0))) / (2.0 * a)
    return np.where(hit & (t > _T_EPS), t, np.inf)


def render(
    scene: Scene,
    k: np.ndarray,
    t_world_cam: np.ndarray,
    width: int,
    height: int,
    max_range: float = DEFAULT_MAX_RANGE_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Render the scene through a pinhole camera.

    Parameters
    ----------
    scene : Scene
        Primitives + materials from synthetic_scene.
    k : (3, 3) array
        Pinhole intrinsics.
    t_world_cam : (4, 4) array
        Camera pose in the world: maps OPTICAL-frame points (+Z forward,
        +X right, +Y down) to world points.
    width, height : int
        Output resolution in pixels.
    max_range : float
        Planar depth beyond this reports 0 ("no return"); RGB keeps the
        shaded color, like a real camera whose depth range is exceeded.

    Returns
    -------
    (rgb, depth_mm) : (H, W, 3) uint8 and (H, W) uint16
        depth_mm is planar camera-Z depth in millimeters, 0 = no return.
    """
    k = np.asarray(k, dtype=np.float64).reshape(3, 3)
    t_world_cam = np.asarray(t_world_cam, dtype=np.float64).reshape(4, 4)
    dirs_cam = _pixel_rays(np.round(k, 9).tobytes(), int(width), int(height))
    rot = t_world_cam[:3, :3]
    origin = t_world_cam[:3, 3]
    # Pure rotation keeps each ray's camera-Z component at 1, so the hit
    # parameter t below remains the planar depth.
    dirs = dirs_cam @ rot.T

    prims: list[tuple[str, object]] = []
    if scene.ground is not None:
        prims.append(("plane", scene.ground))
    prims.extend(("box", b) for b in scene.boxes)
    prims.extend(("sphere", s) for s in scene.spheres)

    n = dirs.shape[0]
    best_t = np.full(n, np.inf)
    hit_prim = np.full(n, -1, dtype=np.int32)
    for i, (kind, prim) in enumerate(prims):
        if kind == "plane":
            t = _plane_t(origin, dirs)
        elif kind == "box":
            t = _box_t(origin, dirs, prim)
        else:
            t = _sphere_t(origin, dirs, prim)
        closer = t < best_t
        best_t[closer] = t[closer]
        hit_prim[closer] = i

    rgb = np.empty((n, 3), dtype=np.uint8)
    rgb[:] = _BACKGROUND_RGB
    # Surface size one pixel covers at the hit: (euclidean range) / fx.
    # shade() uses it to fade speckle octaves that would alias.
    ray_norm = np.linalg.norm(dirs_cam, axis=1)
    fx = float(k[0, 0])
    for i, (_, prim) in enumerate(prims):
        mask = hit_prim == i
        if not mask.any():
            continue
        pts = origin + best_t[mask, None] * dirs[mask]
        footprint = best_t[mask] * ray_norm[mask] / fx
        rgb[mask] = (shade(prim.material, pts, footprint) * 255.0).astype(np.uint8)

    in_range = np.isfinite(best_t) & (best_t <= max_range)
    depth = np.where(in_range, np.clip(best_t * 1000.0, 0.0, 65535.0), 0.0)
    return (
        rgb.reshape(int(height), int(width), 3),
        depth.reshape(int(height), int(width)).astype(np.uint16),
    )
