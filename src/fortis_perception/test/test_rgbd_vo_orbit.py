"""
Orbit-trajectory acceptance test for the RGBD VO core.

Drives RgbdVo (pure, no ROS) with frames rendered along an exact orbit
from fortis_sim_support and bounds the FINAL absolute trajectory error
against the analytic ground truth: < 5% of path length (the sprint
acceptance criterion). importorskip lets the test skip -- not fail --
when this package is tested without sim_support built.
"""

from __future__ import annotations

import numpy as np
import pytest

from fortis_perception.geometry import T_BASE_CAM
from fortis_perception.rgbd_vo import RgbdVo

# Radius clears the scene's object belt (props reach ~0.75 m from center;
# the front camera rides ~0.3 m inside the base circle) so the camera
# never skims through a primitive mid-orbit.
RADIUS_M = 1.6
OMEGA_RAD_S = 0.3
DURATION_S = 8.0
RATE_HZ = 10.0

#: 320x200 render (quarter-area of the real streams, K scaled to match)
#: keeps the 41-frame sweep well under the 60 s budget.
WIDTH, HEIGHT = 320, 200
K = np.array([
    [230.0, 0.0, 160.0],
    [0.0, 230.0, 100.0],
    [0.0, 0.0, 1.0],
])


def _sim_modules():
    """Import the sim_support renderer + trajectory API, skipping when absent."""
    raycaster = pytest.importorskip("fortis_sim_support.raycaster")
    scenes = pytest.importorskip("fortis_sim_support.synthetic_scene")
    trajectory = pytest.importorskip("fortis_sim_support.trajectory")
    return raycaster, scenes, trajectory


def _pose_from_sample(sample):
    """Build the 4x4 world->base pose from a (xyz, quat_xyzw, ...) sample."""
    xyz = np.asarray(sample[0], dtype=float)
    qx, qy, qz, qw = np.asarray(sample[1], dtype=float)
    pose = np.eye(4)
    pose[:3, :3] = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    pose[:3, 3] = xyz
    return pose


def test_orbit_final_ate_under_five_percent_of_path():
    """Bound the final trajectory error to < 5% of path length on an orbit."""
    raycaster, scenes, trajectory = _sim_modules()
    scene = scenes.scene_baseline()
    traj = trajectory.orbit(radius=RADIUS_M, omega=OMEGA_RAD_S)

    vo = RgbdVo(K=K)
    t_cam_base = np.linalg.inv(T_BASE_CAM)
    times = np.arange(0.0, DURATION_S + 1e-9, 1.0 / RATE_HZ)

    gt_first = None
    gt_last = None
    lost = 0
    for t in times:
        gt_last = _pose_from_sample(traj.sample(float(t)))
        if gt_first is None:
            gt_first = gt_last
        t_world_cam = gt_last @ T_BASE_CAM
        rgb, depth_mm = raycaster.render(scene, K, t_world_cam, WIDTH, HEIGHT)
        result = vo.process(
            np.asarray(rgb, dtype=np.uint8),
            np.asarray(depth_mm, dtype=np.uint16),
        )
        if not result.tracking:
            lost += 1

    assert lost <= len(times) // 5, f"tracking lost on {lost}/{len(times)} frames"

    # Both trajectories expressed relative to the starting base pose: the
    # ground truth via the first sample, the estimate via the camera->base
    # similarity around the integrated camera pose.
    gt_rel = np.linalg.inv(gt_first) @ gt_last
    est_rel = T_BASE_CAM @ vo.pose @ t_cam_base

    path_length = RADIUS_M * OMEGA_RAD_S * DURATION_S
    final_error = float(np.linalg.norm(est_rel[:3, 3] - gt_rel[:3, 3]))
    assert final_error < 0.05 * path_length, (
        f"final ATE {final_error:.3f} m exceeds 5% of {path_length:.2f} m path"
    )
