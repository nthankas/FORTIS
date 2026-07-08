"""
Tests for the pure RGBD VO core (fortis_perception.rgbd_vo).

Rendered-frame tests drive the core with fortis_sim_support's synthetic
renderer; importorskip lets them skip -- not fail -- when this package
is tested without sim_support built. The blank-frame test needs no
renderer and always runs.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from fortis_perception.rgbd_vo import RgbdVo

#: 320x200 test intrinsics (~70 deg horizontal FOV): quarter-area of the
#: real 640x400 OAK streams with K halved to match, for render speed.
WIDTH, HEIGHT = 320, 200
K = np.array([
    [230.0, 0.0, 160.0],
    [0.0, 230.0, 100.0],
    [0.0, 0.0, 1.0],
])

#: Camera vantage with good scene coverage: ~1.5 m back from the props
#: clustered near the world origin, half a metre up, looking slightly down.
EYE = np.array([-1.5, 0.0, 0.5])
TARGET = np.array([0.0, 0.0, 0.25])


def _renderer():
    """Import the sim_support renderer, skipping when it is not built."""
    raycaster = pytest.importorskip("fortis_sim_support.raycaster")
    scenes = pytest.importorskip("fortis_sim_support.synthetic_scene")
    return raycaster, scenes


def _look_at(eye, target):
    """Build T_world_cam (optical: X right, Y down, Z forward) toward target."""
    eye = np.asarray(eye, dtype=float)
    forward = np.asarray(target, dtype=float) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    t = np.eye(4)
    t[:3, 0] = right
    t[:3, 1] = down
    t[:3, 2] = forward
    t[:3, 3] = eye
    return t


def _render(raycaster, scene, t_world_cam):
    """Render one RGBD pair at the test resolution."""
    rgb, depth_mm = raycaster.render(scene, K, t_world_cam, WIDTH, HEIGHT)
    return np.asarray(rgb, dtype=np.uint8), np.asarray(depth_mm, dtype=np.uint16)


def test_two_frame_translation_recovered():
    """Recover a pure 5 cm sideways step between two rendered frames."""
    raycaster, scenes = _renderer()
    scene = scenes.scene_baseline()
    t0 = _look_at(EYE, TARGET)
    t1 = t0.copy()
    t1[:3, 3] += 0.05 * t0[:3, 0]  # 5 cm along camera X (right)

    vo = RgbdVo(K=K)
    rgb0, depth0 = _render(raycaster, scene, t0)
    first = vo.process(rgb0, depth0)
    assert first.tracking, "bootstrap frame must adopt a keyframe"

    rgb1, depth1 = _render(raycaster, scene, t1)
    result = vo.process(rgb1, depth1)
    assert result.tracking is True
    assert result.n_inliers >= vo.min_inliers

    translation = result.T_delta[:3, 3]
    # Ground truth keyframe->current motion is (+0.05, 0, 0) in the
    # keyframe camera frame; the signed X check also pins the solvePnP
    # inversion convention.
    assert abs(np.linalg.norm(translation) - 0.05) < 0.005
    assert translation[0] == pytest.approx(0.05, abs=0.005)
    cos_angle = (np.trace(result.T_delta[:3, :3]) - 1.0) / 2.0
    angle = math.acos(min(1.0, max(-1.0, float(cos_angle))))
    assert angle < 0.01


def test_blank_frames_report_tracking_lost():
    """Report tracking=False on featureless frames instead of crashing."""
    vo = RgbdVo(K=K)
    result = None
    for _ in range(3):
        result = vo.process(
            np.zeros((HEIGHT, WIDTH), dtype=np.uint8),
            np.zeros((HEIGHT, WIDTH), dtype=np.uint16),
        )
    assert result.tracking is False
    assert result.T_delta is None
    assert result.n_inliers == 0
    assert np.allclose(vo.pose, np.eye(4))


def test_keyframe_refresh_integrates_long_translation():
    """Integrate ten 2 cm steps to ~0.18 m via keyframe-anchored composition."""
    raycaster, scenes = _renderer()
    scene = scenes.scene_baseline()
    t0 = _look_at(EYE, TARGET)

    vo = RgbdVo(K=K)
    for i in range(10):
        t_i = t0.copy()
        t_i[:3, 3] += 0.02 * i * t0[:3, 0]
        rgb, depth = _render(raycaster, scene, t_i)
        result = vo.process(rgb, depth)
        assert result.tracking, f"tracking lost at frame {i}"

    travelled = float(np.linalg.norm(vo.pose[:3, 3]))
    assert abs(travelled - 0.18) < 0.018  # within 10%
    # 0.18 m total > keyframe_trans_m (0.12): the refresh policy must have
    # fired at least once beyond the bootstrap keyframe.
    assert vo.keyframe_count >= 2

    vo.reset()
    assert np.allclose(vo.pose, np.eye(4))
    assert vo.keyframe_count == 0
