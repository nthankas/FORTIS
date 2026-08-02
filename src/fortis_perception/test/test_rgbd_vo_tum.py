"""
TUM RGB-D benchmark for the pure RGBD VO core (fortis_perception.rgbd_vo).

Replays the public freiburg1_xyz handheld sequence (real Kinect RGB +
registered depth with motion-capture ground truth) straight through
RgbdVo -- no ROS graph -- integrates the keyframe-anchored poses into a
trajectory, rigidly aligns it to ground truth (Umeyama, no scale: RGBD
is metric), and gates the absolute trajectory error. The bound is a
loose anti-flake ceiling (10% of path length); the measured number is
printed for the README.

Skips unless the dataset is present under <repo>/.scratch/tum/ (see the
skipif reason for the download command), so CI without the dataset
stays green.

Measured 2026-08-02 (all 792 associated pairs, ~15 s): ATE RMSE
0.052 m over an 8.01 m ground-truth path (0.7% of path length), zero
tracking losses.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from fortis_perception.rgbd_vo import RgbdVo

#: <repo>/.scratch/ is untracked scratch space; the dataset never enters git.
DATASET = (
    Path(__file__).resolve().parents[3]
    / ".scratch" / "tum" / "rgbd_dataset_freiburg1_xyz"
)

#: Freiburg1 Kinect intrinsics (TUM calibration for the fr1 sequences).
K = np.array([
    [517.3, 0.0, 318.6],
    [0.0, 516.5, 255.3],
    [0.0, 0.0, 1.0],
])

#: TUM depth PNGs are uint16 with 5000 counts per metre; RgbdVo wants
#: millimetres, so one millimetre is 5 counts.
DEPTH_COUNTS_PER_MM = 5.0

#: Subsampling knob: full rate (1) runs in ~15 s and measures 0.7% ATE;
#: stride 2 halves that to ~11 s at 1.1% if this ever needs to be faster.
FRAME_STRIDE = 1

#: Standard TUM association tolerance between rgb and depth timestamps.
MAX_ASSOC_DIFF_S = 0.02


def _read_stamped_lines(path):
    """Parse a TUM index file into (timestamp, rest-of-line-fields) pairs."""
    entries = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        entries.append((float(fields[0]), fields[1:]))
    return entries


def _associate(times_a, times_b, max_diff=MAX_ASSOC_DIFF_S):
    """Greedy best-first timestamp matching (the standard associate.py logic)."""
    candidates = sorted(
        (abs(ta - tb), ta, tb)
        for ta in times_a
        for tb in times_b
        if abs(ta - tb) < max_diff
    )
    taken_a, taken_b, pairs = set(), set(), []
    for _, ta, tb in candidates:
        if ta not in taken_a and tb not in taken_b:
            taken_a.add(ta)
            taken_b.add(tb)
            pairs.append((ta, tb))
    return sorted(pairs)


def _nearest_groundtruth(gt_times, gt_positions, stamp):
    """Return the ground-truth position nearest in time to ``stamp``."""
    i = int(np.searchsorted(gt_times, stamp))
    i = min(max(i, 1), len(gt_times) - 1)
    if abs(gt_times[i - 1] - stamp) < abs(gt_times[i] - stamp):
        i -= 1
    return gt_positions[i]


def _umeyama_align(est, gt):
    """Rigidly align ``est`` onto ``gt`` (Umeyama, no scale) and return it."""
    mu_est = est.mean(axis=0)
    mu_gt = gt.mean(axis=0)
    cov = (gt - mu_gt).T @ (est - mu_est) / len(est)
    u, _, vt = np.linalg.svd(cov)
    s = np.eye(3)
    s[2, 2] = np.sign(np.linalg.det(u @ vt))
    rot = u @ s @ vt
    return est @ rot.T + (mu_gt - rot @ mu_est)


@pytest.mark.slow
@pytest.mark.skipif(
    not (DATASET / "rgb.txt").exists(),
    reason=(
        "TUM freiburg1_xyz not downloaded; fetch with: mkdir -p "
        "~/FORTIS/.scratch/tum && cd ~/FORTIS/.scratch/tum && wget "
        "https://cvg.cit.tum.de/rgbd/dataset/freiburg1/"
        "rgbd_dataset_freiburg1_xyz.tgz && tar xzf rgbd_dataset_freiburg1_xyz.tgz"
    ),
)
def test_freiburg1_xyz_ate_under_ten_percent_of_path():
    """Bound ATE RMSE on freiburg1_xyz to < 10% of the ground-truth path."""
    cv2 = pytest.importorskip("cv2")

    rgb_entries = dict(_read_stamped_lines(DATASET / "rgb.txt"))
    depth_entries = dict(_read_stamped_lines(DATASET / "depth.txt"))
    gt_entries = _read_stamped_lines(DATASET / "groundtruth.txt")
    gt_times = np.array([t for t, _ in gt_entries])
    gt_positions = np.array([[float(v) for v in f[:3]] for _, f in gt_entries])

    pairs = _associate(sorted(rgb_entries), sorted(depth_entries))
    assert len(pairs) > 500, "association produced implausibly few rgb-depth pairs"
    pairs = pairs[::FRAME_STRIDE]

    vo = RgbdVo(K=K)
    est_track, gt_track = [], []
    lost = 0
    for t_rgb, t_depth in pairs:
        gray = cv2.imread(str(DATASET / rgb_entries[t_rgb][0]), cv2.IMREAD_GRAYSCALE)
        depth_png = cv2.imread(str(DATASET / depth_entries[t_depth][0]), cv2.IMREAD_UNCHANGED)
        depth_mm = (depth_png / DEPTH_COUNTS_PER_MM).astype(np.uint16)
        result = vo.process(gray, depth_mm)
        if not result.tracking:
            lost += 1
        # The integrated pose persists across a loss (the next good frame
        # re-anchors there), so every frame contributes a trajectory sample.
        est_track.append(vo.pose[:3, 3])
        gt_track.append(_nearest_groundtruth(gt_times, gt_positions, t_rgb))

    assert lost <= len(pairs) // 5, f"tracking lost on {lost}/{len(pairs)} frames"

    est_track = np.asarray(est_track)
    gt_track = np.asarray(gt_track)
    aligned = _umeyama_align(est_track, gt_track)
    ate_rmse = float(np.sqrt(np.mean(np.sum((aligned - gt_track) ** 2, axis=1))))
    path_length = float(np.sum(np.linalg.norm(np.diff(gt_track, axis=0), axis=1)))

    print(
        f"\nTUM freiburg1_xyz: ATE RMSE {ate_rmse:.3f} m over {path_length:.2f} m "
        f"path ({100.0 * ate_rmse / path_length:.1f}%), "
        f"{len(pairs)} frames (stride {FRAME_STRIDE}), {lost} tracking losses"
    )
    assert ate_rmse < 0.10 * path_length, (
        f"ATE RMSE {ate_rmse:.3f} m exceeds 10% of {path_length:.2f} m path"
    )
